/* SPDX-License-Identifier: GPL-2.0 */

#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <getopt.h>

#include <libubox/ustream.h>
#include <libubox/uloop.h>
#include <libubox/list.h>
#include <libubox/ulog.h>
#include <libubus.h>

#include <uci.h>
#include <uci_blob.h>

#define ULOG_DBG(fmt, ...) ulog(LOG_DEBUG, fmt, ## __VA_ARGS__)

typedef int (*poe_reply_handler)(unsigned char *reply);

/* Careful with this; Only works for set_detection/disconnect_type commands. */
#define PORT_ID_ALL	0x7f
#define MAX_PORT	24
#define MAX_RETRIES	5
#define GET_STR(a, b)	((a) < ARRAY_SIZE(b) ? (b)[a] : NULL)
#define MAX(a, b)	(((a) > (b)) ? (a) : (b))

struct port_config {
	char name[16];
	unsigned int valid : 1;
	unsigned int enable : 1;
	unsigned char priority;
	unsigned char power_up_mode;
	unsigned char power_budget;
};

struct config {
	float budget;
	float budget_guard;

	unsigned int port_count;
	uint8_t pse_id_set_budget_mask;
	struct port_config ports[MAX_PORT];
};

struct port_state {
	const char *status;
	float watt;
	const char *poe_mode;
};

struct state {
	const char *sys_mode;
	unsigned char sys_version;
	const char *sys_mcu;
	const char *sys_status;
	unsigned char sys_ext_version;
	float power_consumption;
	unsigned int num_detected_ports;

	struct port_state ports[MAX_PORT];
	struct uloop_timeout mcu_error_timeout;
};

struct cmd {
	struct list_head list;
	unsigned char cmd[12];
	unsigned int num_retries;
};

static struct ustream_fd stream;
static LIST_HEAD(cmd_pending);
static size_t num_pending;
static unsigned char cmd_seq;
static struct state state;
static struct blob_buf b;

static struct config config = {
	.budget = 65,
	.budget_guard = 7,
	.pse_id_set_budget_mask = 0x01,
};

static uint16_t read16_be(uint8_t *raw)
{
	return (uint16_t)raw[0] << 8 | raw[1];
}

static void write16_be(uint8_t *raw, uint16_t value)
{
	raw[0] = value >> 8;
	raw[1] =  value & 0xff;
}

static void load_port_config(struct uci_context *uci, struct uci_section *s)
{
	const char * name, *id_str, *enable, *priority, *poe_plus;
	unsigned long id;

	id_str = uci_lookup_option_string(uci, s, "id");
	name = uci_lookup_option_string(uci, s, "name");
	enable = uci_lookup_option_string(uci, s, "enable");
	priority = uci_lookup_option_string(uci, s, "priority");
	poe_plus = uci_lookup_option_string(uci, s, "poe_plus");

	if (!id_str || !name) {
		ULOG_ERR("invalid port with missing name and id");
		return;
	}

	id = strtoul(id_str, NULL, 0);
	if (!id || id > MAX_PORT) {
		ULOG_ERR("invalid port id=%lu for %s", id, name);
		return;
	}
	config.port_count = MAX(config.port_count, id);
	id--;

	strncpy(config.ports[id].name, name, sizeof(config.ports[id].name));
	config.ports[id].valid = 1;
	config.ports[id].enable = enable ? !strcmp(enable, "1") : 0;
	config.ports[id].priority = priority ? strtoul(priority, NULL, 0) : 0;
	if (config.ports[id].priority > 3)
		config.ports[id].priority = 3;

	if (poe_plus && !strcmp(poe_plus, "1"))
		config.ports[id].power_up_mode = 3;
}

static void load_global_config(struct uci_context *uci, struct uci_section *s)
{
	const char *budget, *guardband;

	budget = uci_lookup_option_string(uci, s, "budget");
	guardband = uci_lookup_option_string(uci, s, "guard");

	config.budget = budget ? strtof(budget, NULL) : 31.0;
	config.budget_guard = config.budget / 10;
	if (guardband)
		config.budget_guard = strtof(guardband, NULL);
}

static void
config_load(int init)
{
	struct uci_context *uci = uci_alloc_context();
        struct uci_package *package = NULL;

	memset(config.ports, 0, sizeof(config.ports));

	if (!uci_load(uci, "poe", &package)) {
		struct uci_element *e;

		if (init)
			uci_foreach_element(&package->sections, e) {
				struct uci_section *s = uci_to_section(e);

				if (!strcmp(s->type, "global"))
					load_global_config(uci, s);
			}
		uci_foreach_element(&package->sections, e) {
			struct uci_section *s = uci_to_section(e);

			if (!strcmp(s->type, "port"))
				load_port_config(uci, s);
		}
	}

	uci_unload(uci, package);
	uci_free_context(uci);
}

static char *get_board_compatible(void)
{
	char name[128];
	int fd, ret;

	fd = open("/sys/firmware/devicetree/base/compatible", O_RDONLY);
	if (fd < 0)
		return NULL;

	ret = read(fd, name, sizeof(name));
	if (ret < 0)
		return NULL;

	close(fd);

	return strndup(name, ret);
}

static void config_apply_quirks(struct config *config)
{
	char *compatible;

	compatible = get_board_compatible();
	if (!compatible) {
		ULOG_ERR("Can't get 'compatible': %s\n", strerror(errno));
		return;
	}

	if (!strcmp(compatible, "zyxel,gs1900-24hp-v1")) {
		/* Send budget command to first 8 PSE IDs */
		config->pse_id_set_budget_mask = 0xff;
	}

	free(compatible);
}

static void log_packet(int log_level, const char *prefix, const uint8_t d[12])
{
	ulog(log_level,
	     "%s %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
	     prefix, d[0], d[1], d[2], d[3], d[4], d[5],
		     d[6], d[7], d[8], d[9], d[10], d[11]);
}

static int
poe_cmd_send(struct cmd *cmd)
{
	if (state.mcu_error_timeout.pending)
		return -EBUSY;

	log_packet(LOG_DEBUG, "TX ->", cmd->cmd);
	ustream_write(&stream.stream, (char *)cmd->cmd, 12, false);

	return 0;
}

static int
poe_cmd_next(void)
{
	struct cmd *cmd;

	if (list_empty(&cmd_pending))
		return -1;

	cmd = list_first_entry(&cmd_pending, struct cmd, list);

	return poe_cmd_send(cmd);
}

static int
poe_cmd_queue(unsigned char *_cmd, int len)
{
	int i, empty = list_empty(&cmd_pending);
	struct cmd *cmd = malloc(sizeof(*cmd));

	memset(cmd, 0, sizeof(*cmd));
	memset(cmd->cmd, 0xff, 12);
	memcpy(cmd->cmd, _cmd, len);

	cmd_seq++;
	cmd->cmd[1] = cmd_seq;
	cmd->cmd[11] = 0;

	for (i = 0; i < 11; i++)
		cmd->cmd[11] += cmd->cmd[i];

	list_add_tail(&cmd->list, &cmd_pending);
	num_pending++;

	if (empty)
		return poe_cmd_send(cmd);

	return 0;
}

static int poet_cmd_4_port(uint8_t cmd_id, uint8_t port[4], uint8_t data[4])
{
	uint8_t cmd[] = { cmd_id, 0x00, port[0], data[0], port[1], data[1],
					port[2], data[2], port[3], data[3] };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

/* 0x00 - Set port enable
 *	0: Disable
 *	1: Enable
 */
static int
poe_cmd_port_enable(unsigned char port, unsigned char enable)
{
	unsigned char cmd[] = { 0x00, 0x00, port, enable };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

static int poe_cmd_port_mapping_enable(bool enable)
{
	unsigned char cmd[] = { 0x02, 0x00, enable };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

/* 0x06 - Set global port enable
 *	0: Disable PSE Functionality on all Ports
 *	1: Enable PSE Functionality on all Ports
 *	2: Enable Force power Functionality on all ports
 *	3: Enable Force Power with Disconnect Functionality on all Ports
 */
static int
poe_cmd_global_port_enable(unsigned char enable)
{
	unsigned char cmd[] = { 0x06, 0x00, enable };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

/* 0x10 - Set port detection type
 *	1: Legacy Capacitive Detection only
 *	2: IEEE 802.3af 4-Point Detection only (Default)
 *	3: IEEE 802.3af 4-Point followed by Legacy
 *	4: IEEE 802.3af 2-Point detection (Not Supported)
 *	5: IEEE 802.3af 2-Point followed by Legacy
 */
static int
poe_cmd_port_detection_type(unsigned char port, unsigned char type)
{
	unsigned char cmd[] = { 0x10, 0x00, port, type };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

/* 0x11 - Set port classification
 *	0: Disable
 *	1: Enable
 */
static int poe_cmd_port_classification(uint8_t port[4], uint8_t enable[4])
{
	return poet_cmd_4_port(0x11, port, enable);
}

/* 0x13 - Set port disconnect type
 *	0: none
 *	1: AC-disconnect
 *	2: DC-disconnect
 *	3: DC with delay
 */
static int
poe_cmd_port_disconnect_type(unsigned char port, unsigned char type)
{
	unsigned char cmd[] = { 0x13, 0x00, port, type };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

/* 0x15 - Set port power limit type
 *	0: None. Power limit is 16.2W if the connected device is “low power”,
 *	   or the set high power limit if the device is “high power”.
 *	1: Class based. The power limit for class 4 devices is determined by the high power limit.
 *	2: User defined
 */
static int poe_cmd_port_power_limit_type(uint8_t port[4], uint8_t limit[4])
{
	return poet_cmd_4_port(0x15, port, limit);
}

/* 0x16 - Set port power budget
 *	values in 0.2W increments
 */
static int
poe_cmd_port_power_budget(unsigned char port, unsigned char budget)
{
	unsigned char cmd[] = { 0x16, 0x00, port, budget };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

/* 0x17 - Set power management mode
 *	0: None (No Power Management mode) (Default in Semi-Auto mode)
 *	1: Static Power Management with Port Priority(Default in Automode)
 *	2: Dynamic Power Management with Port Priority
 *	3: Static Power Management without Port Priority
 *	4: Dynamic Power Management without Port Priority
 */
static int
poe_cmd_power_mgmt_mode(unsigned char mode)
{
	unsigned char cmd[] = { 0x17, 0x00, mode };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

/* 0x18 - Set global power budget */
static int poe_cmd_global_power_budget(uint8_t pse, float budget, float guard)
{
	uint8_t cmd[] = { 0x18, 0x00, pse, 0x00, 0x00, 0x00, 0x00 };

	write16_be(cmd + 3, budget * 10);
	write16_be(cmd + 5, guard * 10);

	return poe_cmd_queue(cmd, sizeof(cmd));
}

/* 0x1a - Set port priority
 *	0: Low
 *	1: Normal
 *	2: High
 *	3: Critical
 */
static int poe_set_port_priority(uint8_t port[4], uint8_t priority[4])
{
	return poet_cmd_4_port(0x1a, port, priority);
}

/* 0x1c - Set port power-up mode
 *	0: PoE
 *	1: legacy
 *	2: pre-PoE+
 *	3: PoE+
 */
static int poe_set_port_power_up_mode(uint8_t port[4], uint8_t mode[4])
{
	return poet_cmd_4_port(0x1c, port, mode);
}

/* 0x20 - Get system info */
static int
poe_cmd_status(void)
{
	unsigned char cmd[] = { 0x20 };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

static int
poe_reply_status(unsigned char *reply)
{
	const char *mode[] = {
		"Semi-auto I2C",
		"Semi-auto UART",
		"Auto I2C",
		"Auto UART"
	};
	const char *mcu[] = {
		"ST Micro ST32F100 Microcontroller",
		"Nuvoton M05xx LAN Microcontroller",
		"ST Micro STF030C8 Microcontroller",
		"Nuvoton M058SAN Microcontroller",
		"Nuvoton NUC122 Microcontroller"
	};
	const char *status[] = {
		"Global Disable pin is de-asserted:No system reset from the previous query cmd:Configuration saved",
		"Global Disable pin is de-asserted:No system reset from the previous query cmd:Configuration Dirty",
		"Global Disable pin is de-asserted:System reseted:Configuration saved",
		"Global Disable pin is de-asserted:System reseted:Configuration Dirty",
		"Global Disable Pin is asserted:No system reset from the previous query cmd:Configuration saved",
		"Global Disable Pin is asserted:No system reset from the previous query cmd:Configuration Dirty",
		"Global Disable Pin is asserted:System reseted:Configuration saved",
		"Global Disable Pin is asserted:System reseted:Configuration Dirty"
	};

	state.sys_mode = GET_STR(reply[2], mode);
	state.num_detected_ports = reply[3];
	state.sys_version = reply[7];
	state.sys_mcu = GET_STR(reply[8], mcu);
	state.sys_status = GET_STR(reply[9], status);
	state.sys_ext_version = reply[10];

	return 0;
}

/* 0x23 - Get power statistics */
static int
poe_cmd_power_stats(void)
{
	unsigned char cmd[] = { 0x23 };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

static int
poe_reply_power_stats(unsigned char *reply)
{
	state.power_consumption = read16_be(reply + 2) * 0.1;

	return 0;
}

/* 0x25 - Get port config */
static int poe_cmd_port_config(unsigned char port)
{
	unsigned char cmd[] = { 0x25, 0x00, port };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

/* 0x26 - Get extended port config */
static int
poe_cmd_port_ext_config(unsigned char port)
{
	unsigned char cmd[] = { 0x26, 0x00, port };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

static int
poe_reply_port_ext_config(unsigned char *reply)
{
	const char *mode[] = {
		"PoE",
		"Legacy",
		"pre-PoE+",
		"PoE+"
	};

	state.ports[reply[2]].poe_mode = GET_STR(reply[3], mode);

	return 0;
}

/* 0x28 - Get all all port status */
static int poe_cmd_4_port_status(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4)
{
	uint8_t cmd[] = { 0x28, 0x00, p1, 1, p2, 1, p3, 1, p4, 1 };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

static int poe_reply_4_port_status(uint8_t *reply)
{
	int i, port, pstate;

	const char *status[] = {
		[0] = "Disabled",
		[1] = "Searching",
		[2] = "Delivering power",
		[4] = "Fault",
		[5] = "Other fault",
		[6] = "Requesting power",
	};

	for (i = 2; i < 11; i+=2) {
		port = reply[i];
		pstate = reply[i + 1];

		if (port == 0xff) {
			continue;
		} else if (port >= MAX_PORT) {
			ULOG_WARN("Invalid port status packet (port=%d)\n", port);
			return -1;
		}

		state.ports[port].status = GET_STR(pstate & 0xf, status);
	}

	return 0;
}

/* 0x30 - Get port power statistics */
static int
poe_cmd_port_power_stats(unsigned char port)
{
	unsigned char cmd[] = { 0x30, 0x00, port };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

static int
poe_reply_port_power_stats(unsigned char *reply)
{
	int port_idx = reply[2];

	state.ports[port_idx].watt = read16_be(reply + 9) * 0.1;
	return 0;
}

static poe_reply_handler reply_handler[] = {
	[0x20] = poe_reply_status,
	[0x23] = poe_reply_power_stats,
	[0x26] = poe_reply_port_ext_config,
	[0x28] = poe_reply_4_port_status,
	[0x30] = poe_reply_port_power_stats,
};

static void mcu_clear_timeout(struct uloop_timeout *t)
{
	poe_cmd_next();
}

static void handle_poe_f0_reply(struct cmd *cmd, uint8_t *reply)
{
	const char *reason;

	const char *reasons[] = {
		[0xd] = "request-incomplete",
		[0xe] = "request-bad-checksum",
		[0xf] = "not-ready",
	};

	reason = GET_STR((uint8_t)(reply[0] - 0xf0), reasons);
	reason = reason ? reason : "unknown";

	/* Log the first reply, then only log complete failures. */
	if (cmd->num_retries == 0) {
		ULOG_NOTE("MCU rejected command: %s\n", reason);
		log_packet(LOG_NOTICE, "\tCMD:   ", cmd->cmd);
		log_packet(LOG_NOTICE, "\treply: ", reply);
	}

	if (!state.mcu_error_timeout.pending) {
		if (++cmd->num_retries > MAX_RETRIES) {
			ULOG_ERR("Aborting request (%02x) after %d attempts\n",
				 cmd->cmd[0], cmd->num_retries);
			free(cmd);
			return;
		}

		/* Wait for the MCU to recover */
		state.mcu_error_timeout.cb = mcu_clear_timeout;
		uloop_timeout_set(&state.mcu_error_timeout, 100);
	}

	list_add(&cmd->list, &cmd_pending);
}

static int
poe_reply_consume(unsigned char *reply)
{
	struct cmd *cmd = NULL;
	unsigned char sum = 0, i;
	uint8_t cmd_id, cmd_seq;

	log_packet(LOG_DEBUG, "RX <-", reply);

	if (list_empty(&cmd_pending)) {
		ULOG_ERR("received unsolicited reply\n");
		return -1;
	}

	cmd = list_first_entry(&cmd_pending, struct cmd, list);
	list_del(&cmd->list);
	num_pending--;
	cmd_id = cmd->cmd[0];
	cmd_seq = cmd->cmd[1];

	for (i = 0; i < 11; i++)
		sum += reply[i];

	if (reply[11] != sum) {
		ULOG_DBG("received reply with bad checksum\n");
		free(cmd);
		return -1;
	}

	if ((reply[0] & 0xf0) == 0xf0) {
		handle_poe_f0_reply(cmd, reply);
		return -1;
	}

	free(cmd);

	if ((reply[0] != cmd_id) || (reply[0] > ARRAY_SIZE(reply_handler))) {
		ULOG_DBG("received reply with bad command id\n");
		return -1;
	}

	if (reply[1] != cmd_seq) {
		ULOG_DBG("received reply with bad sequence number\n");
		return -1;
	}

	if (reply_handler[reply[0]]) {
	  return reply_handler[reply[0]](reply);
	}

	return 0;
}

static void
poe_stream_msg_cb(struct ustream *s, int bytes)
{
	int len;
	unsigned char *reply = (unsigned char *)ustream_get_read_buf(s, &len);

	if (len < 12)
		return;
	poe_reply_consume(reply);
	ustream_consume(s, 12);
	poe_cmd_next();
}

static void
poe_stream_notify_cb(struct ustream *s)
{
	if (!s->eof)
		return;

	ULOG_ERR("tty error, shutting down\n");
	exit(-1);
}

static int
poe_stream_open(char *dev, struct ustream_fd *s, speed_t speed)
{
	int ret, tty;

	struct termios tio = {
		.c_oflag = 0,
		.c_iflag = 0,
		.c_cflag = speed | CS8 | CREAD | CLOCAL,
		.c_lflag = 0,
		.c_cc = {
			[VMIN] = 1,
		}
	};

	tty = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (tty < 0) {
		ULOG_ERR("%s: device open failed: %s\n", dev, strerror(errno));
		return -1;
	}

	ret = tcsetattr(tty, TCSANOW, &tio);
	if (ret) {
		ULOG_ERR("Can't configure serial port: %s", strerror(errno));
		return -errno;
	}

	s->stream.string_data = false;
	s->stream.notify_read = poe_stream_msg_cb;
	s->stream.notify_state = poe_stream_notify_cb;

	ustream_fd_init(s, tty);
	tcflush(tty, TCIFLUSH);

	return 0;
}

static int poet_setup(const struct port_config *ports, size_t num_ports)
{
	uint8_t port_ids[4], priorities[4], powerup_mode[4], limit_type[4];
	uint8_t enable_all[4] = {1, 1, 1, 1};
	size_t i = 0, num_okay = 0;

	do {
		for ( ; i < num_ports; i++) {
			if (!ports[i].enable)
				continue;

			port_ids[num_okay] = i;
			priorities[num_okay] = ports[i].priority;
			powerup_mode[num_okay] = ports[i].power_up_mode;
			limit_type[num_okay] = (ports[i].power_budget) ? 2 : 1;

			if (++num_okay == 4)
				break;
		};

		memset(enable_all + num_okay, 0xff, 4 - num_okay);
		memset(port_ids + num_okay, 0xff, 4 - num_okay);
		memset(priorities + num_okay, 0xff, 4 - num_okay);
		memset(powerup_mode + num_okay, 0xff, 4 - num_okay);
		memset(limit_type + num_okay, 0xff, 4 - num_okay);

		poe_set_port_priority(port_ids, priorities);
		poe_set_port_power_up_mode(port_ids, powerup_mode);
		poe_cmd_port_classification(port_ids, enable_all);
		poe_cmd_port_power_limit_type(port_ids, limit_type);

		num_okay = 0;
	} while (++i < num_ports);

	return 0;
}

static int
poe_port_setup(void)
{
	size_t i;

	poe_cmd_port_disconnect_type(PORT_ID_ALL, 2);
	poe_cmd_port_detection_type(PORT_ID_ALL, 3);

	for (i = 0; i < config.port_count; i++) {
		if (!config.ports[i].enable || !config.ports[i].power_budget)
			continue;

		poe_cmd_port_power_budget(i, config.ports[i].power_budget);
	}

	poet_setup(config.ports, config.port_count);

	for (i = 0; i < config.port_count; i++)
		poe_cmd_port_enable(i, !!config.ports[i].enable);

	return 0;
}

static void poe_set_power_budget(const struct config *config)
{
	unsigned int pse;

	for (pse = 0; pse < 8; pse++) {
		if (!(config->pse_id_set_budget_mask & (1 << pse)))
			continue;

		poe_cmd_global_power_budget(pse, config->budget,
					    config->budget_guard);
	}
}

static int
poe_initial_setup(void)
{
	poe_cmd_status();
	poe_cmd_power_mgmt_mode(2);
	poe_cmd_port_mapping_enable(false);
	poe_cmd_global_port_enable(0);
	poe_set_power_budget(&config);

	poe_port_setup();
	poe_port_setup();
	poe_port_setup();

	return 0;
}

static void
state_timeout_cb(struct uloop_timeout *t)
{
	size_t i;

	poe_cmd_power_stats();

	for (i = 0; i < config.port_count; i += 4)
		poe_cmd_4_port_status(i, i + 1, i + 2, i + 3);

	for (i = 0; i < config.port_count; i++) {
		poe_cmd_port_config(i);
		poe_cmd_port_ext_config(i);
		poe_cmd_port_power_stats(i);
	}


	for (i = 0; i < config.port_count; i += 4)
		poe_cmd_4_port_status(i, i + 1, i + 2, i + 3);

	for (i = 0; i < config.port_count; i++) {
		poe_cmd_port_config(i);
		poe_cmd_port_ext_config(i);
		poe_cmd_port_power_stats(i);
	}


	for (i = 0; i < config.port_count; i += 4)
		poe_cmd_4_port_status(i, i + 1, i + 2, i + 3);

	for (i = 0; i < config.port_count; i++) {
		poe_cmd_port_config(i);
		poe_cmd_port_ext_config(i);
		poe_cmd_port_power_stats(i);
	}

	for (i = 0; i < config.port_count; i += 4)
		poe_cmd_4_port_status(i, i + 1, i + 2, i + 3);

	for (i = 0; i < config.port_count; i++) {
		poe_cmd_port_ext_config(i);
		poe_cmd_port_power_stats(i);
	}

	for (i = 0; i < config.port_count; i += 4)
		poe_cmd_4_port_status(i, i + 1, i + 2, i + 3);

	for (i = 0; i < config.port_count; i++) {
		poe_cmd_port_ext_config(i);
		poe_cmd_port_power_stats(i);
	}

	ULOG_INFO("TIMEOUT rearmed (%zu pending)\n", num_pending);
	uloop_timeout_set(t, 2 * 1000);
}

static int
ubus_poe_info_cb(struct ubus_context *ctx, struct ubus_object *obj,
		 struct ubus_request_data *req, const char *method,
		 struct blob_attr *msg)
{
	char tmp[16];
	size_t i;
	void *c;

	blob_buf_init(&b, 0);

	snprintf(tmp, sizeof(tmp), "v%u.%u",
		 state.sys_version, state.sys_ext_version);
	blobmsg_add_string(&b, "firmware", tmp);
	if (state.sys_mcu)
		blobmsg_add_string(&b, "mcu", state.sys_mcu);
	blobmsg_add_double(&b, "budget", config.budget);
	blobmsg_add_double(&b, "consumption", state.power_consumption);

	c = blobmsg_open_table(&b, "ports");
	for (i = 0; i < config.port_count; i++) {
		void *p;

		if (!config.ports[i].valid)
			continue;

		p = blobmsg_open_table(&b, config.ports[i].name);

		blobmsg_add_u32(&b, "priority", config.ports[i].priority);

		if (state.ports[i].poe_mode)
			blobmsg_add_string(&b, "mode", state.ports[i].poe_mode);
		if (state.ports[i].status)
			blobmsg_add_string(&b, "status", state.ports[i].status);
		else
			blobmsg_add_string(&b, "status", "unknown");
		if (state.ports[i].watt)
			blobmsg_add_double(&b, "consumption", state.ports[i].watt);

		blobmsg_close_table(&b, p);
	}
	blobmsg_close_table(&b, c);

	ubus_send_reply(ctx, req, b.head);

	return UBUS_STATUS_OK;
}

static const struct blobmsg_policy ubus_poe_sendframe_policy[] = {
	{ "frame", BLOBMSG_TYPE_STRING },
};

static int
ubus_poe_sendframe_cb(struct ubus_context *ctx, struct ubus_object *obj,
		   struct ubus_request_data *req, const char *method,
		   struct blob_attr *msg)
{
	struct blob_attr *tb[ARRAY_SIZE(ubus_poe_sendframe_policy)];
	char *frame, *next, *end;
	size_t cmd_len = 0;
	unsigned long byte_val;
	uint8_t cmd[9];

	blobmsg_parse(ubus_poe_sendframe_policy,
		      ARRAY_SIZE(ubus_poe_sendframe_policy),
		      tb, blob_data(msg), blob_len(msg));
	if (!*tb)
		return UBUS_STATUS_INVALID_ARGUMENT;

	frame = blobmsg_get_string(*tb);
	end = frame + strlen(frame);
	next = frame;

	while ((next < end) && (cmd_len < sizeof(cmd))) {
		errno = 0;
		byte_val = strtoul(frame, &next, 16);
		if (errno || (frame == next) || (byte_val > 0xff))
			return UBUS_STATUS_INVALID_ARGUMENT;

		cmd[cmd_len++] = byte_val;
		frame = next;
	}

	return poe_cmd_queue(cmd, cmd_len);
}

static int
ubus_poe_reload_cb(struct ubus_context *ctx, struct ubus_object *obj,
		   struct ubus_request_data *req, const char *method,
		   struct blob_attr *msg)
{
	config_load(0);
	poe_port_setup();

	return UBUS_STATUS_OK;
}

static const struct ubus_method ubus_poe_methods[] = {
	UBUS_METHOD_NOARG("info", ubus_poe_info_cb),
	UBUS_METHOD_NOARG("reload", ubus_poe_reload_cb),
	UBUS_METHOD("sendframe", ubus_poe_sendframe_cb, ubus_poe_sendframe_policy),
};

static struct ubus_object_type ubus_poe_object_type =
	UBUS_OBJECT_TYPE("poe", ubus_poe_methods);

static struct ubus_object ubus_poe_object = {
	.name = "poe",
	.type = &ubus_poe_object_type,
	.methods = ubus_poe_methods,
	.n_methods = ARRAY_SIZE(ubus_poe_methods),
};

static void
ubus_connect_handler(struct ubus_context *ctx)
{
	int ret;

	ret = ubus_add_object(ctx, &ubus_poe_object);
	if (ret)
		ULOG_ERR("Failed to add object: %s\n", ubus_strerror(ret));
}

int
main(int argc, char ** argv)
{
	int ch;

	struct uloop_timeout state_timeout = {
		.cb = state_timeout_cb,
	};

	struct ubus_auto_conn conn = {
		.cb = ubus_connect_handler,
	};

	ulog_open(ULOG_STDIO | ULOG_SYSLOG, LOG_DAEMON, "realtek-poe");
	ulog_threshold(LOG_INFO);

	while ((ch = getopt(argc, argv, "d")) != -1) {
		switch (ch) {
		case 'd':
			ulog_threshold(LOG_DEBUG);
			break;
		}
	}

	config_load(1);
	config_apply_quirks(&config);

	uloop_init();
	ubus_auto_connect(&conn);

	if (poe_stream_open("/dev/ttyS1", &stream, B19200) < 0)
		return -1;


	poe_initial_setup();
	uloop_timeout_set(&state_timeout, 1000);
	uloop_run();
	uloop_done();

	return 0;
}
