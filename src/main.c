/* SPDX-License-Identifier: GPL-2.0 */

#include "tek-poe.h"

#include <string.h>
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

#define MAX_RETRIES	5

struct mcu {
	struct uloop_timeout error_timeout;
	struct list_head pending_cmds;
	struct ustream_fd stream;
	struct mcu_state state;
	uint8_t cmd_seq;
};

struct cmd {
	struct list_head list;
	uint8_t cmd[12];
	unsigned int num_retries;
};

struct poe_ctx {
	struct mcu mcu;
	struct config config;
	struct ubus_auto_conn conn;
	struct blob_buf blob_buf;
	struct uloop_timeout state_timeout;
	unsigned int hardcore_hacking_mode_en : 1;
};

static const struct poe_dialect *dialect = &broadcom_dialect;

static struct poe_ctx *ubus_to_poe_ctx(struct ubus_context *u)
{
	struct ubus_auto_conn *c = container_of(u, struct ubus_auto_conn, ctx);
	return container_of(c, struct poe_ctx, conn);
}

static void load_port_config(struct config *cfg, struct uci_context *uci,
			     struct uci_section *s)
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
	cfg->port_count = MAX(cfg->port_count, id);
	id--;

	strncpy(cfg->ports[id].name, name, sizeof(cfg->ports[id].name));
	cfg->ports[id].valid = 1;
	cfg->ports[id].enable = enable ? !strcmp(enable, "1") : 0;
	cfg->ports[id].priority = priority ? strtoul(priority, NULL, 0) : 0;
	if (cfg->ports[id].priority > 3)
		cfg->ports[id].priority = 3;

	if (poe_plus && !strcmp(poe_plus, "1"))
		cfg->ports[id].power_up_mode = 3;
}

static void load_global_config(struct config *cfg, struct uci_context *uci,
			       struct uci_section *s)

{
	const char *budget, *guardband;

	budget = uci_lookup_option_string(uci, s, "budget");
	guardband = uci_lookup_option_string(uci, s, "guard");

	cfg->budget = budget ? strtof(budget, NULL) : 31.0;
	cfg->budget_guard = cfg->budget / 10;
	if (guardband)
		cfg->budget_guard = strtof(guardband, NULL);
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

static void config_load(struct config *cfg, int init)
{
	struct uci_context *uci = uci_alloc_context();
	struct uci_package *package = NULL;

	memset(cfg->ports, 0, sizeof(cfg->ports));

	if (!uci_load(uci, "poe", &package)) {
		struct uci_element *e;

		if (init) {
			uci_foreach_element(&package->sections, e) {
				struct uci_section *s = uci_to_section(e);

				if (!strcmp(s->type, "global"))
					load_global_config(cfg, uci, s);
			}

			config_apply_quirks(cfg);
		}
		uci_foreach_element(&package->sections, e) {
			struct uci_section *s = uci_to_section(e);

			if (!strcmp(s->type, "port"))
				load_port_config(cfg, uci, s);
		}
	}

	uci_unload(uci, package);
	uci_free_context(uci);
}

static void log_packet(int log_level, const char *prefix, const uint8_t d[12])
{
	ulog(log_level,
	     "%s %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
	     prefix, d[0], d[1], d[2], d[3], d[4], d[5],
		     d[6], d[7], d[8], d[9], d[10], d[11]);
}

static int mcu_cmd_send(struct mcu *mcu, struct cmd *cmd)
{
	if (mcu->error_timeout.pending)
		return -EBUSY;

	log_packet(LOG_DEBUG, "TX ->", cmd->cmd);
	return ustream_write(&mcu->stream.stream, (void *)cmd->cmd, 12, false);
}

static int mcu_cmd_next(struct mcu *mcu)
{
	struct cmd *cmd;

	if (list_empty(&mcu->pending_cmds))
		return -EAGAIN;

	cmd = list_first_entry(&mcu->pending_cmds, struct cmd, list);

	return mcu_cmd_send(mcu, cmd);
}

int mcu_queue_cmd(struct mcu *mcu, uint8_t *cmd_buf, size_t len)
{
	int i, empty = list_empty(&mcu->pending_cmds);
	struct cmd *cmd = malloc(sizeof(*cmd));

	memset(cmd, 0, sizeof(*cmd));
	memset(cmd->cmd, 0xff, 12);
	memcpy(cmd->cmd, cmd_buf, len);

	mcu->cmd_seq++;
	cmd->cmd[1] = mcu->cmd_seq;
	cmd->cmd[11] = 0;

	for (i = 0; i < 11; i++)
		cmd->cmd[11] += cmd->cmd[i];

	list_add_tail(&cmd->list, &mcu->pending_cmds);

	if (empty)
		return mcu_cmd_send(mcu, cmd);

	return 0;
}

static void mcu_clear_timeout(struct uloop_timeout *t)
{
	struct mcu *mcu = container_of(t, struct mcu, error_timeout);

	mcu_cmd_next(mcu);
}

static void handle_f0_reply(struct mcu *mcu, struct cmd *cmd, uint8_t *reply)
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

	if (!mcu->error_timeout.pending) {
		if (++cmd->num_retries > MAX_RETRIES) {
			ULOG_ERR("Aborting request (%02x) after %d attempts\n",
				 cmd->cmd[0], cmd->num_retries);
			free(cmd);
			return;
		}

		/* Wait for the MCU to recover */
		mcu->error_timeout.cb = mcu_clear_timeout;
		uloop_timeout_set(&mcu->error_timeout, 100);
	}

	list_add(&cmd->list, &mcu->pending_cmds);
}

static int mcu_handle_reply(struct mcu *mcu, uint8_t *reply)
{
	struct cmd *cmd = NULL;
	uint8_t sum = 0, i, cmd_id, cmd_seq;

	log_packet(LOG_DEBUG, "RX <-", reply);

	if (list_empty(&mcu->pending_cmds)) {
		ULOG_ERR("received unsolicited reply\n");
		return -1;
	}

	cmd = list_first_entry(&mcu->pending_cmds, struct cmd, list);
	list_del(&cmd->list);
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
		handle_f0_reply(mcu, cmd, reply);
		return -1;
	}

	free(cmd);

	if ((reply[0] != cmd_id)) {
		ULOG_DBG("received reply with bad command id\n");
		return -1;
	}

	if (reply[1] != cmd_seq) {
		ULOG_DBG("received reply with bad sequence number\n");
		return -1;
	}

	return dialect->handle_reply(&mcu->state, reply, 12);
}

static void poe_stream_msg_cb(struct ustream *s, int bytes)
{
	struct ustream_fd *ufd = container_of(s, struct ustream_fd, stream);
	struct mcu *mcu = container_of(ufd, struct mcu, stream);
	int len;
	uint8_t *reply = (uint8_t *)ustream_get_read_buf(s, &len);

	if (len < 12)
		return;
	mcu_handle_reply(mcu, reply);
	ustream_consume(s, 12);
	mcu_cmd_next(mcu);
}

static void poe_stream_notify_cb(struct ustream *s)
{
	if (!s->eof)
		return;

	ULOG_ERR("tty error, shutting down\n");
	exit(-1);
}

static int poe_stream_open(char *dev, struct ustream_fd *s, speed_t speed)
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

static void state_timeout_cb(struct uloop_timeout *t)
{
	struct poe_ctx *poe = container_of(t, struct poe_ctx, state_timeout);

	dialect->poll_async(&poe->mcu, &poe->config);
	uloop_timeout_set(t, 2 * 1000);
}

static int ubus_poe_info_cb(struct ubus_context *ctx, struct ubus_object *obj,
			    struct ubus_request_data *req, const char *method,
			    struct blob_attr *msg)
{
	struct poe_ctx *poe = ubus_to_poe_ctx(ctx);
	const struct mcu_state *state = &poe->mcu.state;
	const struct config *cfg = &poe->config;
	struct blob_buf *b = &poe->blob_buf;
	char tmp[16];
	size_t i;
	void *c;

	blob_buf_init(b, 0);

	snprintf(tmp, sizeof(tmp), "v%u.%u",
		 state->sys_version, state->sys_ext_version);
	blobmsg_add_string(b, "firmware", tmp);
	if (state->sys_mcu)
		blobmsg_add_string(b, "mcu", state->sys_mcu);
	blobmsg_add_double(b, "budget", cfg->budget);
	blobmsg_add_double(b, "consumption", state->power_consumption);

	c = blobmsg_open_table(b, "ports");
	for (i = 0; i < cfg->port_count; i++) {
		void *p;

		if (!cfg->ports[i].valid)
			continue;

		p = blobmsg_open_table(b, cfg->ports[i].name);

		blobmsg_add_u32(b, "priority", cfg->ports[i].priority);

		if (state->ports[i].poe_mode)
			blobmsg_add_string(b, "mode", state->ports[i].poe_mode);
		if (state->ports[i].status)
			blobmsg_add_string(b, "status", state->ports[i].status);
		else
			blobmsg_add_string(b, "status", "unknown");
		if (state->ports[i].watt)
			blobmsg_add_double(b, "consumption", state->ports[i].watt);

		blobmsg_close_table(b, p);
	}
	blobmsg_close_table(b, c);

	ubus_send_reply(ctx, req, b->head);

	return UBUS_STATUS_OK;
}

static int ubus_poe_debug_cb(struct ubus_context *ctx, struct ubus_object *obj,
			    struct ubus_request_data *req, const char *method,
			    struct blob_attr *msg)
{
	struct poe_ctx *poe = ubus_to_poe_ctx(ctx);
	const struct mcu_state *state = &poe->mcu.state;
	const struct config *cfg = &poe->config;
	struct blob_buf *b = &poe->blob_buf;
	size_t i;
	void *c, *p;

	blob_buf_init(b, 0);

	blobmsg_add_double(b, "reported_budget", state->reported_power_budget);


	blobmsg_add_u32(b, "num_detected_ports", state->num_detected_ports);
	blobmsg_add_u32(b, "port_map_en", state->port_map_en);
	blobmsg_add_u32(b, "device_id", state->device_id);

	if (state->has_ext_cfg_info) {
		blobmsg_add_double(b, "uvlo_threshold", state->uvlo_threshold);
		blobmsg_add_double(b, "ovlo_threshold", state->ovlo_threshold);
		blobmsg_add_u32(b, "pre_alloc", state->pre_alloc);
		blobmsg_add_u32(b, "powerup_mode", state->powerup_mode);
		blobmsg_add_u32(b, "disconnect_type", state->disconnect_type);
		blobmsg_add_u32(b, "ddflag", state->ddflag);
		blobmsg_add_u32(b, "num_pse", state->num_pse);
	}

	c = blobmsg_open_table(b, "ports");
	for (i = 0; i < cfg->port_count; i++) {
		if (!cfg->ports[i].valid)
			continue;

		p = blobmsg_open_table(b, cfg->ports[i].name);

		blobmsg_add_u32(b, "power_limit_type", state->ports[i].power_limit_type);
		blobmsg_add_double(b, "power_budget", state->ports[i].power_budget);
		blobmsg_add_u32(b, "priority", state->ports[i].priority);
		blobmsg_add_u32(b, "primary_pse_output", state->ports[i].primary_pse_output);
		blobmsg_add_u32(b, "mapping", state->ports[i].mapping);

		if (state->ports[i].has_ext_config) {
			blobmsg_add_u32(b, "enabled", state->ports[i].enabled);
			blobmsg_add_u32(b, "auto_powerup", state->ports[i].auto_powerup);
			blobmsg_add_u32(b, "detection_type", state->ports[i].detection_type);
			blobmsg_add_u32(b, "classification_enable", state->ports[i].classification_enable);
			blobmsg_add_u32(b, "disconnect_type", state->ports[i].disconnect_type);
			blobmsg_add_u32(b, "pair", state->ports[i].pair);
		}

		if (state->ports[i].has_detailed_state) {
			blobmsg_add_u32(b, "class_info", state->ports[i].class_info);
			blobmsg_add_u32(b, "pd_type", state->ports[i].pd_type);
			blobmsg_add_u32(b, "mpss_mask", state->ports[i].mpss_mask);
		}

		blobmsg_close_table(b, p);
	}
	blobmsg_close_table(b, c);

	p = blobmsg_open_table(b, "mapping");
	blobmsg_add_u32(b, "enabled", state->port_map_en);
	for (i = 0; i < cfg->port_count; i++) {
		if (!cfg->ports[i].valid)
			continue;

		blobmsg_add_u32(b, cfg->ports[i].name, state->ports[i].mapping);
	}
	blobmsg_close_table(b, p);

	ubus_send_reply(ctx, req, b->head);

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
	struct poe_ctx *poe = ubus_to_poe_ctx(ctx);
	struct mcu *mcu = &poe->mcu;
	char *frame, *next, *end;
	size_t cmd_len = 0;
	unsigned long byte_val;
	uint8_t cmd[9];
	int ret;

	if (!poe->hardcore_hacking_mode_en)
		return UBUS_STATUS_PERMISSION_DENIED;

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

	ret = mcu_queue_cmd(mcu, cmd, cmd_len);
	return (ret < 0) ?  UBUS_STATUS_SYSTEM_ERROR : UBUS_STATUS_OK;
}

static int ubus_poe_reload_cb(struct ubus_context *ctx, struct ubus_object *obj,
			      struct ubus_request_data *req, const char *method,
			      struct blob_attr *msg)
{
	struct poe_ctx *poe = ubus_to_poe_ctx(ctx);

	config_load(&poe->config, 0);
	dialect->init_ports_async(&poe->mcu, &poe->config);

	return UBUS_STATUS_OK;
}

static const struct blobmsg_policy ubus_poe_manage_policy[] = {
	{ "port", BLOBMSG_TYPE_STRING },
	{ "enable", BLOBMSG_TYPE_BOOL },
};

static int ubus_poe_manage_cb(struct ubus_context *ctx, struct ubus_object *obj,
			      struct ubus_request_data *req, const char *method,
			      struct blob_attr *msg)
{
	struct blob_attr *tb[ARRAY_SIZE(ubus_poe_manage_policy)];
	struct poe_ctx *poe = ubus_to_poe_ctx(ctx);
	const struct config *cfg = &poe->config;
	const struct port_config *port;
	struct mcu *mcu = &poe->mcu;
	const char *port_name;
	size_t i;

	blobmsg_parse(ubus_poe_manage_policy,
		      ARRAY_SIZE(ubus_poe_manage_policy),
		      tb, blob_data(msg), blob_len(msg));
	if (!tb[0] || !tb[1])
		return UBUS_STATUS_INVALID_ARGUMENT;

	port_name = blobmsg_get_string(tb[0]);
	for (i = 0; i < cfg->port_count; i++) {
		port = &cfg->ports[i];
		if (!port->enable || strcmp(port_name, port->name))
			continue;
		return dialect->enable_port_async(mcu, i, blobmsg_get_bool(tb[1]));
	}
	return UBUS_STATUS_INVALID_ARGUMENT;
}

static const struct ubus_method ubus_poe_methods[] = {
	UBUS_METHOD_NOARG("info", ubus_poe_info_cb),
	UBUS_METHOD_NOARG("debug", ubus_poe_debug_cb),
	UBUS_METHOD_NOARG("reload", ubus_poe_reload_cb),
	UBUS_METHOD("sendframe", ubus_poe_sendframe_cb, ubus_poe_sendframe_policy),
	UBUS_METHOD("manage", ubus_poe_manage_cb, ubus_poe_manage_policy),
};

static struct ubus_object_type ubus_poe_object_type =
	UBUS_OBJECT_TYPE("poe", ubus_poe_methods);

static struct ubus_object ubus_poe_object = {
	.name = "poe",
	.type = &ubus_poe_object_type,
	.methods = ubus_poe_methods,
	.n_methods = ARRAY_SIZE(ubus_poe_methods),
};

static void ubus_connect_handler(struct ubus_context *ctx)
{
	int ret;

	ret = ubus_add_object(ctx, &ubus_poe_object);
	if (ret)
		ULOG_ERR("Failed to add object: %s\n", ubus_strerror(ret));
}

int main(int argc, char **argv)
{
	int ch;

	struct poe_ctx poe = {
		.state_timeout.cb = state_timeout_cb,
		.conn.cb = ubus_connect_handler,
		.config = {
			.budget = 65,
			.budget_guard = 7,
			.pse_id_set_budget_mask = 0x01,
		},
	};

	INIT_LIST_HEAD(&poe.mcu.pending_cmds);
	ulog_open(ULOG_STDIO | ULOG_SYSLOG, LOG_DAEMON, "realtek-poe");
	ulog_threshold(LOG_INFO);

	while ((ch = getopt(argc, argv, "d")) != -1) {
		switch (ch) {
		case 'd':
			ulog_threshold(LOG_DEBUG);
			poe.hardcore_hacking_mode_en = 1;
			break;
		}
	}

	config_load(&poe.config, 1);

	uloop_init();
	ubus_auto_connect(&poe.conn);

	if (poe_stream_open("/dev/ttyS1", &poe.mcu.stream, B19200) < 0)
		return -1;

	dialect->init_async(&poe.mcu, &poe.config);
	uloop_timeout_set(&poe.state_timeout, 1000);
	uloop_run();
	uloop_done();

	return 0;
}
