/* SPDX-License-Identifier: GPL-2.0 */

#include "tek-poe.h"

#include <errno.h>
#include <string.h>
#include <libubox/ulog.h>

typedef int (*poe_reply_handler)(struct mcu_state *ctx, uint8_t *reply);

static struct dialect_map rtl_dialect_map[0x100] = {
	[MCU_SET_POWER_MGMT_MODE] 	= {0x10, 0},
	[MCU_SET_POWER_BUDGET]		= {0x04, 0},
	// [MCU_ENABLE_PORT_MAPPING]	= {0x02, 0},
	[PORT_ENABLE]			= {0x01, 0},
	// [PORT_ENABLE_CLASSIFICATION]	= {0x11, CMD_IS_4PORT},
	[PORT_SET_DETECTION_TYPE]	= {0x09, CMD_IS_4PORT},
	[PORT_SET_PRIORITY]		= {0x15, CMD_IS_4PORT},
	[PORT_SET_POE_MODE]		= {0x0c, CMD_IS_4PORT},
	[PORT_SET_DISCONNECT_TYPE]	= {0x0f, CMD_IS_4PORT},
	[PORT_SET_POWER_LIMIT_TYPE]	= {0x12, CMD_IS_4PORT},
	[PORT_SET_AUTO_POWERUP]		= {0x08, CMD_IS_4PORT},
	[PORT_SET_POWER_LIMIT]		= {0x13, 0},

	[MCU_GET_SYSTEM_INFO]		= {0x40, 0},
	[MCU_GET_POWER_STATS]		= {0x41, 0},
	[PORT_GET_EXT_CONFIG]		= {0x49, 0},
	// [PORT_GET_SHORT_STATUS]		= {0x28, CMD_IS_4PORT},
	[PORT_GET_POWER_STATS]		= {0x44, 0},
	[MCU_GET_EXT_CONFIG]		= {0x4a, 0},
	[PORT_GET_CONFIG]		= {0x48, 0},
	[PORT_GET_STATUS]		= {0x42, 0},
	[PORT_GET_SHORT_STATUS]		= {0x43, CMD_IS_RETARDED_4PORT},
};

/* Careful with this; Only works for set_detection/disconnect_type commands. */
#define PORT_ID_ALL	0x7f
#define PORT_ID_WRONG_DIALECT		0x61

static struct mcu *hack_mcu;

static int poe_cmd_queue(uint8_t *cmd, int len)
{
	return mcu_queue_cmd(hack_mcu, cmd, len);
}

static int poet_cmd_4_port(uint8_t cmd_id, uint8_t port[4], uint8_t data[4])
{
	uint8_t cmd[] = { cmd_id, 0x00, port[0], data[0], port[1], data[1],
		port[2], data[2], port[3], data[3] };

		return poe_cmd_queue(cmd, sizeof(cmd));
}

static int rtl_cmd_pse_up(uint8_t enable)
{
	uint8_t cmd[] = { 0x00, 0x00, enable };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

static int rtl_cmd_port_enable(struct mcu *mcu, uint8_t port, uint8_t enable)
{
	uint8_t cmd[] = { 0x01, 0x00, port, enable };

	return mcu_queue_cmd(mcu, cmd, sizeof(cmd));
}

static int rtl_cmd_reset_enable(bool enable)
{
	uint8_t cmd[] = { 0x02, 0x00, enable };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

static int rtl_cmd_global_power_budget(uint8_t pse, float budget, float guard)
{
	uint8_t cmd[] = { 0x04, 0x00, pse, 0x00, 0x00, 0x00, 0x00 };

	write16_be(cmd + 3, budget * 10);
	write16_be(cmd + 5, guard * 10);

	return poe_cmd_queue(cmd, sizeof(cmd));
}

static int rtl_set_mystery_parameter(uint8_t ports[4], uint8_t enables[4])
{
	return poet_cmd_4_port(0x08, ports, enables);
}


static int rtl_set_port_power_up_mode(uint8_t ports[4], uint8_t modes[4])
{
	return poet_cmd_4_port(0x0c, ports, modes);
}

static int rtl_cmd_power_mgmt_mode(uint8_t mode)
{
	uint8_t cmd[] = { 0x10, 0x00, mode };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

static int rtl_cmd_port_power_limit_type(uint8_t ports[4], uint8_t priorities[4])
{
	return poet_cmd_4_port(0x12, ports, priorities);
}

static int rtl_cmd_port_power_budget(uint8_t port, uint8_t budget)
{
	uint8_t cmd[] = { 0x13, 0x00, port, budget };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

static int rtl_set_port_priority(uint8_t port[4], uint8_t priority[4])
{
	return poet_cmd_4_port(0x15, port, priority);
}

/* 0x40 - Get system info */
static int rtl_cmd_status(void)
{
	uint8_t cmd[] = { 0x40 };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

static int rtl_reply_status(struct mcu_state *ctx, uint8_t *reply)
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

	ctx->sys_mode = GET_STR(reply[2], mode);
	if (!reply[3] || reply[3] > MAX_PORT)
		ULOG_ERR("num_detected_ports=%d is invalid\n", reply[3]);
	else
		ctx->num_detected_ports = reply[3];
	ctx->port_map_en = reply[4];
	ctx->device_id =  read16_be(reply + 5);
	ctx->sys_version = reply[7];
	ctx->sys_mcu = GET_STR(reply[8], mcu);
	ctx->sys_status = GET_STR(reply[9], status);
	ctx->sys_ext_version = reply[10];

	return 0;
}

static int rtl_cmd_power_stats(void)
{
	uint8_t cmd[] = { 0x41 };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

static int rtl_reply_power_stats(struct mcu_state *ctx, uint8_t *reply)
{
	ctx->power_consumption = read16_be(reply + 2) * 0.1;
	ctx->reported_power_budget = read16_be(reply + 4) * 0.1;

	return 0;
}

static int rtl_cmd_port_status(uint8_t port)
{
	uint8_t cmd[] = { 0x42, 0x00, port };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

static int rtl_reply_port_status(struct mcu_state *mcu, uint8_t *reply)
{
	int port;

	port = reply[2];
	if (port >= MAX_PORT) {
		ULOG_WARN("Invalid port status packet (port=%d)\n", port);
		return -1;
	}

	mcu->ports[port].class_info = reply[5];
	mcu->ports[port].pd_type = reply[6];
	mcu->ports[port].mpss_mask = reply[7];
	mcu->ports[port].has_detailed_state = 1;

	return 0;
}

static int rtl_cmd_4_port_group_status(uint8_t start_port)
{
	uint8_t cmd[] = { 0x43, 0x00, start_port / 4};

	return poe_cmd_queue(cmd, sizeof(cmd));
}

static int rtl_reply_4_port_group_status(struct mcu_state *mcu, uint8_t *reply)
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

	port = reply[2] * 4;
	for (i = 3; i < 11; i += 2, port++) {
		pstate = reply[i];

		if (port == 0xff) {
			continue;
		} else if (port >= MAX_PORT) {
			ULOG_WARN("Invalid port status packet (port=%d)\n", port);
			return -1;
		}

		mcu->ports[port].status = GET_STR(pstate & 0xf, status);
	}

	return 0;
}

static int rtl_cmd_port_power_stats(uint8_t port)
{
	uint8_t cmd[] = { 0x44, 0x00, port};

	return poe_cmd_queue(cmd, sizeof(cmd));
}

static int rtl_reply_port_power_stats(struct mcu_state *mcu, uint8_t *reply)
{
	unsigned int port_idx = reply[2];

	if (port_idx > mcu->num_detected_ports) {
		ULOG_WARN("Invalid port in power stat (port=%d)\n", port_idx);
		return -EPROTO;
	}

	mcu->ports[port_idx].watt = read16_be(reply + 9) * 0.1;

	return 0;
}

static int rtl_cmd_why_u_reset(void)
{
	uint8_t cmd[] = { 0x47, 0x00, 0x01 };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

static int rtl_cmd_port_config(uint8_t port)
{
	uint8_t cmd[] = { 0x48, 0x00, port };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

static int rtl_reply_port_config(struct mcu_state *mcu, uint8_t *reply)
{
	unsigned int port_idx = reply[2];
	struct port_state *port;

	if (port_idx > mcu->num_detected_ports) {
		ULOG_WARN("Invalid port in ext config packet (port=%d)\n", port_idx);
		return -EPROTO;
	}

	port = &mcu->ports[port_idx];

	port->enabled = reply[3];
	port->auto_powerup = reply[4];
	port->detection_type = reply[5];
	port->classification_enable = reply[6];
	port->disconnect_type = reply[7];
	port->pair = reply[8];

	return 0;
}

static int rtl_cmd_port_ext_config(uint8_t port)
{
	uint8_t cmd[] = { 0x49, 0x00, port };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

static int rtl_reply_port_ext_config(struct mcu_state *mcu, uint8_t *reply)
{
	unsigned int port_idx = reply[2];
	struct port_state *port;

	const char *mode[] = {
		"PoE",
		"Legacy",
		"pre-PoE+",
		"PoE+"
	};

	if (port_idx > mcu->num_detected_ports) {
		ULOG_WARN("Invalid port in ext config packet (port=%d)\n", port_idx);
		return -EPROTO;
	}

	port = &mcu->ports[port_idx];

	port->poe_mode = GET_STR(reply[3], mode);
	port->mapping = reply[8];


	port->power_limit_type = reply[4];
	port->power_budget = reply[5] * 0.2;
	port->priority = reply[6];
	port->primary_pse_output = reply[7];
	port->mapping = reply[8];
	port->has_ext_config = 1;

	return 0;
}

static int rtl_cmd_ext_config()
{
	uint8_t cmd[] = { 0x4a };

	return poe_cmd_queue(cmd, sizeof(cmd));
}

static int rtl_reply_ext_config(struct mcu_state *mcu, uint8_t *reply)
{
	mcu->uvlo_threshold = reply[2] * 0.06445 + 33.0;
	mcu->pre_alloc = reply[3];
	mcu->powerup_mode = reply[4];
	mcu->disconnect_type = reply[5];
	mcu->ddflag = reply[6];
	mcu->ovlo_threshold = reply[7]* 0.06445 + 57.0;
	mcu->num_pse = reply[8];
	mcu->has_ext_cfg_info = 1;

	return 0;
}

static int rtl_reply_4_port(struct mcu_state *mcu, uint8_t *reply)
{
	uint8_t port, ret;
	int i;

	for (i = 2; i < 10; i += 2) {
		port = reply[i];
		ret = reply[i + 1];
		if (port == 0xff)
			continue;

		if (ret)
			ULOG_WARN("Command %02x failed for port %d with code %d\n",
				  reply[0], port, ret);
	}

	return 0;
}

static poe_reply_handler reply_handler[] = {
	[PORT_ENABLE]			= rtl_reply_4_port,
	// [0x03] = rtl_reply_4_port,
	[PORT_SET_AUTO_POWERUP]		= rtl_reply_4_port,
	[PORT_SET_POE_MODE]		= rtl_reply_4_port,
	[PORT_SET_POWER_LIMIT_TYPE]	= rtl_reply_4_port,
	[PORT_SET_POWER_LIMIT]		= rtl_reply_4_port,
	// [0x14] = rtl_reply_4_port,
	[PORT_SET_PRIORITY]		= rtl_reply_4_port,
	[MCU_GET_SYSTEM_INFO]		= rtl_reply_status,
	[MCU_GET_POWER_STATS]		= rtl_reply_power_stats,
	[PORT_GET_STATUS]		= rtl_reply_port_status,
	[PORT_GET_SHORT_STATUS]		= rtl_reply_4_port_group_status,
	[PORT_GET_POWER_STATS]		= rtl_reply_port_power_stats,
	[PORT_GET_CONFIG]		= rtl_reply_port_config,
	[PORT_GET_EXT_CONFIG]		= rtl_reply_port_ext_config,
	[MCU_GET_EXT_CONFIG]		= rtl_reply_ext_config,
};

static int poe_default_reply_handler(uint8_t *reply)
{
	int cmd = reply[0];
	int ret = reply[2];

	if (ret)
		ULOG_WARN("Command 0x%x replied with error 0x%x\n", cmd, ret);
	return 0;
}

static int rtl_handle_reply(struct mcu_state *ctx, uint8_t *reply, size_t len)
{
	int command;

	if (len != 12)
		return -EINVAL;

	command = rev_lookup(rtl_dialect_map, reply[0]);
	if (command < 0) {
		ULOG_DBG("rtl: received reply with bad command id\n");
		return -1;
	}

	if (reply_handler[command]){
		return reply_handler[command](ctx, reply);
	} else {
		poe_default_reply_handler(reply);
	}

	return 0;
}

static int poet_setup(const struct port_config *ports, size_t num_ports)
{
	uint8_t port_ids[4], priorities[4], powerup_mode[4], limit_type[4];
	uint8_t disable_all[4] = {0, 0, 0, 0};
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

		memset(disable_all + num_okay, 0xff, 4 - num_okay);
		memset(port_ids + num_okay, 0xff, 4 - num_okay);
		memset(priorities + num_okay, 0xff, 4 - num_okay);
		memset(powerup_mode + num_okay, 0xff, 4 - num_okay);
		memset(limit_type + num_okay, 0xff, 4 - num_okay);

		rtl_set_port_priority(port_ids, priorities);
		rtl_set_port_power_up_mode(port_ids, powerup_mode);
		rtl_set_mystery_parameter(port_ids, disable_all);
		rtl_cmd_port_power_limit_type(port_ids, limit_type);

		num_okay = 0;
	} while (++i < num_ports);

	return 0;
}

static int rtl_port_setup(struct mcu *mcu, const struct config *config)
{
	size_t i;

	// poe_cmd_port_disconnect_type(PORT_ID_ALL, 2);
	// poe_cmd_port_detection_type(PORT_ID_ALL, 3);

	for (i = 0; i < config->port_count; i++) {
		if (!config->ports[i].enable || !config->ports[i].power_budget)
			continue;

		rtl_cmd_port_power_budget(i, config->ports[i].power_budget);
	}

	poet_setup(config->ports, config->port_count);

	for (i = 0; i < config->port_count; i++)
		rtl_cmd_port_enable(mcu, i, !!config->ports[i].enable);

	return 0;
}

static void poe_set_power_budget(const struct config *config)
{
	unsigned int pse;

	for (pse = 0; pse < 8; pse++) {
		if (!(config->pse_id_set_budget_mask & (1 << pse)))
			continue;

		rtl_cmd_global_power_budget(pse, config->budget,
					    config->budget_guard);
	}
}

static int rtl_initial_setup(struct mcu *mcu, const struct config *config)
{
	hack_mcu = mcu;
	rev_map(rtl_dialect_map);

	rtl_cmd_pse_up(true);
	rtl_cmd_why_u_reset();
	rtl_cmd_reset_enable(false);

	rtl_cmd_status();
	rtl_cmd_status();
	rtl_cmd_power_mgmt_mode(2);
	poe_set_power_budget(config);

	rtl_port_setup(mcu, config);

	return 0;
}

static int chicken_reset(struct mcu *mcu)
{
	// rtl_cmd_reset_enable(false);
	rtl_cmd_reset_enable(true);
	return 0;
}

static int rtl_poll(struct mcu *mcu, const struct config *config)
{
	size_t i;

	rtl_cmd_status();
	rtl_cmd_ext_config();
	rtl_cmd_power_stats();

	for (i = 0; i < config->port_count; i += 4)
		rtl_cmd_4_port_group_status(i);

	for (i = 0; i < config->port_count; i++) {
		rtl_cmd_port_config(i);
		rtl_cmd_port_status(i);
		rtl_cmd_port_ext_config(i);
		rtl_cmd_port_power_stats(i);
	}

	return 0;
}

const struct poe_dialect realtek_dialect = {
	.init_async = rtl_initial_setup,
	.init_ports_async = rtl_port_setup,
	.poll_async = rtl_poll,
	.enable_port_async = rtl_cmd_port_enable,
	.handle_reply = rtl_handle_reply,
	.reset = chicken_reset,
};
