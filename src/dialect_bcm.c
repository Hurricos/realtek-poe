/* SPDX-License-Identifier: GPL-2.0 */

#include "tek-poe.h"

#include <errno.h>
#include <string.h>
#include <libubox/ulog.h>

typedef int (*poe_reply_handler)(struct mcu_state *ctx, uint8_t *reply);

/* Careful with this; Only works for set_detection/disconnect_type commands. */
#define PORT_ID_ALL	0x7f
#define PORT_ID_WRONG_DIALECT		0x61

__attribute__((unused))
static const struct dialect_map bcm_dialect_map[0x100] = {
	[MCU_SET_POWER_MGMT_MODE] 	= {0x17, 0},
	[MCU_SET_POWER_BUDGET]		= {0x18, 0},
	[MCU_ENABLE_PORT_MAPPING]	= {0x02, 0},
	[PORT_ENABLE]			= {0x00, 0},
	[PORT_ENABLE_CLASSIFICATION]	= {0x11, CMD_IS_4PORT},
	[PORT_SET_DETECTION_TYPE]	= {0x10, CMD_HAS_ALL_PORT},
	[PORT_SET_PRIORITY]		= {0x1a, CMD_IS_4PORT},
	[PORT_SET_POE_MODE]		= {0xfc, CMD_IS_4PORT},
	[PORT_SET_DISCONNECT_TYPE]	= {0x13, CMD_HAS_ALL_PORT},
	[PORT_SET_POWER_LIMIT_TYPE]	= {0x15, CMD_IS_4PORT},
	[PORT_SET_POWER_LIMIT]		= {0x16, 0},

	[MCU_GET_SYSTEM_INFO]		= {0x20, 0},
	[MCU_GET_POWER_STATS]		= {0x23, 0},
	[PORT_GET_EXT_CONFIG]		= {0x26, 0},
	[PORT_GET_SHORT_STATUS]		= {0x28, CMD_IS_4PORT},
	[PORT_GET_POWER_STATS]		= {0x30, 0},
};

static int poet_cmd_4_port(struct mcu *mcu, uint8_t cmd_id, uint8_t port[4],
			   uint8_t data[4])
{
	uint8_t cmd[] = { cmd_id, 0x00, port[0], data[0], port[1], data[1],
		port[2], data[2], port[3], data[3] };

		return mcu_queue_cmd(mcu, cmd, sizeof(cmd));
}

/* 0x00 - Set port enable
 *	0: Disable
 *	1: Enable
 */
static int poe_cmd_port_enable(struct mcu *mcu, uint8_t port, uint8_t enable)
{
	uint8_t cmd[] = { 0x00, 0x00, port, enable };

	return mcu_queue_cmd(mcu, cmd, sizeof(cmd));
}

static int poe_cmd_port_mapping_enable(struct mcu *mcu, bool enable)
{
	uint8_t cmd[] = { 0x02, 0x00, enable };

	return mcu_queue_cmd(mcu, cmd, sizeof(cmd));
}

/* 0x10 - Set port detection type
 *	1: Legacy Capacitive Detection only
 *	2: IEEE 802.3af 4-Point Detection only (Default)
 *	3: IEEE 802.3af 4-Point followed by Legacy
 *	4: IEEE 802.3af 2-Point detection (Not Supported)
 *	5: IEEE 802.3af 2-Point followed by Legacy
 */
static int poe_cmd_port_detection_type(struct mcu *mcu, uint8_t port,
				       uint8_t type)
{
	uint8_t cmd[] = { 0x10, 0x00, port, type };

	return mcu_queue_cmd(mcu, cmd, sizeof(cmd));
}

/* 0x11 - Set port classification
 *	0: Disable
 *	1: Enable
 */
static int poe_cmd_port_classification(struct mcu *mcu, uint8_t port[4],
				       uint8_t enable[4])
{
	return poet_cmd_4_port(mcu, 0x11, port, enable);
}

/* 0x13 - Set port disconnect type
 *	0: none
 *	1: AC-disconnect
 *	2: DC-disconnect
 *	3: DC with delay
 */
static int poe_cmd_port_disconnect_type(struct mcu *mcu, uint8_t port,
					uint8_t type)
{
	uint8_t cmd[] = { 0x13, 0x00, port, type };

	return mcu_queue_cmd(mcu, cmd, sizeof(cmd));
}

/* 0x15 - Set port power limit type
 *	0: None. Power limit is 16.2W if the connected device is “low power”,
 *	   or the set high power limit if the device is “high power”.
 *	1: Class based. The power limit for class 4 devices is determined by the high power limit.
 *	2: User defined
 */
static int poe_cmd_port_power_limit_type(struct mcu *mcu, uint8_t port[4],
					 uint8_t limit[4])
{
	return poet_cmd_4_port(mcu, 0x15, port, limit);
}

/* 0x16 - Set port power budget
 *	values in 0.2W increments
 */
static int poe_cmd_port_power_budget(struct mcu *mcu, uint8_t port,
				     uint8_t budget)
{
	uint8_t cmd[] = { 0x16, 0x00, port, budget };

	return mcu_queue_cmd(mcu, cmd, sizeof(cmd));
}

/* 0x17 - Set power management mode
 *	0: None (No Power Management mode) (Default in Semi-Auto mode)
 *	1: Static Power Management with Port Priority(Default in Automode)
 *	2: Dynamic Power Management with Port Priority
 *	3: Static Power Management without Port Priority
 *	4: Dynamic Power Management without Port Priority
 */
static int poe_cmd_power_mgmt_mode(struct mcu *mcu, uint8_t mode)
{
	uint8_t cmd[] = { 0x17, 0x00, mode };

	return mcu_queue_cmd(mcu, cmd, sizeof(cmd));
}

/* 0x18 - Set global power budget */
static int poe_cmd_global_power_budget(struct mcu *mcu, uint8_t pse,
				       float budget, float guard)
{
	uint8_t cmd[] = { 0x18, 0x00, pse, 0x00, 0x00, 0x00, 0x00 };

	write16_be(cmd + 3, budget * 10);
	write16_be(cmd + 5, guard * 10);

	return mcu_queue_cmd(mcu, cmd, sizeof(cmd));
}

/* 0x1a - Set port priority
 *	0: Low
 *	1: Normal
 *	2: High
 *	3: Critical
 */
static int poe_set_port_priority(struct mcu *mcu, uint8_t port[4],
				 uint8_t priority[4])
{
	return poet_cmd_4_port(mcu, 0x1a, port, priority);
}

/* 0x1c - Set port power-up mode
 *	0: PoE
 *	1: legacy
 *	2: pre-PoE+
 *	3: PoE+
 */
static int poe_set_port_power_up_mode(struct mcu *mcu, uint8_t port[4],
				      uint8_t mode[4])
{
	return poet_cmd_4_port(mcu, 0x1c, port, mode);
}

/* 0x20 - Get system info */
static int poe_cmd_status(struct mcu *mcu)
{
	uint8_t cmd[] = { 0x20 };

	return mcu_queue_cmd(mcu, cmd, sizeof(cmd));
}

static int poe_reply_status(struct mcu_state *state, uint8_t *reply)
{
	const char *mode[] = {
		"Semi-auto I2C",
		"Semi-auto UART",
		"Auto I2C",
		"Auto UART"
	};
	const char *mcu_names[] = {
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

	state->sys_mode = GET_STR(reply[2], mode);
	state->num_detected_ports = reply[3];
	state->port_map_en = reply[4];
	state->device_id =  read16_be(reply + 5);
	state->sys_version = reply[7];
	state->sys_mcu = GET_STR(reply[8], mcu_names);
	state->sys_status = GET_STR(reply[9], status);
	state->sys_ext_version = reply[10];

	return 0;
}

/* 0x23 - Get port status */
static int poe_cmd_port_status(struct mcu *mcu, uint8_t port)
{
	uint8_t cmd[] = { 0x21, 0x00, port };

	return mcu_queue_cmd(mcu, cmd, sizeof(cmd));
}

static int poe_reply_port_status(struct mcu_state *state, uint8_t *reply)
{
	int port_idx = reply[2];

	state->ports[port_idx].class_info = reply[5];
	state->ports[port_idx].pd_type = reply[6];
	state->ports[port_idx].mpss_mask = reply[7];
	state->ports[port_idx].has_detailed_state = 1;

	return 0;
}

/* 0x23 - Get power statistics */
static int poe_cmd_power_stats(struct mcu *mcu)
{
	uint8_t cmd[] = { 0x23 };

	return mcu_queue_cmd(mcu, cmd, sizeof(cmd));
}

static int poe_reply_power_stats(struct mcu_state *state, uint8_t *reply)
{
	state->power_consumption = read16_be(reply + 2) * 0.1;
	state->reported_power_budget = read16_be(reply + 4) * 0.1;

	return 0;
}

/* 0x25 - Get port config */
static int poe_cmd_port_config(struct mcu *mcu, uint8_t port)
{
	uint8_t cmd[] = { 0x25, 0x00, port };

	return mcu_queue_cmd(mcu, cmd, sizeof(cmd));
}

static int poe_reply_port_config(struct mcu_state *state, uint8_t *reply)
{
	int port_idx = reply[2];

	state->ports[port_idx].enabled = reply[4];
	state->ports[port_idx].auto_powerup = reply[4];
	state->ports[port_idx].detection_type = reply[5];
	state->ports[port_idx].classification_enable = reply[6];
	state->ports[port_idx].disconnect_type = reply[7];
	state->ports[port_idx].pair = reply[8];
	state->ports[port_idx].has_ext_config = 1;

	return 0;
}

/* 0x26 - Get extended port config */
static int poe_cmd_port_ext_config(struct mcu *mcu, uint8_t port)
{
	uint8_t cmd[] = { 0x26, 0x00, port };

	return mcu_queue_cmd(mcu, cmd, sizeof(cmd));
}

static int poe_reply_port_ext_config(struct mcu_state *state, uint8_t *reply)
{
	int port_idx = reply[2];

	const char *mode[] = {
		"PoE",
		"Legacy",
		"pre-PoE+",
		"PoE+"
	};

	state->ports[port_idx].poe_mode = GET_STR(reply[3], mode);
	state->ports[port_idx].power_limit_type = reply[4];
	state->ports[port_idx].power_budget = reply[5] * 0.2;
	state->ports[port_idx].priority = reply[6];
	state->ports[port_idx].primary_pse_output = reply[7];
	/* In the broadcom dialect, pse output and mapping are synonymous. */
	state->ports[port_idx].mapping = state->ports[port_idx].primary_pse_output;

	return 0;
}

/* 0x28 - Get all all port status */
static int poe_cmd_4_port_status(struct mcu *mcu, uint8_t p1, uint8_t p2,
				 uint8_t p3, uint8_t p4)
{
	uint8_t cmd[] = { 0x28, 0x00, p1, 1, p2, 1, p3, 1, p4, 1 };

	return mcu_queue_cmd(mcu, cmd, sizeof(cmd));
}

static int poe_reply_4_port_status(struct mcu_state *state, uint8_t *reply)
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

		state->ports[port].status = GET_STR(pstate & 0xf, status);
	}

	return 0;
}

/* 0x2b - Get extended device config */
static int poe_cmd_get_extended_config(struct mcu *mcu)
{
	uint8_t cmd[] = { 0x2b };

	return mcu_queue_cmd(mcu, cmd, sizeof(cmd));
}

static int poe_reply_extended_config(struct mcu_state *state, uint8_t *reply)
{
	state->uvlo_threshold = reply[2] * 0.06445 + 33.0;
	state->pre_alloc = reply[3];
	state->powerup_mode = reply[4];
	state->disconnect_type = reply[5];
	state->ddflag = reply[6];
	state->ovlo_threshold = reply[7] * 0.06445 + 57.0;
	state->num_pse = reply[8];
	state->has_ext_cfg_info = 1;

	return 0;
}

/* 0x30 - Get port power statistics */
static int poe_cmd_port_power_stats(struct mcu *mcu, uint8_t port)
{
	uint8_t cmd[] = { 0x30, 0x00, port };

	return mcu_queue_cmd(mcu, cmd, sizeof(cmd));
}

static int poe_reply_port_power_stats(struct mcu_state *state, uint8_t *reply)
{
	int port_idx = reply[2];

	state->ports[port_idx].watt = read16_be(reply + 9) * 0.1;
	return 0;
}

static poe_reply_handler reply_handler[] = {
	[0x20] = poe_reply_status,
	[0x21] = poe_reply_port_status,
	[0x23] = poe_reply_power_stats,
	[0x26] = poe_reply_port_ext_config,
	[0x25] = poe_reply_port_config,
	[0x28] = poe_reply_4_port_status,
	[0x2b] = poe_reply_extended_config,
	[0x30] = poe_reply_port_power_stats,
};


static int poe_default_reply_handler(uint8_t *reply)
{
	int cmd = reply[0];
	int ret = reply[2];

	if (ret)
		ULOG_WARN("Command 0x%x replied with error 0x%x\n", cmd, ret);
	return 0;
}

static int poe_reply_consume(struct mcu_state *ctx, uint8_t *reply)
{
	if (reply[0] > ARRAY_SIZE(reply_handler)) {
		ULOG_DBG("bcm: received reply with bad command id\n");
		return -1;
	}

	if (reply_handler[reply[0]]) {
		return reply_handler[reply[0]](ctx, reply);
	} else {
		poe_default_reply_handler(reply);
	}

	return 0;
}

static int bcm_handle_reply(struct mcu_state *ctx, uint8_t *reply, size_t len)
{
	if (len != 12)
		return -EINVAL;

	return poe_reply_consume(ctx, reply);
}

static int poet_setup(struct mcu* mcu, const struct port_config *ports,
		      size_t num_ports)
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

		poe_set_port_priority(mcu, port_ids, priorities);
		poe_set_port_power_up_mode(mcu, port_ids, powerup_mode);
		poe_cmd_port_classification(mcu, port_ids, enable_all);
		poe_cmd_port_power_limit_type(mcu, port_ids, limit_type);

		num_okay = 0;
	} while (++i < num_ports);

	return 0;
}

static int bcm_port_setup(struct mcu *mcu, const struct config *cfg)
{
	size_t i;

	poe_cmd_port_disconnect_type(mcu, PORT_ID_ALL, 2);
	poe_cmd_port_detection_type(mcu, PORT_ID_ALL, 3);

	for (i = 0; i < cfg->port_count; i++) {
		if (!cfg->ports[i].enable || !cfg->ports[i].power_budget)
			continue;

		poe_cmd_port_power_budget(mcu, i, cfg->ports[i].power_budget);
	}

	poet_setup(mcu, cfg->ports, cfg->port_count);

	for (i = 0; i < cfg->port_count; i++)
		poe_cmd_port_enable(mcu, i, !!cfg->ports[i].enable);

	return 0;
}

static void poe_set_power_budget(struct mcu* mcu, const struct config *config)
{
	unsigned int pse;

	for (pse = 0; pse < 8; pse++) {
		if (!(config->pse_id_set_budget_mask & (1 << pse)))
			continue;

		poe_cmd_global_power_budget(mcu, pse, config->budget,
					    config->budget_guard);
	}
}

static int bcm_initial_setup(struct mcu* mcu, const struct config *cfg)
{
	poe_cmd_status(mcu);
	poe_cmd_power_mgmt_mode(mcu, 2);
	poe_cmd_port_mapping_enable(mcu, false);
	poe_set_power_budget(mcu, cfg);

	bcm_port_setup(mcu, cfg);

	return 0;
}

static int bcm_poll(struct mcu *mcu, const struct config *cfg)
{
	size_t i;

	poe_cmd_power_stats(mcu);
	if (/*poe->hardcore_hacking_mode_en*/ 0)
		poe_cmd_get_extended_config(mcu);

	for (i = 0; i < cfg->port_count; i += 4)
		poe_cmd_4_port_status(mcu, i, i + 1, i + 2, i + 3);

	for (i = 0; i < cfg->port_count; i++) {
		if (/*poe->hardcore_hacking_mode_en*/ 0) {
			poe_cmd_port_status(mcu, i);
			poe_cmd_port_config(mcu, i);
		}

		poe_cmd_port_ext_config(mcu, i);
		poe_cmd_port_power_stats(mcu, i);
	}

	return 0;
}

const struct poe_dialect broadcom_dialect = {
	.init_async = bcm_initial_setup,
	.init_ports_async = bcm_port_setup,
	.poll_async = bcm_poll,
	.enable_port_async = poe_cmd_port_enable,
	.handle_reply = bcm_handle_reply,
};
