/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef TEK_POE_H
#define TEK_POE_H

#include <stddef.h>
#include <stdint.h>
#include <libubox/utils.h>

#define ULOG_DBG(fmt, ...) ulog(LOG_DEBUG, fmt, ## __VA_ARGS__)

#define GET_STR(a, b)	((a) < ARRAY_SIZE(b) ? (b)[a] : NULL)
#define MAX(a, b)	(((a) > (b)) ? (a) : (b))
#define MAX_PORT	48

struct port_state {
	const char *status;
	float watt;
	const char *poe_mode;
};

struct port_config {
	char name[16];
	unsigned int valid : 1;
	unsigned int enable : 1;
	uint8_t priority;
	uint8_t power_up_mode;
	uint8_t power_budget;
};

struct port_led_config {
	uint8_t enable:1;
	uint8_t interface:1;
	uint8_t shift_order:1;
	uint8_t led_count:2;
	uint8_t blink_override:2;
	uint8_t state_off;
	uint8_t state_req;
	uint8_t state_err;
	uint8_t state_on;
};

struct port_led_map {
	uint8_t offset;
	uint8_t ports[8];
};

struct system_led_config {
	uint8_t sys_ok:1;
	uint8_t in_gb:2;
	uint8_t out_of_gb:2;
	uint8_t exceeds_ps:2;
	uint8_t out_of_gb_off_delay;
	uint8_t exceeds_ps_off_delay;
	uint8_t map_enable;
};

struct mcu_state {
	const char *sys_mode;
	uint8_t sys_version;
	const char *sys_mcu;
	const char *sys_status;
	uint8_t sys_ext_version;
	float power_consumption;
	unsigned int num_detected_ports;

	struct port_state        ports[MAX_PORT];
	struct port_led_config   portledconfig;
	struct port_led_map      ledmaps[(MAX_PORT + 7) / 8];
	struct system_led_config sysledconfig;
};

struct config {
	float budget;
	float budget_guard;

	unsigned int port_count;
	uint8_t pse_id_set_budget_mask;
	struct port_config ports[MAX_PORT];
};

static inline uint16_t read16_be(uint8_t *raw)
{
	return (uint16_t)raw[0] << 8 | raw[1];
}

static inline void write16_be(uint8_t *raw, uint16_t value)
{
	raw[0] = value >> 8;
	raw[1] =  value & 0xff;
}

#endif /* TEK_POE_H */
