#!/usr/bin/env python3

""" Broadcom PSE decoder and PoE monitor (poem)

The poem decoder decodes a raw or ASCII dump of broadcom PSE packets, and forms
a "big picture" of the PoE configuration and port status. It can alternatively
monitor serial traffic (not yet implemented) through an external serial port
connected in such a way as to monitor the serial lines of the MCU.

Decode a file:
--------------
    ./poem dump_file.raw
    ./poem dump_file.txt

Monitor a serial port:
----------------------
    ./poem /dev/ttyUSB0
"""

import argparse
import pprint

packet_def = {
	0x00 : {
		"name" : "set port enable",
		"fields" : [
			("port", 1),
			("enable", 1),
		]
	},
	0x02 : {
		"name" : "set port map enable",
		"fields" : [
			("enable", 1),
		]
	},
	0x05 : { "name" : "clear counters", "fields" : [] },
	0x06 : {
		"name" : "set global port enable",
		"fields" : [
			("enable", 1),
		]
	},
	0x09 : { "name" : "system reset", "fields" : [] },
	0x0b : {
		"name" : "set device power management",
		"fields" : [
			("pre-alloc", 1),
			("powerup_mode", 1),
			("disconnect_order", 1),
			("gb_hysteresis", 1),
		]
	},
	0x11 : {
		"name" : "set classification enable",
		"fields" : [
			("port", 1),
			("classification", 1),
		]
	},
	0x13 : {
		"name" : "set disconnect type",
		"fields" : [
			("port", 1),
			("disconnect", 1),
		]
	},
	0x15 : {
		"name" : "set power limit type",
		"fields" : [
			("port", 1),
			("limit_type", 1),
			("port", 1),
			("limit_type", 1),
			("port", 1),
			("limit_type", 1),
			("port", 1),
			("limit_type", 1),
		]
	},
	0x16 : {
		"name" : "set power power budget",
		"fields" : [
			("port", 1),
			("power_limit", 1),
			("port", 1),
			("power_limit", 1),
			("port", 1),
			("power_limit", 1),
			("port", 1),
			("power_limit", 1),
		]
	},
	0x17 : {
		"name" : "set power management mode",
		"fields" : [
			("mode", 1),
		]
	},
	0x18 : {
		"name" : "set global power budget",
		"fields" : [
			("pse_ctrl", 1),
			("total_power", 2),
			("guard_band", 2),
		]
	},
	0x1a : {
		"name" : "set port priority",
		"fields" : [
			("port", 1),
			("priority", 1),
		]
	},
	0x1c : {
		"name" : "set port power-up mode",
		"fields" : [
			("port", 1),
			("mode", 1),
			("port", 1),
			("mode", 1),
			("port", 1),
			("mode", 1),
			("port", 1),
			("mode", 1),
		]
	},
	0x1d : {
		"name" : "set port mapping",
		"fields" : [
			("port", 1),
			("mode", 1),
		]
	},
	0x20 : {
		"name" : "get system info",
		"fields" : [
			("mode", 1),
			("max_ports", 1),
			("port_map", 1),
			("id", 2),
			("version", 1),
			("mcu_type", 1),
			("system_status", 1),
			("version_ext", 1),
		]
	},
	0x22 : {
		"name" : "get port counters",
		"fields" : [
			("port", 1),
			#("reset", 1),
			#("overload", 1),
			#("short", 2),
			#("denied", 1),
			#("mps_absent", 1),
			#("invalid_signature", 1),
		]
	},
	0x23 : {
		"name" : "get power statistics",
		"fields" : [
			("consumed", 2),
			("budget", 2),
			("b3", 1),
			("high_power", 1),
			("gb_hysteresis", 1),
		]
	},
	0x25 : {
		"name" : "get port config",
		"fields" : [
			("port", 1),
			("enable", 1),
			("auto_powerup", 1),
			("detection_type", 1),
			("classification_enable", 1),
			("disconnect", 1),
			("pair", 1),
		]
	},
	0x28 : {
		"name" : "get all port status",
		"fields" : [
			("port", 1),
			("status", 1),
			("port", 1),
			("status", 1),
			("port", 1),
			("status", 1),
			("port", 1),
			("status", 1),
		]
	}
}

def decode_packet(pkt, big_picture):
	csum = sum(pkt[0:11]) & 0Xff
	if csum != pkt[11]:
		print(f'{pkt.hex(sep=" ")} Checksum error={hex(csum)} != {hex(pkt[11])}')
		return

	pdef = packet_def.get(pkt[0])
	if not pdef:
		print(f'**** Unexplained {pkt.hex(sep=" ")} ****')
		return

	offset = 2
	descr = ""
	port = None
	pse = None
	for name, size in pdef["fields"]:
		value = 0
		for orf in range(offset, offset + size):
			value = value << 8 | pkt[orf]

		descr += f' {name}={hex(value)}'
		if name == "port":
			port = value
		elif name == "pse_ctrl":
			pse = value

		offset += size
		all_ones = (1 << (size * 8)) - 1
		if value in (all_ones, 255):
			continue

		if not port and not pse:
			big_picture[name] = value
			continue

		if name in ["port", "pse_ctrl"]:
			continue

		if port:
			if not big_picture.get(port):
				big_picture[port] = {}
			big_picture[port][name] = value
		elif pse:
			if not big_picture["pse"].get(pse):
				big_picture["pse"][pse] = {}
			big_picture["pse"][pse][name] = value

	print (f'[{hex(pkt[0])}]{pdef["name"]}:{descr}')

def ascii_to_bin(line):
	try:
		byte_list = [int(num, base=16) for num in line.split()]
	except ValueError:
		return []

	return bytearray(byte_list)

def parse_ascii_dump(filename, big_picture):
	with open(filename, 'r') as ascii_file:
		for line in ascii_file:
			pkt = ascii_to_bin(line)
			if len(pkt) < 12:
				continue
			decode_packet(pkt, big_picture)

def parse_raw_dump(filename, big_picture):
	with open(filename, 'rb') as raw_file:
		while (packet := raw_file.read(12)):
			decode_packet(packet, big_picture)

def main(args):
	big_picture = {"pse" : {}}
	pretty = pprint.PrettyPrinter(indent=4)

	with open(args.dumpfile, 'rb') as dump:
		if dump.isatty():
			raise NotImplementedError("File is a tty")

	try:
		parse_ascii_dump(args.dumpfile, big_picture)
	except UnicodeDecodeError:
		parse_raw_dump(args.dumpfile, big_picture)

	pretty.pprint(big_picture)

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='Broadcom PoE decoder')
	parser.add_argument('dumpfile', type=str,
                    help='File containing serial port dump (binary)')

	main(parser.parse_args())
