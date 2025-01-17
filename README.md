realtek-poe
===========

A system daemon for handling configuration and control of PoE Power Sourcing
Equipment (PSE) based on Broadcom and Realtek PoE controllers.


Getting realtek-poe
-------------------

The realtek-poe is delivered as an OpenWRT package, part of the packages feed:

	# opkg update
	# opkg install realtek-poe


Configuring power delivery
--------------------------

PoE configuration is read from the UCI "poe" section. This is found in the
`/etc/config/poe` file. Documenting the specific of each field is beyond the
scope of this README.

When running as a system daemon, using the provided init.d startup script,
editing the config file will cause a configuration reload. This will apply new
settings for the PoE ports, but leaves global settings untouched. To cause
a reconfiguration manually:

	# ubus call poe reload


Runtime interface
-----------------

In addition to the config file, realtek-poe provides a runtime interface for
querying port status and managing ports. The format is auto-documented by:

	# ubus -v list poe


While port configuration must be changed by editing the config, power to
individual ports may be dynamically toggled with the `manage` command:

	# ubus call poe manage '{"port" : "lan3", "enable" : False}'
