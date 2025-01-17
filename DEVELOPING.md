Development help for realtek-poe
================================

Coding style
------------

realtek-poe uses the Linux kernel coding style:
 - https://www.kernel.org/doc/html/latest/process/coding-style.html


Compiling realtek-poe
---------------------

Manual builds of realtek-poe are intended to use the OpenWRT SDK. The simplest
way to get all the dependencies is to link the the source in the packages
sub-directory of the SDK. The first-time setup involves:

	$ ln -s <path/to>/realtek-poe/ package/realtek-poe
	$ ./scripts/feeds update base
	$ ./scripts/feeds install libubox libubus libuci
	$ make menuconfig


Afterwards, realtek-poe can be built like any other OpenWRT package. Insert the
`V=s` argument to see verbose build messages and compiler messages:

	$ make package/realtek-poe/compile [V=s]

For realtek MIPS builds, the package and realtek-poe binary respectively can be
found under:
 - `bin/packages/mips_4kec/base/`
 - `./staging_dir/target-mips_4kec_musl/root-realtek/usr/bin/realtek-poe`


Design goals of the realtek-poe daemon
--------------------------------------

Broadcom and Realtek PoE controllers are based on a serial bus. A single command
and repsonse may take over 10 milliseconds to complete, while querying the
status of all ports may take hundreds of milliseconds. Additionally, commands
may randomly fail with an error reply.

A simple command and query utility would need to block for extended periods of
time. This has the potential of making unresponsive other components of OpenWRT
witch depend on altering or checking PoE settings. Such a utility will
occasionally be forced to report the protocol errors upstream, complicating the
PoE scheme. Hence, there is a need for a daemon to address both issues.

On the front end, realtek-poe implements a non-blocking interface using ubus
as the transport. The goal of the ubus interface is to be responsive, by hiding
the protocol delays of the serial bus. The backend handles the protocol details,
including retrying commands that report errors. To achieve both goals
**realtek-poe implements an asynchronous design**.

The state of the PoE controller is cached within realtek-poe. This cached state
is reported via ubus without waiting on hardware responses. To ensure the cached
state is reasonably up to date, the PoE controller is queried at regular
intervals. The system is build upon a libubox event loop. For this to work,
**no blocking calls can be made from the realtek-poe event loop**.


Design of realtek-poe around blocking transports
------------------------------------------------

The non-blocking requirement presents a delicate problem for PoE controllers
which use I2C as the serial bus. Linux does not offer a non-blocking API for
I2C transfers. The event loop would be unable to respond to ubus requests, thus
failing to meet the first design requirement. The solution in this case would
be to move the blocking transfers into a separate thread. This is not, as of
this writing, implemented for realtek-poe.
