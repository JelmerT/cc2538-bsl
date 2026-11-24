TI CC13xx/CC2538/CC26xx Serial Boot Loader [![Tests](https://github.com/JelmerT/cc2538-bsl/actions/workflows/test.yml/badge.svg)](https://github.com/JelmerT/cc2538-bsl/actions/workflows/test.yml)
==========================================

This folder contains a python script that communicates with the boot loader of the Texas Instruments CC2538, CC26xx and CC13xx SoCs (System on Chips).
It can be used to erase, program, verify and read the flash of those SoCs with a simple USB to serial converter.

### Requirements

To run this script you need a Python interpreter, Linux and Mac users should be fine, Windows users have a look here: [Python Download][python].

Alternatively, Docker can be used to run this script as a one-liner without the need to install dependencies, see [git-developer/ti-cc-tool](https://github.com/git-developer/ti-cc-tool) for details. The image from this repository runs as a non-root user, so on Linux pass the serial device's group to give it access:

    docker run --rm --device /dev/ttyUSB0 --group-add "$(stat -c %g /dev/ttyUSB0)" \
      -v "$PWD:/fw:ro" -e FIRMWARE_FILE=/fw/firmware.hex \
      ghcr.io/jelmert/cc2538-bsl:main -p /dev/ttyUSB0 -e -w -v

To build the image from a git worktree, set the version explicitly:

    docker build --build-arg SETUPTOOLS_SCM_PRETEND_VERSION_FOR_CC2538_BSL="$(git describe --tags | sed 's/-/.post/;s/-.*//')" .

To communicate with the uart port of the SoC you need a usb to serial converter:
* If you use the SmartRF06 board with an Evaluation Module (EM) mounted on it you can use the on-board ftdi chip. Make sure the "Enable UART" jumper is set on the board. You can have a look [here][contiki cc2538dk] for more info on drivers for this chip on different operating systems.
* If you use a different platform, there are many cheap USB to UART converters available, but make sure you use one with 3.3v voltage levels.

### Dependencies

This script uses the pyserial package to communicate with the serial port and chip (https://pypi.org/project/pyserial/). You can install it by running `pip install pyserial`.

If you want to be able to program your device from an Intel Hex file, you will need to install the IntelHex package: https://pypi.python.org/pypi/IntelHex (e.g. by running `pip install intelhex`).

The script will try to auto-detect whether your firmware is a raw binary or an Intel Hex by using python-magic:
(https://pypi.python.org/pypi/python-magic). You can install it by running `pip install python-magic`. Please bear in mind that installation of python-magic may have additional dependencies, depending on your OS: (https://github.com/ahupp/python-magic#dependencies).

If python-magic is _not_ installed, the script will try to auto-detect the firmware type by looking at the filename extension, but this is sub-optimal. If the extension is `.hex`, `.ihx` or `.ihex`, the script will assume that the firmware is an Intel Hex file. In all other cases, the firmware will be treated as raw binary.

### CC2538

Once you connected the SoC you need to make sure the serial boot loader is enabled. A chip without a valid image (program), as it comes from the factory, will automatically start the boot loader. After you upload an image to the chip, the "Image Valid" bits are set to 0 to indicate that a valid image is present in flash. On the next reset the boot loader won't be started and the image is immediately executed.   
To make sure you don't get "locked out", i.e. not being able to communicate over serial with the boot loader in the SoC anymore, you need to enable the boot loader backdoor in your image (the script checks this before erasing when your image contains the boot loader configuration). When the boot loader backdoor is enabled the boot loader will be started when the chip is reset and a specific pin of the SoC is pulled high or low (configurable).   
The boot loader backdoor can be enabled and configured with the 8-bit boot loader backdoor field in the CCA area in flash. If you set this field to 0xF3FFFFFF the boot loader will be enabled when pin PA3 is pulled low during boot. This translates to holding down the `select` button on the SmartRF06 board while pushing the `EM reset` button.
If you did lock yourself out or there is already an image flashed on your SoC, you will need a jtag programmer to erase the image. This will reset the image valid bits and enable the boot loader on the next reset. The SmartRF06EB contains both a jtag programmer and a USB to uart converter on board.

### CC26xx and CC13xx

The script has been tested with SmartRF06EB + CC2650 EM. The physical wiring on the CC2650 Sensortag does not meet the ROM bootloader's requirements in terms of serial interface configuration. For that reason, interacting with the Sensortag via this script is (and will remain) impossible.

For sensortags CC1350STK:
It is possible to solder cables to R21 and R22 for flashing using the Serial Bootloader. This [issue] has instructions about flashing the CC1350STK sensortag.

For ITead SONOFF Zigbee 3.0 USB Dongle Plus:
For the CC2652P based "SONOFF Zigbee 3.0 USB Dongle Plus" (model "ZBDongle-P") adapter from ITead you need to invoke toggle to activate bootloader with `--bootloader-sonoff-usb` if you do not want to open its enclosure to manually start the bootloader with the boot button on the PCB.

For all the CC13xx and CC26xx families, the ROM bootloader is configured through the `BL_CONFIG` 'register' in CCFG. `BOOTLOADER_ENABLE` should be set to `0xC5` to enable the bootloader in the first place.

This is enough if the chip has not been programmed with a valid image. If a valid image is present, then the remaining fields of `BL_CONFIG` and the `ERASE_CONF` register must also be configured correctly:

* Select a DIO by setting `BL_PIN_NUMBER`
* Select an active level (low/high) for the DIO by setting `BL_LEVEL`
* Enable 'failure analysis' by setting `BL_ENABLE` to `0xC5`
* Make sure the `BANK_ERASE` command is enabled: The `BANK_ERASE_DIS_N` bit in the `ERASE_CONF` register in CCFG must be set. `BANK_ERASE` is enabled by default.

If you are using CC13xx/CC26xxware, the relevant settings are under `startup_files/ccfg.c`. This is the case if you are using Contiki.

Similar to the CC2538, the bootloader will be activated if, at the time of reset, failure analysis is enabled and the selected DIO is found to be at the active level.

As an example, to bind the bootloader backdoor to KEY_SELECT on the SmartRF06EB, you need to set the following:

* `BOOTLOADER_ENABLE = 0xC5` (Bootloader enable. `SET_CCFG_BL_CONFIG_BOOTLOADER_ENABLE` in CC13xx/CC26xxware)
* `BL_LEVEL = 0x00` (Active low. `SET_CCFG_BL_CONFIG_BL_LEVEL` in CC13xx/CC26xxware)
* `BL_PIN_NUMBER = 0x0B` (DIO 11. `SET_CCFG_BL_CONFIG_BL_PIN_NUMBER` in CC13xx/CC26xxware)
* `BL_ENABLE = 0xC5` (Enable "failure analysis". `SET_CCFG_BL_CONFIG_BL_ENABLE` in CC13xx/CC26xxware)

These settings are very useful for development, but enabling failure analysis in a deployed firmware may allow a malicious user to read out the contents of your device's flash or to erase it. Do not enable this in a deployment unless you understand the security implications.

### Usage

The script will automatically select the first serial looking port from a USB to uart converter in `/dev` (OSX, Linux) for uploading. Be careful as on the SmartRF06B board under Linux this might be the jtag interface as apposed to the uart interface. In this case select the correct serial port manually with the `-p` option. Serial port selection under Windows needs testing.

Before uploading your image make sure you start the boot loader on the SoC (`select` + `reset` on CC2538DK).

Install the tool and its `cc2538-bsl` command with `pip install .` from a checkout, or run it in place with `python -m cc2538_bsl.cc2538_bsl`. You can find more info on the different options by running `cc2538-bsl -h`.

To write firmware, the target flash has to be erased first, so pass `-e` together with `-w`. Examples:

* Write a raw binary firmware: `cc2538-bsl -e -w -v example/main.bin`
* Write firmware to a Sonoff Zigbee 3.0 USB Dongle Plus: `cc2538-bsl --bootloader-sonoff-usb -e -w -v CC1352P2_CC2652P_launchpad_coordinator.hex`

Before erasing, the script checks what the write will leave in CCFG and asks for confirmation when:

* the boot loader would be closed after the write. The script works this out from the image's boot loader configuration together with what the erase clears, or, without an erase, the value read from the target;
* a write without erase cannot set CCFG bits the image asks for, such as the backdoor pin or level, because the target has those bits cleared;
* an erase clears CCFG and the image restores only part of it;
* the page erase done by `-W` clears CCFG and the image does not restore it.

An image without CCFG does not ask when an explicit `-e`, or an `-E` that covers the CCFG page, clears CCFG, because that erase was requested. The boot loader check looks only at the boot loader configuration, so it can also ask when the rest of CCFG leaves the image invalid and the ROM boot loader would still run. A write without erase whose image reaches CCFG stops with an error if CCFG cannot be read from the target. On a chip the script does not recognise, or a CC2538 that reports an unknown flash size, the location of CCFG is a guess, so `-w`, `-W`, `-i` and `-D` (and `-e`, when the flash size is unknown) ask before running instead of running these checks. With `-D`, the lockout question is left to the `-D` confirmation.

Pass `--skip-ccfg-check` to skip these CCFG, lockout and IEEE address checks and their questions, for example in scripts without a terminal. The other checks still run. `-f` skips every question, including these, and also the checks that stop `-w` from writing outside flash, `-w` and `-r` from going past the end of flash, and the `-W` page alignment check.

When `-i` sets an IEEE address without erasing its flash page, the script reads the address area first and asks if the new address cannot be written over the old one.

### Remarks

* Programming multiple SoCs at the same time is not yet supported.
* Reading the full flash of a 512K chip takes a really long time, use the length option to only read what you're interested in for now.

If you found a bug or improved some part of the code, please submit an issue or pull request.

##### Authors
Jelmer Tiete (c) 2014, <jelmer@tiete.be>   
Loosly based on [stm32loader] by Ivan A-R <ivan@tuxotronic.org>   

[![Analytics](https://ga-beacon.appspot.com/UA-3496907-10/JelmerT/cc2538-bsl?pixel)](https://github.com/igrigorik/ga-beacon)

[python]: http://www.python.org/download/ "Python Download"
[contiki cc2538dk]: https://github.com/contiki-os/contiki/tree/master/platform/cc2538dk "Contiki CC2538DK readme"
[stm32loader]: https://github.com/jsnyder/stm32loader "stm32loader"
[issue]: https://github.com/JelmerT/cc2538-bsl/issues/78 "issue"
