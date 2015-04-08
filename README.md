TI CC2538 Serial Boot Loader
============================

This folder contains a python script that communicates with the boot loader of the Texas Instruments CC2538 SoC (System on Chip).
It can be used to erase, program, verify and read the flash of the CC2538 with a simple USB to serial converter.

###Requirements

To run this script you need a Python interpreter, Linux and Mac users should be fine, Windows users have a look here: [Python Download][python].

To communicate with the uart port of the CC2538 SoC you need a usb to serial converter:   
* If you use the SmartRF06 board (CC2538DK) you can use the on-board ftdi chip. Make sure the "Enable UART" jumper is set on the board. You can have a look [here][contiki cc2538dk] for more info on drivers for this chip on different operating systems.
* If you use a different platform, there are many cheap USB to UART converters available, but make sure you use one with 3.3v voltage levels.

###Boot Loader Backdoor

Once you connected the SoC you need to make sure the serial boot loader is enabled. A chip without a valid image (program), as it comes from the factory, will automatically start the boot loader. After you upload an image to the chip, the "Image Valid" bits are set to 0 to indicate that a valid image is present in flash. On the next reset the boot loader won't be started and the image is immediately executed.   
To make sure you don't get "locked out", i.e. not being able to communicate over serial with the boot loader in the SoC anymore, you need to enable the boot loader backdoor in your image (the script currently only checks this on firmware for the 512K model). When the boot loader backdoor is enabled the boot loader will be started when the chip is reset and a specific pin of the SoC is pulled high or low (configurable).   
The boot loader backdoor can be enabled and configured with the 8-bit boot loader backdoor field in the CCA area in flash. If you set this field to 0xF3FFFFFF the boot loader will be enabled when pin PA3 is pulled low during boot. This translates to holding down the `select` button on the SmartRF06 board while pushing the `EM reset` button.
If you did lock yourself out or there is already an image flashed on your SoC, you will need a jtag programmer to erase the image. This will reset the image valid bits and enable the boot loader on the next reset. The SmartRF06EB contains both a jtag programmer and a USB to uart converter on board.

###Usage

The script will automatically select the first serial looking port from a USB to uart converter in `/dev` (OSX, Linux) for uploading. Be careful as on the SmartRF06B board under Linux this might be the jtag interface as apposed to the uart interface. In this case select the correct serial port manually with the `-p` option. Serial port selection under Windows needs testing.

Before uploading your image make sure you start the boot loader on the SoC (`select` + `reset` on CC2538DK).
You can find more info on the different options by executing `python cc2538-bsl.py -h`.

###Remarks

* Currently the CC2538-bsl only checks the boot loader backdoor enable bit on 512K model of CC2538.
* Programming multiple SoCs at the same time is not yet supported.
* Automatic detection of the serial port needs testing, especially on Windows.
* Reading the full flash of a 512K chip takes a really long time, use the length option to only read what you're interested in for now.

If you found a bug or improved some part of the code, please submit an issue or pull request.


#####Authors   
Jelmer Tiete (c) 2014, <jelmer@tiete.be>   
Loosly based on [stm32loader] by Ivan A-R <ivan@tuxotronic.org>   

[![Analytics](https://ga-beacon.appspot.com/UA-3496907-10/JelmerT/cc2538-bsl?pixel)](https://github.com/igrigorik/ga-beacon)

[python]: http://www.python.org/download/ "Python Download"
[contiki cc2538dk]: https://github.com/contiki-os/contiki/tree/master/platform/cc2538dk "Contiki CC2538DK readme"
[stm32loader]: https://github.com/jsnyder/stm32loader "stm32loader"