#!/usr/bin/env python3

# Copyright (c) 2014, Jelmer Tiete <jelmer@tiete.be>.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. The name of the author may not be used to endorse or promote
#    products derived from this software without specific prior
#    written permission.

# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
# OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Implementation based on stm32loader by Ivan A-R <ivan@tuxotronic.org>

# Serial boot loader over UART for CC13xx / CC2538 / CC26xx
# Based on the info found in TI's swru333a.pdf (spma029.pdf)
#
# Bootloader only starts if no valid image is found or if boot loader
# backdoor is enabled.
# Make sure you don't lock yourself out!! (enable backdoor in your firmware)
# More info at https://github.com/JelmerT/cc2538-bsl


import sys
import glob
import math
import time
import os
import struct
import binascii
import traceback
import argparse
import subprocess

# version
__version__ = "2.1"

# Verbose level
QUIET = 5

# Default Baud
DEFAULT_BAUD = 500000
DEFAULT_READ_LEN = 0x80000

try:
    import serial
except ImportError:
    print('{} requires the Python serial library'.format(sys.argv[0]))
    print('Please install it with:')
    print('')
    print('   pip3 install pyserial')
    sys.exit(1)


def mdebug(level, message, attr='\n'):
    if QUIET >= level:
        print(message, end=attr, file=sys.stderr)

# Takes chip IDs (obtained via Get ID command) to human-readable names
CHIP_ID_STRS = {0xb964: 'CC2538',
                0xb965: 'CC2538'
                }

RETURN_CMD_STRS = {0x40: 'Success',
                   0x41: 'Unknown command',
                   0x42: 'Invalid command',
                   0x43: 'Invalid address',
                   0x44: 'Flash fail'
                   }

COMMAND_RET_SUCCESS = 0x40
COMMAND_RET_UNKNOWN_CMD = 0x41
COMMAND_RET_INVALID_CMD = 0x42
COMMAND_RET_INVALID_ADR = 0x43
COMMAND_RET_FLASH_FAIL = 0x44


class CmdException(Exception):
    pass


class FirmwareFile:
    HEX_FILE_EXTENSIONS = ('hex', 'ihx', 'ihex')

    def __init__(self, path):
        """
        Read a firmware file and store its data ready for device programming.

        This class will try to guess the file type if python-magic is available.

        If python-magic indicates a plain text file, and if IntelHex is
        available, then the file will be treated as one of Intel HEX format.

        In all other cases, the file will be treated as a raw binary file.

        In both cases, the file's contents are stored in bytes for subsequent
        usage to program a device or to perform a crc check.

        Parameters:
            path -- A str with the path to the firmware file.

        Attributes:
            bytes: A bytearray with firmware contents ready to send to the
            device
        """
        self._crc32 = None

        try:
            from magic import from_file
            file_type = from_file(path, mime=True)

            if file_type == 'text/plain' or file_type == 'text/x-hex':
                mdebug(5, "Firmware file: Intel Hex")
                self.__read_hex(path)
            elif file_type == 'application/octet-stream':
                mdebug(5, "Firmware file: Raw Binary")
                self.__read_bin(path)
            else:
                error_str = "Could not determine firmware type. Magic " \
                            "indicates '%s'" % (file_type)
                raise CmdException(error_str)
        except ImportError:
            if os.path.splitext(path)[1][1:] in self.HEX_FILE_EXTENSIONS:
                mdebug(5, "Your firmware looks like an Intel Hex file")
                self.__read_hex(path)
            else:
                mdebug(5, "Cannot auto-detect firmware filetype: Assuming .bin")
                self.__read_bin(path)

            mdebug(10, "For more solid firmware type auto-detection, install "
                       "python-magic.")
            mdebug(10, "Please see the readme for more details.")

    def __read_hex(self, path):
        try:
            from intelhex import IntelHex
            self.bytes = bytearray(IntelHex(path).tobinarray())
        except ImportError:
            error_str = "Firmware is Intel Hex, but the IntelHex library " \
                        "could not be imported.\n" \
                        "Install IntelHex in site-packages or program " \
                        "your device with a raw binary (.bin) file.\n" \
                        "Please see the readme for more details."
            raise CmdException(error_str)

    def __read_bin(self, path):
        with open(path, 'rb') as f:
            self.bytes = bytearray(f.read())

    def crc32(self):
        """
        Return the crc32 checksum of the firmware image

        Return:
            The firmware's CRC32, ready for comparison with the CRC
            returned by the ROM bootloader's COMMAND_CRC32
        """
        if self._crc32 is None:
            self._crc32 = binascii.crc32(bytearray(self.bytes)) & 0xffffffff

        return self._crc32


class CommandInterface:

    ACK_BYTE = 0xCC
    NACK_BYTE = 0x33

    def open(self, aport=None, abaudrate=DEFAULT_BAUD):
        # Try to create the object using serial_for_url(), or fall back to the
        # old serial.Serial() where serial_for_url() is not supported.
        # serial_for_url() is a factory class and will return a different
        # object based on the URL. For example serial_for_url("/dev/tty.<xyz>")
        # will return a serialposix.Serial object for Ubuntu or Mac OS;
        # serial_for_url("COMx") will return a serialwin32.Serial oject for Windows OS.
        # For that reason, we need to make sure the port doesn't get opened at
        # this stage: We need to set its attributes up depending on what object
        # we get.
        try:
            self.sp = serial.serial_for_url(aport, do_not_open=True, timeout=20, write_timeout=20)
        except AttributeError:
            self.sp = serial.Serial(port=None, timeout=20, write_timeout=20)
            self.sp.port = aport

        if ((os.name == 'nt' and isinstance(self.sp, serial.serialwin32.Serial)) or \
           (os.name == 'posix' and isinstance(self.sp, serial.serialposix.Serial))):
            self.sp.baudrate=abaudrate        # baudrate
            self.sp.bytesize=8                # number of databits
            self.sp.parity=serial.PARITY_NONE # parity
            self.sp.stopbits=1                # stop bits
            self.sp.xonxoff=0                 # s/w (XON/XOFF) flow control
            self.sp.rtscts=0                  # h/w (RTS/CTS) flow control
            self.sp.timeout=1.0               # set the timeout value

        self.sp.open()

    def invoke_bootloader(self, dtr_active_high=False, inverted=False, sonoff_usb=False):
        # Use the DTR and RTS lines to control bootloader and the !RESET pin.
        # This can automatically invoke the bootloader without the user
        # having to toggle any pins.
        #
        # If inverted is False (default):
        # DTR: connected to the bootloader pin
        # RTS: connected to !RESET
        # If inverted is True, pin connections are the other way round
        if inverted:
            set_bootloader_pin = self.sp.setRTS
            set_reset_pin = self.sp.setDTR
        else:
            set_bootloader_pin = self.sp.setDTR
            set_reset_pin = self.sp.setRTS

        if sonoff_usb:
            mdebug(5,'sonoff')
            # this bootloader toggle is added specifically for the
            # ITead Sonoff Zigbee 3.0 USB Dongle. This dongle has an odd
            # connection between RTS DTR and reset and IO15 (imply gate):
            # DTR  RTS  |  RST  IO15
            # 1     1   |   1    1
            # 0     0   |   1    1
            # 1     0   |   0    1
            # 0     1   |   1    0
            set_bootloader_pin(0)
            set_reset_pin(1)

            set_bootloader_pin(1)
            set_reset_pin(0)

            # Let the chip register the reset while the bootloader pin is
            # asserted, then release reset (RTS) so it comes out of reset
            # into the bootloader instead of being held in reset.
            time.sleep(0.002)
            set_reset_pin(1)
        else:
            set_bootloader_pin(1 if not dtr_active_high else 0)
            set_reset_pin(0)
            set_reset_pin(1)

            set_reset_pin(0)
            # Make sure the pin is still asserted when the chip
            # comes out of reset. This fixes an issue where
            # there wasn't enough delay here on Mac.
            time.sleep(0.002)
            set_bootloader_pin(0 if not dtr_active_high else 1)

        # Some boards have a co-processor that detects this sequence here and
        # then drives the main chip's BSL enable and !RESET pins. Depending on
        # board design and co-processor behaviour, the !RESET pin may get
        # asserted after we have finished the sequence here. In this case, we
        # need a small delay so as to avoid trying to talk to main chip before
        # it has actually entered its bootloader mode.
        #
        # See contiki-os/contiki#1533
        time.sleep(0.1)

    def close(self):
        self.sp.close()

    def _wait_for_ack(self, info="", timeout=4):
        stop = time.time() + timeout
        got = bytearray(2)
        while got[-2] != 00 or got[-1] not in (CommandInterface.ACK_BYTE,
                                               CommandInterface.NACK_BYTE):
            got += self._read(1)
            if time.time() > stop:
                raise CmdException("Timeout waiting for ACK/NACK after '%s'"
                                   % (info,))

        # Our bytearray's length is: 2 initial bytes + 2 bytes for the ACK/NACK
        # plus a possible N-4 additional (buffered) bytes
        mdebug(10, "Got %d additional bytes before ACK/NACK" % (len(got) - 4,))

        # wait for ask
        ask = got[-1]

        if ask == CommandInterface.ACK_BYTE:
            # ACK
            return 1
        elif ask == CommandInterface.NACK_BYTE:
            # NACK
            mdebug(10, "Target replied with a NACK during %s" % info)
            return 0

        # Unknown response
        mdebug(10, "Unrecognised response 0x%x to %s" % (ask, info))
        return 0

    def _encode_addr(self, addr):
        byte3 = (addr >> 0) & 0xFF
        byte2 = (addr >> 8) & 0xFF
        byte1 = (addr >> 16) & 0xFF
        byte0 = (addr >> 24) & 0xFF
        return bytes([byte0, byte1, byte2, byte3])

    def _decode_addr(self, byte0, byte1, byte2, byte3):
        return ((byte3 << 24) | (byte2 << 16) | (byte1 << 8) | (byte0 << 0))

    def _calc_checks(self, cmd, addr, size):
        return ((sum(bytearray(self._encode_addr(addr))) +
                 sum(bytearray(self._encode_addr(size))) +
                 cmd) & 0xFF)

    def _write(self, data, is_retry=False):
        if isinstance(data, int):
            assert data < 256
            goal = 1
            written = self.sp.write(bytes([data]))
        elif isinstance(data, (bytes, bytearray)):
            goal = len(data)
            written = self.sp.write(data)
        else:
            raise CmdException("Internal Error. Bad data type: {}"
                               .format(type(data)))

        if written < goal:
            mdebug(10, "*** Only wrote {} of target {} bytes"
                   .format(written, goal))
            if is_retry and written == 0:
                raise CmdException("Failed to write data on the serial bus")
            mdebug(10, "*** Retrying write for remainder")
            if isinstance(data, int):
                return self._write(data, is_retry=True)
            else:
                return self._write(data[written:], is_retry=True)

    def _read(self, length):
        return bytearray(self.sp.read(length))

    def sendAck(self):
        self._write(0x00)
        self._write(0xCC)
        return

    def sendNAck(self):
        self._write(0x00)
        self._write(0x33)
        return

    def receivePacket(self):
        # stop = time.time() + 5
        # got = None
        # while not got:
        got = self._read(2)
        #     if time.time() > stop:
        #         break

        # if not got:
        #     raise CmdException("No response to %s" % info)

        if len(got) < 2:
            raise CmdException("No response packet from target")
        if got[0] < 2:
            raise CmdException("Invalid response packet size %d" % got[0])
        size = got[0]  # rcv size
        chks = got[1]  # rcv checksum
        data = bytearray(self._read(size - 2))  # rcv data
        if len(data) != max(size - 2, 0):
            raise CmdException("Incomplete response packet from target")

        mdebug(10, "*** received %x bytes" % size)
        if chks == sum(data) & 0xFF:
            self.sendAck()
            return data
        else:
            self.sendNAck()
            # TODO: retry receiving!
            raise CmdException("Received packet checksum error")
            return 0

    def sendSynch(self):
        cmd = 0x55

        # flush serial input buffer for first ACK reception
        self.sp.flushInput()

        mdebug(10, "*** sending synch sequence")
        self._write(cmd)  # send U
        self._write(cmd)  # send U
        return self._wait_for_ack("Synch (0x55 0x55)", 8)

    def checkLastCmd(self):
        stat = self.cmdGetStatus()
        if not (stat):
            raise CmdException("No response from target on status request. "
                               "(Did you disable the bootloader?)")

        if stat[0] == COMMAND_RET_SUCCESS:
            mdebug(10, "Command Successful")
            return 1
        else:
            stat_str = RETURN_CMD_STRS.get(stat[0], None)
            if stat_str is None:
                mdebug(0, "Warning: unrecognized status returned "
                          "0x%x" % stat[0])
            else:
                mdebug(0, "Target returned: 0x%x, %s" % (stat[0], stat_str))
            return 0

    def cmdPing(self):
        cmd = 0x20
        lng = 3

        self._write(lng)  # send size
        self._write(cmd)  # send checksum
        self._write(cmd)  # send data

        mdebug(10, "*** Ping command (0x20)")
        if self._wait_for_ack("Ping (0x20)"):
            return self.checkLastCmd()

    def cmdReset(self):
        cmd = 0x25
        lng = 3

        self._write(lng)  # send size
        self._write(cmd)  # send checksum
        self._write(cmd)  # send data

        mdebug(10, "*** Reset command (0x25)")
        if self._wait_for_ack("Reset (0x25)"):
            return 1

    def cmdGetChipId(self):
        cmd = 0x28
        lng = 3

        self._write(lng)  # send size
        self._write(cmd)  # send checksum
        self._write(cmd)  # send data

        mdebug(10, "*** GetChipId command (0x28)")
        if self._wait_for_ack("Get ChipID (0x28)"):
            # 4 byte answ, the 2 LSB hold chip ID
            version = self.receivePacket()
            if self.checkLastCmd():
                if len(version) != 4:
                    raise CmdException("Unreasonable chip id: %s"
                                       % repr(version))
                mdebug(10, "    Version 0x%02X%02X%02X%02X" % tuple(version))
                chip_id = (version[2] << 8) | version[3]
                return chip_id
            else:
                raise CmdException("GetChipID (0x28) failed")

    def cmdGetStatus(self):
        cmd = 0x23
        lng = 3

        self._write(lng)  # send size
        self._write(cmd)  # send checksum
        self._write(cmd)  # send data

        mdebug(10, "*** GetStatus command (0x23)")
        if self._wait_for_ack("Get Status (0x23)"):
            stat = self.receivePacket()
            return stat

    def cmdSetXOsc(self):
        cmd = 0x29
        lng = 3

        self._write(lng)  # send size
        self._write(cmd)  # send checksum
        self._write(cmd)  # send data

        mdebug(10, "*** SetXOsc command (0x29)")
        if self._wait_for_ack("SetXOsc (0x29)"):
            return 1
            # UART speed (needs) to be changed!

    def cmdRun(self, addr):
        cmd = 0x22
        lng = 7

        self._write(lng)  # send length
        self._write(self._calc_checks(cmd, addr, 0))  # send checksum
        self._write(cmd)  # send cmd
        self._write(self._encode_addr(addr))  # send addr

        mdebug(10, "*** Run command(0x22)")
        return 1

    def cmdEraseMemory(self, addr, size):
        cmd = 0x26
        lng = 11

        self._write(lng)  # send length
        self._write(self._calc_checks(cmd, addr, size))  # send checksum
        self._write(cmd)  # send cmd
        self._write(self._encode_addr(addr))  # send addr
        self._write(self._encode_addr(size))  # send size

        mdebug(10, "*** Erase command(0x26)")
        if self._wait_for_ack("Erase memory (0x26)", 10):
            return self.checkLastCmd()

    def cmdSectorEraseCC26xx(self, addr):
        cmd = 0x26
        lng = 7

        self._write(lng)  # send length
        self._write(self._calc_checks(cmd, addr, 0))  # send checksum
        self._write(cmd)  # send cmd
        self._write(self._encode_addr(addr))  # send addr

        mdebug(10, "*** Sector Erase command(0x26)")
        if self._wait_for_ack("Sector Erase (0x26)", 10):
            return self.checkLastCmd()

    def cmdBankErase(self):
        cmd = 0x2C
        lng = 3

        self._write(lng)  # send length
        self._write(cmd)  # send checksum
        self._write(cmd)  # send cmd

        mdebug(10, "*** Bank Erase command(0x2C)")
        if self._wait_for_ack("Bank Erase (0x2C)", 10):
            return self.checkLastCmd()

    def cmdCRC32(self, addr, size):
        cmd = 0x27
        lng = 11

        self._write(lng)  # send length
        self._write(self._calc_checks(cmd, addr, size))  # send checksum
        self._write(cmd)  # send cmd
        self._write(self._encode_addr(addr))  # send addr
        self._write(self._encode_addr(size))  # send size

        mdebug(10, "*** CRC32 command(0x27)")
        if self._wait_for_ack("Get CRC32 (0x27)", 1):
            crc = self.receivePacket()
            if self.checkLastCmd():
                if len(crc) != 4:
                    raise CmdException("Unexpected CRC32 reply length %d"
                                       % len(crc))
                return self._decode_addr(crc[3], crc[2], crc[1], crc[0])

    def cmdCRC32CC26xx(self, addr, size):
        cmd = 0x27
        lng = 15

        self._write(lng)  # send length
        self._write(self._calc_checks(cmd, addr, size))  # send checksum
        self._write(cmd)  # send cmd
        self._write(self._encode_addr(addr))  # send addr
        self._write(self._encode_addr(size))  # send size
        self._write(self._encode_addr(0x00000000))  # send number of reads

        mdebug(10, "*** CRC32 command(0x27)")
        if self._wait_for_ack("Get CRC32 (0x27)", 1):
            crc = self.receivePacket()
            if self.checkLastCmd():
                if len(crc) != 4:
                    raise CmdException("Unexpected CRC32 reply length %d"
                                       % len(crc))
                return self._decode_addr(crc[3], crc[2], crc[1], crc[0])

    def cmdDownload(self, addr, size):
        cmd = 0x21
        lng = 11

        if (size % 4) != 0:  # check for invalid data lengths
            raise Exception('Invalid data size: %i. '
                            'Size must be a multiple of 4.' % size)

        self._write(lng)  # send length
        self._write(self._calc_checks(cmd, addr, size))  # send checksum
        self._write(cmd)  # send cmd
        self._write(self._encode_addr(addr))  # send addr
        self._write(self._encode_addr(size))  # send size

        mdebug(10, "*** Download command (0x21)")
        if self._wait_for_ack("Download (0x21)", 2):
            return self.checkLastCmd()

    def cmdSendData(self, data):
        cmd = 0x24
        lng = len(data)+3
        # TODO: check total size of data!! max 252 bytes!

        self._write(lng)  # send size
        self._write((sum(bytearray(data))+cmd) & 0xFF)  # send checksum
        self._write(cmd)  # send cmd
        self._write(bytearray(data))  # send data

        mdebug(10, "*** Send Data (0x24)")
        if self._wait_for_ack("Send data (0x24)", 10):
            return self.checkLastCmd()

    def cmdMemRead(self, addr):  # untested
        cmd = 0x2A
        lng = 8

        self._write(lng)  # send length
        self._write(self._calc_checks(cmd, addr, 4))  # send checksum
        self._write(cmd)  # send cmd
        self._write(self._encode_addr(addr))  # send addr
        self._write(4)  # send width, 4 bytes

        mdebug(10, "*** Mem Read (0x2A)")
        if self._wait_for_ack("Mem Read (0x2A)", 1):
            data = self.receivePacket()
            if self.checkLastCmd():
                # self._decode_addr(ord(data[3]),
                #                   ord(data[2]),ord(data[1]),ord(data[0]))
                return data

    def cmdMemReadCC26xx(self, addr):
        cmd = 0x2A
        lng = 9

        self._write(lng)  # send length
        self._write(self._calc_checks(cmd, addr, 2))  # send checksum
        self._write(cmd)  # send cmd
        self._write(self._encode_addr(addr))  # send addr
        self._write(1)  # send width, 4 bytes
        self._write(1)  # send number of reads

        mdebug(10, "*** Mem Read (0x2A)")
        if self._wait_for_ack("Mem Read (0x2A)", 1):
            data = self.receivePacket()
            if self.checkLastCmd():
                return data

    def cmdMemWrite(self, addr, data, width):
        if width != len(data):
            raise ValueError("width does not match len(data)")
        if width != 1 and width != 4:
            raise ValueError("width must be 1 or 4")

        cmd = 0x2B
        lng = 8 + len(data)

        content = (
            bytearray([cmd])
            + self._encode_addr(addr)
            + bytearray([1 if (width == 4) else 0])
            + bytearray(data)
        )

        self._write(lng)  # send length
        self._write(sum(content) & 0xFF)  # send checksum
        self._write(content)

        mdebug(10, "*** Mem write (0x2B)")
        if self._wait_for_ack("Mem Write (0x2B)", 2):
            return self.checkLastCmd()


# Complex commands section

    def writeMemory(self, addr, data, force=False):
        lng = len(data)
        # amount of data bytes transferred per packet (theory: max 252 + 3)
        trsf_size = 248
        empty_packet = bytearray((0xFF,) * trsf_size)

        # Boot loader enable check for a full CC2538 512K image. The CLI
        # checks images with ccfg_write_question before erasing, unless
        # --force or --skip-ccfg-check is given.
        if lng == 524288 and addr == 0x00200000:
            # check the boot loader enable bit  (only for 512K model)
            if not ((data[524247] & (1 << 4)) >> 4):
                if not (force or
                        query_yes_no("The boot loader backdoor is not enabled "
                                     "in the firmware you are about to write "
                                     "to the target. You will NOT be able to "
                                     "reprogram the target using this tool if "
                                     "you continue! "
                                     "Do you want to continue?", "no")):
                    raise Exception('Aborted by user.')

        mdebug(5, "Writing %(lng)d bytes starting at address 0x%(addr)08X" %
               {'lng': lng, 'addr': addr})

        offs = 0
        addr_set = 0

        # check if amount of remaining data is less then packet size
        while lng > trsf_size:
            # skip packets filled with 0xFF
            if data[offs:offs+trsf_size] != empty_packet:
                if addr_set != 1:
                    # set starting address if not set
                    if not self.cmdDownload(addr, lng):
                        return 0
                    addr_set = 1
                mdebug(5, " Write %(len)d bytes at 0x%(addr)08X"
                       % {'addr': addr, 'len': trsf_size}, '\r')
                sys.stdout.flush()

                # send next data packet
                if not self.cmdSendData(data[offs:offs+trsf_size]):
                    return 0
            else:   # skipped packet, address needs to be set
                addr_set = 0

            offs = offs + trsf_size
            addr = addr + trsf_size
            lng = lng - trsf_size

        mdebug(5, "Write %(len)d bytes at 0x%(addr)08X" % {'addr': addr,
                                                           'len': lng})
        # While a transfer is in progress this Download is redundant. It is
        # still sent to keep the bytes on the wire unchanged, but only a
        # Download that starts a transfer has to succeed.
        download_ok = self.cmdDownload(addr, lng)
        if addr_set != 1 and not download_ok:
            return 0
        return self.cmdSendData(data[offs:offs+lng])  # send last data packet


class Chip:
    def __init__(self, command_interface):
        self.command_interface = command_interface

        # Some defaults. The child can override.
        self.flash_start_addr = 0x00000000
        self.has_cmd_set_xosc = False
        self.page_size = 2048

    @property
    def flash_end(self):
        return self.flash_start_addr + self.size

    def page_align_up(self, value):
        return int(math.ceil(value / self.page_size) * self.page_size)


    def page_align_down(self, value):
        return int(math.floor(value / self.page_size) * self.page_size)


    def page_to_addr(self, pages):
        addresses = []
        for page in pages:
            addresses.append(int(self.flash_start_addr) +
                             int(page)*self.page_size)
        return addresses

    ccfg_known = True  # False when the CCFG address is only a guess
    size_known = True  # False when the flash size is only a guess
    size_code = None   # the undocumented size code behind a guessed size
    max_flash_size = None  # largest flash the chip family can have

    def size_warning(self):
        """Return a warning when the flash size is a guess, else None.

        The constructor does not print it; main_cli does, and other callers
        should check it after creating the device.
        """
        if self.size_known:
            return None
        return ("Warning: %s. Reads, erases and range checks use this size."
                % guessed_size_note(self))

    def ccfg_range(self):
        """Return the (address, length) of the CCFG area."""
        return self.bootloader_address, 44

    def image_covers_ccfg(self, image, address):
        """Return True if the image spans the whole CCFG area."""
        ccfg_addr, ccfg_len = self.ccfg_range()
        return address <= ccfg_addr and ccfg_addr + ccfg_len <= address + len(image)

    def image_has_ccfg(self, image, address):
        """Return True if the image programs any byte of the CCFG area."""
        ccfg_addr, ccfg_len = self.ccfg_range()
        start = max(ccfg_addr - address, 0)
        end = min(ccfg_addr + ccfg_len - address, len(image))
        return start < end and any(b != 0xFF for b in image[start:end])

    BLANK_WORD = b'\xFF' * 4

    def image_bootloader_bytes(self, image, address):
        """Return {index: byte} for the boot loader word bytes in the image."""
        found = {}
        for i in range(4):
            offset = self.bootloader_address + i - address
            if 0 <= offset < len(image):
                found[i] = image[offset]
        return found

    def image_covers_bootloader_word(self, image, address):
        return len(self.image_bootloader_bytes(image, address)) == 4

    def read_ccfg(self):
        """Return the bytes of the CCFG area on the chip."""
        ccfg_addr, ccfg_len = self.ccfg_range()
        data = bytearray()
        for addr in range(ccfg_addr, ccfg_addr + ccfg_len, 4):
            word = self.read_memory(addr)
            if word is None or len(word) != 4:
                raise CmdException("Short read at address 0x%08X" % addr)
            data += word
        return bytes(data)

    def ccfg_bits_lost(self, image, address, chip_ccfg):
        """Return the CCFG addresses where the chip has bits cleared that
        the image wants set, which a write without erase cannot set.

        Image bytes of 0xFF in the IEEE address fields are left out: there
        0xFF means unset, and -i may have programmed an address.
        """
        ccfg_addr, ccfg_len = self.ccfg_range()
        ieee = [range(a, a + n) for a, n in self.ccfg_address_fields()]
        lost = []
        for i in range(ccfg_len):
            offset = ccfg_addr + i - address
            if not 0 <= offset < len(image):
                continue
            if image[offset] == 0xFF and any(ccfg_addr + i in r for r in ieee):
                continue
            if chip_ccfg[i] & image[offset] != image[offset]:
                lost.append(ccfg_addr + i)
        return lost

    def ccfg_address_fields(self):
        """Return (address, length) of the IEEE address fields in CCFG."""
        return []  # CC2538 keeps its secondary address outside CCFG

    def bootloader_word_after_write(self, image, address, start):
        """Return the boot loader word the chip holds after the write.

        start is the word before the write: BLANK_WORD after an erase, or
        the bytes on the chip, because flash writes can only clear bits.
        """
        word = bytearray(start)
        for i, value in self.image_bootloader_bytes(image, address).items():
            word[i] &= value
        return struct.unpack('<L', bytes(word))[0]

    def image_disables_bootloader(self, image, address, start=None):
        """Return True if writing the image would lock out this bootloader.

        start is the boot loader word before the write, as for
        bootloader_word_after_write; None means BLANK_WORD. An image that
        programs no CCFG byte leaves CCFG blank after an erase, which makes
        the image invalid so the ROM bootloader runs, and leaves it
        unchanged otherwise.
        """
        if not self.image_has_ccfg(image, address):
            return False
        if start is None:
            start = self.BLANK_WORD
        word = self.bootloader_word_after_write(image, address, start)
        return not self._bootloader_enabled(word)

    def crc(self, address, size):
        return getattr(self.command_interface, self.crc_cmd)(address, size)

    def confirm_disable_bootloader(self, force=False):
        if not (force or
                query_yes_no("Disabling the bootloader will prevent you from "
                             "using this script until you re-enable the "
                             "bootloader using JTAG. Do you want to continue?",
                             "no")):
            raise Exception('Aborted by user.')

    def disable_bootloader(self, force=False):
        self.confirm_disable_bootloader(force)

        pattern = struct.pack('<L', self.bootloader_dis_val)

        if self.command_interface.writeMemory(self.bootloader_address, pattern):
            mdebug(5, "    Set bootloader closed done                      ")
        else:
            raise CmdException("Set bootloader closed failed             ")


class CC2538(Chip):
    max_flash_size = 0x80000

    def __init__(self, command_interface):
        super().__init__(command_interface)
        self.flash_start_addr = 0x00200000
        self.has_cmd_set_xosc = True
        self.bootloader_dis_val = 0xefffffff
        self.crc_cmd = "cmdCRC32"

        FLASH_CTRL_DIECFG0 = 0x400D3014
        FLASH_CTRL_DIECFG2 = 0x400D301C
        addr_ieee_address_primary = 0x00280028
        ccfg_len = 44

        # Read out primary IEEE address, flash and RAM size
        model = self.command_interface.cmdMemRead(FLASH_CTRL_DIECFG0)
        self.size = (model[3] & 0x70) >> 4
        if 0 < self.size <= 4:
            self.size *= 0x20000  # in bytes
        else:
            self.size = 0x10000  # in bytes
            # Code 0 and the reserved codes 5-7 are not documented sizes, so
            # the end of flash (and CCFG) is a guess
            self.ccfg_known = False
            self.size_known = False
            self.size_code = (model[3] & 0x70) >> 4  # see size_warning()
        self.bootloader_address = self.flash_start_addr + self.size - ccfg_len
        self.addr_ieee_address_secondary = (self.flash_start_addr + self.size -
                                            ccfg_len - 8)

        sram = (((model[2] << 8) | model[3]) & 0x380) >> 7
        sram = (2 - sram) << 3 if sram <= 1 else 32  # in KB

        pg = self.command_interface.cmdMemRead(FLASH_CTRL_DIECFG2)
        pg_major = (pg[2] & 0xF0) >> 4
        if pg_major == 0:
            pg_major = 1
        pg_minor = pg[2] & 0x0F

        ti_oui = bytearray([0x00, 0x12, 0x4B])
        ieee_addr = self.command_interface.cmdMemRead(
                                            addr_ieee_address_primary)
        ieee_addr_end = self.command_interface.cmdMemRead(
                                            addr_ieee_address_primary + 4)
        if ieee_addr[:3] == ti_oui:
            ieee_addr += ieee_addr_end
        else:
            ieee_addr = ieee_addr_end + ieee_addr

        flash = ("%dKB Flash" % (self.size >> 10) if self.size_known
                 else "%dKB Flash (guessed, size code %d)"
                 % (self.size >> 10, self.size_code))
        mdebug(5, "CC2538 PG%d.%d: %s, %dKB SRAM, CCFG at 0x%08X"
               % (pg_major, pg_minor, flash, sram, self.bootloader_address))
        mdebug(5, "Primary IEEE Address: %s"
               % (':'.join('%02X' % x for x in ieee_addr)))

    def _bootloader_enabled(self, word):
        return bool(word & (1 << 28))

    def erase(self):
        return self.eraseRange(self.flash_start_addr, self.size)

    def eraseRange(self, addr, size):
        mdebug(5, "Erasing %s bytes starting at address 0x%08X"
               % (size, addr))
        return self.command_interface.cmdEraseMemory(addr, size)

    def read_memory(self, addr):
        # CC2538's COMMAND_MEMORY_READ sends each 4-byte number in inverted
        # byte order compared to what's written on the device
        data = self.command_interface.cmdMemRead(addr)
        if data is None or len(data) != 4:
            raise CmdException("Read failed at address 0x%08X" % addr)
        return bytearray([data[x] for x in range(3, -1, -1)])


class CC26xx(Chip):
    # Class constants
    MISC_CONF_1 = 0x500010A0
    PROTO_MASK_BLE = 0x01
    PROTO_MASK_IEEE = 0x04
    PROTO_MASK_BOTH = 0x05

    def __init__(self, command_interface):
        super().__init__(command_interface)
        self.bootloader_dis_val = 0x00000000
        self.crc_cmd = "cmdCRC32CC26xx"
        self.page_size = 4096

        ICEPICK_DEVICE_ID = 0x50001318
        FCFG_USER_ID = 0x50001294
        PRCM_RAMHWOPT = 0x40082250
        FLASH_SIZE = 0x4003002C
        addr_ieee_address_primary = 0x500012F0
        ccfg_len = 88
        ieee_address_secondary_offset = 0x20
        bootloader_dis_offset = 0x30
        sram = "Unknown"

        # Determine CC13xx vs CC26xx via ICEPICK_DEVICE_ID::WAFER_ID and store
        # PG revision
        device_id = self.command_interface.cmdMemReadCC26xx(ICEPICK_DEVICE_ID)
        wafer_id = (((device_id[3] & 0x0F) << 16) +
                    (device_id[2] << 8) +
                    (device_id[1] & 0xF0)) >> 4
        pg_rev = (device_id[3] & 0xF0) >> 4

        # Read FCFG1_USER_ID to get the package and supported protocols
        user_id = self.command_interface.cmdMemReadCC26xx(FCFG_USER_ID)
        package = {0x00: '4x4mm',
                   0x01: '5x5mm',
                   0x02: '7x7mm',
                   0x03: 'Wafer',
                   0x04: '2.7x2.7',
                   0x05: '7x7mm Q1',
                   }.get(user_id[2] & 0x03, "Unknown")

        protocols = user_id[1] >> 4

        # We can now detect the exact device
        if wafer_id == 0xB99A:
            chip = self._identify_cc26xx(pg_rev, protocols)
        elif wafer_id == 0xB9BE:
            chip = self._identify_cc13xx(pg_rev, protocols)
        elif wafer_id == 0xBB41 or wafer_id == 0xBB77 or wafer_id == 0xBB7A:
            chip = self._identify_cc13xx(pg_rev, protocols)
            self.page_size = 8192
        else:
            # Unknown wafer_id: continue with a generic name and 8 KB pages
            # (the layout used by recent CC13xx/CC26xx parts) instead of
            # crashing with an UnboundLocalError. See issues #150, #153.
            chip = "Unknown (wafer_id=0x%X)" % wafer_id
            self.page_size = 8192
            # CC13x4/CC26x4-class parts keep CCFG outside main flash, so
            # its address here is a guess
            self.ccfg_known = False
            mdebug(5, "Warning: unrecognized wafer_id 0x%X, assuming a "
                      "CC13xx/CC26xx-class part with %d byte flash pages. "
                      "Flashing may not work correctly; please report this "
                      "chip." % (wafer_id, self.page_size))

        # Read flash size, calculate and store bootloader disable address
        self.size = self.command_interface.cmdMemReadCC26xx(
                                                FLASH_SIZE)[0] * self.page_size
        self.bootloader_address = self.size - ccfg_len + bootloader_dis_offset
        self.addr_ieee_address_secondary = (self.size - ccfg_len +
                                            ieee_address_secondary_offset)

        # RAM size
        ramhwopt_size = self.command_interface.cmdMemReadCC26xx(
                                                PRCM_RAMHWOPT)[0] & 3
        if ramhwopt_size == 3:
            sram = "20KB"
        elif ramhwopt_size == 2:
            sram = "16KB"
        else:
            sram = "Unknown"

        # Primary IEEE address. Stored with the MSB at the high address
        ieee_addr = self.command_interface.cmdMemReadCC26xx(
                                        addr_ieee_address_primary + 4)[::-1]
        ieee_addr += self.command_interface.cmdMemReadCC26xx(
                                        addr_ieee_address_primary)[::-1]

        mdebug(5, "%s (%s): %dKB Flash, %s SRAM, CCFG.BL_CONFIG at 0x%08X"
               % (chip, package, self.size >> 10, sram,
                  self.bootloader_address))
        mdebug(5, "Primary IEEE Address: %s"
               % (':'.join('%02X' % x for x in ieee_addr)))

    def _identify_cc26xx(self, pg, protocols):
        chips_dict = {
            CC26xx.PROTO_MASK_IEEE: 'CC2630',
            CC26xx.PROTO_MASK_BLE: 'CC2640',
            CC26xx.PROTO_MASK_BOTH: 'CC2650',
        }

        chip_str = chips_dict.get(protocols & CC26xx.PROTO_MASK_BOTH, "Unknown")
        pg_str = "PG unknown (0x%X)" % pg

        if pg == 1:
            pg_str = "PG1.0"
        elif pg == 3:
            pg_str = "PG2.0"
        elif pg == 7:
            pg_str = "PG2.1"
        elif pg == 8 or pg == 0x0B:
            # CC26x0 PG2.2+ or CC26x0R2
            rev_minor = self.command_interface.cmdMemReadCC26xx(
                                                CC26xx.MISC_CONF_1)[0]
            if rev_minor == 0xFF:
                rev_minor = 0x00

            if pg == 8:
                # CC26x0
                pg_str = "PG2.%d" % (2 + rev_minor,)
            elif pg == 0x0B:
                # HW revision R2, update Chip name
                chip_str += 'R2'
                pg_str = "PG%d.%d" % (1 + (rev_minor // 10), rev_minor % 10)

        return "%s %s" % (chip_str, pg_str)

    def _identify_cc13xx(self, pg, protocols):
        chip_str = "CC1310"
        if protocols & CC26xx.PROTO_MASK_IEEE == CC26xx.PROTO_MASK_IEEE:
            chip_str = "CC1350"
        pg_str = "PG unknown (0x%X)" % pg

        if pg == 0:
            pg_str = "PG1.0"
        elif pg == 2 or pg == 3 or pg == 1:
            rev_minor = self.command_interface.cmdMemReadCC26xx(
                                                CC26xx.MISC_CONF_1)[0]
            if rev_minor == 0xFF:
                rev_minor = 0x00
            pg_str = "PG2.%d" % (rev_minor,)

        return "%s %s" % (chip_str, pg_str)

    def _bootloader_enabled(self, word):
        # CCFG.BL_CONFIG: BOOTLOADER_ENABLE and BL_ENABLE (backdoor)
        return (word >> 24) == 0xC5 and (word & 0xFF) == 0xC5

    def ccfg_range(self):
        return self.bootloader_address - 0x30, 88

    def ccfg_address_fields(self):
        # CCFG.IEEE_MAC_0/1 (set by -i) and CCFG.IEEE_BLE_0/1
        ccfg_addr = self.ccfg_range()[0]
        return [(ccfg_addr + 0x20, 8), (ccfg_addr + 0x28, 8)]

    def erase(self):
        mdebug(5, "Erasing all main bank flash sectors")
        return self.command_interface.cmdBankErase()

    def eraseRange(self, addr, size):
        mdebug(5, "Erasing %s bytes starting at address 0x%08X"
               % (size, addr))
        for curAddr in range(addr, addr + size, self.page_size):
            mdebug(6, "Erasing sector starting at address 0x%08X" % curAddr)
            if not self.command_interface.cmdSectorEraseCC26xx(curAddr):
                return 0
        return 1

    def read_memory(self, addr):
        # CC26xx COMMAND_MEMORY_READ returns contents in the same order as
        # they are stored on the device
        data = self.command_interface.cmdMemReadCC26xx(addr)
        if data is None or len(data) != 4:
            raise CmdException("Read failed at address 0x%08X" % addr)
        return data


SKIP_CCFG_HINT = ("Use --skip-ccfg-check to skip the CCFG, lockout and IEEE "
                  "address questions (and the -e question on a chip with a "
                  "guessed flash size), or --force to skip all prompts and "
                  "some checks.")


def query_yes_no(question, default="yes",
                 skip_hint="Use --force to skip prompts."):
    valid = {"yes": True,
             "y": True,
             "ye": True,
             "no": False,
             "n": False}
    if default is None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        try:
            choice = input().lower()
        except EOFError:
            raise Exception("No answer to a confirmation prompt (input is "
                            "closed). " + skip_hint)
        if default is not None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' "
                             "(or 'y' or 'n').\n")


# Convert the entered IEEE address into an integer
def parse_ieee_address(inaddr):
    try:
        addr = int(inaddr, 16)
    except ValueError:
        # inaddr is not a hex string, look for other formats
        if ':' in inaddr:
            octets = inaddr.split(':')
        elif '-' in inaddr:
            octets = inaddr.split('-')
        else:
            raise ValueError("Supplied IEEE address is not a hex integer nor "
                             "a ':' or '-' separated list of 8 bytes")
        if len(octets) != 8:
            raise ValueError("Supplied IEEE address does not contain 8 bytes")
        addr = 0
        for i, b in zip(range(8), octets):
            try:
                octet = int(b, 16)
            except ValueError:
                raise ValueError("IEEE address contains invalid bytes")
            if not 0 <= octet <= 0xFF:
                raise ValueError("IEEE address contains invalid bytes")
            addr += octet << (56-(i*8))
    if not 0 <= addr < 2 ** 64:
        raise ValueError("IEEE address does not fit in 64 bits")
    return addr


def _parse_range_number(value):
    """Parse an int, 0x-prefixed hex, or decimal."""
    if isinstance(value, int):
        return value
    text = value.strip().lower()
    if text.startswith('0x'):
        return int(text, 16)
    return int(text, 10)


def _parse_range_values(device, values):
    if len(values) and len(values) < 3:
        page_addr_range = []
        for value in values:
            try:
                number = _parse_range_number(value)
            except ValueError:
                raise ValueError("Supplied value is not a page or an address")
            if number % int(device.page_size) != 0:
                raise ValueError("Supplied addresses are not page_size: "
                                 "{} aligned".format(device.page_size))
            page_addr_range.append(number)
        return page_addr_range
    else:
        raise ValueError("Supplied range is neither a page or address range")


def parse_page_address_range(device, pg_range):
    """Convert the address/page range into a start address and byte length"""
    values = pg_range.split(',')
    page_addr = []
    # check if first argument is character
    if values[0].isalpha():
        values[0] = values[0].lower()
        if values[0] == 'p' or values[0] == 'page':
            values[1:] = device.page_to_addr(values[1:])
        elif values[0] != 'a' and values[0] != 'address':
            raise ValueError("Prefix is neither a(address) or p(page)")
        page_addr.extend(_parse_range_values(device, values[1:]))
    else:
        page_addr.extend(_parse_range_values(device, values))
    if len(page_addr) == 1:
        return [page_addr[0], device.page_size]
    else:
        return [page_addr[0], (page_addr[1] - page_addr[0])]


def ranges_overlap(ranges, start, length):
    """Return True if any (start, length) range overlaps the given range."""
    return any(r_start < start + length and start < r_start + r_length
               for r_start, r_length in ranges)


def guessed_ccfg_question(device, args, ieee_addr):
    """Return the question for a chip whose CCFG location is a guess, or
    None when no write, -i, -D or (with a guessed size) -e is requested."""
    if device.size_known:
        reason = "This chip was not recognised"
    else:
        reason = "The flash size this chip reports is not documented"
    parts = []
    if args.write or args.erase_write:
        parts.append("the write may reach CCFG, and the boot loader "
                     "lockout check cannot run")
    if ieee_addr is not None:
        parts.append("-i writes the IEEE address at the guessed address "
                     "0x%08X, which may hold application code"
                     % device.addr_ieee_address_secondary)
    if args.disable_bootloader:
        parts.append("-D writes the boot loader word at the guessed "
                     "address 0x%08X, which may hold application code"
                     % device.bootloader_address)
    if args.erase and not device.size_known:
        parts.append("-e erases only the guessed %d KB of flash"
                     % (device.size >> 10))
    if not parts:
        return None
    return ("%s, so the location of its CCFG (%d KB of flash) is a guess: "
            "%s. Pass --skip-ccfg-check to skip this question. Do you want "
            "to continue?" % (reason, device.size >> 10, "; ".join(parts)))


def ieee_write_question(device, ieee_bytes, image, address, erase_ranges):
    """Return the question to ask before -i writes the IEEE address, or None.

    The address is written without its own erase, so it is ANDed with the
    bytes on the chip after any erase and image write in this run.
    erase_ranges are all erases in this run, including the -W one. image
    is None without a write.
    """
    ieee_addr = device.addr_ieee_address_secondary
    ieee_page = device.page_align_down(ieee_addr)
    if image is not None and ranges_overlap([(address, len(image))],
                                            ieee_page, device.page_size):
        hint = ("Erase its flash page first with -e, -W, or an -E range "
                "that covers it.")
    elif image is not None:
        hint = ("Erase its flash page first with -e or an -E range that "
                "covers it.")
    else:
        hint = ("Erase its flash page first with an -E range that covers "
                "it, and write that page again.")
    if ranges_overlap(erase_ranges, ieee_addr, len(ieee_bytes)):
        start = bytearray(b'\xFF' * len(ieee_bytes))
    else:
        try:
            start = bytearray()
            for addr in range(ieee_addr, ieee_addr + len(ieee_bytes), 4):
                start += device.read_memory(addr)
        except (CmdException, serial.SerialException) as err:
            raise CmdException("Could not read the IEEE address area from "
                               "the target to check the -i write (%s). %s "
                               "or pass --skip-ccfg-check."
                               % (err, hint.rstrip('.') + ','))
    if image is not None:
        for i in range(len(start)):
            offset = ieee_addr + i - address
            if 0 <= offset < len(image):
                start[i] &= image[offset]
    if all(start[i] & b == b for i, b in enumerate(ieee_bytes)):
        return None
    return ("The IEEE address area at 0x%08X on the target already has bits "
            "cleared that this write cannot set, so the address stored "
            "will not be the one given with -i. %s Do you want to "
            "continue?" % (ieee_addr, hint))


def ccfg_write_question(device, image, address, erase_ranges, write_erase,
                        disabling=False):
    """Return the question to ask before a write changes CCFG, or None.

    erase_ranges are the erases asked for with -e or -E. write_erase is
    the page range -W erases, or None. disabling is set for -D, which
    asks about the lockout itself. Asks at most one question, with a
    lockout taking priority over a firmware that will not boot.
    """
    ccfg_addr, ccfg_len = device.ccfg_range()
    explicit = ranges_overlap(erase_ranges, ccfg_addr, ccfg_len)
    implicit = (write_erase is not None
                and ranges_overlap([write_erase], ccfg_addr, ccfg_len))
    # Erase ranges are page aligned (_parse_range_values, page_align_*) and
    # CCFG sits in the last page, so any erase that touches CCFG clears all
    # of it, including the boot loader word.
    erased = explicit or implicit
    erase_hint = (" Erase the CCFG page first with -e, -W, or an -E range "
                  "that covers it.")
    lockout = (" You will NOT be able to reprogram the target using this "
               "tool if you continue! Do you want to continue?")
    caveat = (" If the target then holds a valid image, you will NOT be "
              "able to reprogram it using this tool! Do you want to "
              "continue?")
    lost = []
    bl_lost = False
    if erased:
        start = device.BLANK_WORD
    elif ranges_overlap([(address, len(image))], ccfg_addr, ccfg_len):
        # Without an erase the write is ANDed with CCFG on the chip
        try:
            chip_ccfg = device.read_ccfg()
        except (CmdException, serial.SerialException) as err:
            # The link may be out of step after a failed read, so stop
            raise CmdException("Could not read CCFG from the target to "
                               "check this write without erase (%s). Erase "
                               "the CCFG page first with -e, -W, or an -E "
                               "range that covers it, or pass "
                               "--skip-ccfg-check." % err)
        bl_offset = device.bootloader_address - ccfg_addr
        start = chip_ccfg[bl_offset:bl_offset + 4]
        lost = device.ccfg_bits_lost(image, address, chip_ccfg)
        # When the image sets an enabled word, its pin and level bytes
        # matter even where they are 0xFF, so check every byte of the word.
        # A blank image word leaves the chip word as it is, and a disabled
        # one is handled by the lockout check.
        image_word = device.bootloader_word_after_write(
            image, address, device.BLANK_WORD)
        bl_lost = (device.image_covers_bootloader_word(image, address)
                   and image_word != 0xFFFFFFFF
                   and device._bootloader_enabled(image_word)
                   and any(start[i] & value != value for i, value in
                           device.image_bootloader_bytes(image,
                                                         address).items()))
    else:
        start = None  # CCFG stays as it is
    # The boot loader word has its own checks: bl_lost and the lockout one
    bl_word = range(device.bootloader_address, device.bootloader_address + 4)
    lost = [a for a in lost if a not in bl_word]
    if disabling:
        # -D replaces the boot loader word and asks about it itself
        bl_lost = False
    if (not disabling and start is not None
            and device.image_disables_bootloader(image, address, start)):
        image_word = device.bootloader_word_after_write(
            image, address, device.BLANK_WORD)
        if (device.image_covers_bootloader_word(image, address)
                and not device._bootloader_enabled(image_word)):
            return ("The boot loader backdoor is not enabled in the "
                    "firmware you are about to write to the target." + lockout)
        if bl_lost:
            return ("The boot loader word on the target has bits cleared "
                    "that a write without erase cannot set, so the boot "
                    "loader backdoor will not be enabled." + erase_hint
                    + lockout)
        if not erased and device._bootloader_enabled(
                struct.unpack('<L', bytes(start))[0]):
            return ("The boot loader configuration in the firmware you are "
                    "about to write disables the backdoor that is enabled "
                    "on the target." + lockout)
        # The image does not fully set the word. Whether the ROM boot loader
        # still runs then depends on the rest of CCFG, which is not checked.
        if erased:
            return ("This erase clears the boot loader configuration in "
                    "CCFG and the firmware you are about to write does not "
                    "fully restore it." + caveat)
        return ("The boot loader backdoor is not enabled on the target and "
                "the firmware you are about to write does not enable it."
                + caveat)
    if bl_lost:
        return ("The boot loader word on the target has bits cleared that "
                "a write without erase cannot set, so the backdoor pin or "
                "level will differ from the firmware and you may not be able "
                "to enter the boot loader." + erase_hint
                + " Do you want to continue?")
    if lost:
        return ("CCFG on the target has bits cleared at %s that a write "
                "without erase cannot set, so CCFG will not match the "
                "firmware." % ", ".join(
                    "0x%08X" % a for a in sorted(set(a & ~3 for a in lost)))
                + erase_hint + " Do you want to continue?")
    if not erased:
        return None
    has_ccfg = device.image_has_ccfg(image, address)
    if has_ccfg and device.image_covers_ccfg(image, address):
        return None
    if has_ccfg:
        return ("This erase includes the CCFG area and the image only "
                "restores part of it. The firmware may not boot until the "
                "rest of CCFG is written. Do you want to continue?")
    if explicit:
        return None  # an app-only image after an erase that was asked for
    return ("The -W erase includes the CCFG area and the image does not "
            "restore it. The firmware will not boot until CCFG is written. "
            "Do you want to continue?")


def guessed_size_note(device):
    """Describe a guessed flash size for messages."""
    code = ("unknown flash size code %d" % device.size_code
            if device.size_code is not None else "unknown flash size")
    return "%s, assuming %d KB" % (code, device.size >> 10)


def check_erase_range(device, start, length):
    """Reject an erase range that is empty or not entirely inside flash."""
    if length <= 0:
        raise ValueError("Erase range end must be after its start")
    if start < device.flash_start_addr or start + length > device.flash_end:
        message = ("Erase range 0x%08X-0x%08X is outside flash "
                   "(0x%08X-0x%08X)" % (start, start + length,
                                        device.flash_start_addr,
                                        device.flash_end))
        if not device.size_known:
            message += " (%s)" % guessed_size_note(device)
        raise ValueError(message)


def check_guessed_flash_end(device, address, length):
    """Refuse a write that reaches past a guessed end of flash.

    Flash there cannot be erased, because erases stop at the guessed end,
    so the write would be ANDed with old data. --force does not apply.
    """
    if (length <= 0 or device.size_known
            or device.max_flash_size is None):
        return
    max_end = device.flash_start_addr + device.max_flash_size
    if address < max_end and address + length > device.flash_end:
        raise CmdException("Write 0x%08X-0x%08X reaches past the end of "
                           "flash at 0x%08X, which is a guess (%s), so that "
                           "flash cannot be erased first."
                           % (address, address + length, device.flash_end,
                              guessed_size_note(device)))


def check_write(device, address, length, force=False, erase_write=False):
    """Run every check on a write before anything is sent.

    Alignment and the guessed end of flash come first, so the later errors
    do not suggest a --force that cannot help.
    """
    if address % 4:
        raise CmdException("Write address 0x%08X is not 4-byte aligned."
                           % address)
    check_guessed_flash_end(device, address, length)
    check_write_target(device, address, force, erase_write)
    check_flash_bounds(device, address, length, "Write", force)


def check_write_target(device, address, force=False, erase_write=False):
    """Reject write addresses the bootloader cannot program as flash.

    Part of check_write, which also checks alignment and length.
    """
    if not device.flash_start_addr <= address < device.flash_end:
        message = ("Write address 0x%08X is outside flash (0x%08X-0x%08X)."
                   % (address, device.flash_start_addr, device.flash_end))
        if erase_write:
            raise CmdException(message + " Erase-write only works on flash.")
        if not force:
            raise CmdException(message + " Use --force to continue anyway.")
        mdebug(5, "Warning: " + message)
    if erase_write and address % device.page_size:
        message = ("Erase-write address 0x%08X is not aligned to a %d byte "
                   "page, so the data before it in that page would be erased."
                   % (address, device.page_size))
        if not force:
            raise CmdException(message + " Use --force to continue anyway.")
        mdebug(5, "Warning: " + message)


def check_flash_bounds(device, address, length, action, force=False):
    """Reject an operation that starts in flash and runs past its end.

    Addresses outside flash (e.g. RAM) are left to the bootloader.
    """
    if (device.flash_start_addr <= address < device.flash_end
            and address + length > device.flash_end):
        message = ("%s of %d bytes at 0x%08X runs past the end of flash "
                   "(0x%08X)." % (action, length, address, device.flash_end))
        if not force:
            raise CmdException(message + " Use --force to continue anyway.")
        mdebug(5, "Warning: " + message)


def get_read_length(device, address, requested, force=False):
    """Return the 4-byte aligned read length, defaulting to the end of flash."""
    if requested is not None:
        length = requested
    elif device.flash_start_addr <= address < device.flash_end:
        # Whole words only, so the default never crosses the end of flash
        length = (device.flash_end - address) & ~0x03
        if not length:
            raise CmdException("Less than one 4-byte word left to read at "
                               "0x%08X." % address)
        return length
    else:
        length = DEFAULT_READ_LEN

    length = (length + 3) & ~0x03
    check_flash_bounds(device, address, length, "Read", force)
    return length


def check_output_path(path):
    """Fail before any device access if the read output cannot be saved."""
    if os.path.isdir(path):
        raise Exception('Output path %s is a directory.' % path)
    if os.path.exists(path):
        if not os.access(path, os.W_OK):
            raise Exception('Output file %s is not writable.' % path)
        return
    # A dangling symlink is created at its target
    probe = os.path.realpath(path) if os.path.islink(path) else path
    try:
        with open(probe, 'xb'):
            pass
        os.unlink(probe)
    except OSError as err:
        raise Exception('Cannot write output file %s: %s' % (path, err))


def reset_target(cmd):
    """Leave the bootloader without hiding the error being reported."""
    try:
        cmd.cmdReset()
    except (Exception, KeyboardInterrupt) as err:
        reason = 'interrupted' if isinstance(err, KeyboardInterrupt) else err
        mdebug(5, "Warning: could not reset target: %s" % reason)


def _is_own_checkout(root):
    """Return True if root is this project's repo, not one vendoring it."""
    if not os.path.exists(os.path.join(root, '.git')):
        return False
    try:
        with open(os.path.join(root, 'pyproject.toml'), encoding='utf-8',
                  errors='replace') as f:
            return any(line.strip() == 'name = "cc2538-bsl"' for line in f)
    except OSError:
        return False


def version():
    # Use the installed package's version, set from git tags at build time.
    try:
        from importlib.metadata import version as package_version
        return package_version('cc2538-bsl')
    except Exception:
        pass

    # Not installed: use this checkout's tags, never those of the current dir.
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _is_own_checkout(root):
        try:
            output = subprocess.check_output(
                ['git', '-C', root, 'describe', '--tags', '--match', '[0-9]*'],
                stderr=subprocess.DEVNULL, timeout=5)
            return output.decode('utf-8').strip()
        except Exception:
            pass

    return __version__


class _VersionAction(argparse.Action):
    """Print the version, resolving it only when --version is used."""

    def __call__(self, parser, namespace, values, option_string=None):
        sys.stdout.write('%s %s\n' % (parser.prog, version()))
        parser.exit()


def cli_setup():
    parser = argparse.ArgumentParser()

    parser.add_argument('-q', action='store_true', help='Quiet')
    parser.add_argument('-V', action='store_true', help='Verbose')
    parser.add_argument('-f', '--force', action='store_true', help='Force operation(s) without asking any questions')
    parser.add_argument('-e', '--erase', action='store_true', help='Mass erase')
    parser.add_argument('-E', '--erase-page', help='Receives an absolute address(a) range or page(p) range, default is address(a) eg: -E p,1,4, -E a,0x00001000,0x00003000 (CC2538 flash starts at 0x00200000). Addresses are decimal unless prefixed with 0x')
    parser.add_argument('-w', '--write', action='store_true', help='Write')
    parser.add_argument('-W', '--erase-write', action='store_true', help='Write after erasing the pages the image covers (avoids a mass erase). Whole pages are erased, so other data in the last page is lost. A start address that is not page aligned requires --force.')
    parser.add_argument('-v', '--verify', action='store_true', help='Verify (CRC32 check)')
    parser.add_argument('-r', '--read', action='store_true', help='Read')
    parser.add_argument('-l', '--len', type=int, help='Length of read (default: to the end of flash)')
    parser.add_argument('-p', '--port', help='Serial port (default: first USB-like port in /dev)')
    parser.add_argument('-b', '--baud', type=int, help=f"Baud speed (default: {DEFAULT_BAUD})")
    parser.add_argument('-a', '--address', type=int, help='Target address')
    parser.add_argument('-i', '--ieee-address', help='Set the secondary 64 bit IEEE address')
    parser.add_argument('--bootloader-active-high', action='store_true', help='Use active high signals to enter bootloader')
    parser.add_argument('--bootloader-invert-lines', action='store_true', help='Inverts the use of RTS and DTR to enter bootloader')
    parser.add_argument('--bootloader-sonoff-usb', action='store_true', help='Toggles RTS and DTR in the correct pattern for Sonoff USB dongle')
    parser.add_argument('--skip-ccfg-check', action='store_true', help='Skip the CCFG, boot loader lockout and IEEE address checks and their questions (and the question for -e on a chip with a guessed flash size), but keep the other checks')
    parser.add_argument('-D', '--disable-bootloader', action='store_true', help='After finishing, disable the bootloader')
    parser.add_argument('--version', action=_VersionAction, nargs=0, help="show program's version number and exit")
    parser.add_argument('file', nargs='?', default=None, help='Firmware file to write or verify, or output file for read')

    args = parser.parse_args()
    if args.port == '':
        parser.error('the serial port given with -p is empty')
    if args.file is None and (args.write or args.erase_write or args.verify
                              or args.read):
        parser.error('a file is required with -w, -W, -v or -r')
    # Connecting pulses the reset lines, so never do it on an auto-selected
    # port unless an operation was requested.
    if not args.port and not (
            args.erase or args.erase_page is not None or args.write or args.erase_write
            or args.verify or args.read or args.ieee_address is not None
            or args.disable_bootloader):
        parser.error('no operation requested; use -p PORT to only identify '
                     'the target')

    return args

def main_cli():
    global QUIET

    args = cli_setup()

    force_speed = False

    if args.baud:
        force_speed = True
    else:
        args.baud = DEFAULT_BAUD

    if args.V:
        QUIET = 10
    elif args.q:
        QUIET = 0

    # Reset on failure only while flash is unchanged. After a failed erase or
    # write the target stays in its bootloader so the operation can be retried.
    reset_on_failure = False

    try:
        if (args.write and args.read) or (args.erase_write and args.read):
            if not (args.force or
                    query_yes_no("You are reading and writing to the same "
                                 "file. This will overwrite your input file. "
                                 "Do you want to continue?", "no")):
                raise Exception('Aborted by user.')
        if ((args.erase or args.erase_page) and args.read
                and not (args.write or args.erase_write)):
            if not (args.force or
                    query_yes_no("You are about to erase your target before "
                                 "reading. Do you want to continue?", "no")):
                raise Exception('Aborted by user.')

        if (args.read and not (args.write  or args.erase_write)
            and args.verify):
            raise Exception('Verify after read not implemented.')

        if args.len is not None and args.len <= 0:
            raise Exception('Length must be positive but %d was provided'
                            % (args.len,))

        # Validate the firmware before touching the device
        firmware = None
        if args.write or args.erase_write or args.verify:
            mdebug(5, "Reading data from %s" % args.file)
            firmware = FirmwareFile(args.file)
            if not firmware.bytes:
                raise Exception('Firmware file %s is empty.' % args.file)
            if (args.write or args.erase_write) and len(firmware.bytes) % 4:
                raise Exception('Firmware size %d is not a multiple of 4.'
                                % len(firmware.bytes))

        ieee_addr = None
        if args.ieee_address is not None:
            ieee_addr = parse_ieee_address(args.ieee_address)

        if args.read:
            check_output_path(args.file)

        # Try and find the port automatically
        if not args.port:
            ports = []

            # Get a list of all USB-like names in /dev
            for name in ['ttyACM',
                         'tty.usbserial',
                         'ttyUSB',
                         'tty.usbmodem',
                         'tty.SLAB_USBtoUART']:
                ports.extend(glob.glob('/dev/%s*' % name))

            ports = sorted(ports)

            if ports:
                # Found something - take it
                args.port = ports[0]
            else:
                raise Exception('No serial port found.')

        cmd = CommandInterface()
        cmd.open(args.port, args.baud)
        cmd.invoke_bootloader(args.bootloader_active_high,
                              args.bootloader_invert_lines,
                              args.bootloader_sonoff_usb)
        mdebug(5, "Opening port %(port)s, baud %(baud)d"
               % {'port': args.port, 'baud': args.baud})

        mdebug(5, "Connecting to target...")

        if not cmd.sendSynch():
            raise CmdException("Can't connect to target. Ensure boot loader "
                               "is started. (no answer on synch sequence)")
        reset_on_failure = True

        # if (cmd.cmdPing() != 1):
        #     raise CmdException("Can't connect to target. Ensure boot loader "
        #                        "is started. (no answer on ping command)")

        chip_id = cmd.cmdGetChipId()
        chip_id_str = CHIP_ID_STRS.get(chip_id, None)

        if chip_id_str is None:
            mdebug(10, '    Unrecognized chip ID. Trying CC13xx/CC26xx')
            device = CC26xx(cmd)
        else:
            mdebug(10, "    Target id 0x%x, %s" % (chip_id, chip_id_str))
            device = CC2538(cmd)

        size_warning = device.size_warning()
        if size_warning:
            # Printed even with -q: reads without -l stop at this size
            print(size_warning + " Writes that reach past it are refused.",
                  file=sys.stderr)

        # Choose a good default address unless the user specified -a
        if args.address is None:
            args.address = device.flash_start_addr

        if not force_speed and device.has_cmd_set_xosc:
            if cmd.cmdSetXOsc():  # switch to external clock source
                cmd.close()
                args.baud = 1000000
                cmd.open(args.port, args.baud)
                mdebug(6, "Opening port %(port)s, baud %(baud)d"
                       % {'port': args.port, 'baud': args.baud})
                mdebug(6, "Reconnecting to target at higher speed...")
                if (cmd.sendSynch() != 1):
                    raise CmdException("Can't connect to target after clock "
                                       "source switch. (Check external "
                                       "crystal)")
            else:
                raise CmdException("Can't switch target to external clock "
                                   "source. (Try forcing speed)")

        # Check sizes before the first destructive operation
        erase_ranges = []
        if args.erase:
            erase_ranges.append((device.flash_start_addr, device.size))
        if args.erase_page is not None:
            erase_range = parse_page_address_range(device, args.erase_page)
            check_erase_range(device, erase_range[0], erase_range[1])
            erase_ranges.append((erase_range[0], erase_range[1]))
        if args.write or args.erase_write:
            check_write(device, args.address, len(firmware.bytes),
                        args.force, args.erase_write)
        write_erase = None
        if args.erase_write:
            # Cover every page the image touches
            erase_start = device.page_align_down(args.address)
            erase_end = min(device.page_align_up(args.address +
                                                 len(firmware.bytes)),
                            device.flash_end)
            write_erase = (erase_start, erase_end - erase_start)
        check_ccfg = not (args.force or args.skip_ccfg_check)
        if check_ccfg and not device.ccfg_known:
            question = guessed_ccfg_question(device, args, ieee_addr)
            if question and not query_yes_no(question, "no", SKIP_CCFG_HINT):
                raise Exception('Aborted by user.')
        if (args.write or args.erase_write) and check_ccfg \
                and device.ccfg_known:
            # Only asks questions, and may read the target, so skip it
            # entirely with --force or --skip-ccfg-check
            question = ccfg_write_question(device, firmware.bytes,
                                           args.address, erase_ranges,
                                           write_erase,
                                           args.disable_bootloader)
            if question and not query_yes_no(question, "no", SKIP_CCFG_HINT):
                raise Exception('Aborted by user.')
        if ieee_addr is not None and check_ccfg and device.ccfg_known:
            all_erases = erase_ranges + ([write_erase] if write_erase else [])
            writing = args.write or args.erase_write
            question = ieee_write_question(
                device, struct.pack('<Q', ieee_addr),
                firmware.bytes if writing else None,
                args.address, all_erases)
            if question and not query_yes_no(question, "no", SKIP_CCFG_HINT):
                raise Exception('Aborted by user.')
        if args.disable_bootloader:
            device.confirm_disable_bootloader(args.force)
        if args.read:
            read_length = get_read_length(device, args.address, args.len,
                                          args.force)

        if args.erase:
            reset_on_failure = False
            mdebug(5, "    Performing mass erase")
            if device.erase():
                mdebug(5, "    Erase done")
            else:
                raise CmdException("Erase failed")

        if args.erase_page is not None:
            reset_on_failure = False
            mdebug(5, "    Performing partial erase")
            if device.eraseRange(erase_range[0], erase_range[1]):
                mdebug(5, "    Partial erase done                  ")
            else:
                raise CmdException("Partial erase failed")

        if args.write:
            reset_on_failure = False
            if cmd.writeMemory(args.address, firmware.bytes, True):
                mdebug(5, "    Write done                                ")
            else:
                raise CmdException("Write failed                       ")

        if args.erase_write:
            reset_on_failure = False
            if device.eraseRange(erase_start, erase_end - erase_start):
                mdebug(5, "    Erase before write done                 ")
            else:
                raise CmdException("Erase before write failed")
            if cmd.writeMemory(args.address, firmware.bytes, True):
                mdebug(5, "    Write done                              ")
            else:
                raise CmdException("Write failed                       ")

        if args.verify:
            mdebug(5, "Verifying by comparing CRC32 calculations.")

            crc_local = firmware.crc32()
            # CRC of target will change according to length input file
            crc_target = device.crc(args.address, len(firmware.bytes))

            if crc_target is None:
                raise CmdException("CRC read failed: could not verify flash")

            if crc_local == crc_target:
                mdebug(5, "    Verified (match: 0x%08x)" % crc_local)
            else:
                raise Exception("NO CRC32 match: Local = 0x%x, "
                                "Target = 0x%x" % (crc_local, crc_target))

        if ieee_addr is not None:
            reset_on_failure = False
            mdebug(5, "Setting IEEE address to %s"
                       % (':'.join(['%02x' % b
                                    for b in struct.pack('>Q', ieee_addr)])))
            ieee_addr_bytes = struct.pack('<Q', ieee_addr)

            if cmd.writeMemory(device.addr_ieee_address_secondary,
                               ieee_addr_bytes):
                mdebug(5, "    "
                          "Set address done                                ")
            else:
                raise CmdException("Set address failed                       ")

        if args.read:
            length = read_length

            mdebug(5, "Reading %s bytes starting at address 0x%x"
                   % (length, args.address))

            # Only touch the output file once the whole read has succeeded
            read_data = bytearray()
            for i in range(0, length >> 2):
                # reading 4 bytes at a time
                rdata = device.read_memory(args.address + (i * 4))
                mdebug(5, " 0x%x: 0x%02x%02x%02x%02x"
                       % (args.address + (i * 4), rdata[0], rdata[1],
                          rdata[2], rdata[3]), '\r')
                read_data += rdata
            with open(args.file, 'wb') as f:
                f.write(read_data)
            mdebug(5, "    Read done                                ")

        if args.disable_bootloader:
            reset_on_failure = False
            device.disable_bootloader(True)  # confirmed before any erase

        reset_on_failure = False
        cmd.cmdReset()

    except KeyboardInterrupt:
        if reset_on_failure:
            reset_target(cmd)
        exit('ERROR: Interrupted by user.')
    except Exception as err:
        if QUIET >= 10:
            traceback.print_exc()
        if reset_on_failure:
            reset_target(cmd)
        exit('ERROR: %s' % str(err))

if __name__ == "__main__":
    main_cli()
