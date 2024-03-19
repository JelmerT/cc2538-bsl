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

from subprocess import Popen, PIPE

import sys
import glob
import math
import time
import os
import struct
import binascii
import traceback
import argparse

# version
__version__ = "2.1"

# Verbose level
QUIET = 5

# Default Baud
DEFAULT_BAUD = 500000

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


class FirmwareFile(object):
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

            if file_type == 'text/plain':
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
        if self._crc32 == None:
            self._crc32 = binascii.crc32(bytearray(self.bytes)) & 0xffffffff

        return self._crc32


class CommandInterface(object):

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
            self.sp = serial.serial_for_url(aport, do_not_open=True, timeout=10, write_timeout=10)
        except AttributeError:
            self.sp = serial.Serial(port=None, timeout=10, write_timeout=10)
            self.sp.port = aport

        if ((os.name == 'nt' and isinstance(self.sp, serial.serialwin32.Serial)) or \
           (os.name == 'posix' and isinstance(self.sp, serial.serialposix.Serial))):
            self.sp.baudrate=abaudrate        # baudrate
            self.sp.bytesize=8                # number of databits
            self.sp.parity=serial.PARITY_NONE # parity
            self.sp.stopbits=1                # stop bits
            self.sp.xonxoff=0                 # s/w (XON/XOFF) flow control
            self.sp.rtscts=0                  # h/w (RTS/CTS) flow control
            self.sp.timeout=0.5               # set the timeout value

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

    def _wait_for_ack(self, info="", timeout=1):
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
        if type(data) == int:
            assert data < 256
            goal = 1
            written = self.sp.write(bytes([data]))
        elif type(data) == bytes or type(data) == bytearray:
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
            if type(data) == int:
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

        size = got[0]  # rcv size
        chks = got[1]  # rcv checksum
        data = bytearray(self._read(size - 2))  # rcv data

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
        return self._wait_for_ack("Synch (0x55 0x55)", 2)

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
            if stat_str == None:
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
                assert len(version) == 4, ("Unreasonable chip "
                                           "id: %s" % repr(version))
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

    def writeMemory(self, addr, data):
        lng = len(data)
        # amount of data bytes transferred per packet (theory: max 252 + 3)
        trsf_size = 248
        empty_packet = bytearray((0xFF,) * trsf_size)

        # Boot loader enable check
        # TODO: implement check for all chip sizes & take into account partial
        # firmware uploads
        if (lng == 524288):  # check if file is for 512K model
            # check the boot loader enable bit  (only for 512K model)
            if not ((data[524247] & (1 << 4)) >> 4):
                if not (conf['force'] or
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
                    self.cmdDownload(addr, lng)
                    addr_set = 1
                mdebug(5, " Write %(len)d bytes at 0x%(addr)08X"
                       % {'addr': addr, 'len': trsf_size}, '\r')
                sys.stdout.flush()

                # send next data packet
                self.cmdSendData(data[offs:offs+trsf_size])
            else:   # skipped packet, address needs to be set
                addr_set = 0

            offs = offs + trsf_size
            addr = addr + trsf_size
            lng = lng - trsf_size

        mdebug(5, "Write %(len)d bytes at 0x%(addr)08X" % {'addr': addr,
                                                           'len': lng})
        self.cmdDownload(addr, lng)
        return self.cmdSendData(data[offs:offs+lng])  # send last data packet


class Chip(object):
    def __init__(self, command_interface):
        self.command_interface = command_interface

        # Some defaults. The child can override.
        self.flash_start_addr = 0x00000000
        self.has_cmd_set_xosc = False
        self.page_size = 2048


    def page_align_up(self, value):
        return int(math.ceil(value / self.page_size) * self.page_size)


    def page_align_down(self, value):
        return int(math.floor(value / self.page_size) * self.page_size)


    def page_to_addr(self, pages):
        addresses = []
        for page in pages:
            addresses.append(int(device.flash_start_addr) +
                             int(page)*self.page_size)
        return addresses

    def crc(self, address, size):
        return getattr(self.command_interface, self.crc_cmd)(address, size)

    def disable_bootloader(self):
        if not (conf['force'] or
                query_yes_no("Disabling the bootloader will prevent you from "
                             "using this script until you re-enable the "
                             "bootloader using JTAG. Do you want to continue?",
                             "no")):
            raise Exception('Aborted by user.')

        pattern = struct.pack('<L', self.bootloader_dis_val)

        if cmd.writeMemory(self.bootloader_address, pattern):
            mdebug(5, "    Set bootloader closed done                      ")
        else:
            raise CmdException("Set bootloader closed failed             ")


class CC2538(Chip):
    def __init__(self, command_interface):
        super(CC2538, self).__init__(command_interface)
        self.flash_start_addr = 0x00200000
        self.addr_ieee_address_secondary = 0x0027ffcc
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
        self.bootloader_address = self.flash_start_addr + self.size - ccfg_len

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

        mdebug(5, "CC2538 PG%d.%d: %dKB Flash, %dKB SRAM, CCFG at 0x%08X"
               % (pg_major, pg_minor, self.size >> 10, sram,
                  self.bootloader_address))
        mdebug(5, "Primary IEEE Address: %s"
               % (':'.join('%02X' % x for x in ieee_addr)))

    def erase(self):
        mdebug(5, "Erasing %s bytes starting at address 0x%08X"
               % (self.size, self.flash_start_addr))
        return self.command_interface.cmdEraseMemory(self.flash_start_addr,
                                                     self.size)

    def read_memory(self, addr):
        # CC2538's COMMAND_MEMORY_READ sends each 4-byte number in inverted
        # byte order compared to what's written on the device
        data = self.command_interface.cmdMemRead(addr)
        return bytearray([data[x] for x in range(3, -1, -1)])


class CC26xx(Chip):
    # Class constants
    MISC_CONF_1 = 0x500010A0
    PROTO_MASK_BLE = 0x01
    PROTO_MASK_IEEE = 0x04
    PROTO_MASK_BOTH = 0x05

    def __init__(self, command_interface):
        super(CC26xx, self).__init__(command_interface)
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

        if pg == 0:
            pg_str = "PG1.0"
        elif pg == 2 or pg == 3 or pg == 1:
            rev_minor = self.command_interface.cmdMemReadCC26xx(
                                                CC26xx.MISC_CONF_1)[0]
            if rev_minor == 0xFF:
                rev_minor = 0x00
            pg_str = "PG2.%d" % (rev_minor,)

        return "%s %s" % (chip_str, pg_str)

    def erase(self):
        mdebug(5, "Erasing all main bank flash sectors")
        return self.command_interface.cmdBankErase()

    def read_memory(self, addr):
        # CC26xx COMMAND_MEMORY_READ returns contents in the same order as
        # they are stored on the device
        return self.command_interface.cmdMemReadCC26xx(addr)


def query_yes_no(question, default="yes"):
    valid = {"yes": True,
             "y": True,
             "ye": True,
             "no": False,
             "n": False}
    if default == None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = input().lower()
        if default != None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' "
                             "(or 'y' or 'n').\n")


# Convert the entered IEEE address into an integer
def parse_ieee_address(inaddr):
    try:
        return int(inaddr, 16)
    except ValueError:
        # inaddr is not a hex string, look for other formats
        if ':' in inaddr:
            bytes = inaddr.split(':')
        elif '-' in inaddr:
            bytes = inaddr.split('-')
        if len(bytes) != 8:
            raise ValueError("Supplied IEEE address does not contain 8 bytes")
        addr = 0
        for i, b in zip(range(8), bytes):
            try:
                addr += int(b, 16) << (56-(i*8))
            except ValueError:
                raise ValueError("IEEE address contains invalid bytes")
        return addr


def _parse_range_values(device, values):
    if len(values) and len(values) < 3:
        page_addr_range = []
        try:
            for value in values:
                try:
                    if int(value) % int(device.page_size) != 0:
                        raise ValueError("Supplied addresses are not page_size: "
                                         "{} aligned".format(device.page_size))
                    page_addr_range.append(int(value))
                except ValueError:
                    if int(value, 16) % int(device.page_size) != 0:
                        raise ValueError("Supplied addresses are not page_size: "
                                         "{} aligned".format(device.page_size))
                    page_addr_range.append(int(value, 16))
            return page_addr_range
        except ValueError:
            raise ValueError("Supplied value is not a page or an address")
    else:
        raise ValueError("Supplied range is neither a page or address range")


def parse_page_address_range(device, pg_range):
    """Convert the address/page range into a start address and byte length"""
    values = pg_range.split(',')
    page_addr = []
    # check if first argument is character
    if values[0].isalpha():
        values[0].lower()
        if values[0] == 'p' or values[0] == 'page':
            if values[0] == 'p':
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


def version():
    # Get the version using "git describe".
    try:
        p = Popen(['git', 'describe', '--tags', '--match', '[0-9]*'],
                  stdout=PIPE, stderr=PIPE)
        p.stderr.close()
        line = p.stdout.readlines()[0]
        return line.decode('utf-8').strip()
    except:
        # We're not in a git repo, or git failed, use fixed version string.
        return __version__


def cli_setup():
    parser = argparse.ArgumentParser()

    parser.add_argument('-q', action='store_true', help='Quiet')
    parser.add_argument('-V', action='store_true', help='Verbose')
    parser.add_argument('-f', '--force', action='store_true', help='Force operation(s) without asking any questions')
    parser.add_argument('-e', '--erase', action='store_true', help='Mass erase')
    parser.add_argument('-E', '--erase-page', help='Receives an address(a) range or page(p) range, default is address(a) eg: -E a,0x00000000,0x00001000, -E p,1,4')
    parser.add_argument('-w', '--write', action='store_true', help='Write')
    parser.add_argument('-W', '--erase-write', action='store_true', help='Write after erasing section to write to (avoids a mass erase). Rounds up section to erase if not page aligned.')
    parser.add_argument('-v', '--verify', action='store_true', help='Verify (CRC32 check)')
    parser.add_argument('-r', '--read', action='store_true', help='Read')
    parser.add_argument('-l', '--len', type=int, default=0x80000, help='Length of read')
    parser.add_argument('-p', '--port', help='Serial port (default: first USB-like port in /dev)')
    parser.add_argument('-b', '--baud', type=int, help=f"Baud speed (default: {DEFAULT_BAUD})")
    parser.add_argument('-a', '--address', type=int, help='Target address')
    parser.add_argument('-i', '--ieee-address', help='Set the secondary 64 bit IEEE address')
    parser.add_argument('--bootloader-active-high', action='store_true', help='Use active high signals to enter bootloader')
    parser.add_argument('--bootloader-invert-lines', action='store_true', help='Inverts the use of RTS and DTR to enter bootloader')
    parser.add_argument('--bootloader-sonoff-usb', action='store_true', help='Toggles RTS and DTR in the correct pattern for Sonoff USB dongle')
    parser.add_argument('-D', '--disable-bootloader', action='store_true', help='After finishing, disable the bootloader')
    parser.add_argument('--version', action='version', version='%(prog)s ' + version())
    parser.add_argument('file')

    return parser.parse_args()

if __name__ == "__main__":
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

    try:
        if (args.write and args.read) or (args.erase_write and args.read):
            if not (args.force or
                    query_yes_no("You are reading and writing to the same "
                                 "file. This will overwrite your input file. "
                                 "Do you want to continue?", "no")):
                raise Exception('Aborted by user.')
        if ((args.erase and args.read) or (args.erase_page and args.read)
            and not (args.write or args.erase_write)):
            if not (args.force or
                    query_yes_no("You are about to erase your target before "
                                 "reading. Do you want to continue?", "no")):
                raise Exception('Aborted by user.')

        if (args.read and not (args.write  or args.erase_write)
            and args.verify):
            raise Exception('Verify after read not implemented.')

        if args.len < 0:
            raise Exception('Length must be positive but %d was provided'
                            % (args.len,))

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
        if args.write or args.erase_write or args.verify:
            mdebug(5, "Reading data from %s" % args.file)
            firmware = FirmwareFile(args.file)

        mdebug(5, "Connecting to target...")

        if not cmd.sendSynch():
            raise CmdException("Can't connect to target. Ensure boot loader "
                               "is started. (no answer on synch sequence)")

        # if (cmd.cmdPing() != 1):
        #     raise CmdException("Can't connect to target. Ensure boot loader "
        #                        "is started. (no answer on ping command)")

        chip_id = cmd.cmdGetChipId()
        chip_id_str = CHIP_ID_STRS.get(chip_id, None)

        if chip_id_str == None:
            mdebug(10, '    Unrecognized chip ID. Trying CC13xx/CC26xx')
            device = CC26xx(cmd)
        else:
            mdebug(10, "    Target id 0x%x, %s" % (chip_id, chip_id_str))
            device = CC2538(cmd)

        # Choose a good default address unless the user specified -a
        if not args.address:
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

        if args.erase:
            mdebug(5, "    Performing mass erase")
            if device.erase():
                mdebug(5, "    Erase done")
            else:
                raise CmdException("Erase failed")

        if args.erase_page:
            erase_range = parse_page_address_range(device, args.erase_page)
            mdebug(5, "Erasing %d bytes at addres 0x%x"
                   % (erase_range[1], erase_range[0]))
            cmd.cmdEraseMemory(erase_range[0], erase_range[1])
            mdebug(5, "    Partial erase done                  ")

        if args.write:
            # TODO: check if boot loader back-door is open, need to read
            #       flash size first to get address
            if cmd.writeMemory(args.address, firmware.bytes):
                mdebug(5, "    Write done                                ")
            else:
                raise CmdException("Write failed                       ")

        if args.erase_write:
            # TODO: check if boot loader back-door is open, need to read
            #       flash size first to get address
            # Round up to ensure page alignment
            erase_len = device.page_align_up(len(firmware.bytes))
            erase_len = min(erase_len, device.size)
            if cmd.cmdEraseMemory(args.address, erase_len):
                mdebug(5, "    Erase before write done                 ")
            if cmd.writeMemory(args.address, firmware.bytes):
                mdebug(5, "    Write done                              ")
            else:
                raise CmdException("Write failed                       ")

        if args.verify:
            mdebug(5, "Verifying by comparing CRC32 calculations.")

            crc_local = firmware.crc32()
            # CRC of target will change according to length input file
            crc_target = device.crc(args.address, len(firmware.bytes))

            if crc_local == crc_target:
                mdebug(5, "    Verified (match: 0x%08x)" % crc_local)
            else:
                cmd.cmdReset()
                raise Exception("NO CRC32 match: Local = 0x%x, "
                                "Target = 0x%x" % (crc_local, crc_target))

        if args.ieee_address:
            ieee_addr = parse_ieee_address(args.ieee_address)
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
            length = args.len

            # Round up to a 4-byte boundary
            length = (length + 3) & ~0x03

            mdebug(5, "Reading %s bytes starting at address 0x%x"
                   % (length, args.address))
            with open(args.file, 'wb') as f:
                for i in range(0, length >> 2):
                    # reading 4 bytes at a time
                    rdata = device.read_memory(args.address + (i * 4))
                    mdebug(5, " 0x%x: 0x%02x%02x%02x%02x"
                           % (args.address + (i * 4), rdata[0], rdata[1],
                              rdata[2], rdata[3]), '\r')
                    f.write(rdata)
                f.close()
            mdebug(5, "    Read done                                ")

        if args.disable_bootloader:
            device.disable_bootloader()

        cmd.cmdReset()

    except Exception as err:
        if QUIET >= 10:
            traceback.print_exc()
        exit('ERROR: %s' % str(err))
