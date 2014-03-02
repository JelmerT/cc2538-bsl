#!/usr/bin/env python

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

# Serial boot loader over UART for CC2538
# Based on the info found in TI's swru333a.pdf (spma029.pdf)
#
# Bootloader only starts if no valid image is found or if boot loader
# backdoor is enabled.
# Make sure you don't lock yourself out!! (enable backdoor in your firmware)
# More info at https://github.com/JelmerT/cc2538-bsl

from __future__ import print_function

import sys, getopt
import serial
import glob
import time
import tempfile
import os
import subprocess
import struct
import binascii

try:
    from progressbar import *
    #TODO fix progressbar
    usepbar = 0
except:
    usepbar = 0

# Verbose level
QUIET = 5

def mdebug(level, message, attr='\n'):
    if QUIET >= level:
        print(message, end=attr, file=sys.stderr)

# Takes chip IDs (obtained via Get ID command) to human-readable names
CHIP_ID_STRS = {0xb964: 'CC2538'}

RETURN_CMD_STRS =  {0x40: 'Success',
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

class CommandInterface(object):
    def open(self, aport='/dev/tty.usbserial-000013FAB', abaudrate=500000) :
        self.sp = serial.Serial(
            port=aport,
            baudrate=abaudrate,     # baudrate
            bytesize=8,             # number of databits
            parity=serial.PARITY_NONE,
            stopbits=1,
            xonxoff=0,              # enable software flow control
            rtscts=0,               # disable RTS/CTS flow control
            timeout=0.5             # set a timeout value, None for waiting forever
        )

    def close(self):
        self.sp.close()


    def _wait_for_ack(self, info="", timeout=0):
        stop = time.time() + timeout
        got = None
        while not got:
            got = self.sp.read(2)
            if time.time() > stop:
                break

        if not got:
            mdebug(10, "No response to %s" % info)
            return 0

        # wait for ask
        ask = ord(got[1])

        if ask == 0xCC:
            # ACK
            return 1
        elif ask == 0x33:
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
        return (chr(byte0) + chr(byte1) + chr(byte2) + chr(byte3))

    def _decode_addr(self, byte0, byte1, byte2, byte3):
        return ((byte3 << 24) | (byte2 << 16) | (byte1 << 8) | (byte0 << 0))

    def _calc_checks(self,cmd,addr,size):
        return (chr((sum(bytearray(self._encode_addr(addr)))
                    +sum(bytearray(self._encode_addr(size)))
                    +cmd)
                    &0xFF))


    def sendAck(self):
        self.sp.write(chr(0x00))
        self.sp.write(chr(0xCC))
        return

    def sendNAck(self):
        self.sp.write(chr(0x00))
        self.sp.write(chr(0x33))
        return


    def receivePacket(self):
        # stop = time.time() + 5
        # got = None
        # while not got:
        got = self.sp.read(2)
        #     if time.time() > stop:
        #         break

        # if not got:
        #     raise CmdException("No response to %s" % info)

        size = ord(got[0]) #rcv size
        chks = ord(got[1]) #rcv checksum
        data = self.sp.read(size-2) # rcv data

        mdebug(10, "*** received %x bytes" % size)
        if chks == sum(ord(i) for i in data)&0xFF:
            self.sendAck()
            return data
        else:
            self.sendNAck()
            #TODO: retry receiving!
            raise CmdException("Received packet checksum error")
            return 0

    def sendSynch(self):
        cmd = 0x55

        mdebug(10, "*** sending synch sequence")
        self.sp.write(chr(cmd)) #send U
        self.sp.write(chr(cmd)) #send U
        return self._wait_for_ack("Synch (0x55 0x55)")

    def checkLastCmd(self):
        stat = self.cmdGetStatus()
        if not (stat):
            raise CmdException("No response from target on status request. (Did you disable the bootloader?)")

        if ord(stat) == COMMAND_RET_SUCCESS:
            mdebug(10, "Command Successful")
            return 1
        else:
            stat_str = RETURN_CMD_STRS.get(ord(stat), None)
            if stat_str is None:
                mdebug(0, 'Warning: unrecognized status returned 0x%x' % ord(stat))
            else:
                mdebug(0, "Target returned: 0x%x, %s" % (ord(stat), stat_str))
            return 0


    def cmdPing(self):
        cmd = 0x20
        lng = 3

        self.sp.write(chr(lng)) #send size
        self.sp.write(chr(cmd)) #send checksum
        self.sp.write(chr(cmd)) # send data

        mdebug(10, "*** Ping command (0x20)")
        if self._wait_for_ack("Ping (0x20)"):
            return self.checkLastCmd()

    def cmdReset(self):
        cmd = 0x25
        lng = 3

        self.sp.write(chr(lng)) #send size
        self.sp.write(chr(cmd)) #send checksum
        self.sp.write(chr(cmd)) # send data

        mdebug(10, "*** Reset command (0x25)")
        if self._wait_for_ack("Reset (0x25)"):
            return 1

    def cmdGetChipId(self):
        cmd = 0x28
        lng = 3

        self.sp.write(chr(lng)) #send size
        self.sp.write(chr(cmd)) #send checksum
        self.sp.write(chr(cmd)) # send data

        mdebug(10, "*** GetChipId command (0x28)")
        if self._wait_for_ack("Get ChipID (0x28)"):
            version = self.receivePacket() #4 byte answ, the 2 LSB hold chip ID
            if self.checkLastCmd():
                return version
            else:
                raise CmdException("GetChipID (0x28) failed")

    def cmdGetStatus(self):
        cmd = 0x23
        lng = 3

        self.sp.write(chr(lng)) #send size
        self.sp.write(chr(cmd)) #send checksum
        self.sp.write(chr(cmd)) # send data

        mdebug(10, "*** GetStatus command (0x23)")
        if self._wait_for_ack("Get Status (0x23)"):
            stat = self.receivePacket()
            return stat

    def cmdSetXOsc(self):
        cmd = 0x29
        lng = 3

        self.sp.write(chr(lng)) #send size
        self.sp.write(chr(cmd)) #send checksum
        self.sp.write(chr(cmd)) # send data

        mdebug(10, "*** SetXOsc command (0x29)")
        if self._wait_for_ack("SetXOsc (0x29)"):
            return 1
            # UART speed (needs) to be changed!

    def cmdRun(self, addr):
        cmd=0x22
        lng=7

        self.sp.write(chr(lng)) #send length
        self.sp.write(self._calc_checks(cmd,addr,0)) #send checksum
        self.sp.write(chr(cmd)) # send cmd
        self.sp.write(self._encode_addr(addr)) #send addr

        mdebug(10, "*** Run command(0x22)")
        return 1

    def cmdEraseMemory(self, addr, size):
        cmd=0x26
        lng=11

        self.sp.write(chr(lng)) #send length
        self.sp.write(self._calc_checks(cmd,addr,size)) #send checksum
        self.sp.write(chr(cmd)) # send cmd
        self.sp.write(self._encode_addr(addr)) #send addr
        self.sp.write(self._encode_addr(size)) #send size

        mdebug(10, "*** Erase command(0x26)")
        if self._wait_for_ack("Erase memory (0x26)",10):
            return self.checkLastCmd()

    def cmdCRC32(self, addr, size):
        cmd=0x27
        lng=11

        self.sp.write(chr(lng)) #send length
        self.sp.write(self._calc_checks(cmd,addr,size)) #send checksum
        self.sp.write(chr(cmd)) # send cmd
        self.sp.write(self._encode_addr(addr)) #send addr
        self.sp.write(self._encode_addr(size)) #send size

        mdebug(10, "*** CRC32 command(0x27)")
        if self._wait_for_ack("Get CRC32 (0x27)",1):
            crc=self.receivePacket()
            if self.checkLastCmd():
                return self._decode_addr(ord(crc[3]),ord(crc[2]),ord(crc[1]),ord(crc[0]))

    def cmdDownload(self, addr, size):
        cmd=0x21
        lng=11

        if (size % 4) != 0: #check for invalid data lengths
            raise Exception('Invalid data size: %i. Size must be a multiple of 4.' % size)

        self.sp.write(chr(lng)) #send length
        self.sp.write(self._calc_checks(cmd,addr,size)) #send checksum
        self.sp.write(chr(cmd)) # send cmd
        self.sp.write(self._encode_addr(addr)) #send addr
        self.sp.write(self._encode_addr(size)) #send size

        mdebug(10, "*** Download command (0x21)")
        if self._wait_for_ack("Download (0x21)",2):
            return self.checkLastCmd()

    def cmdSendData(self, data):
        cmd=0x24
        lng=len(data)+3
        #TODO: check total size of data!! max 252 bytes!

        self.sp.write(chr(lng)) #send size
        self.sp.write(chr((sum(bytearray(data))+cmd)&0xFF)) #send checksum
        self.sp.write(chr(cmd)) # send cmd
        self.sp.write(bytearray(data)) # send data

        mdebug(10, "*** Send Data (0x24)")
        if self._wait_for_ack("Send data (0x24)",10):
            return self.checkLastCmd()

    def cmdMemRead(self, addr): #untested
        cmd=0x2A
        lng=8

        self.sp.write(chr(lng)) #send length
        self.sp.write(self._calc_checks(cmd,addr,4)) #send checksum
        self.sp.write(chr(cmd)) # send cmd
        self.sp.write(self._encode_addr(addr)) #send addr
        self.sp.write(chr(4)) #send width, 4 bytes

        mdebug(10, "*** Mem Read (0x2A)")
        if self._wait_for_ack("Mem Read (0x2A)",1):
            data = self.receivePacket()
            if self.checkLastCmd():
                return data#self._decode_addr(ord(data[3]),ord(data[2]),ord(data[1]),ord(data[0]))

    def cmdMemWrite(self, addr, data, width): #untested
        #TODO: check width for 1 or 4 and data size
        cmd=0x2B
        lng=10

        self.sp.write(chr(lng)) #send length
        self.sp.write(self._calc_checks(cmd,addr,0)) #send checksum
        self.sp.write(chr(cmd)) # send cmd
        self.sp.write(self._encode_addr(addr)) #send addr
        self.sp.write(bytearray(data)) # send data
        self.sp.write(chr(width)) #send width, 4 bytes

        mdebug(10, "*** Mem write (0x2B)")
        if self._wait_for_ack("Mem Write (0x2B)",2):
            return checkLastCmd()


# Complex commands section

    def writeMemory(self, addr, data):
        lng = len(data)
        trsf_size = 248 #amount of data bytes transferred per packet (theory: max 252 + 3)
        empty_packet = [255]*trsf_size #empty packet (filled with 0xFF)

        # Boot loader enable check
        # TODO: implement check for all chip sizes & take into account partial firmware uploads
        if (lng == 524288): #check if file is for 512K model
            if not ((data[524247] & (1 << 4)) >> 4): #check the boot loader enable bit  (only for 512K model)
                if not query_yes_no("The boot loader backdoor is not enabled "\
                    "in the firmware you are about to write to the target. "\
                    "You will NOT be able to reprogram the target using this tool if you continue! "\
                    "Do you want to continue?","no"):
                    raise Exception('Aborted by user.')

        mdebug(5, "Writing %(lng)d bytes starting at address 0x%(addr)X" %
               { 'lng': lng, 'addr': addr})

        if usepbar:
            widgets = ['Writing: ', Percentage(),' ', ETA(), ' ', Bar()]
            pbar = ProgressBar(widgets=widgets, maxval=lng, term_width=79).start()

        offs = 0
        addr_set = 0

        while lng > trsf_size: #check if amount of remaining data is less then packet size
            if data[offs:offs+trsf_size] != empty_packet: #skip packets filled with 0xFF
                if addr_set != 1:
                    self.cmdDownload(addr,lng) #set starting address if not set
                    addr_set = 1
                if usepbar:
                    pbar.update(pbar.maxval-lng)
                else:
                    mdebug(5, " Write %(len)d bytes at 0x%(addr)X" % {'addr': addr, 'len': trsf_size}, '\r')
                    sys.stdout.flush()
                self.cmdSendData(data[offs:offs+trsf_size]) #send next data packet
            else:   #skipped packet, address needs to be set
                addr_set = 0

            offs = offs + trsf_size
            addr = addr + trsf_size
            lng = lng - trsf_size

        if usepbar:
            pbar.update(pbar.maxval-lng)
            pbar.finish()
        else:
            mdebug(5, "Write %(len)d bytes at 0x%(addr)X" % {'addr': addr, 'len': lng}, '\r')
        self.cmdDownload(addr,lng)
        return self.cmdSendData(data[offs:offs+lng]) #send last data packet

def query_yes_no(question, default="yes"):
    valid = {"yes":True,   "y":True,  "ye":True,
             "no":False,     "n":False}
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
        choice = raw_input().lower()
        if default is not None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' "\
                             "(or 'y' or 'n').\n")

def usage():
    print("""Usage: %s [-hqVewvr] [-l length] [-p port] [-b baud] [-a addr] [file.bin]
    -h          This help
    -q          Quiet
    -V          Verbose
    -e          Erase (full)
    -w          Write
    -v          Verify (CRC32 check)
    -r          Read
    -l length   Length of read
    -p port     Serial port (default: first USB-like port in /dev)
    -b baud     Baud speed (default: 500000)
    -a addr     Target address

    ./%s -e -w -v example/main.bin

    """ % (sys.argv[0],sys.argv[0]))

def read(filename):
    """Read the file to be programmed and turn it into a binary"""
    with open(filename, 'rb') as f:
        bytes = f.read()
        return [ord(x) for x in bytes]

if __name__ == "__main__":

    conf = {
            'port': 'auto',
            'baud': 500000,
            'force_speed' : 0,
            'address': 0x00200000,
            'erase': 0,
            'write': 0,
            'verify': 0,
            'read': 0,
            'len': 0x80000,
            'fname':'',
        }

# http://www.python.org/doc/2.5.2/lib/module-getopt.html

    try:
        opts, args = getopt.getopt(sys.argv[1:], "hqVewvrp:b:a:l:")
    except getopt.GetoptError as err:
        # print help information and exit:
        print(str(err)) # will print something like "option -a not recognized"
        usage()
        sys.exit(2)

    for o, a in opts:
        if o == '-V':
            QUIET = 10
        elif o == '-q':
            QUIET = 0
        elif o == '-h':
            usage()
            sys.exit(0)
        elif o == '-e':
            conf['erase'] = 1
        elif o == '-w':
            conf['write'] = 1
        elif o == '-v':
            conf['verify'] = 1
        elif o == '-r':
            conf['read'] = 1
        elif o == '-p':
            conf['port'] = a
        elif o == '-b':
            conf['baud'] = eval(a)
            conf['force_speed'] = 1
        elif o == '-a':
            conf['address'] = eval(a)
        elif o == '-l':
            conf['len'] = eval(a)
        else:
            assert False, "Unhandled option"

    try:
        #Sanity checks
        if conf['write'] or conf['read'] or conf['verify']:    #check for input/output file
            try:
                args[0]
            except:
                raise Exception('No file path given.')

        if conf['write'] and conf['read']:
            if not query_yes_no("You are reading and writing to the same file. This will overwrite your input file. "\
            "Do you want to continue?","no"):
                raise Exception('Aborted by user.')
        if conf['erase'] and conf['read'] and not conf['write']:
            if not query_yes_no("You are about to erase your target before reading. "\
            "Do you want to continue?","no"):
                raise Exception('Aborted by user.')

        if conf['read'] and not conf['write'] and conf['verify']:
            raise Exception('Verify after read not implemented.')

        # Try and find the port automatically
        if conf['port'] == 'auto':
            ports = []

            # Get a list of all USB-like names in /dev
            for name in ['tty.usbserial', 'ttyUSB', 'tty.usbmodem']:
                ports.extend(glob.glob('/dev/%s*' % name))

            ports = sorted(ports)

            if ports:
                # Found something - take it
                conf['port'] = ports[0]
            else:
                raise Exception('No serial port found.')

        cmd = CommandInterface()
        cmd.open(conf['port'], conf['baud'])
        mdebug(5, "Opening port %(port)s, baud %(baud)d" % {'port':conf['port'],
                                                      'baud':conf['baud']})
        if conf['write'] or conf['verify']:
            mdebug(5, "Reading data from %s" % args[0])
            data = read(args[0])

        mdebug(5, "Connecting to target...")

        if not cmd.sendSynch():
            raise CmdException("Can't connect to target. Ensure boot loader is started. (no answer on synch sequence)")

        if conf['force_speed'] != 1:
            if cmd.cmdSetXOsc(): #switch to external clock source
                cmd.close()
                conf['baud']=1000000
                cmd.open(conf['port'], conf['baud'])
                mdebug(6, "Opening port %(port)s, baud %(baud)d" % {'port':conf['port'],'baud':conf['baud']})
                mdebug(6, "Reconnecting to target at higher speed...")
                if (cmd.sendSynch()!=1):
                    raise CmdException("Can't connect to target after clock source switch. (Check external crystal)")
            else:
                raise CmdException("Can't switch target to external clock source. (Try forcing speed)")

        # if (cmd.cmdPing() != 1):
        #     raise CmdException("Can't connect to target. Ensure boot loader is started. (no answer on ping command)")

        chip_id = cmd.cmdGetChipId()
        assert len(chip_id) == 4, "Unreasonable chip id: %s" % repr(chip_id)
        chip_id_num = (ord(chip_id[2]) << 8) | ord(chip_id[3])
        chip_id_str = CHIP_ID_STRS.get(chip_id_num, None)

        if chip_id_str is None:
            mdebug(0, 'Warning: unrecognized chip ID 0x%x' % chip_id_num)
        else:
            mdebug(5, "    Target id 0x%x, %s" % (chip_id_num, chip_id_str))

        if conf['erase']:
            # we only do full erase for now (CC2538)
            address = 0x00200000 #flash start addr for cc2538
            size = 0x80000 #total flash size cc2538
            mdebug(5, "Erasing %s bytes starting at address 0x%x" % (size, address))

            if cmd.cmdEraseMemory(address, size):
                mdebug(5, "    Erase done")
            else:
                raise CmdException("Erase failed")

        if conf['write']:
            #TODO: check if boot loader back-door is open, need to read flash size first to get address
            if cmd.writeMemory(conf['address'], data):
                mdebug(5, "    Write done                                ")
            else:
                raise CmdException("Write failed                       ")

        if conf['verify']:
            mdebug(5,"Verifying by comparing CRC32 calculations.")

            crc_local = (binascii.crc32(bytearray(data))& 0xffffffff)
            crc_target = cmd.cmdCRC32(conf['address'],len(data)) #CRC of target will change according to length input file

            if crc_local == crc_target:
                mdebug(5, "    Verified (match: 0x%08x)" % crc_local)
            else:
                cmd.cmdReset()
                raise Exception("NO CRC32 match: Local = 0x%x, Target = 0x%x" % (crc_local,crc_target))

        if conf['read']:
            length = conf['len']
            if length < 4:  #reading 4 bytes at a time
                length = 4
            else:
                length = length + (length % 4)

            mdebug(5, "Reading %s bytes starting at address 0x%x" % (length, conf['address']))
            f = file(args[0], 'w').close() #delete previous file
            for i in range(0,(length/4)):
                rdata = cmd.cmdMemRead(conf['address']+(i*4)) #reading 4 bytes at a time
                mdebug(5, " 0x%x: 0x%02x%02x%02x%02x" % (conf['address']+(i*4), ord(rdata[3]), ord(rdata[2]), ord(rdata[1]), ord(rdata[0])), '\r')
                file(args[0], 'ab').write(''.join(reversed(rdata)))
            mdebug(5, "    Read done                                ")

        cmd.cmdReset()

    except Exception, err:
        print('ERROR: %s' % str(err))
