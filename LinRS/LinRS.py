from __future__ import print_function
import base64
import serial
from ctypes import *
import sys
import pprint
import struct

class InvalidResponseException(Exception):
    pass

class LinRSRequest(object):
    def __init__(self, **kwargs):
        self.header = '01'
        self.main_id = kwargs.pop('main_id', '00')
        self.sub_id = kwargs.pop('sub_id', '00')
        self.drive_id = kwargs.pop('macid', '01')
        self.data = kwargs.pop('data', '0000')
        self.end = '04'

    def __str__(self):
        print(self.__repr__())
        return base64.b16decode(self.__repr__(), True)

    def __repr__(self):
        data_len = (len(self.data) / 2) + 3
        return self.header + self.drive_id + '{0:02x}'.format(data_len) + '02' + self.sub_id + self.main_id + self.data + self.end

class LinRSResponse(object):
    def __init__(self, bytes=()):

        if int(bytes[0].encode('hex'), 16) != 0x02 or \
           int(bytes[-1].encode('hex'), 16) != 0x04:
            raise InvalidResponseException("Fixed ID start data or end telegram missing (%s, %s)" % (bytes[0].encode('hex'), bytes[-1].encode('hex')))

        self.communication_state = base64.b16encode(bytes[3])
        self.status_word = base64.b16encode(bytes[5]) + base64.b16encode(bytes[4])
        self.state_var   = base64.b16encode(bytes[7]) + base64.b16encode(bytes[6])
        self.actual_position = struct.unpack("<I", struct.pack(">I", int(bytes[8:12].encode('hex'), 16))) # + (bytes[9] << 8) + (bytes[10] << 16) + (bytes[11] << 24))

    def __str__(self):
        pp = pprint.PrettyPrinter(indent=4)
        return pp.pformat(self.__dict__)

    def __repr__(self):
        return("LinRSResponse (state: %s)" % self.communication_state)

class ControlWordBits(BigEndianStructure):
    _pack_   = 1
    _fields_ = [
                  ('phase_search',      c_uint16, 1),
                  ('reserved1',         c_uint16, 1),
                  ('goto_initial',      c_uint16, 1),
                  ('clearance_check',   c_uint16, 1),
                  ('home',              c_uint16, 1),
                  ('special_mode',      c_uint16, 1),
                  ('jog_minus',         c_uint16, 1),
                  ('jog_plus',          c_uint16, 1),
                  ('error_acknowledge', c_uint16, 1),
                  ('goto_position',     c_uint16, 1),
                  ('not_freeze',        c_uint16, 1),
                  ('not_abort',         c_uint16, 1),
                  ('enable_operation',  c_uint16, 1),
                  ('not_quick_stop',    c_uint16, 1),
                  ('voltage_enable',    c_uint16, 1),
                  ('switch_on',         c_uint16, 1)
               ]

class ControlWord(Union):
    _anonymous_ = ('bit',)
    _fields_ = [
                ('bit',     ControlWordBits ),
                ('asBytes', c_uint16        )
               ]

class Line(object):
    def __init__(self, serial_port):
        self.serial_port = serial_port

    def close(self):
        self.con.close()

    def connect(self):
        self.con = serial.Serial(port=self.serial_port,
                                 baudrate=38400,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 bytesize=serial.EIGHTBITS
                                 )
        return self.con

class Drive(object):

    def __init__(self, connection, drive_id='01'):
        self.id = drive_id
        self.token = '02'
        self.connection = connection

    def writeControlWord(self, cw):
        message = LinRSRequest(main_id='01', sub_id='00')
        message.data = '{0:04x}'.format(cw.asBytes)
        self.connection.write(str(message))
        self.control_word = cw
        return self._read_response()

    def waitForState(self, state, timeout=15):
        count = 0
        while self.getStateVar() != state:
            count += 1
            time.sleep(0.1)
            if timeout > 0 and (count * 0.1) > timeout:
                raise Exception("Timed out waiting for state %s", state)

    def encode(self, mm):
        scaled = mm * 10000
        data = ['0', '0', '0', '0', '0', '0', '0', '0']
        n = 1
        char = hex(scaled)[-n]
        while char != 'x':
            data[n-1] = char
            n += 1
            char = hex(scaled)[-n]
        sorted_data = [data[i] for i in (1, 0, 3, 2, 5, 4, 7, 6)]
        return ''.join(sorted_data).upper()

    def responseRequest(self, subtype):
        message = LinRSRequest(main_id='00', sub_id=subtype)
        self.connection.write(str(message))
        return self._read_response()

    def getStateVar(self):
        return self.responseRequest('04').state_var

    def getStatusWord(self):
        return self.responseRequest('02')

    def getWarnWord(self):
        return self.responseRequest('03')

    def get_status(self):
        return self.responseRequest('01')

    def _read_response(self):
        (start, macid, size) = self.connection.read(3)

        start = int(start.encode('hex'), 16)
        macid = int(macid.encode('hex'), 16)
        size  = int(size.encode('hex'), 16)

        if start != 0x01:
            raise InvalidResponseException("Incorrect Fixed ID Telegram Start: %s" % base64.b16encode(start))

        if macid != int(self.id, 16):
            raise InvalidResponseException("Response from wrong controller (chaining unsupported)")

        return LinRSResponse(bytes=self.connection.read(size + 1))

    def home(self):
        try:
            self.control_word.bit.home = 1
        except:
            self.control_word = ControlWord(switch_on=1, voltage_enable=1, not_quick_stop=1, enable_operation=1, home=1)

        self.writeControlWord(self.control_word)
        self.waitForState('090F')
        self.control_word.bit.home = 0
        self.writeControlWord(self.control_word)

    def move_to_pos(self, pos, print_details=False):
        if self.token == '02':
            self.token = '01'
        else:
            self.token = '02'

        dataString = ("01" + self.id + "09020002" + self.token + "02" +
                      self.encode(pos) + "04")
        print('TX = '+dataString + '(move to pos)')
        data = base64.b16decode(dataString)
        self.connection.write(data)
        inputWord = self._read_response()

        if print_details:
            self._parse_response(inputWord)
        return inputWord

    def move_home(self):
        dataString = "01" + self.id + "0902000202020000000004"

        print('TX = '+dataString + ' (move home)')
        data = base64.b16decode(dataString)
        self.connection.write(data)
        return self._read_response()

    def _parse_response(self, response):
        print('RX = ' + response)
        if response[0:2] == '01':
            print('01 Write Control Word')
        else:
            print(response[0:2] + " Unknown header")
        print(response[2:4] + ' Address')
        length = int(response[4:6], 16)
        print(response[4:6] + ' Data length')
        temp = 0
        while temp < length:
            print(response[6+2*temp:8+2*temp])
            temp += 1
        print(response[6+2*temp:8+2*temp] + ' End telegram')

    def getState(self):
        state_var = self.getStateVar()
        main_state = state_var[0:2]
        sub_state = state_var[2:4]

        if main_state == '08':
            print(sub_state)
            sub_int = int(sub_state, 16)
            if sub_int & 0x40:
                return 'C8'
            elif int(sub_state, 16) & 0x20:
                return 'A8'
            elif sub_state == '80':
                return '88'
            else:
                 return '08'

        return main_state

    def initialize(self):
        sequence = ('00', '01', '02', '05', '06', '08', '09', 'C8')
        index = 0
        while index < len(sequence) - 1:
            state = self.getState()
            index = sequence.index(state)
            print("Current state: %s" % self.states[state]['description'])
            try:
                print("Next state:    %s" % self.states[sequence[index+1]]['description'] )
            except:
                return
            (function, param) = self.states[state]['to'][sequence[index+1]]
            if param is not None:
                function(self, param)
            else:
                function(self)

    states = {
           '00': { 'description': 'Not Ready To Switch On',
                   'to':          { '01': ( writeControlWord,
                                            ControlWord(switch_on=0) ) }
                 },
           '01': { 'description': 'Switch On Disabled',
                   'to':          { '02': ( writeControlWord,
                                            ControlWord(switch_on=0, voltage_enable=1, not_quick_stop=1) ) }
                 },
           '02': { 'description': 'Ready To Switch On',
                   'to':          { '05': ( writeControlWord,
                                            ControlWord(switch_on=1, voltage_enable=1, not_quick_stop=1) ) }
                 },
           '03':   { 'description': 'Setup Error',
                   'to':          { '04': ( writeControlWord,
                                            ControlWord(switch_on=0) ) }
                 },
           '04':   { 'description': 'Error',
                   'to':          { '00': ( writeControlWord,
                                            ControlWord(error_acknowledge=1) ) }
                 },
           '05': { 'description': 'HW Tests',
                   'to':          { '06': ( waitForState,
                                            '0006' ) }
                 },
           '06': { 'description': 'Ready to Operate',
                   'to':          { '08': ( writeControlWord,
                                            ControlWord(switch_on=1, voltage_enable=1, not_quick_stop=1, enable_operation=1) ) }
                 },
           '08': { 'description': 'Operation enabled (not homed)',
                   'to':          { 'C8': ( home,
                                            None ),
                                    '09': ( home,
                                            None ) }
                 },
           # Operation Enabled substates are functionally similar to main
           # states. So they are re-mapped onto unused main-state values
           # here for code simplicity
           '88': { 'description': 'Operation enabled (Homed)',
                   'to':          { 'C8': ( waitForState,
                                            'C8' ) }
                 },
           'A8': { 'description': 'Operation enabled (Motion Active)',
                   'to':          { 'C8': ( waitForState,
                                            'C8' ) }
                 },
           'C8': { 'description': 'Operation enabled (In Target Position)' },
           '09': { 'description': 'Homing',
                   'to':          { 'C8': ( home,
                                            None ) }
                 },
           '10': { 'description': 'Clearance Check' }
    }

if __name__ == '__main__':

    cw = ControlWord()
    cw.bit.home = 1
    message = LinRSRequest(main_id='01', sub_id='00')
    message.data = '{0:04x}'.format(cw.asBytes)

    print(repr(message))

    ## Test code for module
    import time
    con = Line('/dev/ttyUSB0').connect()
    lin = Drive(con, '01')

    print (lin.getStateVar())

    lin.initialize()

    lin.move_home()
    print(lin.move_to_pos(0, False))
    time.sleep(0.2)
    print(lin.move_to_pos(10, False))
    time.sleep(0.2)
    print(lin.move_to_pos(50, False))
    time.sleep(0.2)
    print(lin.move_to_pos(100, False))
    time.sleep(0.2)
    print(lin.move_to_pos(150, False))
    time.sleep(0.2)
    print(lin.move_to_pos(100, False))
    time.sleep(0.2)
    print(lin.move_to_pos(50, False))

    con.close()
    # print(lin.encode(10))
