import base64,serial
from numpy.ma.core import mod
from base64 import b16encode
from macpath import join


class Line:
    
    def __init__(self, serial_port):
        self.serial_port = serial_port
        
    def close(self):
        self.con.close()
        
    def connect(self):
        con = serial.Serial(
                            port=self.serial_port,
                            baudrate=38400,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS
                            )
        return con

class Drive:
    def __init__(self, connection, drive_id='01'):
        self.id = drive_id
        self.token = '02'
        self.connection = connection
    
    def encode(self, mm):
        scaled = mm * 10000
        data = ['0','0','0','0','0','0','0','0']
        n=1
        char = hex(scaled)[-n]
        while char != 'x':
            data[n-1] = char
            n+=1
            char = hex(scaled)[-n]
        sorted_data = [data[1],data[0],data[3],data[2],data[5],data[4],data[7],data[6]]
        return ''.join(sorted_data).upper()        
    
    def get_status(self):
        dataString = "01" + self.id + "05020001000004"
        data = base64.b16decode(dataString)
        self.connection.write(data)
        i=30
        loop=True
        inputWord = ""
        while loop:
            i-=1
            inputByte = base64.b16encode(self.connection.read())
            inputWord += inputByte
            if i<0:
                loop=False
            if inputByte == '04':
                loop=False
        return inputWord
    
    def move_to_pos(self, pos, print_details=False):
        if self.token == '02':
            self.token ='01'
        else:
            self.token = '02'
    
        dataString = "01" + self.id + "09020002" + self.token + "02" + self.encode(pos) + "04"
        print 'TX = '+dataString + '(move to pos)'
        data = base64.b16decode(dataString)
        self.connection.write(data)
        i=30
        loop=True
        inputWord = ""
        while loop:
            i-=1
            inputByte = base64.b16encode(self.connection.read())
            inputWord += inputByte
            if i<0:
                loop=False
            if inputByte == '04':
                loop=False
        if print_details:
            self._get_response(inputWord)
        return inputWord
    
    def move_home(self):
        dataString = "01" + self.id + "0902000202020000000004"
        
        print 'TX = '+dataString + ' (move home)'
        data = base64.b16decode(dataString)
        self.connection.write(data)
        i=30
        loop=True
        inputWord = ""
        while loop:
            i-=1
            inputByte = base64.b16encode(self.connection.read())
            inputWord += inputByte
            if i<0:
                loop=False
            if inputByte == '04':
                loop=False
        return inputWord
    
    def _get_response(self, response):
        print 'RX = ' + response
        if response[0:2] == '01':
            print '01 Write Control Word'
        else:
            print response[0:2] + " Unknown header"
        print response[2:4] + ' Address'
        length = int(response[4:6],16)
        print response[4:6] + ' Data length'
        temp=0
        while temp < length:
            print response[6+2*temp:8+2*temp]
            temp+=1
        print response[6+2*temp:8+2*temp] + ' End telegram'

if __name__ == '__main__':
    ##Test code for module
    import time
    con = Line('COM11').connect()
    lin = Drive(con, '01')
    lin.move_home()
    print lin.move_to_pos(0, False)
    time.sleep(0.2)
    print lin.move_to_pos(10, False)
    time.sleep(0.2)
    print lin.move_to_pos(50, False)
    time.sleep(0.2)
    print lin.move_to_pos(100, False)
    time.sleep(0.2)
    print lin.move_to_pos(150, False)
    time.sleep(0.2)
    print lin.move_to_pos(100, False)
    time.sleep(0.2)
    print lin.move_to_pos(50, False)
    
    con.close()
    #print lin.encode(10)