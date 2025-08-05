import serial
import struct
from time import sleep

class Leptrino(serial.Serial):
  def __init__(self, Port = '/dev/ttyACM0', Baudrate = 460800):
    super(Leptrino, self).__init__()
    self.port = Port
    self.baudrate = Baudrate
    self.parity = serial.PARITY_NONE
    self.stopbits = serial.STOPBITS_ONE
    self.timeout = 0.1
   

  def connect(self):
    self.open()
    self.stop_sending_data()
    sleep(0.1)
    while (self.inWaiting() > 0):
      self.read(self.inWaiting())

    self.recieve_product_info()
    self.recieve_sensor_rated_value()
    
  def _send_command(self, data):
    DLE = 0x10
    STX = 0x02
    ETX = 0x03

    send_data = []
    for c in data:
      if c == DLE:
        send_data.append(DLE)
      send_data.append(c)
    
    CommandCharList = [DLE, STX] + send_data + [DLE, ETX]  # + BCC

    BCC = 0x00   #initialize BCC

    for tmp in data + [ETX]:
      BCC = BCC ^ tmp

    CommandCharList.append(BCC)

    commandStr = ""

    for tmp in CommandCharList:
      commandStr = commandStr + struct.pack("B", tmp)

    self.write(commandStr)

  def recieve_product_info(self):
    command = [0x04, 0xFF, 0x2A, 0x00]
    self._send_command(command)
    resp = self.read_data()
    
    self.product_type = ''
    for i in range(16):
      if resp[6+i] != '/x00':
        self.product_type = self.product_type + resp[6+i]
    
    self.serial_number = ''
    for i in range(8):
      self.serial_number = self.serial_number + resp[6+16+i]
    self.serial_number = int(self.serial_number)

    self.firm_version = ''
    for i in range(4):
      self.firm_version = self.firm_version + resp[6+16+8+i]
    self.firm_version = int(self.firm_version)

    self.output_rate = ''
    for i in range(6):
      self.output_rate = self.output_rate + resp[6+15+8+4+i]
    self.output_rate = int(self.output_rate)

  def recieve_sensor_rated_value(self):
    command = [0x04, 0xFF, 0x2B, 0x00]
    self._send_command(command)
    resp = self.read_data()
    self.Fx_max = struct.unpack('f', resp[6]  + resp[7]  + resp[8]  + resp[9] )[0]
    self.Fy_max = struct.unpack('f', resp[10] + resp[11] + resp[12] + resp[13])[0]
    self.Fz_max = struct.unpack('f', resp[14] + resp[15] + resp[16] + resp[17])[0]
    self.Mx_max = struct.unpack('f', resp[18] + resp[19] + resp[20] + resp[21])[0]
    self.My_max = struct.unpack('f', resp[22] + resp[23] + resp[24] + resp[25])[0]
    self.Mz_max = struct.unpack('f', resp[26] + resp[27] + resp[28] + resp[29])[0]

  def hand_shake(self):
    command = [0x04, 0xFF, 0x30, 0x00]
    self._send_command(command)
    resp = self.read_data()

    Fx = struct.unpack('h', resp[6]  + resp[7] )[0] * self.Fx_max /10000.0
    Fy = struct.unpack('h', resp[8]  + resp[9] )[0] * self.Fy_max /10000.0
    Fz = struct.unpack('h', resp[10] + resp[11])[0] * self.Fz_max /10000.0
    Mx = struct.unpack('h', resp[12] + resp[13])[0] * self.Mx_max /10000.0
    My = struct.unpack('h', resp[14] + resp[15])[0] * self.My_max /10000.0
    Mz = struct.unpack('h', resp[16] + resp[17])[0] * self.Mz_max /10000.0

    return Fx, Fy, Fz, Mx, My, Mz

  def start_sending_data(self):
    command = [0x04, 0xFF, 0x32, 0x00]
    self._send_command(command)
    
  def stop_sending_data(self):
    command = [0x04, 0xFF, 0x33, 0x00]
    self._send_command(command)

  def read_data(self):
    readbuf = []
    flag = True

    while flag:
      char = self.read()
      readbuf.append(char)
      if char == '\x10':
        char = self.read()

        if char == '\x10':
          continue

        elif char == '\x03':
          readbuf.append(char)
          flag = False
          readbuf.append(self.read())

        else:
          readbuf.append(char)

    return readbuf
