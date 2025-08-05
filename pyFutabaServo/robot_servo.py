import serial
import struct
from time import sleep

class serial_converter(serial.Serial):
  def __init__(self, Port, Baudrate):
    super(serial_converter, self).__init__()
    self.port = Port
    self.baudrate = Baudrate
    self.parity = serial.PARITY_NONE
    self.stopbits = serial.STOPBITS_ONE
    self.timeout = 0.1
    self.connect()

    
  def connect(self):
    self.isBusy = False
    if self.isOpen == True:
      return True

    else:  
      try:
        self.open()
        return True
    
      except:
        return False

      
  def buffer_clear(self):
    self.flushInput()

    

class futaba_servo(object):
  def __init__(self, serial_conv, servo_id, zero_position = 0):
    self._serial_conv = serial_conv
    self._id = servo_id
    self._zero_pos = zero_position

  def is_open(self):
    return self._serial_conv.isOpen()

  def _send_command(self, Flag, Address, Length, Count, Data=[]):
    command = [0xFA, 0xAF, self._id]

    for tmp in [Flag, Address, Length, Count, Data]:
      if isinstance(tmp, int):
        command.append(tmp)
        
      elif isinstance(tmp, list):
        command = command + tmp

      else:
        raise ValueError()

    Sum = command[2]

    for tmp in command[3:]:
      Sum = Sum ^ tmp
      
    command.append(Sum)
    send_data = ""
    for tmp in command:
      send_data = send_data + struct.pack("B", tmp)

    self._serial_conv.write(send_data)
#    sleep(0.005)


  def set_compliance_param(self, CW_Margin, CCW_Margin, CW_Slope, CCW_Slope, Punch):
    data = [CW_Margin, CCW_Margin, CW_Slope, CCW_Slope, Punch & 0x00FF, (Punch & 0xFF00) >> 8]

    while self._serial_conv.isBusy:
      sleep(0.001)

    self._serial_conv.isBusy = True
    self._serial_conv.buffer_clear()
    self._send_command(0x01, 0x18, 0x06, 0x01, data)
    ret_packet = self._serial_conv.read()
    self._serial_conv.isBusy = False
    
    if ret_packet == '\x07':
      return True
      #print  self._serial_conv.port + " ID: " + str(self._id), "OK"

    else:
      print "retry to send command:", self._serial_conv.port, " ID: ", self._id
      self.set_compliance_param(CW_Margin, CCW_Margin, CW_Slope, CCW_Slope, Punch)
    

  def move(self, Position, Time):
    pos = int(Position + self._zero_pos)
    tm = int(Time)
    data = [pos & 0x00FF, (pos & 0xFF00) >> 8 , tm & 0x00FF, (tm & 0xFF00) >> 8 ]

    while self._serial_conv.isBusy:
      sleep(0.001)

    self._serial_conv.isBusy = True
    self._serial_conv.buffer_clear()
    self._send_command(0x01, 0x1E, 0x04, 0x01, data)
    ret_packet = self._serial_conv.read()
    self._serial_conv.isBusy = False
    
    if ret_packet == '\x07':
      return True
      #print  self._serial_conv.port + " ID: " + str(self._id), "OK"

    else:
      print "retry to send command:", self._serial_conv.port, " ID: ", self._id
      self.move(Position, Time)

      
  def torque(self, Mode):
    """
    Set Servo's torque
    Mode = 0 : Off
         = 1 : On
         = 2 : Brake
    """

    data = Mode & 0x00FF
    while self._serial_conv.isBusy:
      sleep(0.001)

    self._serial_conv.isBusy = True
    self._serial_conv.buffer_clear()
    self._send_command(0x01, 0x24, 0x01, 0x01, data)

    ret_packet = self._serial_conv.read()
    self._serial_conv.isBusy = False
    
    if ret_packet == '\x07':
      return True
      #print  self._serial_conv.port + " ID: " + str(self._id), "OK"

    else:
      print "retry to send command:", self._serial_conv.port, " ID: ", self._id
      self.torque(Mode)      

  
  def get_status(self):
    while self._serial_conv.isBusy:
      sleep(0.001)

    self._serial_conv.isBusy = True
    
    self._serial_conv.buffer_clear()
    self._send_command(0x09, 0x00, 0x00, 0x01)
    readbuf = ""
    for x in range(26):
      char = self._serial_conv.read()
      if char == "":
        readbuf = "timeout"
        break
      readbuf = readbuf + char

    self._serial_conv.isBusy = False

    ##check header
    if readbuf[0] == "\xFD" and readbuf[1] == "\xDF":
      angle = struct.unpack('h', readbuf[7] + readbuf[8])[0]
      time = struct.unpack('h', readbuf[9] + readbuf[10])[0]
      speed = struct.unpack('h', readbuf[11] + readbuf[12])[0]
      load = struct.unpack('h', readbuf[13] + readbuf[14])[0]
      temperature = struct.unpack('h', readbuf[15] + readbuf[16])[0]
      voltage = struct.unpack('h', readbuf[17] + readbuf[18])[0]

      return {"Angle": angle - self._zero_pos,           
              "Time": time,
              "Speed": speed,
              "Load": load,
              "Temperature": temperature,
              "Voltage": voltage}
      
    else:
      print "retry to send command:", self._serial_conv.port, " ID: ", self._id
      return self.get_status()



class servo_cluster(object):
  def __init__(self, servo_list, activate_sequence = [], zero_position = []):
    self.servo_list = servo_list
    self._num_servo = len(servo_list)

    if zero_position == []:
      zero_position = [0 for i in range(self._num_servo)]

    self.set_zero_position(zero_position)
    
    if activate_sequence == []:
      activate_sequence = [range(self._num_servo)]
    
    self._activate_sequence = activate_sequence

  def is_open(self):
    for servo in self.servo_list:
      if servo.is_open() == False:
        return False

    return True

  def set_zero_position(self, angle_list):
    for i in range(self._num_servo):
      self.servo_list[i]._zero_pos = angle_list[i]

  def activate(self):
    for activate_list in self._activate_sequence:
      for i in activate_list:
        self.servo_list[i].torque(1)
        self.servo_list[i].move(0,200)

      sleep(2.0)
 
  def deactivate(self):
    self.torque(0)

  def torque(self, mode):
    if isinstance(mode, int):
      for servo in self.servo_list:
        servo.torque(mode)

    if isinstance(mode, list):
      for i in range(self._num_servo):
        self.servo_list[i].torque(mode[i])


  def set_compliance_param(self, servo_index, CW_Margin, CCW_Margin, CW_Slope, CCW_Slope, Punch):
    if isinstance(servo_index, int):
      self.servo_list[servo_index].set_compliance_param(CW_Margin, CCW_Margin, CW_Slope, CCW_Slope, Punch)

    if isinstance(servo_index, list):
      for i in servo_index:
        self.servo_list[i].set_compliance_param(CW_Margin, CCW_Margin, CW_Slope, CCW_Slope, Punch)

  def move(self, position_list, move_time = 20, servo_index = [], reverse = []):
    if servo_index == []:
      servo_index = range(self._num_servo)
    if reverse == []:
      reverse = [False for i in range(self._num_servo)]

    for i,j in enumerate(servo_index):
      angle = position_list[i]
      if angle is None:
        continue
      if reverse[i]:
        angle *= -1
      self.servo_list[j].move(angle, move_time)

  def get_status(self):
    angle = []
    time = []
    speed = []
    load = []
    temperature = []
    voltage = []
    for servo in self.servo_list:
      data = servo.get_status()
      angle.append(data["Angle"])
      time.append(data["Time"])
      speed.append(data["Speed"])
      load.append(data["Load"])
      temperature.append(data["Temperature"])
      voltage.append(data["Voltage"])

    return {"Angle": angle,           
            "Time": time,
            "Speed": speed,
            "Load": load,
            "Temperature": temperature,
            "Voltage": voltage}

