# pyFutabaServo
Using Futaba's servo motor from Python

## Installation

```
sudo gpasswd -a USERNAME dialout
sudo apt-get install python-serial
reboot
```

## Setting Up 
### Everytime you boot up the PC
```
sudo ./startup.sh
```

## How To Use
### Import 
```
from robot_servo import *
```
Serial converter
```
ttyUSB0 = serial_converter('/dev/ttyUSB0', 115200)
```
Servo
```
servo_list = [futaba_servo(ttyUSB0, id) for id in [1,2,3,4,5,6]]
```
Servo cluster(e.g., hand, arm....)
```
activate_sequence = [[0,3],[1,4],[2,5]]

#it means that when starting up,
#first, servo_list[0] and servo_list[3] are move to initial position.
#next, [1] and [4] are move.
#finnaly, [2] and [5] are move.

zero_position = [0,-200,0,0,-200,0]
hand = servo_cluster(servo_list, activate_sequence, zero_position)
```

### start and stop
start and go initial posture
```
hand.activate()
```
stop
```
hand.deactivate()
```

### setting torque

off   : 0  
on    : 1  
brake : 2  

```
# all servo
hand.torque(1)
# one by one
hand.torque([0,0,0,1,1,1])
```

### move servo
1[deg] -> 10  
1[sec] -> 100  

```
angles = [-200, -100, 0, 0, 100, 200] # -20, -10, 0, 0, 10, 20 [deg]
move_time = 100 # 1sec
hand.move(angles, move_time)
```

### get servo status
```
hand.get_status()
```

### set compliance parameters
```
servo_index = [0,1,2] # set parameters of servo_list[0], [1] and [2].
CW_Margin = 2
CCW_Margin = 2
CW_Slope = 10
CCW_Slope = 10
Punch = 180
hand.set_compliance_param(servo_index, CW_Margin, CCW_Margin, CW_Slope, CCW_Slope, Punch)
```
#### Margin
It is allow able range of the angle around the goal angle.  
If the error between the present angle and the goal position is in the set range,
the servo recognized itself to be in the goal position and stop moving.  
The unit is 0.1 degree and the settable range is 0~255 for both directions.
#### Slope
It is the range that output torque of the servo increases in proportion to the error between the present angle and aim angle.
The flexibility of the servo increases in proportion to this value.  
The unit is 1.0 degree, and the settable range is 0~255 for each directions.
#### Punch
It is the minimum torque (electric current) that is generated when present angle of the servo exceeds the range of Margin.  
The unit is 0.01% of the maximum torque and the settable range is 0~10000.


#### Initial Parameters
| Servo Type | Margin | Slope | Punch |
|:-----------|:-------|:------|:------|
| RS405CB, RS406CB | 1 (0.1[deg]) | 4 (4.0[deg]) | 1300 (13%) |
| RS301CR | 2 (0.2[deg]) | 10 (10.0[deg]) | 180 (1.8%) |
| RS302CD | 2 (0.2[deg]) | 15 (15.0[deg]) | 200 (2.0%) |
