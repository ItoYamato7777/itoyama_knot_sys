# Setup Ubuntu
```
sudo apt-get update
sudo apt-get upgrade
LANG=C xdg-user-dirs-gtk-update
gsettings set org.gnome.desktop.input-sources xkb-options "['ctrl:nocaps']"
sudo modprobe ftdi-sio
sudo chmod 777 /sys/bus/usb-serial/drivers/ftdi_sio/new_id
sudo echo "1115 0008" > /sys/bus/usb-serial/drivers/ftdi_sio/new_id
sudo gpasswd -a USERNAME dialout
sudo apt-get install git gitk
sudo apt-get install tmux
sudo apt-get install python-pip
sudo pip install --upgrade pip
mkdir ~/Downloads/openRTM-aist_install/
cd ~/Downloads/openRTM-aist_install/
wget https://raw.githubusercontent.com/OpenRTM/OpenRTM-aist/master/scripts/pkg_install_ubuntu.sh
sudo sh ./pkg_install_ubuntu.sh -l c++ -l python -l openrtp -d --yes
sudo apt-get install python-numpy python-scipy
sudo python -mpip install -U matplotlib
sudo pip install -U scikit-learn
```


## Install emacs
Check [here](http://negitoro:8080/takizawa/emacs_el).

## Install nVidia GPU Driver
Download .run file from nVidia's webpage. (e.g., NVIDIA-Linux-x86_64-285.05.09.run)
1. Press Ctrl + Alt + F1
2. login
3. cd to the location of the file 
4. run the following commands:
```
sudo service lightdm stop
chmod +x ./NVIDIA-Linux-x86_64-285.05.09.run
sudo ./NVIDIA-Linux-x86_64-285.05.09.run
```

## Install Open3D
```
sudo pip install open3d
```

## Install OpenCV
Check [here](http://negitoro:8080/takizawa/Install-OpenCV).


## Setting up for rtc_handle

Describe the network address of the HIRO and DT13D04 (ensenso is connected) in /etc/hosts
```
192.168.4.195   hiro022
192.168.4.196   DT99B03
```

Add JointDataTypes.idl into /dist-packages/OpenRTM_aist/RTM_IDL where OpenRTM-aist-Python1x is installed.  
(In my case, that is /usr/local/python2.7/dist-packages/OpenRTM_aist/RTM_IDL.)  
Then, idl-compile that.
```
sudo omniidl -bpython JointDataTypes.idl
```

## Serial port setting

| Port | Device |
|:-----|:-------|
|/dev/ttyUSB0|Serial converter for right hand servo motors|  
|/dev/ttyUSB1|Serial converter for left hand servo motors|  
|/dev/ttyUSB2|Serial converter for teaching hand servo motors|  
|/dev/ttyACM0|HOKUYO Simple URG for Right hand|  
|/dev/ttyACM1|HOKUYO Simple URG for Left hand|  
|/dev/lepR (ttyACM2) |Leptrino Force Sensor for Right Hand|  
|/dev/lepL (ttyACM3) |Leptrino Force Sensor for Left Hand|  

## make dirs
```
cd
mkdir rtmnaming
cd rtmnaming
mkdir 2809
mkdir 9876
```

# How to install
```
$ mkdir FOLDER_NAME
$ cd FOLDER_NAME
$ git clone http://192.168.0.9:8080/git/takizawa/knotting_env.git IncludeFiles
$ cd IncludeFiles
$ ./setup.sh
```

# How to use
## Every time you boot up the PC,
### On main PC (DT18-B01)
Starting naming server
```
cd ~/rtmnaming/9876
rtm-naming 9876
```

Setting up RSC-U485
```
sudo modprobe ftdi-sio
sudo chmod 777 /sys/bus/usb-serial/drivers/ftdi_sio/new_id
sudo echo "1115 0008" > /sys/bus/usb-serial/drivers/ftdi_sio/new_id
```

### On Sub PC (DT13-D04)
Run RTC_Ensenso
```
cd workspace/RTC_Ensenso
./build/src/EnsensoComp
```

## Import knot_sys
run python
```
cd 
cd FOLDER_NAME
python
```
import knot_sys
```
>>> from knot_sys import *
```
When "Calibrate Joints?[Y]/n: " is displayed, Press "y" and Enter key.  
When "Push [OK] to Servo ON all" is displayed, press Enter key.
