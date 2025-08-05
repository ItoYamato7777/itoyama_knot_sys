# Ensenso_handle

Getting ensenso's data through RTC_Ensenso on python

RTC_Ensenso is [here](http://negitoro:8080/takizawa/RTC_Ensenso "RTC_Ensenso")

## How to install
```
omniidl -bpython EnsensoService.idl
```
download [rtmtools](https://github.com/takashi-suehiro/rtmtools "rtmtools")

set up rtmtools path and naming server on which ensenso rtc is running.

- ensenso_handle.py l.1: path of rtmtools
- ensenso_handle.py l.2: naming server

## How to use
### add path to ensenso_handle and import 
```
import sys
sys.path.append(PATH_ENSENSO_HANDLE)
import ensenso_handle as ensenso
```
### viewing images
```
ensenso.show_all_images()
```

### getting data
```
port = "color"
img, timestamp = ensenso.capture(port)
```
port:
- color
- pointMap
- ir_L
- ir_R
- renderPointMap
- renderPointMapTexture

### getting synchronized data
```
port_list = ["renderPointMap", "renderPointMapTexture"]
img_list, timestamp = ensenso.capture(port_list)
```

### Changing projection mode
```
mode = "led"
ensenso.set_projection_mode(mode)
```
mode:
- off
- projector
- led

### turn on/off display mode of RTC
```
mode = "on"
ensenso.switch_monitor(mode)
```
mode:
- on
- off

### Getting/setting renderPointMap parameters
```
params = ensenso.get_rpm_params()
print "pixelsize: ", params["pixelsize"]
print "width:  ", params["width"]
print "height: ", params["height"]
ensenso.set_rpm_params(pixelsize = 1.0, width = 1280, height = 764)
```
