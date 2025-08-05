modprobe ftdi-sio
chmod 777 /sys/bus/usb-serial/drivers/ftdi_sio/new_id
echo "1115 0008" > /sys/bus/usb-serial/drivers/ftdi_sio/new_id
