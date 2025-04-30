import network   # handles connecting to WiFi
import urequests # handles making and servicing network requests
import socket
import dht
import time
from time import sleep

# Connect to network
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
# prevent the wireless chip from
# activating power-saving mode when it is idle
wlan.config(pm = 0xa11140)
# set a static IP address for Pico
# your router IP could be very different eg:
# 192.168.1.1
# Or comment out if using a dynamic connection
wlan.ifconfig(('ip', 'subnet', 'gateway', 'dns'))
# Fill in your network name (ssid) and password here:
ssid = 'ssid'
#password = 'password if needed'
wlan.connect(ssid)#, password)

led = machine.Pin('LED', machine.Pin.OUT)
for i in range(wlan.status()):
    led.on()
    time.sleep(2)
    led.off()
    time.sleep(2)
    print('Connected')
    status = wlan.ifconfig()
    print('ip = ' + status[0])
