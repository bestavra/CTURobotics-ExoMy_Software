#!/usr/bin/env python
# Python Program to Get IP Address
#
# Instructions:
#
# o Place file where you want it /usr/local/bin or leave it in ExoMy folder
# o Add to crontab: sudo crontab -e
#
# Run Every 5 mins. If once a min change */5 to *
# If once every 2 mins change */5 to */2 ...
#
# * * * * * python /home/pi/ExoMy_Software/scripts/get_ip.py >> /dev/null 2>&1
#
# Change /etc/default/cron and add "EXTRA_OPTS="-L 0" to stop logging cronjob in syslog
# Restart cron: /etc/init.d/cron force-reload
#
##################################################################
import socket
import json

def get_ip_address():
    ip_address = '';
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("192.0.0.1",80))
    ip_address = s.getsockname()[0]
    s.close()
    return ip_address

data = {}
data['host'] = {'ip-address': get_ip_address() }

with open('/home/pi/ExoMy_Software/scripts/temp/hostdata.json', 'w') as f:
    json.dump(data, f)