#!/usr/bin/env python3

from pymavlink import mavutil
import fcntl, os
import errno
import socket
import pickle
import time
import sys
import jetson.inference
import jetson.utils
from datetime import datetime

HEADERSIZE = 8
LOG_INTERVAL = 1
GLOBAL_POSITION_TIMEOUT = 3

def recive_data(sock):
    buffer = b''
    try:
        buffer = sock.recv(HEADERSIZE)
    except socket.error as e:
        err = e.args[0]
        if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
            # print("nothing recived")
            return b''
        else:
            print(e)
            sys.exit(1)
    if len(buffer) != 0:
        print("alleget packet length: {}".format(buffer.decode("utf-8")))
        full_msg = b''
        while len(full_msg) < int(buffer):
            full_msg = full_msg + sock.recv(int(buffer) - len(full_msg))
        decode = pickle.loads(full_msg)
        return decode
    


GROUND_STATION = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
GROUND_STATION.connect(('192.168.1.4', 11234))
fcntl.fcntl(GROUND_STATION, fcntl.F_SETFL, os.O_NONBLOCK)

init_msg = pickle.dumps('DRONE')
init_msg = '{:<{}}'.format(len(init_msg), HEADERSIZE).encode('utf-8') + init_msg
GROUND_STATION.send(init_msg)

HUSKY = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
HUSKY.connect(('192.168.1.2', 11234))
fcntl.fcntl(HUSKY, fcntl.F_SETFL, os.O_NONBLOCK)

init_msg = pickle.dumps('DRONE', protocol=2)
init_msg = '{:<{}}'.format(len(init_msg), HEADERSIZE).encode('utf-8') + init_msg
HUSKY.send(init_msg)

loc = {'lat': -91, 'lon': -181, 'type': 'none'}

'''------------initialize jetson------------'''
net = jetson.inference.detectNet()
video_input = jetson.utils.videoSource()

output = jetson.utils.videoOutput("rtp://192.168.1.3:1234") # use for remote viewing

#record to avi
dir_original ='/home/souyu/Documents/REU_2022_recordings'
now = datetime.now()
time_stamp = now.strftime("%Y-%m-%d_%H_%M_%S")
output = jetson.utils.videoOutput(os.path.join(dir_original, time_stamp + '.avi'))

print("Image Detection has started!")
print(time_stamp)
confidence = 0
'''------------jetson initialized------------'''

connection = mavutil.mavlink_connection(device = '/dev/ttyACM0', baud = 57600)

connection.wait_heartbeat()
    
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))
last_log = time.time()
last_global_position_recv = time.time()

while True:
    mavlink_msg = connection.recv_match()
    if mavlink_msg != None and mavlink_msg.get_type() == 'GLOBAL_POSITION_INT':
        loc['lat'] = mavlink_msg.lat
        loc['lon'] = mavlink_msg.lon
        loc['type'] = 'GLOBAL_POSITION_INT'
        last_global_position_recv = time.time()
        print(loc)
    if mavlink_msg != None and time.time() - last_global_position_recv > GLOBAL_POSITION_TIMEOUT and mavlink_msg.get_type() == 'GPS_RAW_INT':
        loc['lat'] = mavlink_msg.lat
        loc['lon'] = mavlink_msg.lon
        loc['type'] = 'GPS_RAW_INT'
        last_global_position_recv = time.time()
        print("WARNING: No 'GLOBAL_POSITION_INT' detected within timeout window")
        print(loc)

    decode = recive_data(GROUND_STATION)
    if decode == '1':
        print("sending location to husky")
        packet = pickle.dumps(loc, protocol=2)
        packet = '{:<{}}'.format(len(packet), HEADERSIZE).encode('utf-8') + packet
        HUSKY.send(packet)
    
    img = video_input.Capture()
    detections = net.Detect(img, overlay="box,labels,conf")
    if len(detections) > 0:
        for detection in detections:
            counter = net.GetClassDesc(detection.ClassID)
            if counter == 'person':
                confidence = detection.Confidence
                jetson.utils.cudaDeviceSynchronize() # Allows to take both video and photo at same time
                GS_report = pickle.dumps("Detected a person! Confidence is {:f}".format(confidence))
                GS_report = '{:<{}}'.format(len(GS_report), HEADERSIZE).encode('utf-8') + GS_report
                GROUND_STATION.send(GS_report)
                print("Detected a person! Confidence is {:f}. Sending location to Husky.".format(confidence))
                packet = pickle.dumps(loc, protocol=2)
                packet = '{:<{}}'.format(len(packet), HEADERSIZE).encode('utf-8') + packet
                HUSKY.send(packet)
    output.Render(img)
    
    if time.time() - last_log >= LOG_INTERVAL:
        packet = pickle.dumps(loc)
        packet = '{:<{}}'.format(len(packet), HEADERSIZE).encode('utf-8') + packet
        last_log = time.time()
        GROUND_STATION.send(packet)
    
    time.sleep(0.1)
    
