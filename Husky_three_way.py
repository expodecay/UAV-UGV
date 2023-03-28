import rospy
from gps_common.msg import GPSFix
from sensor_msgs.msg import NavSatFix
from new_mission_lib_test import send_mission
import fcntl, os
import errno
import socket
import pickle
import time
import sys

HEADERSIZE = 8
LOG_INTERVAL = 1
loc = {'lat': -91, 'lon': -181}


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


def send_data(data, sock, h_size):
    packet = pickle.dumps(data)
    packet = '{:<{}}'.format(len(packet), h_size).encode('utf-8') + packet
    sock.send(packet)


def update_location():
    loc = {'lat': data.latitude, 'lon': data.longitude}


GROUND_STATION = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
GROUND_STATION.connect(('192.168.1.4', 11234))
fcntl.fcntl(GROUND_STATION, fcntl.F_SETFL, os.O_NONBLOCK)

send_data('HUSKY', GROUND_STATION, HEADERSIZE)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('', 11234))
s.listen(5)

clientsocket, address = s.accept()
fcntl.fcntl(clientsocket, fcntl.F_SETFL, os.O_NONBLOCK)

id = b''
while len(id) == 0:
    id = recive_data(clientsocket)
print("Drone connected")

DRONE = {'socket': None, 'addr': None}

if id == 'DRONE':
    DRONE['socket'] = clientsocket
    DRONE['addr'] = address

target_loc = {'lat': -91, 'lon': -181}

# subscribe to GPS topic and post it to loc dictionary
# in the call back function do not forget
last_log = time.time()
# create a publisher

rospy.init_node("comunication_node")

pub = rospy.Publisher("/drone_goal", GPSFix, queue_size=10)
rospy.Subscriber("/piksi_position/navsatfix_spp", NavSatFix, update_location)
print("publisher initialized")

while True:
    if time.time() - last_log >= LOG_INTERVAL:
        last_log = time.time()
        send_data(loc, GROUND_STATION, HEADERSIZE)

    target = recive_data(DRONE['socket'])
    print(target)
    if type(target) == type({}):
        print("INFO: Recived target location: " + str(target))
        target_loc = target
        send_mission(target_loc, {"lat": 34.059319, "lon": -117.820521})
        # publish location to the topic
        msg = GPSFix()
        msg.latitude = target_loc['lat']
        msg.longitude = target_loc['lon']
        pub.publish(msg)
    time.sleep(0.4)
