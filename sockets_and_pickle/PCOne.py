import socket
import random
import time
import pickle
from point import point

nums = []																# what we are gonna send (a list of ppints)
packet = bytes()														# list will be in byte form
headersize = 8															# each send packege will have a header that goes before data



for i in range(100):													# creating list
	nums.append(point(random.randint(0, 100), random.randint(0, 100)))
print(nums)
	
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)					# initializing the socket to IPV4 and streaming of data
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)					# use sockets that are not timed out yet
s.bind(('', 11234))														# use global IP and port number 11234
s.listen(5)																# accept up to 5 devices to this port
clientsocket, address = s.accept()										# accept (idk how many) devices

packet = pickle.dumps(nums)												# create binary out of points list

packet = '{:<{}}'.format(len(packet), headersize) + packet				# create header that contains the length of the binary data

while True:																# send data constantly
	print(nums)															# print list
	clientsocket.send(bytes(packet))									# send to the client that we accepted
	time.sleep(2)														# wait 2 sec
print("end")