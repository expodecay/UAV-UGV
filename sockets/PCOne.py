import socket
import random
import time
import pickle
from point import point

nums = []
packet = bytes()
headersize = 8



for i in range(100):
	nums.append(point(random.randint(0, 100), random.randint(0, 100)))
print(nums)
	
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('', 11234))
s.listen(5)
clientsocket, address = s.accept()

packet = pickle.dumps(nums)

packet = '{:<{}}'.format(len(packet), headersize) + packet

while True:
	print(nums)
	clientsocket.send(bytes(packet))
	time.sleep(2)
print("end")