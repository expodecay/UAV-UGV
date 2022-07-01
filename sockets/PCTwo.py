import socket
import pickle
from point import point

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.connect(('192.168.1.5', 11234))
counter = -1000;
full_msg = bytes()

while counter < 1:
	msg = s.recv(8)
	if len(msg) != 0:
		print("alleget packet length: {}".format(msg.decode("utf-8")))
		while len(full_msg) < int(msg):
			full_msg = full_msg + s.recv(int(msg) - len(full_msg))
		
		decode = pickle.loads(full_msg)
		# print("actuall length: {}".format(len(full_msg.decode("utf-8"))))
		print(decode)
		full_msg = bytes()
		counter = counter + 1