from multiprocessing.pool import ThreadPool
import fcntl, os
import errno
import socket
import time
import pickle
import sys
import keyboard


HEADERSIZE = 8
recipients = []

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
        # print("alleget packet length: {}".format(buffer.decode("utf-8")))
        full_msg = b''
        while len(full_msg) < int(buffer):
            full_msg = full_msg + sock.recv(int(buffer) - len(full_msg))
        decode = pickle.loads(full_msg)
        return decode

def flush_socket(sock):
	while True:
		try:
			sock.recv(1024)
		except socket.error as e:
			err = e.args[0]
			if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
				# print("nothing recived")
				break
			else:
				print(e)
				sys.exit(1)

def get_recipient(soc):
	clientsocket, address = soc.accept()
	fcntl.fcntl(clientsocket, fcntl.F_SETFL, os.O_NONBLOCK)
	return clientsocket, address

Husky = {'socket': None, 'addr': None}
Drone = {'socket': None, 'addr': None}


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('', 11234))
s.listen(5)

pool = ThreadPool(processes=1)
thread = pool.apply_async(get_recipient, (s,))

while len(recipients) < 2:				# change this to 2 when you add another 
    if thread.ready():
        clientsocket, address = thread.get()
        client = (clientsocket, address)
        recipients.append(client)
        print(address)
        thread = pool.apply_async(get_recipient, (s,))
        print(len(recipients))
		
decode = b''
while len(decode) == 0:
	decode = recive_data(recipients[0][0])
	
if decode == 'HUSKY':
	Husky['socket'] = recipients[0][0]
	Husky['addr'] = recipients[0][1]
	Drone['socket'] = recipients[1][0] 
	Drone['addr'] = recipients[1][1]
	flush_socket(Husky['socket'])
	flush_socket(Drone['socket'])
elif decode == 'DRONE':
	Drone['socket'] = recipients[0][0] 
	Drone['addr'] = recipients[0][1]
	Husky['socket'] = recipients[1][0]
	Husky['addr'] = recipients[1][1]
	flush_socket(Husky['socket'])
	# ret = Drone['socket'].recv(10)
	flush_socket(Drone['socket'])
		
print(Drone)

while True:
	husky_sais = recive_data(Husky['socket'])
	if len(husky_sais) != 0:
		print("HUSKY: " + str(husky_sais))
	
	drone_sais = recive_data(Drone['socket'])
	if len(drone_sais) != 0:
		print("DRONE: " + str(drone_sais))
	
	if keyboard.is_pressed('k'):
		# print("k is pressed")
		drone_command = '1'
		drone_command = pickle.dumps(drone_command)
		drone_command = '{:<{}}'.format(len(drone_command), HEADERSIZE).encode('utf-8') + drone_command
		print(drone_command)
		Drone['socket'].send(drone_command)

		
	time.sleep(0.2)