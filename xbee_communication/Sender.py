import serial
import time

x = 10
y = 20
z = 15

def main():
	
	ser = serial.Serial('/dev/ttyUSB0') 			# initializes USB port
	print(ser.name)									# checks if the port exsists
	while True:
		packet = str(x) + "," + str(y) + "," + str(z) + chr(1)
		ser.write(packet)								# send the packet
		time.sleep(1)									# sleep for 1 second

main()
	