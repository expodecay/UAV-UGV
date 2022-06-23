import serial
import time




def main():
	
	ser = serial.Serial('/dev/ttyUSB1')
	print(ser.name)
	while True:
		length = ser.in_waiting
		recived_packet = ser.read(length)
		if length > 0:
			print(recived_packet)
			if recived_packet[-2] != 'o':
				print("//Not a full packet//")
			else:
				print("Full packet")
		else:
			print("buffer is empty")
		time.sleep(200.0/1000.0)

main()