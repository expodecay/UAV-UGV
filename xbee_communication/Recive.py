import serial
import time




def main():
	
	ser = serial.Serial('/dev/ttyUSB1') 		# initializes a USB port. This is Ubuntu address
	print(ser.name) 							# Check if port exsists
	while True:
		length = ser.in_waiting 				# gets the length of the buffer
		recived_packet = ser.read(length)		# reads the entire length of the buffer
		if length > 0:			
			print(recived_packet)				# prints read buffer
			if recived_packet[-1] != chr(1):	# In this case every packet that sender script sends ends in special character
				print("//Not a full packet//")	# that has ASCII code of 1. This is used to check if the wnrierty of the packet was recived
			else:
				print("Full packet")			# if we have 'o' as the last element then we have full packet
		else:
			print("buffer is empty")			# if length of the buffer is 0 then it is empty
		time.sleep(200.0/1000.0)				# sleep and let other processes run

main()
