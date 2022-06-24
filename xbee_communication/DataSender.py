import serial
import time

disable_requests = True

pause = 0.3

x = 12.43345
y = 24.23432
z = 17.23434
	

def main():
	
	ser = serial.Serial('/dev/ttyUSB0') 			# initializes USB port
	print(ser.name)									# checks if the port exsists
	noRequestCounter = 0							# this variable hold number of cycles there was no send requests
	while True:
					# reciver will send a packege with special chacacter (ASCII code 1) indicating that it is ready to read.	
		length = ser.in_waiting 					# gets the length of the buffer
		if length > 0:
			recived_packet = ser.read(length)		# reads the entire length of the buffer
			if recived_packet.find(chr(1)) != -1 or disable_requests:		
							# checks if the special character is present or if requests are diabled
				noRequestCounter = 0				# reset noRequestCounter
				packet = chr(1) + str(x) + "," + str(y) + "," + str(z) + chr(1)	# create a packet
				ser.write(packet)					# send the packet
			elif recived_packert.find(chr(2)) != -1:# ASCII-code-2 stands for regular ping
				noRequestCounter = 0				# reset noRequest counter
		else:
			noRequestCounter += 1					# increase time out counter by 1
			if noRequestCounter * pause >= 5:		#if there is no pings from Resicer then warn the user
				print("WARNING: no pings from Resiver in 5 seconds.")
		time.sleep(pause)								# sleep for pause second

main()
	