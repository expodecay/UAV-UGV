import serial
import time


fullMsg = ""

send_requests = True

def getAmsg(recived):
	packet = ""
	lastMarker = -2
	read = False
	stopReading = False
	for i in range(len(recived)-1, -1, -1):
		if recived[i] == chr(1):
			if read and i+1 != lastRead:
				stopReading = True
				break
			else:
				read = True
				lastRead = i
		elif read and not stopReading:
			packet = recived[i] + packet
	if read and stopReading:
		return packet
	else:
		return ""

	#-------------------DEBUG CODE---------------------------------------
"""	
def printIntArr(arr):
	print("ASCII codes are:")
	for i in arr:
		print(i + " | ")
"""


	#----------------END OF DEBUG CODE-----------------------------------
			

def main():
	
	ser = serial.Serial('/dev/ttyUSB0') 		# initializes a USB port. This is Ubuntu address
	print(ser.name) 							# Check if port exsists
	recived_packet = ""
	all_packets = ""
	skipCounter = 0
	skipLimit = 10
	while True:
		length = ser.in_waiting 				# gets the length of the buffer
		recived_packet = ser.read(length)		# reads the entire length of the buffer
		all_packets += recived_packet			# adds recived
		msg = getAmsg(all_packets)				# tries to get a full message out of recived data
		if len(msg) > 0:	
			# print("All of it: " + all_packets)
			# print("All recived data: " + recived_packet)# prints all read data
			print("Message: " + msg)			# prints full message
			fullMsg = msg						# bring packets into global space
			all_packets = ""					# flush all packets
		else:
			if len(recived_packet) == 0:		# (comment this out later!!!)
				print("buffer is empty")		# if length of the buffer is 0 then it is empty
			else:
				print("//No full message found//")# if we recived packets but did not get a message, we have half of the message
				# printIntArr(all_packets)
		if send_requests:
			ser.write(chr(1))					# if requests are enabled then send one
		else:
			ser.write(chr(2))					# if not then just ping
		time.sleep(700.0/1000.0)				# sleep and let other processes run

main()