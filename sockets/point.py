class point:
	def __init__(self):
		self.x = 0
		self.y = 0
		self.z = 0
	
	def __init__(self, x, y,):
		self.x = x
		self.y = y
		self.z = 0
	
	def __str__(self):
		return "(" + str(self.x) + ", " + str(self.y) + ", " + str(self.z) + ")"
	
	def __repr__(self):
		return self.__str__()