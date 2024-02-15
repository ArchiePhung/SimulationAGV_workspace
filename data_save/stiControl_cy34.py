def check_CYrun():	
	# -
	if self.step_check11 == 0:
		if self.status_conveyor11.status == 4:
			if self.status_CPD.input2 == 0:
				self.step_check11 = 1
				print("Băng tải trôi qua check lần 1")
			else:
				self.enable_moving = 0
	elif self.step_check11 == 1:
		if self.status_CPD.input2 == 0:
			self.step_check11 = 2
			print("Băng tải đã trôi qua check lần 2")
		else:
			self.flagWarning_transfer11 = 1
			self.enable_moving = 0
	elif self.step_check11 == 2:
		if self.status_CPD.input2 == 0:
			self.step_check11 = 3
			print("Băng tải trôi qua check lần 3")
		else:
			self.flagWarning_transfer11 = 1
			self.enable_moving = 0
	elif self.step_check11 == 3:
		if self.status_CPD.input2 == 0:
			print("Băng tải trôi qua check lần 4")
			self.completed_after_mission1 = 1
			self.step_check11 = 0
		else:
			self.flagWarning_transfer11 = 1
			self.enable_moving = 0