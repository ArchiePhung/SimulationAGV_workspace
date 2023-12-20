# AGV_OC8.2
Frame_CAN 

---------- Received ----------
Byte_0: ID OC.

Byte_1: Mission.
	+ Reset:    0
	+ Received: 1
	+ Transmit: 2
	+ Rotation_forward : 10
	+ Rotation_opposite: 11

Byte_2: Percent speed.
	+ 50 <--> 100 %

Byte_3: 
Byte_4: 
Byte_5: 
Byte_6: 
Byte_7: 

---------- Transmit ----------
Byte_0: Status1.
	+ Free: 0
	+ Runing Received: 1
	+ Runing Transmit: 2
	+ Completed Received: 3
	+ Completed Transmit: 4
	+ Rotation_forward Runing : 10
	+ Rotation_opposite Runing: 11

Byte_1: Status2.
	+ Free: 0
	+ Runing Received: 1
	+ Runing Transmit: 2
	+ Completed Received: 3
	+ Completed Transmit: 4
	
Byte_2: status sensor limit ahead 1.
	+ Not: 0
	+ Yes: 1
Byte_3: status sensor limit behind 2.
	+ Not: 0
	+ Yes: 1
Byte_4: status sensor check rack 1.
	+ Not: 0
	+ Yes: 1
Byte_5: status sensor limit ahead 2.
	+ Not: 0
	+ Yes: 1
Byte_6: status sensor limit behind 2.
	+ Not: 0
	+ Yes: 1
Byte_7: status sensor check rack 2.
	+ Not: 0
	+ Yes: 1

------------------- Bố trí vị trí băng tải -------------------
    |  1  |  3  |  5  |
Đầu |  2  |  4  |  6  |