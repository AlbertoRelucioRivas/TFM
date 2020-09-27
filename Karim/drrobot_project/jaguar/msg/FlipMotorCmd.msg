#this is used for Flip Motor Drive Command, all the motors work in position-control mode
int32 leftFrontCmd	# left front flip motor cmd, will send to drive baord-3, channel 1, like "!PR 1 -30"
int32 rightFrontCmd	# right front flip motor cmd, will send to drive baord-3, channel 2, like "!PR 2 50"
int32 leftRearCmd	# left rear flip motor cmd, will send to drive baord-4, channel 1, like "!PR 1 0", will keep same position
int32 rightRearCmd	# right rear flip motor cmd, will send to drive baord-4, channel 2, like "!PR 2 -100"
