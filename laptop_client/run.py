import time 
from command_interface import CommandInterface

CI = CommandInterface()

# start code
#CI.connect_to_bot(host=host, port=port)

while True:
	print('Press ENTER to start recording: ')
	input()
	# time.sleep(0.3)
	response = CI.listen_to_command(mode='manual', duration=1)
	command = CI.parse_response(response)
	if not isinstance(command, type(None)):
		if command[0] == 'f' or command == '4':
			CI.send_message_to_bot('forward')
		if command[0] == 'b':
			CI.send_message_to_bot('backward')
		if command[0] == 'l':
			CI.send_message_to_bot('left')
		if command[0] == 'r':
			CI.send_message_to_bot('right')
		if command[0] == 's':
			CI.send_message_to_bot('stop')
	else:
		print('Did not get command. Please record again')
