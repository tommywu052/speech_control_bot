import socket
import sys

DELIM = '\r'
BUFFER_SIZE = 1024
import os
python_file_dir_path = os.path.dirname(os.path.realpath(__file__))
print("socket2file.py is in: {}".format(python_file_dir_path))
OUTPUT_FILE = python_file_dir_path + "/command.txt"
# Initial write to file to prevent File Not Found
with open(OUTPUT_FILE, 'w+') as outfile:
    outfile.write("stop")

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the port
server_address = ('0.0.0.0', 10000) # NOTE: for LAN connections, host should be 0.0.0.0
print('starting up on {} port {}'.format(*server_address))
sock.bind(server_address)

# Listen for incoming connections
sock.listen(1) # NOTE: if we want more than one client connection, need multi-thread

while True:
    # Wait for a connection
    connection, client_address = sock.accept()
    print('connection from', client_address)

    # If connection OK, listen on any client messages
    while True:
        data = connection.recv(BUFFER_SIZE) # NOTE: if client not sending, code will block here
        data_decoded = data.decode('ascii')
        if data_decoded:
            print('received from client: {}'.format(data_decoded))
            if data_decoded[-1] == DELIM:
                print('DELIM reached, sending response to client\n')
                connection.sendall('got your message: {}{}'.format(data_decoded, DELIM).encode('ascii')) # make sure that client is able to read response in one go
                with open(OUTPUT_FILE, 'w') as outfile:
                    outfile.write(data_decoded.strip())
                
        else:
            print('client terminated, closing connection to allow for next client connection')
            connection.close()
            break
