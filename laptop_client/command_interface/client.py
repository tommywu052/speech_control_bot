import socket, time

class ClientSocket(object):

    def __init__(self, delim='\r', buffer_size=1024, server_response_timeout=3):
        self.DELIM = delim
        self.BUFFER_SIZE = buffer_size
        self.SERVER_RESPONSE_TIMEOUT = server_response_timeout # in seconds
        self.sock = 0
        self.server_address = ('', 0)


    def init_connection(self, host='192.168.43.250', port=10000):
        if self.sock != 0: # a socket has been initiated before
            self.sock.close()
            print('terminating old connection to {} port {}'.format(*self.server_address))

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_address = (host, port)
        print('connecting to {} port {}'.format(*self.server_address))
        self.sock.connect(self.server_address)
        self.sock.settimeout(self.SERVER_RESPONSE_TIMEOUT)

    def send_message(self, message): # message is a string
        message = (message + self.DELIM).encode('ascii')
        self.sock.send(message)
        while True:
            try:
                data = self.sock.recv(self.BUFFER_SIZE) # NOTE: without timeout catching, code blocks here
                data_decoded = data.decode('ascii')
                if data_decoded:
                    print('received from server: {}'.format(data_decoded))
                    if data_decoded[-1] == self.DELIM:
                            # print('DELIM reached, closing client socket')
                       break
                else:
                    print('no response from server')
                    breaks
            except self.socket.timeout as e:
                err = e.args[0]
                if err == 'timed out':
                    print('TIMEOUT: no response from server')
                    break

if __name__ == '__main__':
    client_socket = ClientSocket(delim='\r', buffer_size=1024, server_response_timeout=3)
    client_socket.init_connection(host='192.168.43.250', port=10000)
    client_socket.send_message('heyman')

        
        


