from .microphone import Microphone
from .client import ClientSocket
import json


class CommandInterface(object):
    def __init__(self):
        # initializing a Microphone object will run a 
        # piece of code to select the input source
        self.mic    = Microphone()
        self.client = ClientSocket(delim='\r', buffer_size=1024, server_response_timeout=3) 
        
        self.__setup()
 
    #################################################
    #               Command related methods         #
    #################################################

    def __setup(self):
        with open('config.json', 'r') as f:
            config  = json.load(f)
        api_key = config['api_key']
        host    = config['host']
        port    = config['port']
        
        self.mic.set_api_key(api_key)
        self.connect_to_bot(host=host, port=port)

    def listen_to_command(self, mode='auto', duration=None):
        return self.mic.get_command(mode, duration)

    def parse_response(self, response):
        # returns the decoded text if speech is detected
        # else returns None 
        try:
            cmd = response.pop('DisplayText')
            cmd = cmd.replace('.', '')
            cmd = cmd.lower()
            return cmd
        except KeyError:
            return None
        
    #################################################
    #               Socket related methods          #
    #################################################
    def send_message_to_bot(self, msg):
        try:
            self.client.send_message(msg)
        except Exception as err:
            print('An error occured. Message not sent.')
            raise err

    def connect_to_bot(self, host, port):
        try:
            assert (isinstance(host, str))
        except AssertionError:
            raise TypeError(f'Host IP has to be string, not {type(host).__name__}')
        try:
            assert (isinstance(port, int))
        except AssertionError:
            raise TypeError(f'Port has to be int, not {type(port).__name__}')
        self.client.init_connection(host, port)


if __name__ == '__main__':
    CI = CommandInterface()
    HOST = '192.168.43.250'
    PORT = 10000
    
    # initialize a control center client and establish a connection to the bot
    CI.connect_to_bot(host=HOST, port=PORT)
    
    while True:
        print('Press ENTER to start recording: ')
        input()
        response = CI.listen_to_command()
        command = CI.parse_response(response)
        CI.send_message_to_bot(command)
    
