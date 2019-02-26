import json
import pyaudio
import requests
import speech_recognition as sr

audio = pyaudio.PyAudio()


class Microphone(object):

    def __init__(self):
        self.microphone_index = None
        self.sample_rate = None 
        self.recording_devices = {}
        self.output_devices = {}

        self.__scan_devices()
        self.__set_recording_device_index()

        """
        Azure ASR configuration parameters
        """
        self.__API_KEY  = None
        self.__REGION   = 'westus'
        self.__MODE     = 'interactive' 
        self.__LANG     = 'en-US'#'zh-TW'
        self.__FORMAT   = 'simple'
        self.__AT_URL   = 'https://api.cognitive.microsoft.com/sts/v1.0/issueToken'


    def __scan_devices(self):
        device_count = audio.get_device_count()

        for idx in range(device_count):
            device = audio.get_device_info_by_index(idx)
            
            if self.__is_recording_device(device):
                self.recording_devices[idx] = device
            else:
                self.output_devices[idx] = device

    def __set_recording_device_index(self):
        print('#'*60)
        print('Available devices:')
        for v in self.recording_devices.values():
            print(f"Recording Device: {v['name']:20} | Device ID: {v['index']}")
        print('#'*60)
        print('\n')

        while True:
            print('Please select recording device by device ID: ', end='')
            deviceID = input()
            try:
                int(deviceID)
            except ValueError:
                print('Error: deviceID cannot be a {}. '.format(type(deviceID).__name__), end='')
                print('Please re-enter.\n')
                continue

            deviceID = int(deviceID)

            if deviceID in self.recording_devices:
                self.microphone_index = deviceID
                break
            else:
                print('Error: Invalid device ID. Please re-enter.\n')
    
    def __set_sample_rate(self):
        while True:
            print('Enter new sample rate: ', end='')
            rate = input()
            
            try:
                int(rate)
            except ValueError:
                print(f'Error: sample rate cannot be a {type(rate).__name__}', end='')
                print('Please re-enter\n')
                continue
            rate = int(rate)
            self.sample_rate = rate
            break

    def __is_recording_device(self, device):
        assert isinstance(device['maxInputChannels'], int)
        return True if device['maxInputChannels'] > 0 else False

    def __get_token(self):
        headers = {'Ocp-Apim-Subscription-Key': self.__API_KEY}
        response =  requests.post(self.__AT_URL, headers=headers)
        token = response.content
        return token

    def __speech2text(self, token, audio):
        url  =f'https://{self.__REGION}.stt.speech.microsoft.com/speech/recognition/{self.__MODE}/cognitiveservices/v1?language={self.__LANG}&format={self.__FORMAT}'
        headers = {
                'Accept': 'application/json',
                'Ocp-Apim-Subscription-Key': self.__API_KEY,
                'Transfer-Encoding': 'chunked',
                'Content-type': 'audio/wav; codec=audio/pcm; samplerate=16000',
                'Authorization': f'Bearer {token}'
                }
        response = requests.post(url, headers=headers, data=audio)
        results = json.loads(response.content)
        return results
    
    def __chunk_data(self, raw_audio, chunk_size=1024): 
        for i in range(0, len(raw_audio), chunk_size):
            yield raw_audio[i:i+chunk_size]

    def __not_safe_record(self, mode, duration):
        """
        may raise OSError because device true sample rate is not the same as
        registered in the device
        """
        recognizer = sr.Recognizer()
        with sr.Microphone(self.microphone_index, sample_rate=self.sample_rate) as source:
                print('Use your voice to control the robot: ')
                if mode == 'auto':
                    audio = recognizer.listen(source)
                elif mode == 'manual':
                    audio = recognizer.record(source, duration=duration)
        return audio

    def set_api_key(self, api_key):
        if self.__API_KEY is not None:
            print('Overwriting existing API key')
            self.__API_KEY = api_key
        else:
            print('Initiating microphone API key')
            self.__API_KEY = api_key 

    def record(self, mode, duration):
        """
        we catch the error in the public method to ensure the user can always
        get the desired recording
        """
        try:
            audio = self.__not_safe_record(mode, duration)
        except OSError as err:
            if err.errno == -9997:
                print('#'*100)
                print('Device TRUE sample rate is different than the one listed IN the device')
                print('You need to manually enter the TRUE sample rate')
                print('To find out the TRUE sample rate of your device, please run the script "check_device sh" ')
                print('#'*100)
                self.__set_sample_rate()
                audio = self.record(mode, duration)
            else:
                raise err
        return audio
    
    def get_command(self, mode='auto', duration=None):
        """
        we can let the recognizer audio terminate by setting the mode to auto
        or we can set the mode to manual and specify a duration
        """
        if mode == 'manual':
            try:
                assert (duration is not None)
            except AssertionError as error:
                raise AssertionError('Listening in manual mode but duration is not specified')
        
        audio = self.record(mode=mode, duration=duration)
        audio = audio.get_wav_data()
        bytes_generator = self.__chunk_data(audio)
        token = self.__get_token()
        
        return self.__speech2text(audio=bytes_generator, token=token)

if __name__ == '__main__':
    mic = Microphone()
    print(mic.get_command())
        
        


