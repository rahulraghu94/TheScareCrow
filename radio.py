from RF24 import *

class Radio:

    # Only the first byte needs to be different. We can open 6 pipes
    # simultaneously for reading (0-5), but the pipe 0 is used for
    # writing also. Only one pipe can be used for writing.
    # pipes = ["1Node", "2Node"]
    pipes = [0x65646f4e32, 0x65646f4e31]
    payloadSize = 32

    def __init__(self, channel=0x0A, pa_level=RF24_PA_MAX, ce_pin=49):
        self.r = RF24(ce_pin, 0)
        self.r.begin()
        self.r.setRetries(5,15)
        self.r.setChannel(channel)
        self.r.setPALevel(pa_level)
        self.r.payloadSize = self.payloadSize
        self.r.openWritingPipe(self.pipes[0]);
        self.r.openReadingPipe(1, self.pipes[1]);
        self.r.startListening();

    def print_details(self):
        self.r.printDetails()

    def radio_available(self):
        return self.r.available()

    def get_data(self):
        if self.r.available():
            data = ""
            while self.r.available():
                len = self.payloadSize
                data = data + self.r.read(len).decode('utf-8')
            return data
        else:
            return None

    def send_data(self, data):
        self.r.stopListening()
        radio.write(data)
        self.r.startListening()
