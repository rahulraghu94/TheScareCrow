from RF24 import *

class Radio:

    # Only the first byte needs to be different. We can open 6 pipes
    # simultaneously for reading (0-5), but the pipe 0 is used for
    # writing also. Only one pipe can be used for writing.
    pipes = ["1Node", "2Node"]

    def __init__(self, channel=1, pa_level="RF24_PA_MAX", ce_pin=49):
        self.r = RF24(ce_pin, 0)
        self.r.begin()
        self.r.setChannel(channel)
        self.r.setPALevel(pa_level)
        self.r.enableDynamicPayloads()
        self.r.setRetries(5,15)
        self.r.openWritingPipe(pipes[0]);
        self.r.openReadingPipe(1, pipes[1]);
        self.r.startListening();

    def print_details():
        self.r.printDetails()

    def get_data():
        if self.r.available():
            data = []
            while self.r.available():
                len = self.r.getDynamicPayloadSize()
                data.append(self.r.read(len).decode('utf-8'))
            return data
        else:
            return None

    def send_data(data):
        self.r.stopListening()
        radio.write(data)
        self.r.startListening()
