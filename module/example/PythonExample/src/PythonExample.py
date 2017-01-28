#!/usr/bin/env python3

from message.example import ExampleMessage, PythonResponse
from nuclear import Reactor, on, Trigger, Single, With, Every

@Reactor
class PythonExample(object):

    def __init__(self):
        print("Constructing PythonExample in python")

    @on(Trigger(ExampleMessage), With(ExampleMessage), Single())
    def example_callback(self, trigger_data, with_data):

        print('Python got a message', trigger_data.timestamp)

        msg = PythonResponse('sup c++')

        self.emit(msg)

    @on(Every(2.0))
    def every_callback(self):

        msg = PythonResponse('Python got an every')
        self.emit(msg)
