"""Mock objects for testing"""


class MockObject(object):
    def __init__(self):
        self.log = []
        self.mocks = {}
        self.mock_called = 'call'

    def __getattr__(self, name):
        if name == 'log' or name.startswith('mock'):
            return self.__dict__[name]

        if name in self.mocks:
            if callable(self.__dict__['mocks'][name]):
                self.mock_called = name
                return self.__dict__['mocks'][name]
            self.log.append(['get', name])
            return self.__dict__['mocks'][name]
        self.log.append(['get', name])

    def __setattr__(self, name, value):
        if name in ['log', 'mocks', 'mock_called']:
            self.__dict__[name] = value
        else:
            self.__dict__['log'].append(['set', name, value])
            self.__dict__['mocks'][name] = value

    def mock_call(self, *args, **kwargs):
        self.log.append([self.mock_called, args, kwargs])
        pass

    def mock_reset_log(self):
        self.log = []

class MockGPIO(MockObject):
    def __init__(self):
        super(MockGPIO, self).__init__()

        self.mocks = {
            'OUT': 'OUT',
            'IN': 'IN',
            'HIGH': 'HIGH',
            'LOW': 'LOW',
            'BOARD': 'BOARD',
            'setup': self.mock_call,
            'output': self.mock_call,
            'setmode': self.mock_call
            }

class MockPWM(MockObject):
    def __init__(self):
        super(MockPWM, self).__init__()

        self.mocks = {
            'write': self.mock_call
            }
