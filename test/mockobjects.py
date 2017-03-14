"""Mock objects for testing"""


class MockObject(object):
    def __init__(self):
        self.log = []
        self.mocks = {}

    def __getattr__(self, name, *args, **kwargs):
        self.log.add(['get', name, args, kwargs])

        if name in self.mocks:
            if callable(self.mocks[name]):
                return self.mocks[name](*args, **kwargs)
            return self.mocks[name]

    def __setattr__(self, name, value):
        self.log.add(['set', name, value])

        self.mocks[name] = value


class MockGPIO(MockObject):
    def __init__(self):
        super(MockGPIO, self).__init__()

        self.mocks = {
            'OUT': 'OUT',
            'IN': 'IN',
            'HIGH': 'HIGH',
            'LOW': 'LOW',
            'BOARD': 'BOARD'
            }


class MockPWM(MockObject):
    pass
