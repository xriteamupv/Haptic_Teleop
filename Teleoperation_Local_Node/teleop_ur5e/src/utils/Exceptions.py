##### ROBOT CONTROL #######

class IncorrectMapping(Exception):
    def __init__(self, mapping, message="Argument mapping is not in [0 ... 3] range"):
        self.value = mapping
        self.message = "Value {self.value} of mapping is not in [0 ... 3] range"
        super().__init__(self.message)

class IncorrectDuration(Exception):
    def __init__(self, duration, message="Argument duration is not in [0 ... 2] range"):
        self.value = duration
        self.message = "Value {self.value} of duration is not in [0 ... 2] range"
        super().__init__(self.message)

class IncorrectController(Exception):
    def __init__(self, controller, message="Argument controller is not in [0 ... 2] range"):
        self.value = controller
        self.message = "Value {self.value} of controller is not in [0 ... 2] range"
        super().__init__(self.message)

class IncorrectInitialPosition(Exception):
    def __init__(self, initial_pos, message="Argument initialpos is not in [0 ... 8] range"):
        self.value = initial_pos
        self.message = "Value {self.value} of initialpos is not in [0 ... 8] range"
        super().__init__(self.message)

class IncoherentSpecification(Exception):
    def __init__(self, message="Arguments oneaxis and twoaxis were specified simultaneously"):
        self.message = message
        super().__init__(self.message)

class IncorrectOneAxis(Exception):
    def __init__(self, oneaxis, message="Argument oneaxis is not in [0 ... 2] range"):
        self.value = oneaxis
        self.message = "Value {self.value} of oneaxis is not in [0 ... 2] range"
        super().__init__(self.message)

class IncorrectTwoAxis(Exception):
    def __init__(self, twoaxis, message="Argument twoaxis is not in [0 ... 2] range"):
        self.value = twoaxis
        self.message = "Value {self.value} of twoaxis is not in [0 ... 2] range"
        super().__init__(self.message)

class IncorrectVelocity(Exception):
    def __init__(self, velocity, message="Argument velocity is not in [0 ... 2] range"):
        self.value = velocity
        self.message = "Value {self.value} of velocity is not in [0 ... 2] range"
        super().__init__(self.message)

class IncorrectAcceleration(Exception):
    def __init__(self, acceleration, message="Argument acceleration is not in [0 ... 2] range"):
        self.value = acceleration
        self.message = "Value {self.value} of acceleration is not in [0 ... 2] range"
        super().__init__(self.message)

class IncorrectPrecision(Exception):
    def __init__(self, precision, message="Argument precision is not in [0.0 ... 0.12] range"):
        self.value = precision
        self.message = "Value {self.value} of precision is not in [0.0 ... 0.12] range"
        super().__init__(self.message)

class IncorrectWaitTime(Exception):
    def __init__(self, wait_time, message="Argument waittime is not in [0 ... 8] range"):
        self.value = wait_time
        self.message = "Value {self.value} of waittime is not in [0 ... 8] range"
        super().__init__(self.message)

##### HAND TRACKING #######

class IncorrectWidth(Exception):

    def __init__(self, initial_pos, message="Argument width is not in [360 ... 1080] range"):
        self.value = initial_pos
        self.message = "Value {self.value} of width is not in [360 ... 1080] range"
        super().__init__(self.message)

class IncorrectHeight(Exception):

    def __init__(self, initial_pos, message="Argument height is not in [240 ... 1080] range"):
        self.value = initial_pos
        self.message = "Value {self.value} of height is not in [240 ... 1080] range"
        super().__init__(self.message)

class IncorrectDetectionConfidence(Exception):

    def __init__(self, initial_pos, message="Argument min_detection_confidence is not in [0.0 ... 1.0] range"):
        self.value = initial_pos
        self.message = "Value {self.value} of min_detection_confidence is not in [0.0 ... 1.0] range"
        super().__init__(self.message)

class IncorrectTrackingConfidence(Exception):

    def __init__(self, initial_pos, message="Argument min_tracking_confidence is not in [0.0 ... 1.0] range"):
        self.value = initial_pos
        self.message = "Value {self.value} of min_tracking_confidence is not in [0.0 ... 1.0] range"
        super().__init__(self.message)

class IncorrectTolerance(Exception):

    def __init__(self, initial_pos, message="Argument tolerance is not in [0 ... 2] range"):
        self.value = initial_pos
        self.message = "Value {self.value} of tolerance is not in [0 ... 2] range"
        super().__init__(self.message)

class IncorrectTracking(Exception):

    def __init__(self, initial_pos, message="Argument tracking is not in [0 ... 2] range"):
        self.value = initial_pos
        self.message = "Value {self.value} of tracking is not in [0 ... 2] range"
        super().__init__(self.message)

class IncorrectGlovesColor(Exception):

    def __init__(self, initial_pos, message="Argument gloves_color is not in [0 ... 3] range"):
        self.value = initial_pos
        self.message = "Value {self.value} of gloves_color is not in [0 ... 3] range"
        super().__init__(self.message)