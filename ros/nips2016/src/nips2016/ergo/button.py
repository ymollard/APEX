import RPi.GPIO as GPIO

class Button(object):
    def __init__(self, params):
        self.params = params
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.params['button_pin'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    @property
    def pressed(self):
        return not GPIO.input(self.params['button_pin'])
