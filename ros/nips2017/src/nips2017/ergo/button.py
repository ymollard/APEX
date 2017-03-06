import rospy
try:
    import RPi.GPIO as GPIO
except ImportError:
    gpio_available = False
else:
    gpio_available = True


class Button(object):
    def __init__(self, params):
        self.params = params
        if gpio_available:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.params['button_pin'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
        else:
            rospy.logwarn("Ergo hasn't found the GPIO, button will not work")
    
    @property
    def pressed(self):
        if gpio_available:
            return not GPIO.input(self.params['button_pin'])
        else:
            return False
