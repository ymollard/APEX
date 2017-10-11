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
            GPIO.setup(self.params['pause_button_pin'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.params['pause_led_pin'], GPIO.OUT)
        else:
            rospy.logwarn("Ergo hasn't found the GPIO, button will not work")

    def switch_led(self, value=None):
        if value is None:
            self.led_on = not self.led_on
        elif value in [True, False]:
            self.led_on = value
        GPIO.output(self.params['pause_led_pin'], self.led_on)

    @property
    def pressed(self):
        if gpio_available:
            return not GPIO.input(self.params['pause_button_pin'])
        else:
            return False
