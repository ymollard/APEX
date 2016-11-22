#!/home/poppy/miniconda/bin/python
import rospy
from std_msgs.msg import UInt8
from colorsys import hsv_to_rgb

try:
    from neopixel import Adafruit_NeoPixel, Color
except ImportError as e:
    raise ImportError(repr(e) + "\nGet it from https://github.com/ymollard/rpi_ws281x")


class LightController(object):
    LED_COUNT = 43      # Number of LED pixels.
    LED_PIN = 18      # GPIO pin connected to the pixels (must support PWM!).
    LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
    LED_DMA = 5       # DMA channel to use for generating signal (try 5)
    LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
    LED_INVERT = False   # True to invert the signal (when using NPN transistor level shift)

    def __init__(self):
        self.led_count = 43
        self.led_pin = 18
        self.leds = Adafruit_NeoPixel(self.LED_COUNT, self.LED_PIN, self.LED_FREQ_HZ, self.LED_DMA, self.LED_INVERT, self.LED_BRIGHTNESS)

    def cb_light(self, msg):
        hue = msg.data/255.  # Hue between (0., 1.)
        r, g, b = map(int, hsv_to_rgb(hue, 1, 255))
        self.set_all(r, g, b)

    def set_all(self, r, g, b):
        for i in range(self.LED_COUNT):
            self.leds.setPixelColor(i, Color(g, r, b))
        self.leds.show()

    def run(self):
        try:
            self.leds.begin()
        except RuntimeError as e:
            raise RuntimeError(repr(e) + "\nAre you running this script with root permissions?")
        else:
            self.set_all(0, 0, 0)
            rospy.Subscriber("nips2016/environment/light", UInt8, self.cb_light)
            rospy.spin()

if __name__ == '__main__':
    rospy.init_node("light_controlller")
    LightController().run()

