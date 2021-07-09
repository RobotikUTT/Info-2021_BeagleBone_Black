import time
import logging
from queue import deque
import Adafruit_BBIO.GPIO as GPIO

from ..engine_base.node_base import NodeBase, subscribe_to_event
from ..events.bbb_gpio_events import EventBBBgpioChangeLedState, EventBBBgpioTiretteState, RequestBBBgpioWaitForTirette, RequestBBBgpioTiretteState, ReplyBBBgpioTiretteState

bbb_gpio_logger = logging.getLogger('BBB_GPIO')


# Leds from left to right
PIN_LED_0_RED = 'P9_23'
PIN_LED_1_PINK = 'P9_25'
PIN_LED_2_GREEN = 'P9_27'
PIN_LED_3_RED = 'P9_30'
PIN_LED_4_RED = 'P8_17'
PIN_LED_5_RED = 'P8_18'
PIN_LED_6_EMPTY = 'P8_12'
PIN_LED_7_EMPTY = 'P8_15'

leds_pin = [
	PIN_LED_0_RED, PIN_LED_1_PINK,
	PIN_LED_2_GREEN, PIN_LED_3_RED,
	PIN_LED_4_RED, PIN_LED_5_RED,
	PIN_LED_6_EMPTY, PIN_LED_7_EMPTY
]

# GPIO from left to right (square pads are GND)
PIN_0_COLOR_SWITCH = 'P8_7'
PIN_1_TIRETTE = 'P8_8'
pin_2_BAU = 'P8_9'


class BBBgpioNode(NodeBase):
	tirette_pulled = False
	pending_requests_wait_for_tirette = None

	def __init__(self, **kwargs):
		super().__init__(**kwargs)
		self.pending_requests_wait_for_tirette = deque()
		for pin in [PIN_LED_0_RED, PIN_LED_1_PINK, PIN_LED_2_GREEN, PIN_LED_3_RED, PIN_LED_4_RED, PIN_LED_5_RED, PIN_LED_6_EMPTY, PIN_LED_7_EMPTY]:
			GPIO.setup(pin, GPIO.OUT)
		for pin in [PIN_0_COLOR_SWITCH, PIN_1_TIRETTE, pin_2_BAU]:
			GPIO.setup(pin, GPIO.IN)

	def start(self):
		super().start()
		self.tirette_pulled = GPIO.input(PIN_1_TIRETTE)
		# self.add_event_detect(None)
		if not self.tirette_pulled:
			GPIO.add_event_detect(PIN_1_TIRETTE, GPIO.RISING, callback=self.on_tirette_pull, bouncetime=100)
		else:
			GPIO.add_event_detect(PIN_1_TIRETTE, GPIO.FALLING, callback=self.on_tirette_push, bouncetime=100)

	def on_tirette_pull(self, pin):
		GPIO.remove_event_detect(PIN_1_TIRETTE)
		self.tirette_pulled = True
		self.publish_event(EventBBBgpioTiretteState(self.tirette_pulled))
		while self.pending_requests_wait_for_tirette:
			req = self.pending_requests_wait_for_tirette.pop()
			rep = ReplyBBBgpioTiretteState(True)
			self.reply_to_request(req, rep)
		bbb_gpio_logger.info('Tirette pulled !')
		time.sleep(1)
		GPIO.add_event_detect(PIN_1_TIRETTE, GPIO.FALLING, callback=self.on_tirette_push, bouncetime=100)

	def on_tirette_push(self, pin):
		GPIO.remove_event_detect(PIN_1_TIRETTE)
		self.tirette_pulled = False
		self.publish_event(EventBBBgpioTiretteState(self.tirette_pulled))
		bbb_gpio_logger.info('Tirette back in place.')
		time.sleep(1)
		GPIO.add_event_detect(PIN_1_TIRETTE, GPIO.RISING, callback=self.on_tirette_pull, bouncetime=100)

	@subscribe_to_event(EventBBBgpioTiretteState)
	def add_event_detect(self, ev):
		pass
		# time.sleep(1)
		# if not self.tirette_pulled:
		# 	GPIO.add_event_detect(PIN_1_TIRETTE, GPIO.RISING, callback=self.on_tirette_pull, bouncetime=100)
		# else:
		# 	GPIO.add_event_detect(PIN_1_TIRETTE, GPIO.FALLING, callback=self.on_tirette_push, bouncetime=100)


	@subscribe_to_event(EventBBBgpioChangeLedState)
	def change_led_state(self, ev):
		GPIO.output(leds_pin[ev.led_nb], GPIO.HIGH if ev.led_ON else GPIO.LOW)

	@subscribe_to_event(RequestBBBgpioTiretteState)
	def reply_tirette_state(self, req):
		rep = ReplyBBBgpioTiretteState(self.tirette_pulled)
		self.reply_to_request(req, rep)

	@subscribe_to_event(RequestBBBgpioWaitForTirette)
	def handle_request_wait_for_tirette(self, req):
		if self.tirette_pulled:
			rep = ReplyBBBgpioTiretteState(True)
			self.reply_to_request(req, rep)
		else:
			self.pending_requests_wait_for_tirette.append(req)
