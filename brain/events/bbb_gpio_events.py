from ..engine_base.event_base import EventBase, EventRequestBase, EventReplyBase


class EventBBBgpioChangeLedState(EventBase):

	def __init__(self, led_nb: int, led_ON: bool, frequency: int = None, period: int = None, duty_cylce: float = 0.5, **kwargs):
		super().__init__(**kwargs)
		self.led_nb = led_nb
		self.led_ON = led_ON
		if led_ON:
			if frequency is None and period is None:
				self.period = 0.7
			elif period is None:
				self.period = 1 / frequency


class EventBBBgpioTiretteState(EventBase):

	def __init__(self, tirette_pulled, **kwargs):
		super().__init__(**kwargs)
		self.tirette_pulled = tirette_pulled


class RequestBBBgpioTiretteState(EventRequestBase):
	pass


class RequestBBBgpioWaitForTirette(EventRequestBase):
	pass


class ReplyBBBgpioTiretteState(EventReplyBase):

	def __init__(self, tirette_pulled, **kwargs):
		super().__init__(**kwargs)
		self.tirette_pulled = tirette_pulled
