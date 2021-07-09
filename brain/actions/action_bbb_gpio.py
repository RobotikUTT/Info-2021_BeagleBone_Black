import time
import logging

from ..engine_base.action_base import ActionBase
from ..events.bbb_gpio_events import EventBBBgpioChangeLedState, RequestBBBgpioWaitForTirette, RequestBBBgpioTiretteState


action_logger = logging.getLogger('action')


class ActionWaitForTirette(ActionBase):

	def run(self):
		# If the tirette is not in place at startup
		self.check_for_interruption()
		rep = self.parent_node.publish_request(RequestBBBgpioTiretteState())
		if rep.tirette_pulled:
			action_logger.warning('Tirette not in place at startup.')
			while rep.tirette_pulled:
				self.check_for_interruption()
				time.sleep(0.2)
				self.check_for_interruption()
				self.parent_node.publish_event(EventBBBgpioChangeLedState(0, False))
				time.sleep(0.2)
				self.check_for_interruption()
				rep = self.parent_node.publish_request(RequestBBBgpioTiretteState())
				self.parent_node.publish_event(EventBBBgpioChangeLedState(0, True))
			action_logger.info('Tirette now in place !')
			time.sleep(2)

		# Wait for the tirette to be pulled
		rep = self.parent_node.publish_request(RequestBBBgpioWaitForTirette())

		# GoGoGo !
