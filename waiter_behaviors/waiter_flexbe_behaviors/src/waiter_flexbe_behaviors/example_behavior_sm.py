#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from waiter_flexbe_states.global_looking_state import GlobalLookingState
from waiter_flexbe_states.holding_state import HoldingState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Aug 21 2015
@author: Philipp Schillinger
'''
class ExampleBehaviorSM(Behavior):
	'''
	This is a simple example for a behavior.
	'''


	def __init__(self):
		super(ExampleBehaviorSM, self).__init__()
		self.name = 'Example Behavior'

		# parameters of this behavior
		self.add_parameter('waiting_time', 3)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		log_msg = "Hello World!"
		# x:83 y:390
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:185 y:134
			OperatableStateMachine.add('Holding',
										HoldingState(prefix="/hook/camera", hold_time=self.waiting_time),
										transitions={'success': 'GlobalLooking', 'failed': 'finished'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:489 y:196
			OperatableStateMachine.add('GlobalLooking',
										GlobalLookingState(prefix="/hook/camera"),
										transitions={'success': 'finished', 'aborted': 'finished'},
										autonomy={'success': Autonomy.Off, 'aborted': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
