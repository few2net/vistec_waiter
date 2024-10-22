#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from waiter_flexbe_states.global_move_state import GlobalMoveState
from waiter_flexbe_states.home_move_state import HomeMoveState
from waiter_flexbe_states.idle_state import IdleState
from waiter_flexbe_states.local_move_state import LocalMoveState
from waiter_flexbe_states.looking_state import LookingState
from waiter_flexbe_states.wait_state import WaitState as waiter_flexbe_states__WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Aug 21 2015
@author: Philipp Schillinger
'''
class testbehavior01SM(Behavior):
	'''
	This is a simple example for a behavior.
	'''


	def __init__(self):
		super(testbehavior01SM, self).__init__()
		self.name = 'test behavior 01'

		# parameters of this behavior
		self.add_parameter('prefix', '/hook/camera')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		log_msg = "Hello World!"
		# x:30 y:560
		_state_machine = OperatableStateMachine(outcomes=['finished'])
		_state_machine.userdata.global_pos = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:246 y:60
			OperatableStateMachine.add('Idle',
										IdleState(prefix="/hook/camera"),
										transitions={'home': 'Homing', 'look': 'Look', 'abort': 'Idle'},
										autonomy={'home': Autonomy.Off, 'look': Autonomy.Off, 'abort': Autonomy.Off})

			# x:54 y:273
			OperatableStateMachine.add('Homing',
										HomeMoveState(prefix="/hook/camera"),
										transitions={'success': 'Idle', 'abort': 'finished'},
										autonomy={'success': Autonomy.Off, 'abort': Autonomy.Off})

			# x:757 y:611
			OperatableStateMachine.add('Local move',
										LocalMoveState(prefix="/hook/camera", max_retry=5, tolerance=1.7),
										transitions={'error': 'Local move', 'abort': 'Idle', 'success': 'Wait'},
										autonomy={'error': Autonomy.Off, 'abort': Autonomy.Off, 'success': Autonomy.Off},
										remapping={'global_pos': 'global_pos'})

			# x:757 y:53
			OperatableStateMachine.add('Look',
										LookingState(prefix="/hook/camera", wait_duration=20),
										transitions={'home': 'Homing', 'has_global': 'Global move', 'abort': 'Idle', 'clear': 'Idle', 'timeout': 'Idle'},
										autonomy={'home': Autonomy.Off, 'has_global': Autonomy.Off, 'abort': Autonomy.Off, 'clear': Autonomy.Off, 'timeout': Autonomy.Off},
										remapping={'global_pos': 'global_pos'})

			# x:366 y:609
			OperatableStateMachine.add('Wait',
										waiter_flexbe_states__WaitState(prefix="/hook/camera"),
										transitions={'abort': 'Idle', 'timeout': 'Look', 'success': 'Homing', 'failed': 'Idle'},
										autonomy={'abort': Autonomy.Off, 'timeout': Autonomy.Off, 'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1050 y:254
			OperatableStateMachine.add('Global move',
										GlobalMoveState(prefix="/hook/camera", max_retry=3, tolerance=1.7),
										transitions={'abort': 'Idle', 'home': 'Homing', 'has_local': 'Local move', 'retry': 'Global move', 'wait': 'Look', 'error': 'Idle', 'success': 'Homing', 'failed': 'Homing'},
										autonomy={'abort': Autonomy.Off, 'home': Autonomy.Off, 'has_local': Autonomy.Off, 'retry': Autonomy.Off, 'wait': Autonomy.Off, 'error': Autonomy.Off, 'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'global_pos': 'global_pos'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
