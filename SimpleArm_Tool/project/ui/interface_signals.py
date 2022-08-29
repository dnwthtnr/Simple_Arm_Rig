import importlib

from project import main

importlib.reload( main )




class InterfaceSignals:

	def __init__( self ):

		self.commands = main.DataManager()

		self.finger_locators = [ ]

		self.arm_locators = [ ]

		self.locator_groups = [ ]

		self.control_size = 55

		

	def signal_spawn_locators( self, amount = int( ), limb = [ 'arm', 'finger' ] ):

		locator_list, locator_group = self.commands.spawn_locator( 
												amount = amount, 
												limb = limb 
												)

		self.locator_groups.append( locator_group )
		
		if limb == 'arm':

			for loc in locator_list:

				self.arm_locators.append( loc )

		if limb == 'finger':

			self.finger_locators.append( locator_list )

	def signal_convert_locators( self ):

		self.commands.locator_cleanup( groups = self.locator_groups )

		finger_data = self.commands.spawn_finger_joints( 
														locators = self.finger_locators, 
														controllerSize = self.control_size 
														)

		base_joints, fk_joints, ik_joints, fk_parent_node, pole_vector, ik_wrist, ik_handle = self.commands.spawn_arm_joints( 
																											locators = self.arm_locators, 
																											controllerSize = self.control_size,
																											fingerJoints = finger_data[ 0 ]
																											)

		self.geometry_data = self.commands.spawn_system_blend( 	
													baseJoints = base_joints, 
													fkJoints = fk_joints, 
													ikJoints = ik_joints, 
													fkControls = fk_parent_node, 
													poleVector = pole_vector, 
													ikWrist = ik_wrist, 
													ikHandle = ik_handle,
													fingerJoints = finger_data[ 0 ],
													fingerControls = finger_data[ 1 ], 
													controllerSize = self.control_size 
													)

	def signal_bind_mesh( self ):

		self.commands.bind_mesh( 
								baseJoint = self.geometry_data[ 1 ],
								geoGroup = self.geometry_data[ 0 ]
								)
