import maya.cmds as cmds

# TODO: skin weights
# TODO: UI to go through rigging and skin teps

class DataManager:

	def __init__( self, verbosity = 1 ):

		# used to control how much output is given
		self.verbosity = verbosity
		
		self.arm_locators = [ ]

		self.finger_locators = [ ]
	
	# spawn locators for use defining joint locations: RETURNS LIST of arm locators / LIST of LISTS holding finger locators
	def spawn_locator( self, amount = 3, limb = [ 'arm', 'finger' ] ):

		fingerNestList = [ ]
		
		for count, i in enumerate( range( amount ) ):

			# if arm positions locators on first 3 grid spaces
			if limb == 'arm':

				position = ( count, 0, 0 )

				name = 'arm_loc_0%s' % ( count + 1 )

			# if finger positions locators 
			if limb == 'finger':
				
				# locs need to be closer together
				locationFactor = ( count + 1 ) * .2

				name = 'finger0%s_loc_0%s' % ( len( self.finger_locators ), count + 1 )

				# if theres an arm position locators after wrist
				if len( self.arm_locators ) > 0:

					# wrist x translate
					endJointLocationX = self.get_object_position( 
							inputList = self.arm_locators )[ -1 ][ 0 ]

					# add translate factor to wrists position
					positionX = locationFactor + endJointLocationX

					# position if an arm is present
					position = ( positionX, 0, 0 )

				else:

					# position if there is no arm
					position = ( locationFactor, 0, 0 )
			
			# create locator
			loc = cmds.sphere( 
					name = name, 
					radius = .4  )

			# center pivot
			cmds.xform( 
					loc, 
					centerPivots = True )

			# set position
			cmds.xform( 
					loc, 
					translation = position )

			# if arm being created add locs to list
			if limb == 'arm':

				self.arm_locators.append( loc[ 0 ] )

			# if finger being created add locs to list
			if limb == 'finger':

				# scale down locs
				cmds.xform( 
						loc, 
						scale = [ .2, .2, .2 ] )

				fingerNestList.append( loc[ 0 ] )

		# return arm list if present
		if limb == 'arm':

			print( self.arm_locators )

			# put locs in group
			locGroup = cmds.group( 
					self.arm_locators, 
					name = 'arm_loc_GRP' )

			# get base loc position
			pivotLocation = cmds.xform( 
					self.arm_locators[ 0 ], 
					query = True, 
					translation = True, 
					worldSpace = True )
			
			# move group pivot to 1st loc center
			cmds.move( 
					pivotLocation[ 0 ], 
					pivotLocation[ 1 ],
					pivotLocation[ 2 ], 
					locGroup + '.scalePivot', 
					locGroup + '.rotatePivot', 
					absolute=True )

			if self.verbosity > 0:

				print( 'armGroup : %s' % ( locGroup ) )
		
			return self.arm_locators, locGroup
		
		# nest finger loc list in list holding all fingers
		else:

			self.finger_locators.append( fingerNestList )

			# put locs in group
			locGroup = cmds.group( 
					fingerNestList, 
					name = 'finger0%s_loc_GRP' % ( len( self.finger_locators ) - 1 ) )

			# get base loc position
			pivotLocation = cmds.xform( 
					fingerNestList[ 0 ], 
					query = True, 
					translation = True, 
					worldSpace = True 
					)
			
			# move group pivot to 1st loc center
			cmds.move( 
					pivotLocation[ 0 ], 
					pivotLocation[ 1 ],
					pivotLocation[ 2 ],
					locGroup + '.scalePivot', 
					locGroup + '.rotatePivot', 
					absolute=True )
			
			if self.verbosity > 0:

				print( 'fingerGroup : %s' % ( locGroup ) )

			return fingerNestList, locGroup

	# clears transforms on finger and arm groups
	def locator_cleanup( self, groups = [ list( ), str( ) ] ):

		# if list
		if isinstance( groups, list ):

			# for group in list of groups
			for group in groups:

				self.freeze_transformations( 
						input = group )
		else:

			self.freeze_transformations( input = groups
				 )
	
	# spawn finger joints : RETURNS LIST of LISTS containing all finger joints and base nodes
	def spawn_finger_joints( self, locators = [ ], controllerSize = int( ) ):

		# note: locators is a list of lists: containing each finger
		
		# if no fingers present then pass
		if len( locators ) < 1:

			pass
		
		# will hold all finger lists
		fingersJointList = [ ]

		# will hold all finger controller lists
		fingersControllerList = [ ]

		# will hold first index of finger lists to get parent group
		finger_locators = [ ]

		# for finger locators in finger list
		for count, finger in enumerate( locators ):

			name = ( 'finger0%s' % ( count ) )
			
			# create and align joints for each finger
			fingerJointList = self.spawn_joints( 
					inputList = finger, 
					name = name, 
					alignToList = True )
		
			# take chain out of hierarchy
			self.hierarchy_remove( 
					input = fingerJointList[ 0 ] )

			self.freeze_transformations( 
					input = fingerJointList )

			fingersJointList.append( fingerJointList )

			# populate finger_locators with first index of each finger
			finger_locators.append( finger[ 0 ] )
				
		# orient all finger joints
		self.orient_joints( 
				jointLists = fingersJointList,
				tipJointLocation = None,
				tipJoint = True )

		# for each finger base in list of finger bases
		for fingerIndex in finger_locators:
			
			# get parent of finger base
			locatorGroup = self.get_object_parent( 
					input = fingerIndex, 
					type = 'None' )

			# delete parent group of finger base
			cmds.delete( locatorGroup )

		# create controllers for joint chain
		fingersControllerList = self.spawn_finger_controls( 
				inputJoints = fingersJointList, 
				controllerSize = controllerSize )

		return [ fingersJointList, fingersControllerList ]

	# create controls for fingers and constrain: RETURNS LIST of LISTS holding controllers for each finger
	def spawn_finger_controls( self, inputJoints = [ ], controllerSize = int( ) ):

		# controller scale
		fingerControlSize = controllerSize * .08
		
		# list to hold compensation groups
		compensateNodeList = [ ]

		returnList = [ ]

		# controller list - holds multiple lists of finger controllers
		fingerControlsList = [ ]

		# for finger joints in finger joints list
		for count, fingerJoints in enumerate( inputJoints ):

			if self.verbosity > 0:

				print( 'spawn_finger_controls:::\ncount: %s \nfingerJoints: %s\ninputJoints: %s' % ( count, fingerJoints, inputJoints ) )

			# finger name
			name = ( 'finger_0%s' % ( count ) )
		
			# create controls for single finger
			fingerControls = self.spawn_controls( 
					inputList = fingerJoints,
					name = name,
					alignToList = True,
					curveRotation = [ 0, 90, 0 ],
					curveSize = fingerControlSize )

			# set each control yellow
			for ctrl in fingerControls:

				self.set_color( 
						item = ctrl, 
						color = [ 1, 1, 0 ] )
			
			# add single finger controls to list of all finger controls
			fingerControlsList.append( fingerControls )

			
			# for current iterating finger control and joint
			for i, ( parentItem, childItem ) in enumerate( zip( fingerControls, fingerJoints ) ):

				name = ( 'finger%s_control_constraint_0%s' % ( count, i ) )

				# constrain control to joint
				cmds.parentConstraint( 
						parentItem, 
						childItem, 
						maintainOffset = True, 
						name = name )

			# create compensation nodes for each controller per finger
			compensateNodeList = self.spawn_compensation_node( 
					target = fingerControls )

			returnList.append( compensateNodeList[ 0 ] )
			
			# parent current fingers controllers into hierarchy
			self.parent_list( 
					inputList = compensateNodeList, 
					transformNodePresent = True 
					)

		if self.verbosity > 0:

				print( ':::spawn_finger_controls:::\nreturnList: %s' % ( returnList ) )
		
		return returnList

	# get loc position and spawn joints/controllers: RETURNS 5 LISTS - [ base_joints, fk_joints, ik_joints, fk_controls, ik_controls ]
	def spawn_arm_joints( self, locators = [ ], controllerSize = int( ), fingerJoints = [ ] ):

		if self.verbosity > 0:

			print( 'locators : %s' % ( locators ) )
			print( 'locatorParent: %s' % ( self.get_object_parent( input = locators[ 0 ], type = 'None' ) )  )

		# create and align joints
		baseJointList = self.spawn_joints( 
				inputList = locators, 
				name = 'arm_BASE', 
				alignToList = True )
		
		# take chain out of hierarchy
		self.hierarchy_remove( 
				input = baseJointList[ 0 ] )

		# freeze transforms
		self.freeze_transformations( 
				input = baseJointList )

		if self.verbosity > 0:

			print( 'locators : %s' % ( baseJointList[ 0 ] ) )
			print( 'locatorParent: %s' % ( self.get_object_parent( input = baseJointList[ 0 ], type = 'None' ) )  )

		# create ik joints for each arm loc
		ikJointList = self.spawn_joints( 
				inputList = locators, 
				name = 'arm_IK', 
				alignToList = True )

		# parent joints to world
		self.hierarchy_remove( 
				input = ikJointList[ 0 ] )

		# freeze transforms
		self.freeze_transformations( 
				input = ikJointList )

		# create fk joints for each arm loc
		fkJointList = self.spawn_joints( 
				inputList = locators, 
				name = 'arm_FK', 
				alignToList = True )

		# parent joints to world
		self.hierarchy_remove( 
				input = fkJointList[ 0 ] )

		# freeze transforms
		self.freeze_transformations( 
				input = fkJointList )
		
		# get hand location to orient wrist
		if len( fingerJoints ) > 0:
			
			# lists to hold joint positions
			fingerLocationX = [ ]

			fingerLocationY = [ ]

			fingerLocationZ = [ ]

			print( fingerJoints  )
			
			# for finger joints in list of all finger joints
			for finger in fingerJoints:

				if self.verbosity > 0:

					print( ':::LocatorToJoints:::\nfingerJoints: %s \nfinger: %s' % ( fingerJoints, finger ) )

				# position of finger base joint
				fingerBaseJointLoc = cmds.xform( 
						finger[ 0 ], query = True,  translation = True )

				# add individual values to corresponding lists
				fingerLocationX.append( 
						fingerBaseJointLoc[ 0 ] )

				fingerLocationY.append( 
						fingerBaseJointLoc[ 1 ] )

				fingerLocationZ.append( 
						fingerBaseJointLoc[ 2 ] )
			
			fingerLocationList = [ 
					fingerLocationX, 
					fingerLocationY, 
					fingerLocationZ 
					]
			
			# final hand position
			handPosition = [ ]
			
			# for list in lists
			for locationList in fingerLocationList:

				if self.verbosity > 0:

					print( ':::LocatorToJoints:::\nfingerLocationList: %s \nlocationList: %s' % ( 
																								fingerLocationList, 
																								locationList ) )

				# location equals average
				location = sum( locationList ) / len( locationList )

				handPosition.append( location )

			if self.verbosity > 0:

				print( ':::LocatorToJoints:::\nhandPosition: %s' % ( handPosition ) )

			tipJointLocation = handPosition

			# create hand joint for each chain
			handBaseJoint = cmds.joint( 
					name = 'hand_BASE_JNT', 
					position = tipJointLocation )
			
			# parent joints to world
			self.hierarchy_remove( 
					input = handBaseJoint )

			# freeze transforms
			self.freeze_transformations( 
					input = handBaseJoint )

			# parent hand under wrist
			cmds.parent( 
					handBaseJoint, 
					baseJointList[ -1 ] )

			# add to base joint list
			baseJointList.append( handBaseJoint )
			
			# FK
			handFkJoint = cmds.joint( 
					name = 'hand_FK_JNT', 
					position = tipJointLocation )
			
			# parent joints to world
			self.hierarchy_remove( 
					input = handFkJoint )

			# freeze transforms
			self.freeze_transformations( 
					input = handFkJoint )

			# parent hand under wrist
			cmds.parent( 
					handFkJoint, 
					fkJointList[ -1 ] )

			# add to base joint list
			fkJointList.append( handFkJoint )
			
			# IK
			handIkJoint = cmds.joint( 
					name = 'hand_IK_JNT', 
					position = tipJointLocation )

			# parent joints to world
			self.hierarchy_remove( 
					input = handIkJoint )

			# freeze transforms
			self.freeze_transformations( 
					input = handIkJoint )

			# parent hand under wrist
			cmds.parent( 
					handIkJoint, 
					ikJointList[ -1 ] )

			# add to base joint list
			ikJointList.append( handIkJoint )
		
		# hold all joint lists
		allJointsList = [ 
				baseJointList, 
				fkJointList, 
				ikJointList 
				]
		
		# if there are fingers
		if len( fingerJoints )  > 0:

			tipJoint = False
		
		else:

			tipJoint = True

			tipJointLocation = None
		
		# orient joints
		self.orient_joints( 
				jointLists = allJointsList,
				tipJointLocation = tipJointLocation,
				tipJoint = tipJoint
				)

		
		# get parent of first of locators in list
		locatorGroup = self.get_object_parent( 
				input = locators[ 0 ], 
				type = 'None' )
		
		# delete locator group
		cmds.delete( 
				locatorGroup )
		
		# spawn FK controls
		fkTopNode = self.spawn_controls_fk( 
				inputJoints = fkJointList[ 0:3 ], 
				controllerSize = controllerSize )

		if self.verbosity > 0:

			print( ':::LocatorToJoints:::\nfkTopNode: %s' % ( fkTopNode ) )

		# spawn IK controls
		poleVector, ikWrist, ikHandle = self.spawn_controls_ik( 
				inputJoints = ikJointList[ 0:3 ], 
				controllerSize = controllerSize )


		
		return baseJointList, fkJointList, ikJointList, fkTopNode[ 0 ], poleVector[ 0 ], ikWrist[ 0 ], ikHandle
	
	# creates fk controls and constrains them to joints: RETURNS STRING name of top node in fk control hierarchy
	def spawn_controls_fk( self, inputJoints = [ ], controllerSize = int( ) ):

		# list to hold compensation groups
		compensateNodeList = [ ]

		fkControllerList = [ ]

		controlScale = controllerSize * .2

		fkControllerList = self.spawn_controls( 
				inputList = inputJoints,
				name = 'arm_FK',
				alignToList = True,
				curveRotation = [ 0, 90, 0 ],
				curveSize = controlScale )

		# set each control red
		for ctrl in fkControllerList:

			self.set_color( 
					item = ctrl, 
					color = [ 1, 0, 0 ] )
		
		# constrain fk controls to joints
		for count, ( parentItem, childItem ) in enumerate( zip( fkControllerList, inputJoints ) ):

			name = ( 'FK_control_constraint_0%s' % ( count ) )

			cmds.parentConstraint( 
					parentItem, 
					childItem, 
					maintainOffset = True, 
					name = name )

		# create compensation nodes for each controller
		compensateNodeList = self.spawn_compensation_node( 
				target = fkControllerList )

		# parent controllers into hierarchy
		self.parent_list( 
				inputList = compensateNodeList, 
				transformNodePresent = True )

		return compensateNodeList
	
	# creates ik controls and constrains them to joints: RETURNS 3 STR of each controller
	def spawn_controls_ik( self, inputJoints = [ ], controllerSize = int( ) ):

		ikControlList = [ ]
		
		ikJointPositionList = self.get_object_position( 
				inputList = inputJoints )

		wristControlScale = controllerSize * .2

		vectorControlScale = controllerSize * .1



		# get start ik joint
		startJoint = inputJoints[ 0 ]
		
		# get final ik joint
		endJoint = inputJoints[ -1 ]

		# get middle index
		middleIndex = self.get_index_middle( 
				inputList = inputJoints )

		# middle joint
		middleJoint = inputJoints[ middleIndex ]

		# set pole vector location 
		poleVectorLocation = self.list_math( 
				inputList = ikJointPositionList[ middleIndex ], 
				operation = 'subtraction', 
				targetIndex = 2, 
				mathFactor = 3 )

		# set control names
		poleVectorName = 'poleVector_IK_CTRL'

		wristControlName = 'wrist_IK_CTRL'
		
		# create wrist control
		wristControl = cmds.circle( 
				name = wristControlName )
		
		# rotate and scale control
		cmds.xform( 
				wristControl, 
				rotation = [ 0, 90, 0 ], 
				
				scale = [ 
						wristControlScale,
						 wristControlScale, 
						 wristControlScale 
						 ] )

		# freeze transforms
		self.freeze_transformations( 
				input = wristControl )

		# align ctrl to joint position
		self.object_align( 
				target = wristControl, 
				destination = endJoint )
		
		# create pole vector control
		poleVectorControl = cmds.circle( 
				name = poleVectorName )

		# align pole vector ctrl to elbow and translate back
		cmds.xform( 
				poleVectorControl, 
				translation = poleVectorLocation, 
				scale = [ 
						vectorControlScale, 
						vectorControlScale, 
						vectorControlScale ] )

		# freeze transforms
		self.freeze_transformations( 
				input = poleVectorControl )

		# delete creation history
		cmds.delete( 
				wristControl, 
				poleVectorControl, 
				constructionHistory = True )

		poleVectorControl = poleVectorControl[ 0 ]

		wristControl = wristControl[ 0 ]

		# set ctrls color blue
		self.set_color( 
				item = poleVectorControl, 
				color = [ 0, 0, 1 ] )

		self.set_color( 
				item = wristControl, 
				color = [ 0, 0, 1 ] )

		# add controls to list
		ikControlList.append( poleVectorControl )

		ikControlList.append( wristControl )
		
		# create compensation nodes
		compensationNode = self.spawn_compensation_node( 
				target = ikControlList )

		# set ik name before creation so it can be used for constraining
		ikHandleName = 'arm_IK_handle'
		
		# create ikHandle
		ikHandle = cmds.ikHandle( 
				name = ikHandleName, 
				startJoint = startJoint, 
				endEffector = endJoint )

		if self.verbosity > 0:

			print( ':::spawn_controls_ik:::\nikhandle : %s' % ( ikHandle ) )
			print( ':::spawn_controls_ik:::\nikHandle_effectedJoints: %s' % ( cmds.ikHandle( 
																				ikHandleName,
																				query = True, 
																				jointList = True
																				) ) )

		# pole vector constraint
		cmds.poleVectorConstraint( 
				poleVectorControl, 
				ikHandleName, 
				name = 'arm_poleVector_constraint' )

		# constrain wrist controller
		cmds.pointConstraint( 
				wristControl, 
				ikHandleName, 
				maintainOffset = True, 
				name = 'ik_point_constraint' )

		cmds.orientConstraint( 
				wristControl, 
				endJoint, 
				maintainOffset = False, 
				name = 'wrist_orient_constraint' )

		# return poleVector, ikWrist, ikHandle
		return compensationNode[ 0 ], compensationNode[ 1 ], ikHandleName

	# creates ikfk blend: RETURN GROUP
	def spawn_system_blend( 
					self, 
					verbosity = 1, 
					baseJoints = [ ], 
					fkJoints = [ ], 
					ikJoints = [ ], 
					fkControls = [ list( ), str( ) ],
					poleVector = [ ], 
					ikWrist = [ ], 
					ikHandle = str( ), 
					fingerJoints = [ ], 
					fingerControls = [ ], 
					controllerSize = int( ) 
					):

		# blender scale
		blenderControlScale = controllerSize * 1.5

		if len( fingerJoints ) > 0:
			
			print( fingerControls )

			# all controls in 1 list
			condensedFingerControls = [ ]

			# if fingers present
			if len( fingerControls ) > 0:

				# for list of one fingers controls in a list of all fingers controls
				for count, fingerControl in enumerate( fingerControls ):

					condensedFingerControls.append( fingerControl[ 0 ] )
				
				# group finger base nodes
				fingerGroup = cmds.group( 
						condensedFingerControls, 
						name = 'fingers_CTRL_GRP' )

			# for finger list in all finger lists
			for listObject in fingerJoints:

				if self.verbosity > 0:

					print( 'listObject : %s' % ( listObject ) )

				# add finger joints under wrist
				cmds.parent( 
						listObject[ 0 ], 
						baseJoints[ -1 ] )

			# parent group to base joint
			cmds.parentConstraint( 
					baseJoints[ -1 ], 
					fingerGroup, 
					maintainOffset = True )
		
		blenderAttrLong = 'IK_FK_Blender'
		
		# creat blender
		blenderHistory = cmds.circle( 
				name = 'arm_blender_CTRL' )

		# offset right above the wrist
		blenderLocation = [ 
							0, 
							0, 
							0 
							]

		# set blender position
		cmds.xform( 
				blenderHistory, 
				translation = blenderLocation, 
				scale = [ blenderControlScale, blenderControlScale, 0.1 ], 
				rotation = [ 90, 0, 0 ] )

		# clear blender transforms
		cmds.makeIdentity( 
				blenderHistory, 
				apply = True, 
				rotate = True, 
				translate = True, 
				scale = True )
		
		# delete blender history
		cmds.delete( 
				blenderHistory, 
				constructionHistory = True )

		# blender becomes list after history delete
		blender = blenderHistory[ 0 ]

		self.set_color( 
				item = blender, 
				color = [ 1, 1, 1 ] )

		# get all blender attributes
		blenderAttributes = cmds.listAttr( 
				blender, 
				keyable = True )

		if self.verbosity > 0:

			print( 'blenderAttributes : %s' % ( blenderAttributes ) )
		
		# for attr in list of all attributes
		for attribute in blenderAttributes:

			attributeFullName = '%s.%s' % ( blender, attribute )
			
			# lock and hide all attributes
			cmds.setAttr( 
					attributeFullName, 
					lock = True, 
					keyable = False, 
					channelBox = False )

		# add blender attribute
		cmds.addAttr( 
				blender, 
				longName = blenderAttrLong, 
				attributeType = 'float', 
				minValue = 0.0, 
				maxValue = 1.0, 
				hidden = False, 
				keyable = True, 
				readable = True, 
				writable = True )

		# blender attributes reference name
		blenderAttr = '%s.%s' % ( blender, blenderAttrLong )

		# if fkControls is list
		if isinstance( fkControls, list ):
		
			# link blender to fk ctrl visibility
			for ctrl in fkControls:

				visAttr = '%s.visibility' % ( ctrl )
				
				cmds.connectAttr( 
						blenderAttr, 
						visAttr )

		else:

			visAttr = '%s.visibility' % ( fkControls )

			cmds.connectAttr( 
					blenderAttr, 
					visAttr )
		
		# create reverse node to connect to ik controls
		reversedBlender = cmds.createNode( 
				'reverse', 
				name = 'blenderReverse' )

		# connect blender to reverse
		cmds.connectAttr( 
				blenderAttr, 
				'%s.inputX'  % ( reversedBlender ) )

		# reverse output
		reversedOutput = '%s.outputX' % ( reversedBlender )

		# link blender to ik ctrl visibility
		for ctrl in [poleVector, ikWrist]:

			visAttr = '%s.visibility' % ( ctrl )
			
			cmds.connectAttr( 
					reversedOutput, 
					visAttr )
		
		# hook up ik and fk blend
		for count, joint in enumerate( baseJoints[ 0 : 3 ] ):

			if self.verbosity > 0:

				print( 'count : %s \njoint : %s' % ( count, joint ) )

			# create blend color for each joint
			blendColors = cmds.createNode( 
					'blendColors', 
					name = 'valueBlender_0%s' % ( count ) )

			blendColor1 = ( '%s.color1' % ( blendColors ) )

			blendColor2 = ( '%s.color2' % ( blendColors ) )

			blendColorsBlender = ( '%s.blender' % ( blendColors ) )

			blendColorsOutput = ( '%s.output' % ( blendColors ) )

			# base joints rotate
			baseRotateAttr = '%s.rotate' % joint
			
			# joints rotate attr
			fkRotateAttr = '%s.rotate' % fkJoints[ count ]

			ikRotateAttr = '%s.rotate' % ikJoints[ count ]

			# connect blender
			cmds.connectAttr( 
					blenderAttr, 
					blendColorsBlender )

			# connect rotation to blend
			cmds.connectAttr( 
					fkRotateAttr, 
					blendColor1 )

			cmds.connectAttr( 
					ikRotateAttr, 
					blendColor2 )

			# connect output to base
			cmds.connectAttr( 
					blendColorsOutput, 
					baseRotateAttr )

		# group ik and fk joints together
		blendGroup = cmds.group( 
				fkJoints[ 0 ],
				ikJoints[ 0 ],
				name = 'blend_joint_GRP')
		
		# put all controls in 1 group for cleanup
		controls = [ ]
		
		# get fk control parent
		controls.append( fkControls[ 0 ] )

		# add ik fk blender
		controls.append( blender )

		# if fingers present
		if len( fingerControls ) > 0:
			
			# add finger group
			controls.append( fingerGroup )

		# add ikWrist
		controls.append( ikWrist )

		# add poleVector
		controls.append( poleVector )
		
		if self.verbosity > 0:

			print( ':::spawn_system_blend:::\nikhandle : %s' % ( ikHandle ) )
			print( ':::spawn_system_blend:::\nikHandle_effectedJoints: %s' % ( cmds.ikHandle( 
																				ikHandle,
																				query = True, 
																				jointList = True
																				) ) )
		
		geoGroup = self.cleanup_scene( 
				joints = baseJoints[ 0 ], 
				controls = controls, 
				dontTouch =  [ ikHandle, blendGroup ] )

		return [ geoGroup, baseJoints[ 0 ]]

	# groups scene: RETURNS GROUP
	def cleanup_scene( self, joints = [ ], controls = [ ], dontTouch = [ list( ), str( ) ] ):

		self.set_color( 
				item = controls[ 0 ], 
				color = [ 1, 0, 0 ] )
		
		# group
		dontTouchGroup = cmds.group( 
				dontTouch, 
				name = 'DONT_TOUCH' )

		jointGroup = cmds.group( 
				joints, 
				name = 'JNT' )
		
		controlGroup = cmds.group( 
				controls, 
				name = 'CTRL' )

		geoGroup = cmds.group( 
				empty = True, 
				name = 'GEO' )

		rigGroup = cmds.group( 
				[ 
				jointGroup, 
				controlGroup, 
				dontTouchGroup, 
				geoGroup ], 
				name = 'RIG' )

		# create CTRL display layer
		controlLayer = cmds.createDisplayLayer( 
				name = 'CTRL_layer', 
				empty = True,  )

		# add group to layer
		cmds.editDisplayLayerMembers( 
				controlLayer, 
				controlGroup )

		# create JNT display layer
		jointLayer = cmds.createDisplayLayer( 
				name = 'JNT_layer', 
				empty = True, )

		# add group to layer
		cmds.editDisplayLayerMembers( 
				jointLayer, 
				jointGroup )
		
		cmds.setAttr( 
				jointLayer + '.visibility', 
				0 )

		# create DONT_TOUCH display layer
		dontTouchLayer = cmds.createDisplayLayer( 
				name = 'DONT_TOUCH_layer', 
				empty = True )

		# add group to layer
		cmds.editDisplayLayerMembers( 
				dontTouchLayer, 
				dontTouchGroup )
		
		cmds.setAttr( 
				dontTouchLayer + '.visibility', 
				0 )

		# create GEO display layer
		geoLayer = cmds.createDisplayLayer( 
				name = 'GEO_layer', 
				empty = True )

		# add group to layer
		cmds.editDisplayLayerMembers( 
				geoLayer, 
				geoGroup )
		
		cmds.setAttr( 
				geoLayer + '.displayType', 
				2 )
		
		return geoGroup
	
	# binds base joints to mesh: NO RETURN
	def bind_mesh( self, baseJoint = str( ), geoGroup = str( ) ):
		
		mesh = cmds.ls( 
						sl = True 
						)
		
		# add mesh to group
		cmds.parent( 
					mesh, 
					geoGroup 
					)

		# bind skin
		cmds.skinCluster( 
							baseJoint, 
							mesh, 
							name = 'arm_skinCluster',
							maximumInfluences = 4,
							obeyMaxInfluences = True,
							)
			
	# ------------------------------------------Use-case Functions---------------------------------------------------------- #
	
	# creates joints from list: RETURNS LIST of joints
	def spawn_joints( self, inputList = [ ], name = str( ), alignToList = False ):

		# list to hold joints
		returnList = [ ]

		# iterate over list
		for count, item in enumerate( inputList ):

			filledName = None
			joint = None

			filledName = ( '%s_0%s_JNT' % ( name, count + 1 ) )

			joint = cmds.joint( name = filledName )

			if alignToList == True:

				self.object_align( target = joint, destination = item )

			returnList.append( joint )

		return returnList
	
	# creates controls from list: RETURNS LIST of controls
	def spawn_controls( self, inputList = [ ], name = str( ), alignToList = True, curveRotation = [ ], curveSize = int( ) ):

		#list to hold controls
		returnList = [ ]

		# iterate over list
		for count, item in enumerate( inputList ):

			filledName = ( '%s_0%s_CTRL' % ( name, count + 1 ) )

			circle = cmds.circle( 
							name = filledName 
							)

			# delete creation history
			cmds.delete( 
					circle, 
					constructionHistory = True 
					)

			# history delete turns it into list
			circle = circle[ 0 ]

			# rotate and scale ctrl
			cmds.xform( 
					circle, 
					rotation = curveRotation, 
					scale = [ curveSize, curveSize, curveSize ] 
					)

			# freeze transforms
			self.freeze_transformations( 
							input = circle 
							)

			if alignToList == True:
				
				# align controller
				self.object_align( 
							target = circle, 
							destination = item
							)

			returnList.append( circle )

		return returnList
	
	# Finds middle index: RETURNS INTEGER list index
	def get_index_middle( self, inputList = [  ] ):

		listLength = float( len( inputList ) )

		listMiddlePerfect = listLength / 2

		# get list middle term
		if listLength % 2 != 0:

			# if remainder is present subtract .5 off middle
			middleIndex = int( listMiddlePerfect - .5 )

		else:

			# if no remainder then perfect middle is correct
			middleIndex = int( listMiddlePerfect )

		if self.verbosity > 0:

			print( 'indexRemainder : %s' % ( listLength % 2 ) )
			print( 'middleIndex : %s' % ( middleIndex ) )

		return middleIndex

	# gets object positions from list of joints: RETURNS LIST holding position
	def get_object_position( self, inputList = [ ] ):

		# list to return
		returnList = [ ]

		# iteratively get object positions
		for obj in inputList:
			
			position = cmds.xform( obj, query = True, translation = True, worldSpace = True )

			returnList.append( position )

			if self.verbosity > 0:

				print( 'position : %s' % ( position ) )

		return returnList

	# parents given list: NO RETURN
	def parent_list( self, inputList = [ ], transformNodePresent = False ):

		# list to hold backwards
		reversedList = [ ]
		
		# reverse list
		for obj in reversed( inputList ):

			reversedList.append( obj )

		# iterate
		for count, obj in enumerate( reversedList ):

			if self.verbosity > 0:

				print( 'parent_list:: \ncount : %s \nobj : %s' % ( count, obj ) )

			# if next iteration in less than list length - parent
			if count + 1 < len( reversedList ):

				nextObject = reversedList[ count + 1 ]

				# if object has transform node
				if transformNodePresent == True:

					# get next objects transform node
					nextObjectIndexRelative = cmds.listRelatives( nextObject, type = 'transform' )

					cmds.parent( obj, nextObjectIndexRelative )

				else:

					cmds.parent( obj, nextObject )

		return None

	# deletes all maya objects in list: NO RETURN
	def delete_list( self, inputList = [ ] ):

		for obj in inputList:

			cmds.delete( obj )

	# perform math operation on specified list member: RETURNS LIST after operation
	def list_math( self, inputList = [ ], operation = str( ), targetIndex = int( ), mathFactor = 1.0 ):

		returnList = [ ]
		
		for count, obj in enumerate( inputList ):
		
			if count == targetIndex:
				
				if operation == 'subtraction':
				
					item = inputList[ targetIndex ] - mathFactor

				elif operation == 'addition':

					item = inputList[ targetIndex ] + mathFactor

			else:

				item = obj

			returnList.append( item )

		return returnList

	# create node to compensate for object transforms: RETURNS NEW NODE holding old node
	def spawn_compensation_node( self, target = [ ] ):

		returnList = [ ]

		for obj in target:

			# set name
			compensationNodeName = '%s_transComp_GRP' % ( obj )
		
			# duplicate object
			compensateNode = cmds.duplicate( obj, transformsOnly = True, name = compensationNodeName )

			# add node to list
			returnList.append( compensateNode )
			
			# parent controller under transform node
			cmds.parent( obj, compensateNode )

		return returnList

	# align maya target to objects transforms: NO RETURN
	def object_align( self, target = str( ), destination = str( ) ):

		if self.verbosity > 0:

			print( ':::object_align:::\ntarget: %s\n destination: %s' % ( target, destination ) )
		
		constraint = cmds.parentConstraint( destination, target, maintainOffset = False )

		cmds.delete( constraint )

	# correctly orients joint chains: NO RETURN
	def orient_joints( self, jointLists = [ ], tipJointLocation = [ None, list( ) ], tipJoint = [ True, False ] ):

		for count, jointList in enumerate( jointLists ):

			if self.verbosity > 0:

				print( ':::orient_joints:::\ncount : %s \njointList : %s \nfirst joint : %s \noriented: %s' % ( count, jointList, jointList[ 0 ], jointList[ -2 ] ) )
		
			# orient joint chain
			cmds.joint( jointList[ 0 ], edit = True, orientJoint = 'xyz', secondaryAxisOrient = 'yup', children = True )
				
			if tipJoint == False:

				continue

			else:
				
				# create tip joint
				tipJoint = cmds.joint( name = 'tipJoint' )
				
				# get 2nd to last joint - was the last joint oriented correctly
				oriented = jointList[ -2 ]

				if self.verbosity > 0:

					print( 'oriented : %s' % ( oriented ) )

				if tipJointLocation == None:
					
					# align tip joint to last oriented joint
					self.object_align( target = tipJoint, destination = oriented )

					# get end joints x-translation
					translation = cmds.xform( jointList[ -1 ], query = True, translation = True, objectSpace = True )

					tipJointParent = cmds.listRelatives( tipJoint, parent = True )
				
					if tipJointParent != None:
						
						# parent tip joint under world if not already
						cmds.parent( tipJoint, world = True )
				
					# parent tip joint under 2nd to last joint so transforms are relative
					cmds.parent( tipJoint, oriented )

					# get whether or not joints are on positive or negative by subtracting 2nd to last from last
					jointSubtractor = cmds.xform( 
								jointList[ -2 ], 
								query = True, 
								translation = True,
								worldSpace = True 
								)
					
					jointBase = cmds.xform( 
										jointList[ -1 ], 
										query = True, 
										translation = True,
										worldSpace = True 
										)
					
					side = jointBase[ 0 ] - jointSubtractor[ 0 ]

					if int( side ) > 0:

						transDouble = 2

					else:
						 
						 transDouble = -2
					
					# double end joints x value
					translateValue = translation[ 0 ] * transDouble
					
					# translate tip joint out past end joint
					cmds.xform( tipJoint, translation = [ translateValue, 0, 0 ], objectSpace = True )

				else:

					cmds.xform( tipJoint, translation = tipJointLocation )
				
				# parent tip joint under original end joint
				cmds.parent( tipJoint, jointList[ -1 ] )

				# clear rotations on tip joint
				cmds.makeIdentity( tipJoint, apply = True, rotate = True )
				
				# orient joint chain with tip joint
				cmds.joint( jointList[ 0 ], edit = True, orientJoint = 'xyz', secondaryAxisOrient = 'yup', children = True )

				# delete tip joint
				cmds.delete( tipJoint )

	# freeze transforms: NO RETURN
	def freeze_transformations( self, input = [ list( ), str( ) ] ):

		if type( input ) == list( ):
		
			for item in input:
				
				cmds.makeIdentity( 
									item, 
									apply = True,
									translate = True, 
									rotate = True, 
									scale = True 
									)

		else:

			cmds.makeIdentity( 
									input, 
									apply = True,
									translate = True, 
									rotate = True, 
									scale = True 
									)

	# gets parent node: RETURNS STRING
	def get_object_parent( self, input = str( ), type = str( ) ):

		if type == 'None':

			returnString = cmds.listRelatives( 
											input, 
											parent = True
											)
		else:

			returnString = cmds.listRelatives( 
											input, 
											parent = True,
											type = type
											)

		return returnString

	# takes object out of hierarchy: NO RETURN
	def hierarchy_remove( self, input = str( ) ):

		parent = cmds.listRelatives( 
							input, 
							parent = True 
							)

		if self.verbosity > 0:

			print( ':::hierarchy_remove:::\ninput: %s\nparent: %s' % ( input, parent ) )
		
		if parent != 'None':
			
			# take chain out of hierarchy
			cmds.parent( input, world = True )

	# iteratively constrain: NO RETURN
	def constrain_lists( self, constraint = [ 'parent', 'point', 'orient' ], name = str( ), parent = [ list( ), str( ) ], child = [ list( ), str( ) ] ):
		
		# holds constraints
		returnList = [ ]

		if type( parent ) == str( ):
			
			newName = ( '%s_01_%sConstraint' % ( name, constraint ) )
			# set constraint
			execString = ( 'cmds.%sConstraint( %s, %s, name = %s, maintainOffset = True )' % ( constraint, parent, child, newName ) )

			eval( execString  ) 

		else:

			for count, ( itemParent , itemChild ) in enumerate( zip( parent, child ) ):

				newName = ( '%s_0%s_%sConstraint' % ( name, count, constraint ) )

				execString = ( 'cmds.%sConstraint( %s, %s, name = %s, maintainOffset = True )' % ( constraint, str( itemParent ), str( itemChild ), newName ) )

				exec( execString) 

	# set RGB color of controller
	def set_color( self, item = str( ), color = [ 0, 0, 0 ] ):

		rgb = [ 'R', 'G', 'B' ]

		# enable override
		cmds.setAttr( item + '.overrideEnabled', 1 )

		# enable color override
		cmds.setAttr( item + '.overrideRGBColors', 1 )

		for colorChannel, colorValue in zip( rgb, color ):

			cmds.setAttr( item + ".overrideColor%s" % colorChannel, colorValue )