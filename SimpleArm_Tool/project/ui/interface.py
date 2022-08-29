from maya import cmds as cmds

import importlib

from project.ui import interface_signals

importlib.reload( interface_signals )

class AutoRiggingWindow( ):

	def __init__( self ):

		self.signals = interface_signals.InterfaceSignals()

		self.build( )

		self.update( )

	def build( self ):

		window_size = [500,500]

		layout_size = [ window_size[ 0 ] / 2, window_size[ 1 ] / 2 ]

		button_size = [ window_size[0]/2, window_size[1]/3 ]

		
		self.window = cmds.window( 
				title = 'Auto Rigging Window', 
				widthHeight = window_size,
				sizeable = True, )

		self.grid_layout = cmds.gridLayout( 
				parent = self.window,
				autoGrow = True,
				numberOfColumns = 2,
				cellWidthHeight = layout_size )

		self.arm_locator_button = cmds.button( 
				parent = self.grid_layout,
				width = button_size[0],
				height = button_size[1],
				label = 'Spawn Arm Locators',
				command = lambda *_: self.signals.signal_spawn_locators( amount = 3, limb = 'arm' ),
				actOnPress = True )

		self.finger_locator_button = cmds.button( 
				parent = self.grid_layout,
				width = button_size[0],
				height = button_size[1],
				label = 'Spawn Finger Locators',
				command = lambda *_: self.signals.signal_spawn_locators( amount = 3, limb = 'finger' ),
				actOnPress = True )

		self.locator_convert_button = cmds.button( 
				parent = self.grid_layout,
				width = button_size[0],
				height = button_size[1],
				label = 'Create Joints',
				command = lambda *_: self.signals.signal_convert_locators( ),
				actOnPress = True )

		self.bind_mesh_button = cmds.button( 
				parent = self.grid_layout,
				width = button_size[0],
				height = button_size[1],
				label = 'Skin Mesh\n(Select Mesh)',
				command = lambda *_: self.signals.signal_bind_mesh( ),
				actOnPress = True )

	def update( self ):

		cmds.showWindow( self.window )