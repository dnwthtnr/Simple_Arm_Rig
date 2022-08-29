import sys

fileDirectory = "G:\\__work\\SimpleArm_Tool"

import importlib

# INSERT PROJECT PATH
sys.path.append( fileDirectory )

from project.ui import interface as ui

importlib.reload( ui )

window = ui.AutoRiggingWindow( )