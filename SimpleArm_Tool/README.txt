						*Arm Auto-Rigger*
						
						author: Tanner Dunworth

This tool is designed to allow you to define poitions on a standard arm mesh to 
spawn joints, create an IK FK blend, and bring you to the point where all that's 
left to do is paint weights on the given mesh!

Prerequisites

- Autodesk Maya ( was developoed using Maya 2022 but should work for other versions as well )


Installation

- Place the 'SimpleArm_Tool' folder wherever you would like on your computer

- Navigate into the 'SimpleArm_Tool' folder

- Open 'shelfButton.py'

- Set the variable 'fileDirectory' equal to 'C:\\your\\download\\directory\\SimpleArm_Tool'

- Copy the contents of 'shelfButton.py' and, within Maya, either execute the code through a 
  python tab in the script editor or create a shelf button that runs the code


How to Use

- Once you've executed the code you will be met with a simple GUI with 4 buttons ( Spawn Arm Locators, Spawn Finger Locators, Create Joints, and Skin Mesh )

- To get started with a given arm mesh press 'Spawn Arm Locators' and 3 nurbs spheres will spawn within a group. You can use both the group and individual
  spheres to mark off your shoulder, elbow, and wrist locations with 'arm_loc_01' being your shoulder and 'arm_loc_03' being your wrist

- After placing your arm locators you can move onto the 'Spawn Finger Locators' button and do the same as you did with the arm. You can create as many 
  different fingers as you want, you could make a million-fingered arm monstrosity if you so choose, it shouldn't cause any issues!

- Once you're happy with the placement of all your new fingers you can go ahead and click on the 'Create Joints' button and your joints, controllers, 
  as well as IK FK blender will spawn. They're going to be a fixed size within this current version, I had the controller size customizable prior, with a slider,
  but found it to be a bit troublesome trying to get the correct size without having to invest more time than I was aloughted into just making the control size 
  customizable

- Now once you're ready to paint weights just select your mesh and click on the 'Skin Mesh' button

- And now it's time to paint weights by hand!


Developer Note:

  None