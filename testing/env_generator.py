#Author: Chaitanya Pb

import numpy as np
import random
import matplotlib.pyplot as plt

# Global variables

ARGOS_FILE_NAME = "collective_transport.argos"

LOOP_FUNC_PATH = "build/testing/libtrajectory_loop_functions"
USER_FUNC_PATH = "build/testing/libtrajectory_loop_functions"

# Arena
ARENA_SIZE = 10
SCALE = 0.8

# Object
OBJ_DIM = (0.2, 0.2, 0.1)
OBJ_MASS = 0.5
OBJ_POS = [-4, 4]

# Goal
GOAL_DIM = (0.2, 0.2, 0.0001)
GOAL_MASS = 0.0001
GOAL_POS = [4, -4]

# Swarm
SWARM_SIZE = 6
CAGE_RADIUS = max(OBJ_DIM[0:2]) + 0.01

# Controller
CONTROLLER_TYPE = "cpp"
CONTROLLER_NAME = "khivbz"
CONTROLLER_PATH = "build/testing/libcollective_transport"

# Obstacles
NUM_OBS = 1
MAX_OBS_DIM = 1.5
MAX_OBS_PERI = 5.0
OBS_HEIGHT = 0.5

# Obstacle Image
IM_SCALE = 100
IM_OFFSET = (int(ARENA_SIZE/2.0)*IM_SCALE, int(ARENA_SIZE/2.0)*IM_SCALE)
ACT_IM = np.ones([int(IM_SCALE*ARENA_SIZE), int(IM_SCALE*ARENA_SIZE)])
EXP_IM = np.ones([int(IM_SCALE*ARENA_SIZE), int(IM_SCALE*ARENA_SIZE)])

# Function writes the initial code for ARGoS file
def code_initial(argosfile):

	xml_def = "<?xml version='1.0' ?>\n"
	argos_def = "\n<!-- ARGoS Configuration file -->\n\n<argos-configuration>\n"

	fw_init = "\n\t<!-- Basic Framework -->\n\n\t<framework>\n"
	fw_core = "\n\t\t<system threads='0' />\n\t\t<experiment length='0' ticks_per_second='10' random_seed='256' />\n"
	fw_end = "\n\t</framework>\n"
	fw_def = fw_init + fw_core + fw_end

	argosfile.write(xml_def + argos_def + fw_def)

# Function writes the ending code for ARGoS file
def code_ending(argosfile):

	end_def = "\n</argos-configuration>\n"

	argosfile.write(end_def)

# Function writes the physics, media and visualization code
def code_pmv(argosfile):

	phy_def = "\n\t<!-- Physics Engine -->\n\n\t<physics_engines>\n\t\t<dynamics2d id='dyn2d' />\n\t</physics_engines>\n"

	loop_def = "\n\t<!-- Loop Functions -->\n\n\t<loop_functions library='" + LOOP_FUNC_PATH + "' label='trajectory_loop_functions' />\n"

	media_def = "\n\t<!-- Communication Media -->\n\n\t<media>\n\t\t<range_and_bearing id='rab' />\n\t\t<led id='leds' />\n\t</media>\n"

	vis_init = "\n\t<!-- Visualization -->\n\n\t<visualization>\n\t\t<qt-opengl>\n"

	if CONTROLLER_TYPE == "buzz": 
		vis_mid = "\t\t\t<user_functions label='buzz_qt' />\n"
	else:
		vis_mid = ""

	user_def = "\n\t\t\t<!-- User Functions -->\n\n\t\t\t<user_functions library='" + USER_FUNC_PATH + "' label='trajectory_qtuser_functions' />\n"

	vis_end = "\n\t\t</qt-opengl>\n\t</visualization>\n"
	vis_def = vis_init + vis_mid + user_def + vis_end

	argosfile.write(phy_def + loop_def + media_def + vis_def)

# Function writes the controller code for ARGoS file
def code_controller(argosfile, controller):

	con_init = "\n\t<!-- Controllers -->\n\n\t<controllers>\n"
	argosfile.write(con_init)

	# Choose between Buzz and C++ controller
	if controller == "buzz":

		bz_comment = "\n\t\t<!-- Buzz Controller -->\n"
		bz_init = "\n\t\t<buzz_controller_kheperaiv id='" + CONTROLLER_NAME + "'>\n"
		
		act_init = "\t\t\t<actuators>\n"
		act1 = "\t\t\t\t<differential_steering implementation='default' />\n"
		act2 = "\t\t\t\t<leds implementation='default' medium='leds' />\n"
		act3 = "\t\t\t\t<range_and_bearing implementation='default' />\n"
		act_end = "\t\t\t</actuators>\n"
		act_def = act_init + act1 + act2 + act3 + act_end
		
		sen_init = "\t\t\t<sensors>\n"
		sen1 = "\t\t\t\t<kheperaiv_proximity implementation='default' show_rays='true' />\n"
		sen2 = "\t\t\t\t<range_and_bearing implementation='medium' medium='rab' show_rays='true' noise_std_dev='0' />\n"
		sen3 = "\t\t\t\t<positioning implementation='default' />\n"
		sen_end = "\t\t\t</sensors>\n"
		sen_def = sen_init + sen1 + sen2 + sen3 + sen_end

		par_init = "\t\t\t<params>\n"
		params = "\t\t\t\t<wheel_turning hard_turn_angle_threshold='90' soft_turn_angle_threshold='70' no_turn_angle_threshold='10' max_speed='10' />\n"
		par_end = "\t\t\t</params>\n"
		par_def = par_init + params + par_end

		bz_end = "\t\t</buzz_controller_kheperaiv>\n"

		argosfile.write(bz_comment + bz_init + act_def + sen_def + par_def + bz_end)

	elif controller == "cpp":

		cpp_comment = "\n\t\t<!-- C++ Controller -->\n"
		cpp_init = "\n\t\t<collective_transport_controller id='" + CONTROLLER_NAME + "' library='" + CONTROLLER_PATH + "'>\n"
		
		act_init = "\t\t\t<actuators>\n"
		act1 = "\t\t\t\t<differential_steering implementation='default' />\n"
		act2 = "\t\t\t\t<leds implementation='default' medium='leds' />\n"
		act3 = "\t\t\t\t<range_and_bearing implementation='default' />\n"
		act_end = "\t\t\t</actuators>\n"
		act_def = act_init + act1 + act2 + act3 + act_end
		
		sen_init = "\t\t\t<sensors>\n"
		sen1 = "\t\t\t\t<kheperaiv_proximity implementation='default' show_rays='true' />\n"
		sen2 = "\t\t\t\t<kheperaiv_ultrasound implementation='default' show_rays='true' />\n"
		sen3 = "\t\t\t\t<positioning implementation='default' />\n"
		sen_end = "\t\t\t</sensors>\n"
		sen_def = sen_init + sen1 + sen2 + sen3 + sen_end

		par_def = "\t\t\t<params alpha='7.5' delta='0.1' velocity='5' />\n"

		cpp_end = "\t\t</collective_transport_controller>\n"

		argosfile.write(cpp_comment + cpp_init + act_def + sen_def + par_def + cpp_end)

	else:
		pass

	con_end = "\n\t</controllers>\n"
	argosfile.write(con_end)

# Function writes code for boundary walls
def code_boundary(argosfile):

	bw_comment = "\n\t\t<!-- Boundary Walls -->\n"
	argosfile.write(bw_comment)

	code_box(argosfile, "wall_north", [0.1, ARENA_SIZE, 1], [(ARENA_SIZE/2.0) - 0.05, 0, 0, 0, 0, 0], move=False)
	code_box(argosfile, "wall_south", [0.1, ARENA_SIZE, 1], [-(ARENA_SIZE/2.0) + 0.05, 0, 0, 0, 0, 0], move=False)
	code_box(argosfile, "wall_east", [ARENA_SIZE - 0.2, 0.1, 1], [0, -(ARENA_SIZE/2.0) + 0.05, 0, 0, 0, 0], move=False)
	code_box(argosfile, "wall_west", [ARENA_SIZE - 0.2, 0.1, 1], [0, (ARENA_SIZE/2.0) - 0.05, 0, 0, 0, 0], move=False)

# Function writes code for a box entity
def code_box(argosfile, box_id, size, pose, move=False, mass=1.0):
	
	if move:
		moveTag = "true"
		mass = mass
	else:
		moveTag = "false"
		mass = 1.0
		edit_image(size, pose)

	box_init = "\n\t\t<box id='" + box_id + "' size='" + str(size[0]) + ", " + str(size[1]) + ", " + str(size[2]) + "' "
	box_move = "movable='" + moveTag + "' mass='" + str(mass) + "'>\n"
	box_pos = "\t\t\t<body position='" + str(pose[0]) + ", " + str(pose[1]) + ", " + str(pose[2]) + "' "
	box_ori = "orientation='" + str(pose[3]) + ", " + str(pose[4]) + ", " + str(pose[5]) + "' />\n"
	box_end = "\t\t</box>\n"
	box_def = box_init + box_move + box_pos + box_ori + box_end

	argosfile.write(box_def)

# Function writes code for the object to move
def code_object(argosfile, pose=None):
	
	global OBJ_POS

	obj_comment = "\n\t\t<!-- Object -->\n"
	argosfile.write(obj_comment)

	mass = OBJ_MASS
	size = OBJ_DIM

	if pose is None:
		position = [random.uniform(-(SCALE*ARENA_SIZE)/2.0, (SCALE*ARENA_SIZE)/2.0), random.uniform(-(SCALE*ARENA_SIZE)/2.0, (SCALE*ARENA_SIZE)/2.0), 0]
		orientation = [0, 0, 0]
		OBJ_POS = [position[0], position[1]]
		pose = position + orientation
	else:
		OBJ_POS = [pose[0], pose[1]]
		pose = [pose[0], pose[1], 0, 0, 0, 0]

	code_box(argosfile, "object", size, pose, move=True, mass=mass)

# Function writes code for the goal
def code_goal(argosfile, pose=None):
	
	global GOAL_POS

	goal_comment = "\n\t\t<!-- Goal -->\n"
	argosfile.write(goal_comment)

	mass = GOAL_MASS
	size = GOAL_DIM

	if pose is None:
		position = [random.uniform(-(SCALE*ARENA_SIZE)/2.0, (SCALE*ARENA_SIZE)/2.0), random.uniform(-(SCALE*ARENA_SIZE)/2.0, (SCALE*ARENA_SIZE)/2.0), 0]
		orientation = [0, 0, 0]
		GOAL_POS = [position[0], position[1]]
		pose = position + orientation
	else:
		GOAL_POS = [pose[0], pose[1]]
		pose = [pose[0], pose[1], 0, 0, 0, 0]

	code_box(argosfile, "goal", size, pose, move=True, mass=mass)

# Function writes code for spawning robots
def code_robot(argosfile, robot_id, pose, controller):
	
	rbt_init = "\n\t\t<kheperaiv id='" + robot_id + "'>\n"
	rbt_pos = "\t\t\t<body position='" + str(pose[0]) + ", " + str(pose[1]) + ", " + str(pose[2]) + "' "
	rbt_ori = "orientation='" + str(pose[3]) + ", " + str(pose[4]) + ", " + str(pose[5]) + "' />\n"
	rbt_con = "\t\t\t<controller config='" + controller + "' />\n"
	rbt_end = "\t\t</kheperaiv>\n"
	rbt_def = rbt_init + rbt_pos + rbt_ori + rbt_con + rbt_end

	argosfile.write(rbt_def)

# Function writes code for random obstacles
def code_obstacle(argosfile):
	
	obs_comment = "\n\t\t<!-- Obstacles -->\n"
	argosfile.write(obs_comment)

	for i in range(NUM_OBS):
		one_dim = random.uniform(0, MAX_OBS_DIM)
		other_dim = (MAX_OBS_PERI/2.0) - one_dim
		size = [one_dim, other_dim, OBS_HEIGHT]

		position = [random.uniform(-(SCALE*ARENA_SIZE)/2.0, (SCALE*ARENA_SIZE)/2.0), random.uniform(-(SCALE*ARENA_SIZE)/2.0, (SCALE*ARENA_SIZE)/2.0), 0]
		orientation = [0, 0, 0]
		pose = position + orientation
		code_box(argosfile, "obs" + str(i), size, pose, move=False)

# Function writes code for spawning robots in "caged" configuration
def code_caging(argosfile):

	rbt_comment = "\n\t\t<!-- Robots -->\n"
	argosfile.write(rbt_comment)

	angles = np.linspace(0, 2*np.pi, SWARM_SIZE+1)
	for i in range(len(angles)-1):
		position = [OBJ_POS[0] + CAGE_RADIUS*np.cos(angles[i]), OBJ_POS[1] + CAGE_RADIUS*np.sin(angles[i]), 0]
		orientation = [0, 0, 0]
		pose = position + orientation
		code_robot(argosfile, "khiv" + str(i), pose, CONTROLLER_NAME)

# Function writes the arena code for ARGoS file
def code_arena(argosfile):

	arn_init = "\n\t<!-- Arena -->\n\n\t<arena size='" + str(ARENA_SIZE) + ", " + str(ARENA_SIZE) + ", 2' center='0, 0, 0.5'>\n"
	argosfile.write(arn_init)

	code_boundary(argosfile)
	code_object(argosfile, pose=OBJ_POS)
	code_caging(argosfile)
	code_goal(argosfile, pose=GOAL_POS)
	code_obstacle(argosfile)

	arn_end = "\n\t</arena>\n"
	argosfile.write(arn_end)

# Function makes edits to the image
def edit_image(size, pose):

	global ACT_IM, EXP_IM
	global IM_SCALE, IM_OFFSET
	global CAGE_RADIUS

	act_size = (IM_SCALE*size[0], IM_SCALE*size[1])
	exp_size = (IM_SCALE*(size[0]+CAGE_RADIUS), IM_SCALE*(size[1]+CAGE_RADIUS))
	im_pos = (int(IM_SCALE*pose[0] + IM_OFFSET[0]), int(IM_SCALE*pose[1] + IM_OFFSET[1]))

	for x in range(int(-act_size[0]/2.0), int(act_size[0]/2.0)):
		for y in range(int(-act_size[1]/2.0), int(act_size[1]/2.0)):
			try:
				ACT_IM[im_pos[0] + x, im_pos[1] + y] = 0
			except:
				pass

	for x in range(int(-exp_size[0]/2.0), int(exp_size[0]/2.0)):
		for y in range(int(-exp_size[1]/2.0), int(exp_size[1]/2.0)):
			try:
				EXP_IM[im_pos[0] + x, im_pos[1] + y] = 0
			except:
				pass

# Main code
if __name__ == "__main__":

	with open(ARGOS_FILE_NAME, "w+") as argosfile:
		code_initial(argosfile)
		code_controller(argosfile, CONTROLLER_TYPE)
		code_arena(argosfile)
		code_pmv(argosfile)
		code_ending(argosfile)

	plt.imsave("actual_map.png", ACT_IM, cmap="gray")
	plt.imsave("expanded_map.png", EXP_IM, cmap="gray")
	plt.imshow(EXP_IM, cmap="gray")
	plt.show()
