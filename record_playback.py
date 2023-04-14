# -------------------------------------------
# Import classes
# -------------------------------------------
from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.scene import Scene
import rospy
from sensor_msgs.msg import Image

from PyKDL import Frame, Rotation, Vector
import numpy as np
import os
import time
import signal
import subprocess

import sys
import argparse

from surgical_robotics_challenge.simulation_manager import SimulationManager
import time

# -------------------------------------------
# parse arguments
# -------------------------------------------
argv = sys.argv
parser = argparse.ArgumentParser(description='indicate the mode')

parser.add_argument('-m', '--mode', type=str, required=True,
                    help='r stands for recording and p stands for playback')
args = parser.parse_args()
ros_topic = "/ambf/env/psm1/baselink/Command /ambf/env/psm2/baselink/Command /ambf/env/psm1/Actuator0/Command /ambf/env/psm2/Actuator0/Command"

# -------------------------------------------
# Helper: break
# -------------------------------------------
def add_break(s):
    time.sleep(s)
    print('-------------')


# -------------------------------------------
# Helper: kill rosbag process
# -------------------------------------------
def terminate_process_and_children(p):
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    assert retcode == 0, "ps command returned %d" % retcode
    for pid_str in ps_output.split(b'\n')[:-1]:
            os.kill(int(pid_str), signal.SIGINT)
    p.terminate()


# -------------------------------------------
# Instantiation
# -------------------------------------------
rospy.init_node("record_teleop")
simulation_manager = SimulationManager('my_surture_example')
time.sleep(0.5)
world_handle = simulation_manager.get_world_handle()

psm1 = PSM(simulation_manager, 'psm1')
psm2 = PSM(simulation_manager, 'psm2')
ecm = ECM(simulation_manager, 'CameraFrame')
scene = Scene(simulation_manager)

# -------------------------------------------
# Record Data from Teleoperation
# -------------------------------------------
add_break(0.5)
print("Resetting the world")
world_handle.reset()
add_break(3.0)

if args.mode == 'r':
    command = "rosbag record -O ambf_data.bag {0}".format(ros_topic)
    rosbag_process = subprocess.Popen(command.split(' '))
else:
    command = "rosbag play ambf_data.bag"
    rosbag_process = subprocess.Popen(command.split(' '))

while not rospy.is_shutdown():
    pass

print("--> Done")
terminate_process_and_children(rosbag_process)
