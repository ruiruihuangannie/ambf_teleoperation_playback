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

from surgical_robotics_challenge.simulation_manager import SimulationManager
import time

import keyboard


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
ros_topic =
    ['/ambf/env/psm1/measured_cp',
    '/ambf/env/psm1/measured_jp',
    '/ambf/env/psm2/measured_cp',
    '/ambf/env/psm2/measured_jp']

simulation_manager = SimulationManager('auto_surturing')
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

command = "rosbag record -O ambf_data.bag {0}".format(ros_topic)
rosbag_process = subprocess.Popen(command.split(' '))

while True:
    try:
        if keyboard.is_pressed('q'):
            terminate_process_and_children(rosbag_process)
            print('--> Data record complete!!')
            break
        
reindex_cmd = "rosbag reindex ambf_data.bag {0}".format(ros_topic)
os.system(reindex_cmd)
sanity_cmd = "rosbag info ambf_data.bag {0}".format(ros_topic)
os.system(sanity_cmd)

# -------------------------------------------
# Playback Data
# -------------------------------------------
command = "rosbag play -r 1000 ambf_data.bag {0}".format(ros_topic)
rosbag_process = subprocess.Popen(command.split(' '))
terminate_process_and_children(rosbag_process)
print('--> Playback complete!')