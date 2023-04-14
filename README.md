AMBF Record & Playback Data
===

## Overview

The objective of this program is to collect data from the latest AMBF phantom assets and replay it using rosbag. The program offers two modes: `r` for "recording" and `p` for "playback." In `r` mode, the program subscribes to the low-level kinematics data of the joints, while in `p` mode, it replays the motions.

## Requirements

For first time users, please make sure to correctly set up the environment by following all steps and links below:

1. Setup AMBF (please update to make sure it is the latest version)
2. Setup Surgical robotics challenge
3. Setup dVRK
4. Copy the current repo
   
   ```shell
   git clone https://github.com/ruiruihuangannie/ambf_teleoperation_playback
   ```

# Usage

## Step 1

### Terminal #1: `roscore`

```shell
source ~/catkin_ws/devel/setup.bash
roscore
```

### Terminal #2: dVRK GUI console

```shell
source ~/catkin_ws/devel/setup.bash
roscd dvrk_config
rosrun dvrk_robot dvrk_console_json -j jhu-daVinci/console-MTML-MTMR.json
```

### Terminal #3: surgical robotics challenge (17 is the new asset)

```shell
source ~/ambf/build/devel/setup.bash
cd ~/surgical_robotics_challenge/
ambf_simulator --launch_file launch.yaml -l 0,1,3,4,14,17 -p 120 -t 1 --override_max_comm_freq 120
```

### Terminal #4-5: teleoperation scripts (left and right)

```shell
source ~/ambf/build/devel/setup.bash
cd ~/surgical_robotics_challenge/scripts/surgical_robotics_challenge/teleoperation
python3 mtm_multi_psm_control.py --mtm MTML -c mtml --one 1 --two 0
```

In another console:

```shell
source ~/ambf/build/devel/setup.bash
cd ~/surgical_robotics_challenge/scripts/surgical_robotics_challenge/teleoperation
python3 mtm_multi_psm_control.py --mtm MTMR -c mtmr --one 0 --two 1
```

### Terminal #6: the recorder/playback console

```shell
source ~/ambf/build/devel/setup.bash
python3 record_playback.py -m r
```

When the `recording` mode begins, a `[Info]` line should be displayed in the console. When finish recording data, press `ctrl+c` to exit the recorder. When successfully recorded, a `.bag` file should appear in the current repo.

### Before going into the `playback mode`, please make sure that:

1. The scene (particularly the rigid bodies) is reset. This can be done wtih ctrl+R.
2. The teleoperation scripts (opened previuosly as T#4 and 5) are exited.

Then run this: 

```shell
source ~/ambf/build/devel/setup.bash
python3 record_playback.py -m p
```

When the `playback` mode begins, a `[info]` line should be displayed in the console. When finish playing back, the process would automatically halt.

## Conclusion

This program provides a simple way to streamline recording and playing-back of AMBF data using rosbag. I hope this is helpful!
