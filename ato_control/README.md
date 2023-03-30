# Contorller Manual

This section describes how to control the arm.

## Get Started

### Prerequisites

1. Activate venv and install dependencies
2. If using PS4 controller, connect to bluetooth. Some hardware doesn't work well with Raspberry Pi OS, you may try to connect by holding `Share` + `PS` button.

### Start the Controller
```
./scripts/activate_arm.sh
```

The PS4 controller signals are updated by `/dev/input/js*` on ubuntu. This script would try connect to the lastest connected joystick device. To specify a target input device, e.g. js6, use `./ato_control/scripts/activate_arm.sh -j 6`

## Using a PS4 controller

### Short Story

* 2 Joysticks to control 2 segments, `L1 R1` to swtich targeted segments.
* `L2 R2` to control gripper.
* `Share` to `enable` detailed logging, `Option` to disconnect.
* `Arrow keys` to switch modes.
* `L3` (press left joystick) move to Home positions.
* `R3` to replay a saved trajectory.

### Long Story

Each mode is mapped to an arrow key, via
```
def __update_input_states(self, key, value):
  ...
def __controller_mode_to_button_mapping(self):
  ...
```

Results in mapping:
```
{
    UP: IN_CARTESIAN_MODE,
    RIGHT: IN_JOINT_SPACE_MODE,
    DOWN: IN_SETTING_MODE,
    LEFT: IN_TRAJECTORY_EDITING_MODE,
}
```

## Modes

The controller can be in one of four possible modes, defined by:

```
def controller_modes(self):.
 - IN_CARTESIAN_MODE
   - Under development and disabled -- limited functionality with preformings to be improved
 - IN_JOINT_SPACE_MODE
   - Control each servo motor individually
 - IN_SETTING_MODE
   - Calibrating between physical and logical position of motors, change velocity
 - IN_TRAJECTORY_EDITING_MODE
   - To create a trajectory, either by setting waypoints and pauses, or recording one live.
"""
```

#### IN_CARTESIAN_MODE

Disabled, currently under development, sorry.

#### IN_JOINT_SPACE_MODE

Left joystick to control the 1st segment, right for the 2nd. Pushing vertically for roll, horizontally for pitch.

Press R1, so that left for the 2nd, right for the 3rd. Press again for 3rd and 4th. Press L1 to reverse. 

#### IN_SETTING_MODE

Mainly used for advanced calibration.

#### IN_TRAJECTORY_EDITING_MODE

* `L3` save the trajectory in editting, and reset_trajectory_in_editing buffer
* `Circle (O)` reset_trajectory_in_editing buffer
* `Cross (X)` append the current position as waypoint
* `Triagle` append a pause
* `Square` toggle trajectory segment recording, each segment would be appended to trajectory in editting

## Using a Kyeboard

To be implemented.

# Software Architecture

This package uses the following components to implement control flow of the arm

-- root obj of arm: ArmController
  -- contains: Ps4Joystick
  -- contains: RaspberryPi
  -- contains multiple: ServoInterface
      -- each has reference to the RaspberryPi

## More Example Commands

#### Start arm controller without connecting to RPI and also enable visualization
> $ ./ato_control/scripts/activate_arm.sh

#### To start arm controller to interface with RaspberryPI (RPI), with joystick at /dev/input/js0 (-j 0)
> $ ./ato_control/scripts/activate_arm.sh -j 0

#### If you have ros installed, you can examine the URDF models by

> $ sudo apt-get install libatlas-base-dev -y
> $ ./ato_control/scripts/download_urdf_mesh_stl.sh && roslaunch urdf_tutorial display.launch model:=./urdf/ato_3_seg_270_deg.urdf