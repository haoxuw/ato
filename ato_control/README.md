# Architecture

This package uses the following components to implement control flow of the arm

-- root obj of arm: ArmController
  -- contains: Ps4Joystick
  -- contains: RaspberryPi
  -- contains multiple: ServoInterface
      -- each has reference to the RaspberryPi

## Example commands

#### Start arm controller without connecting to RPI and also enable visualization (-v)
> $ ./ato_control/scripts/activate_arm.sh -j 0 -v

#### To start arm controller to interface with RaspberryPI (RPI), with joystick at /dev/input/js0 (-j 0)
> $ sudo pigpiod && ./ato_control/scripts/activate_arm.sh -j 0

#### If you have ros installed, you can examine the URDF models by

> $ sudo apt-get install libatlas-base-dev -y
> $ ./ato_control/scripts/download_urdf_mesh_stl.sh && roslaunch urdf_tutorial display.launch model:=./ato_control/ato_3_seg.urdf