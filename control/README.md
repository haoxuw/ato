# Architecture

This package uses the following components to implement control flow of the arm

-- root obj of arm: ArmController
  -- contains: Ps4Joystick
  -- contains: RaspberryPi
  -- contains multiple: ServoInterface
      -- each has reference to the RaspberryPi

## Example commands

#### To start arm controller to interface with RaspberryPI (RPI), with joystick at /dev/input/js0 (-j 0)
> $ sudo pigpiod && control/scripts/activate_arm.sh -j 0

#### Start arm controller without connecting to RPI and also enable visualization (-v)
> $ control/scripts/activate_arm.sh -j 0 -v

#### If you have ros installed, you can examine the URDF models by

> $ sudo apt-get install libatlas-base-dev -y
> $ ./scripts/download_urdf_mesh_stl.sh && roslaunch urdf_tutorial display.launch model:=ato_3_seg.urdf