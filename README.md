# ato

## What is this?

Ato is an open-source robotic arm that can be built at home with a $200 budget.

## Demos

<img alt="A 3 segments arm, visualized with dummy servo motors" src="https://github.com/haoxuw/ato/blob/master-resources/images/snapshots_processed/processed_arm_multi_seg_3.png" width="360">

<img alt="Assembling a segment" src="https://github.com/haoxuw/ato/blob/master-resources/images/snapshots_processed/merged_seg_install__.png" width="360">

<img alt="A gripper with sockets to mount servo on its jaw" src="https://github.com/haoxuw/ato/blob/master-resources/images/snapshots_processed/merged_gripper_dragon__.png" width="360">

## Why design another arm

Many existing commercial robotic arms are robust, precise and packed with sensors. They are great, but their $10k~$100k price tag is also prohibitively expansive for the general public.

This arm is designed to be simple and inexpensive, by stacking modularized and interchangeable 3D printable segments. The first 4DoF arm cost around 120 CAD to build, including online orders and shipping costs (to Toronto) of standardized servos, nuts and bolts, and material costs of 3D printing, although excluding the cost for a Raspberry Pi or Arduino, and assumed access to a 3D printing facility or home printer.

A 6 Degrees-of-Freedom (DoF) arm may be built using 3 identical segments. A segment can be assembled by only three 3d printed objects: A bone and 2 mirrored halves of a joint.

Ato is competent for learning, or for proof of concept (PoC) projects. Although it's not built for heavy duty tasks, the 3D printed parts are fairly robust, and cost next to nothing to replace.

## What is in this repo

This repo is composed of 2 sub projects of ato:
 * Mesh: Allow you to build an arm: Modeling and configuration of 3D printable meshes
 * Control: Allow you to control an arm: Joint position space control using a PS4 controller for joint positions
 * ML-based control: Next steps
   * ML-based cartesian position
   * Full trajectories control

list of neat features
 * mesh:
  -- Visualize printable STLs, and their compositional parts on a virtual shelf
 * control:
  -- Joint space control
  -- Cartesian space control using IK
  -- Control with PS4 controller
  -- Visualized arm simulation using matlab plots
  -- Trajectory replay
 * learning
 -- Training auto save, modify data/lr and continue
 ## Try it out!

For details visit the README.md in each project folder. Meanwhile here's a bird eye view.

### Build the arm

The arm would be assembled as illustrated:
![arm_components_diagram](https://github.com/haoxuw/ato/blob/master-resources/images/illustrations/arm_components_diagram.png)

To manufacture the 3D Printable Structures, you have some options:

#### Download released mesh

> STL files are available at the [resources branch](https://github.com/haoxuw/ato/tree/master-resources/printable_mesh).

Meanwhile the CD pipeline keeps the following STL in sync with master branch:
+ [Segment](https://printable-mesh.s3.us-east-2.amazonaws.com/master/SegmentAugmented_printable_mesh.stl)
+ [BoneSeparable](https://printable-mesh.s3.us-east-2.amazonaws.com/master/BoneSeparableVertical_printable_mesh.stl)
+ [BoneSeparablePitchOnly](https://printable-mesh.s3.us-east-2.amazonaws.com/master/pitch_only_BoneSeparableVertical_pitch_only__printable_mesh.stl)
+ [Gripper](https://printable-mesh.s3.us-east-2.amazonaws.com/master/GripperDragonWithServo_printable_mesh.stl)

#### Make your customized mesh

Built on top of cadquery, the code in /mesh/ defines the 3D printable meshes of the arm, while strictly and literally practicing object oriented programming.

In case you may want to tweak the structural design, or configuration, such as to alter length or girth. Once your changes passes the continuous integration git actions pipeline (CI), it would be greatly appreciated if you would contribute back to this open source project via pull requests.

### 3D print those mesh

Images to be added. Recommended settings to be added.

### Assembling the physical arm

Video to be added.
