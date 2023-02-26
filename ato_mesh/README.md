# mesh

## Build pre-defined arm

### Bill of materials (BOM)

### Download pre-defined mesh

STL files are available at the [resources branch](https://github.com/haoxuw/ato/tree/master-resources/printable_mesh).

### Make your customized mesh

Built on top of cadquery, the code in /mesh/ defines the 3D printable meshes of the arm, while strictly and literally practicing object oriented programming.

In case you may want to tweak the structural design, or configuration, such as to alter length or girth. Once your changes passes the continuous integration git actions pipeline (CI), it would be greatly appreciated if you would contribute back to this open source project via pull requests.

#### Make modifications

##### Installation

## Example commands to generate STL files

> $ ./ato_mesh/scripts/delete_all_stl.sh && ./ato_mesh/scripts/generate_shelf.sh --export -o mesh/generated

## Code architecture

### High-level walk through

To configure an segment with customized length girth etc. Define a new class under segment_configuration.py . Functionality of the variables are [illustrated here](https://github.com/haoxuw/ato/blob/master-resources/images/illustrations/segment_config_diagram.png)
### Naming conventions

## Demo

### Build

### Alternatives
While it's a bad idea, in theory, one may build ![a theoretical 10-segment arm](https://github.com/haoxuw/ato/blob/master-resources/images/snapshots_processed/processed_arm_multi_seg_a.png)
