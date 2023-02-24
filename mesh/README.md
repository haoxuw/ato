# mesh

## Build pre-defined arm

### Bill of materials (BOM)

### Download pre-defined mesh

## Build customized arm

### Make modifications

#### Installation

#### Example commands to generate STL files

> $ ./scripts/delete_all_stl.sh && ./scripts/generate_shelf.sh --export -o generated

## Code architecture

### High-level walk through

To configure an segment with customized length girth etc. Define a new class under segment_configuration.py . Functionality of the variables are [illustrated here](https://github.com/haoxuw/ato/blob/master-resources/images/illustrations/segment_config_diagram.png)
### Naming conventions

## Demo

### Build

### Alternatives
While it's a bad idea, in theory, one may build ![a theoretical 10-segment arm](https://github.com/haoxuw/ato/blob/master-resources/images/snapshots_processed/processed_arm_multi_seg_a.png)
