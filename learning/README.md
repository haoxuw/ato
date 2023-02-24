# Prerequisites

## Install Cuda GPU support
e.g.

```
$ sudo apt install nvidia-cuda-toolkit
$ sudo apt purge nvidia-*
$ sudo add-apt-repository ppa:graphics-drivers/ppa
$ sudo apt update
$ sudo apt install nvidia-470
```

## Example commands to train inverse kinematics

### Generate training data with the control package
> $ ./control/scripts/activate_arm.sh --generate_training_data --training_data_filepath_prefix ~/.ato/training_data

### Start training using the data generated above
> $ ./learning/scripts/training.sh --training_data_filepath_prefix ~/.ato/training_data --max_epochs 256 --depth 12 --width 256 --new_ik

### you can kill the above process anytime, and resume training by
> $ ./learning/scripts/training.sh --training_data_filepath_prefix ~/.ato/training_data --max_epochs 256 --depth 12 --width 256 --fit_ik