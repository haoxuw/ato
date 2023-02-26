import argparse

from learn_kinematics import forward_kinematics, inverse_kinematics, pl_module_wrapper

# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.


def get_args():
    parser = argparse.ArgumentParser()
    # todo: specify 6 segments
    parser.add_argument(
        "--training_data_filepath_prefix",
        type=str,
        default="~/.ato/training_data_forward_kinematics",
        help="Expecting $(training_data_filepath_prefix)_[forward_kinematics|inverse_kinematics]_[xy].npy",
    )
    parser.add_argument("--max_epochs", type=int, default=-1)
    parser.add_argument("--depth", type=int, default=8)
    parser.add_argument("--width", type=int, default=48)
    parser.add_argument("--new_fk", action="store_true")
    parser.add_argument("--new_ik", action="store_true")
    parser.add_argument("--fit_fk", action="store_true")
    parser.add_argument("--fit_ik", action="store_true")
    parser.add_argument(
        "--ckpt_version",
        type=str,
        default="latest",
        help="Only effective for fit_fk or fit_ik",
    )
    return parser.parse_args()


args = get_args()


def fit_both_kinematics_models():

    if args.new_fk or args.fit_fk:
        if args.new_fk:
            forward_kinematics_model = forward_kinematics.ForwardKinematics(
                data_x_filepath=f"{args.training_data_filepath_prefix}_forward_kinematics_x.npy",
                data_y_filepath=f"{args.training_data_filepath_prefix}_forward_kinematics_y.npy",
                width=args.width,
                depth=args.depth,
                batch_size=48,
            )
        else:
            forward_kinematics_model = (
                forward_kinematics.ForwardKinematics.load_from_checkpoint(
                    args.ckpt_version
                )
            )

        assert forward_kinematics_model is not None
        forward_kinematics_model.fit(
            max_epochs=args.max_epochs,
            load_ckpt=not args.new_fk,
            ckpt_version=args.ckpt_version,
        )

    if args.new_ik or args.fit_ik:
        if args.new_ik:
            inverse_kinematics_model = inverse_kinematics.InverseKinematics(
                width=int(args.width * 2),
                depth=int(args.depth),
                data_x_filepath=f"{args.training_data_filepath_prefix}_inverse_kinematics_x.npy",
                data_y_filepath=f"{args.training_data_filepath_prefix}_inverse_kinematics_y.npy",
                batch_size=48,
            )
        else:
            inverse_kinematics_model = (
                inverse_kinematics.InverseKinematics.load_from_checkpoint(
                    args.ckpt_version
                )
            )

        assert inverse_kinematics_model is not None
        inverse_kinematics_model.fit(
            max_epochs=args.max_epochs,
            load_ckpt=not args.new_ik,
            ckpt_version=args.ckpt_version,
        )


if __name__ == "__main__":
    fit_both_kinematics_models()
