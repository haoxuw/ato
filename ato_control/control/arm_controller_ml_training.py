# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.
import logging
import os

import numpy as np
from control import arm_controller, motion


class ArmControllerMlTraining(arm_controller.ArmController):
    @staticmethod
    def __get_endeffector_pose_intrinsic(actuator_positions, index=None, size=None):
        if index is not None and size is not None and (index - 1) % 10000 == 0:
            logging.info(f"Processed {index//1000}k ({int(100.0 * index / size)}%)")
        return motion.ActuatorPositions(
            positions=actuator_positions
        ).forward_kinematics()["endeffector_pose_intrinsic"]

    def __generate_training_data_forward_kinematics(
        self,
        size,
    ):
        # fk and ir both use positions normalized to relu6, so they would match
        # meanwhile this allows ir to use a relu6 as the last layer to prevent out of bound positions
        multiple_actuator_positions_relu6 = np.array(
            [
                np.random.uniform(
                    low=0, high=6, size=len(self._get_indexed_rotation_ranges())
                )
                for _ in range(size)
            ]
        )
        matching_endeffector_pose_intrinsic = np.array(
            [
                self.__get_endeffector_pose_intrinsic(
                    actuator_positions=self._denormalize_relu6(relu6_val),
                    index=index,
                    size=size,
                )
                for index, relu6_val in enumerate(multiple_actuator_positions_relu6)
            ],
        )
        training_x = np.array(
            multiple_actuator_positions_relu6[:, :6], dtype=np.float32
        )  # [:6] to discard gripper configs
        training_y = np.array(matching_endeffector_pose_intrinsic, dtype=np.float32)

        logging.info(
            f"Generated data, example training_data[:10],\nX: \n{training_x[:10]}\nY: \n{training_y[:10]}"
        )

        return training_x, training_y

    def __generate_training_data_inverse_kinematics(
        self,
        size,
        noise_range_in_degrees=(-5, 5),
    ):
        multiple_actuator_positions_relu6 = np.array(
            [
                np.random.uniform(
                    low=0,
                    high=6,
                    size=len(self._get_indexed_rotation_ranges()),
                )
                for _ in range(size)
            ]
        )
        training_x = []
        training_y = []
        for index, current_actuator_positions_relu6 in enumerate(
            multiple_actuator_positions_relu6
        ):
            noise_in_degrees = np.random.uniform(
                low=noise_range_in_degrees[0],
                high=noise_range_in_degrees[1],
                size=len(current_actuator_positions_relu6),
            )
            current_actuator_positions_degrees = self._denormalize_relu6(
                current_actuator_positions_relu6
            )
            current_endeffector_pose_intrinsic = self.__get_endeffector_pose_intrinsic(
                current_actuator_positions_degrees,
            )
            target_actuator_positions_degrees = (
                current_actuator_positions_degrees + noise_in_degrees
            )
            target_actuator_positions_relu6 = (
                motion.ActuatorPositions.normalize_to_relu6(
                    sequence=target_actuator_positions_degrees
                )
            )
            target_endeffector_pose_intrinsic = self.__get_endeffector_pose_intrinsic(
                actuator_positions=target_actuator_positions_degrees,
                index=index,
                size=size,
            )
            # remove gripper position
            current_actuator_positions_relu6 = current_actuator_positions_relu6[:-1]
            target_actuator_positions_relu6 = target_actuator_positions_relu6[:-1]

            training_x.append(
                np.concatenate(
                    [
                        current_actuator_positions_relu6[: self.num_segments * 2],
                        current_endeffector_pose_intrinsic,
                        target_endeffector_pose_intrinsic,
                    ]
                )
            )
            training_y.append(target_actuator_positions_relu6)
        training_x = np.array(training_x, dtype=np.float32)
        training_y = np.array(training_y, dtype=np.float32)
        assert training_x.shape == (
            size,
            self.num_segments * 2 + 12,
        ), training_x.shape
        assert training_y.shape == (size, self.num_segments * 2), training_y.shape

        logging.info(
            f"Generated data, example training_data[:10],\nX: \n{training_x[:10]}\nY: \n{training_y[:10]}"
        )

        return training_x, training_y

    def generate_training_data(
        self,
        size,
        training_data_filepath_prefix="~/.ato/training_data",
        forward=True,
        inverse=True,
    ):
        assert isinstance(size, int) and size > 1, size
        if forward:
            training_x, training_y = self.__generate_training_data_forward_kinematics(
                size=size
            )
            file_path = os.path.expanduser(
                f"{training_data_filepath_prefix}_forward_kinematics_x.npy"
            )
            with open(file_path, "wb") as fout:
                logging.info(f"Saving training_x to {file_path}")
                np.save(fout, training_x)
                fout.close()

            file_path = os.path.expanduser(
                f"{training_data_filepath_prefix}_forward_kinematics_y.npy"
            )
            with open(file_path, "wb") as fout:
                logging.info(f"Saving training_y to {file_path}")
                np.save(fout, training_y)
                fout.close()

        if inverse:
            training_x, training_y = self.__generate_training_data_inverse_kinematics(
                size=size
            )
            file_path = os.path.expanduser(
                f"{training_data_filepath_prefix}_inverse_kinematics_x.npy"
            )
            with open(file_path, "wb") as fout:
                logging.info(f"Saving training_x to {file_path}")
                np.save(fout, training_x)
                fout.close()

            file_path = os.path.expanduser(
                f"{training_data_filepath_prefix}_inverse_kinematics_y.npy"
            )
            with open(file_path, "wb") as fout:
                logging.info(f"Saving training_y to {file_path}")
                np.save(fout, training_y)
                fout.close()
