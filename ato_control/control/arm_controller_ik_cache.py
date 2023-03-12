# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.
import logging
import os
from collections import deque
import numpy as np
from control import arm_controller, motion


class ArmControllerIkCache(arm_controller.ArmController):
    def __init__(
        self,
        evaluation_depth=2,
        segment_lengths=(210, (210) * 2, 147),
        **kwargs,
    ):
        super().__init__(**kwargs)
        self.evaluation_depth = evaluation_depth

        # segment_lengths = (base_link, (movable), gripper)
        horizontal_reach = np.sum(segment_lengths[1]) + segment_lengths[2]
        vertical_reach = segment_lengths[0] + horizontal_reach
        self.reach = (
            (-horizontal_reach, horizontal_reach),
            (-horizontal_reach, horizontal_reach),
            (0, vertical_reach),
        )  # although the real reach is a dome, we roughly define it as a rectangle

    @staticmethod
    def __get_range(reach, index, unit):
        low, high = int(reach[index][0] / unit), int(reach[index][1] / unit + 1)
        assert low < high, (low, high)
        for val in range(low, high):
            yield val

    @staticmethod
    # generate a dictionary of ik value dictionaries,
    # the first level key are depths
    # the second level key are discrete integer tuples, which maps into floating (x,y,z) by
    # tuple / 2^(depth) == (x,y,z)
    # this way we can (A) handle missing values (i.e. unreachable), (B) avoid using floating number as keys
    def __evaluate_ik_cache(evaluation_depth, reach, multiple_initial_positions):
        ik_caches = {}
        count = 0
        unit = 2**evaluation_depth
        logging.error(unit)

        def to_fix_point(float_vector):
            return tuple(round(float(val), 2) for val in float_vector)

        for target_positions_vector in [
            multiple_initial_positions
        ]:  # todo: may add more
            ik_cache = {}
            target_positions = motion.ActuatorPositions(
                positions=target_positions_vector
            )
            target_pose = target_positions.forward_kinematics_ikpy()
            xyz = to_fix_point(target_pose.xyz)
            rpy = to_fix_point(target_pose.rpy)
            eval_queue = deque([(xyz, target_positions)])
            while eval_queue:
                xyz, position = eval_queue.popleft()
                for direction in (-1, 1):
                    for delta in (
                        (direction, 0, 0),
                        (0, direction, 0),
                        (0, 0, direction),
                    ):
                        # solve for each of the 6 directions
                        target_xyz = np.array(xyz) + np.array(delta) * unit
                        pass
            for x_quantized in ArmControllerIkCache.__get_range(reach, 0, unit):
                logging.info(
                    f"Solving {count}th point: {float(x_quantized)} * {unit} == {float(x_quantized) * unit}, {(x_quantized * unit - reach[0][0]) / (reach[0][1] - reach[0][0]) * 100}%"
                )
                for y_quantized in ArmControllerIkCache.__get_range(reach, 1, unit):
                    for z_quantized in ArmControllerIkCache.__get_range(reach, 2, unit):
                        target_pose = motion.EndeffectorPose(
                            pose=[
                                x_quantized * unit,
                                y_quantized * unit,
                                z_quantized * unit,
                                0,
                                0,
                                0,
                            ]
                        )
                        target_positions, _ = target_pose.inverse_kinematics_ikpy(
                            initial_joint_positions=None,
                            solver_mode="Forward",
                        )
                        if target_positions is not None:
                            ik_cache[(x_quantized, y_quantized, z_quantized)] = tuple(
                                pos for pos in target_positions
                            )
                            count += 1
        ik_caches[rpy] = ik_cache
        logging.info(
            f"Generated {count} data points, example ik_cache,\nX: \n{ik_caches}"
        )
        return ik_caches

    def generate_ik_cache(
        self,
        ik_cache_filepath_prefix="~/.ato/ik_cache",
    ):
        ik_cache = ArmControllerIkCache.__evaluate_ik_cache(
            evaluation_depth=self.evaluation_depth,
            reach=self.reach,
            multiple_initial_positions=[self.home_positions],
        )
        file_path = os.path.expanduser(f"{ik_cache_filepath_prefix}_ik_cache.npy")
        with open(file_path, "wb") as fout:
            logging.info(f"Saving ik_cache to {file_path}")
            np.save(fout, ik_cache)
            fout.close()
