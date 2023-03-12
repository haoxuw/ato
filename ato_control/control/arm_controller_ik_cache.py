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
from control.config_and_enums.controller_enums import SolverMode


# moving arm with cached IK has two main benefits
# 1. reduce computation delay, especially when running on device
# 2. using BFS search from the home position makes sure not only the IK solution is viable in isolation,
# but also reachable with minimal joints delta from the previous pose, tracing back to home pose
class ArmControllerIkCache(arm_controller.ArmController):
    def __init__(
        self,
        evaluation_depth=3,
        grouped_segment_lengths=(
            210,
            (210) * 2,
            147,
        ),  # todo parse from arm_segments_config
        **kwargs,
    ):
        super().__init__(**kwargs)
        self.evaluation_depth = evaluation_depth

        # grouped_segment_lengths = (base_link_fixed_to_mount, (segment_1_movable ...), gripper)
        grouped_segment_lengths = (
            self._indexed_segment_lengths[0],
            self._indexed_segment_lengths[1:-1],
            self._indexed_segment_lengths[-1],
        )
        horizontal_reach = (
            np.sum(grouped_segment_lengths[1]) + grouped_segment_lengths[2]
        )
        vertical_reach = grouped_segment_lengths[0] + horizontal_reach
        self.reach = (
            (-horizontal_reach, horizontal_reach),
            (-horizontal_reach, horizontal_reach),
            (0, vertical_reach),
        )  # although the real reach is a dome, we roughly define it as a rectangle

    @staticmethod
    def __within_reach(target_xyz, reach):
        return all(
            axis_reach[0] <= target_axis_val <= axis_reach[1]
            for target_axis_val, axis_reach in zip(target_xyz, reach)
        )

    @staticmethod
    # generate a dictionary of ik value dictionaries,
    # the first level key are depths
    # the second level key are discrete integer tuples, which maps into floating (x,y,z) by
    # tuple / 2^(depth) == (x,y,z)
    # this way we can (A) handle missing values (i.e. unreachable), (B) avoid using floating number as keys
    def __evaluate_ik_cache(
        evaluation_depth,
        reach,
        multiple_initial_positions,
        size=None,
        forward_only=True,
    ):
        ik_caches = {}
        viable = 0
        infeasible = 0
        unit = 2**evaluation_depth

        def to_fix_point(float_vector):
            return tuple(round(float(val), 2) for val in float_vector)

        for target_positions_vector in multiple_initial_positions:  # todo: may add more
            ik_cache = {}
            target_positions = motion.ActuatorPositions(
                positions=target_positions_vector
            )
            target_pose = target_positions.forward_kinematics_ikpy()
            xyz = to_fix_point(target_pose[:3])
            if forward_only:
                target_rpy = to_fix_point(
                    (-90, 0, 0)
                )  # along positive Y, i.e. horizontal forward
            else:
                target_rpy = to_fix_point(target_pose[3:6])
            eval_queue = deque([(xyz, target_positions.actuator_positions)])
            while eval_queue:
                xyz, current_positions = eval_queue.popleft()
                for direction in (-1, 1):
                    for delta in (
                        (direction, 0, 0),
                        (0, direction, 0),
                        (0, 0, direction),
                    ):
                        # solve for each of the 6 directions
                        actual_delta = np.array(delta) * unit
                        target_xyz = to_fix_point(np.array(xyz) + actual_delta)
                        if not ArmControllerIkCache.__within_reach(
                            target_xyz=target_xyz, reach=reach
                        ):
                            continue

                        if target_xyz in ik_cache:
                            if ik_cache[target_xyz] is not None:
                                continue

                        target_pose = np.concatenate([target_xyz, target_rpy])
                        target_pose = motion.EndeffectorPose(pose=target_pose)
                        if forward_only:  # forward only, faster
                            (
                                validated_positions,
                                _,
                            ) = target_pose.inverse_kinematics_ikpy(
                                solver_mode=SolverMode.FORWARD,
                                initial_joint_positions=current_positions,
                            )
                        else:
                            (
                                validated_positions,
                                _,
                            ) = target_pose.inverse_kinematics_ikpy(
                                solver_mode=SolverMode.ALL,
                                initial_joint_positions=current_positions,
                            )
                        if validated_positions is not None:
                            actuator_positions = validated_positions[:-1]
                            ik_cache[target_xyz] = actuator_positions
                            eval_queue.append((target_xyz, actuator_positions))
                            if viable % 100 == 0:
                                # density is a rough estimation because some pose marked infeasible may be solved by other path
                                logging.info(
                                    f"Solved {viable}th point: {target_xyz} @ {actuator_positions} ~{round(viable / (viable+infeasible+1) * 100,2)}% density"
                                )
                            viable += 1
                            if size is not None and size < viable:
                                eval_queue = []
                                break
                        else:
                            ik_cache[target_xyz] = None
                            infeasible += 1
                            logging.debug(
                                f"Failed Solving {viable}th point: {target_xyz}"
                            )
            ik_caches[target_rpy] = ik_cache
        logging.info(
            f"Generated {viable} data points, example ik_cache,\nX: \n{ik_caches}"
        )
        return ik_caches

    def generate_ik_cache(
        self,
        ik_cache_filepath_prefix="~/.ato/ik_cache",
        size=None,
    ):
        ik_cache = ArmControllerIkCache.__evaluate_ik_cache(
            evaluation_depth=self.evaluation_depth,
            reach=self.reach,
            multiple_initial_positions=[self.home_positions],
            size=size,
        )
        file_path = os.path.expanduser(f"{ik_cache_filepath_prefix}_ik_cache.npy")
        with open(file_path, "wb") as fout:
            logging.info(f"Saving ik_cache to {file_path}")
            np.save(fout, ik_cache)
            fout.close()
