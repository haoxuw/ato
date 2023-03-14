# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.
import logging
import queue
from datetime import datetime

import numpy as np
from control import arm_controller, motion
from control.config_and_enums.controller_enums import SolverMode


# controlling arm with cached IK (rather than solving on the fly) has two main benefits
# 1. reduce computation delay, especially when running on device
# 2. using BFS search from the home position makes sure not only the IK solution is feasible in isolation,
# but also reachable with minimal joints delta from the previous pose, tracing back to home pose
# Drawback: during runtime, each ik has to be approximated using linear interpolation, while joint positions
# and endeffector pose does not share a linear relationship
class ArmControllerIkCache(arm_controller.ArmController):
    def __init__(
        self,
        **kwargs,
    ):
        super().__init__(**kwargs)

        # grouped_segment_lengths = (base_link_fixed_to_mount, (segment_1_movable ...), gripper)
        grouped_segment_lengths = (
            self._indexed_segment_lengths[0],
            self._indexed_segment_lengths[1:],
            self._gripper_length,
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
    def solve_target_pose(robot_chain, target_pose, current_positions, forward_only):
        if forward_only:  # forward only, faster
            solver_mode = SolverMode.FORWARD
        else:
            solver_mode = SolverMode.ALL
        validated_positions = motion.EndeffectorPose.inverse_kinematics_ikpy(
            robot_chain=robot_chain,
            target_pose=target_pose,
            solver_mode=solver_mode,
            initial_joint_positions=current_positions,
        )
        return validated_positions

    @staticmethod
    # generate a dictionary of ik value dictionaries,
    # the first level key are depths
    # the second level key are discrete integer tuples, which maps into floating (x,y,z) by
    # tuple / 2^(depth) == (x,y,z)
    # this way we can (A) handle missing values (i.e. unreachable), (B) avoid using floating number as keys
    def __evaluate_ik_cache(
        reach,
        multiple_initial_positions,
        evaluation_unit=64,
    ):
        # initialize utility data structures
        task_queue = queue.Queue()
        ik_caches = {
            "feasible": 0,
            "infeasible": 0,
            "evaluation_unit": evaluation_unit,
        }

        robot_chain = motion.EndeffectorPose.get_ikpy_robot_chain()
        for (
            initial_positions_vector
        ) in multiple_initial_positions:  # todo: may add more
            initial_positions = motion.ActuatorPositions(
                joint_positions=initial_positions_vector
            )
            initial_pose_vector = initial_positions.forward_kinematics_ikpy(
                robot_chain=robot_chain
            ).pose
            initial_xyz = motion.EndeffectorPose.to_fix_point(initial_pose_vector[:3])
            initial_rpy = motion.EndeffectorPose.to_fix_point(initial_pose_vector[3:6])

            ik_caches[initial_rpy] = {
                "initial_xyz": initial_xyz,
                "initial_positions": initial_positions_vector,
                "ik_cache": {},
            }

            # initialize
            task_queue.put((initial_rpy, initial_xyz, initial_positions_vector))

        kwargs = {
            "evaluation_unit": evaluation_unit,
            "ik_caches": ik_caches,
            "task_queue": task_queue,
            "reach": reach,
        }
        ik_caches = ArmControllerIkCache.evaluate_ik_point(**kwargs)

        logging.info(
            f"Generated {ik_caches['feasible']} data points out of {ArmControllerIkCache.solver_counter}, ik_cache.keys() == \n{ik_caches.keys()}"
        )
        return ik_caches

    solver_counter = 0

    @staticmethod
    def evaluate_ik_point(
        evaluation_unit,
        ik_caches,
        task_queue,
        reach,
    ):
        robot_chain = motion.EndeffectorPose.get_ikpy_robot_chain()
        total_duration = 0
        total_points = 1

        while not task_queue.empty():
            task = task_queue.get()

            target_rpy, current_xyz, current_positions = task
            for direction in (-1, 1):
                for delta in (
                    (direction, 0, 0),
                    (0, direction, 0),
                    (0, 0, direction),
                ):
                    # solve for each of the 6 directions
                    actual_delta = np.array(delta) * evaluation_unit
                    target_xyz = motion.EndeffectorPose.to_fix_point(
                        np.array(current_xyz) + actual_delta
                    )
                    if not ArmControllerIkCache.__within_reach(
                        target_xyz=target_xyz, reach=reach
                    ):
                        continue

                    ArmControllerIkCache.solver_counter += 1

                    if target_xyz in ik_caches[target_rpy]["ik_cache"]:
                        if ik_caches[target_rpy]["ik_cache"][target_xyz] is not None:
                            continue

                    assert target_rpy == (-90, 0, 0), "Not supported"
                    target_pose = motion.EndeffectorPose(
                        pose=np.concatenate([target_xyz, target_rpy])
                    )

                    start_time = datetime.now()
                    validated_positions = ArmControllerIkCache.solve_target_pose(
                        robot_chain=robot_chain,
                        target_pose=target_pose,
                        current_positions=current_positions,
                        forward_only=(target_rpy == (-90, 0, 0)),
                    )
                    duration = (datetime.now() - start_time).total_seconds()
                    total_duration += duration
                    total_points += 1

                    if validated_positions is not None:
                        joint_positions = validated_positions.joint_positions
                        ik_caches[target_rpy]["ik_cache"][target_xyz] = joint_positions
                        task_queue.put((target_rpy, target_xyz, joint_positions))

                        # update stats
                        if ik_caches["feasible"] % 1000 == 0:
                            # density is a rough estimation because some pose marked infeasible may be solved by other path
                            density = round(
                                ik_caches["feasible"]
                                / (ik_caches["feasible"] + ik_caches["infeasible"] + 1)
                                * 100,
                                2,
                            )
                            avg_time = round(total_duration * 1000 / total_points, 2)
                            logging.info(
                                f"Solved {ik_caches['feasible']}th point: {target_xyz} @ {validated_positions} ~{density}% density, avg_time {avg_time}ms"
                            )
                        ik_caches["feasible"] += 1
                    else:
                        ik_caches[target_rpy]["ik_cache"][target_xyz] = None

                        # update stats
                        ik_caches["infeasible"] += 1
                        logging.debug(
                            f"Failed Solving {ik_caches['feasible']}th point: {target_xyz}"
                        )
        return ik_caches

    def generate_ik_cache(self, evaluation_unit=128):
        ik_caches = ArmControllerIkCache.__evaluate_ik_cache(
            reach=self.reach,
            multiple_initial_positions=[self.home_positions[: self.num_segments * 2]],
            evaluation_unit=evaluation_unit,
        )

        file_path = self._get_ik_cache_file_path(
            ik_cache_filepath_prefix=self._ik_cache_filepath_prefix
        )
        with open(file_path, "wb") as fout:
            logging.info(f"Saving ik_cache to {file_path}")
            np.save(fout, ik_caches)
            fout.close()

        # logging.error(ik_caches[(-90, 0, 0)]["ik_cache"])
