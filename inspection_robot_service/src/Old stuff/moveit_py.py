#!/usr/bin/env python3

import time
import rclpy
from rclpy.logging import get_logger

from moveit.planning import MoveItPy

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

def main():
    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py_planning_scene")

    # instantiate MoveItPy instance and get planning component
    panda = MoveItPy(node_name="moveit_py_planning_scene")
    panda_arm = panda.get_planning_component("panda_arm")
    planning_scene_monitor = panda.get_planning_scene_monitor()
    logger.info("MoveItPy instance created")

    # ###################################################################
    # # Plan with collision objects
    # ###################################################################

    # add_collision_objects(planning_scene_monitor)
    # panda_arm.set_start_state(configuration_name="ready")
    # panda_arm.set_goal_state(configuration_name="extended")
    # plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)

    # ###################################################################
    # # Check collisions
    # ###################################################################
    # with planning_scene_monitor.read_only() as scene:
    #     robot_state = scene.current_state
    #     original_joint_positions = robot_state.get_joint_group_positions("panda_arm")

    #     # Set the pose goal
    #     pose_goal = Pose()
    #     pose_goal.position.x = 0.25
    #     pose_goal.position.y = 0.25
    #     pose_goal.position.z = 0.5
    #     pose_goal.orientation.w = 1.0

    #     # Set the robot state and check collisions
    #     robot_state.set_from_ik("panda_arm", pose_goal, "panda_hand")
    #     robot_state.update()  # required to update transforms
    #     robot_collision_status = scene.is_state_colliding(
    #         robot_state=robot_state, joint_model_group_name="panda_arm", verbose=True
    #     )
    #     logger.info(f"\nRobot is in collision: {robot_collision_status}\n")

    #     # Restore the original state
    #     robot_state.set_joint_group_positions(
    #         "panda_arm",
    #         original_joint_positions,
    #     )
    #     robot_state.update()  # required to update transforms

    # time.sleep(3.0)

    # ###################################################################
    # # Remove collision objects and return to the ready pose
    # ###################################################################

    # with planning_scene_monitor.read_write() as scene:
    #     scene.remove_all_collision_objects()
    #     scene.current_state.update()

    # panda_arm.set_start_state_to_current_state()
    # panda_arm.set_goal_state(configuration_name="ready")
    # plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)


if __name__ == "__main__":
    main()