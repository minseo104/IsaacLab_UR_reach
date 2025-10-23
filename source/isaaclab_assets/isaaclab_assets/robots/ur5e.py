# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Universal Robots.

The following configuration parameters are available:

* :obj:`UR10_CFG`: The UR10 arm without a gripper.

Reference: https://github.com/ros-industrial/universal_robot
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##


UR5E_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        #usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/UniversalRobots/UR10/ur10_instanceable.usd",
        #usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/UniversalRobots/UR10/ur10_instanceable.usd",

        #asset_path="/workspace/isaaclab/ur_robotiq_usd/",
        #usd_dir="/workspace/isaaclab/ur_robotiq_usd/",
        usd_path="/workspace/isaaclab/ur_robotiq_usd/Collected_ur5e/ur5e.usd",    #Working
        #usd_path="/workspace/isaaclab/source/isaaclab_assets/data/Robots/UR5E/ur_robotiq.usd",

        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": -1.57,
            "elbow_joint": 1.57,
            "wrist_1_joint": -1.57,
            "wrist_2_joint": -1.57,
            "wrist_3_joint": 0,
            #"hande_left_finger_joint": 0.024,
            #"hande_right_finger_joint": 0.024,

        },
    ),
    actuators={
        "shoulder_pan": ImplicitActuatorCfg(
            joint_names_expr=["shoulder_pan_joint"],
            velocity_limit=100.0,
            effort_limit=87.0,
            stiffness=800.0,
            damping=40.0,
        ),
        "shoulder_lift": ImplicitActuatorCfg(
            joint_names_expr=["shoulder_lift_joint"],
            velocity_limit=100.0,
            effort_limit=87.0,
            stiffness=800.0,
            damping=40.0,
        ),
        "elbow_joint": ImplicitActuatorCfg(
            joint_names_expr=["elbow_joint"],
            velocity_limit=100.0,
            effort_limit=87.0,
            stiffness=800.0,
            damping=40.0,
        ),
        "wrist_1": ImplicitActuatorCfg(
            joint_names_expr=["wrist_1_joint"],
            velocity_limit=100.0,
            effort_limit=87.0,
            stiffness=800.0,
            damping=40.0,
        ),
        "wrist_2": ImplicitActuatorCfg(
            joint_names_expr=["wrist_2_joint"],
            velocity_limit=100.0,
            effort_limit=87.0,
            stiffness=800.0,
            damping=40.0,
        ),
        "wrist_3": ImplicitActuatorCfg(
            joint_names_expr=["wrist_3_joint"],
            velocity_limit=100.0,
            effort_limit=87.0,
            stiffness=800.0,
            damping=40.0,
        ),        
    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of UR-10 arm using implicit actuator models."""
