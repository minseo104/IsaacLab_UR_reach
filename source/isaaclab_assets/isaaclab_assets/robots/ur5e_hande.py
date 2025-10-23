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


UR5E_ROBOTIQ_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(

        usd_path="/workspace/isaaclab/source/isaaclab_assets/data/Robots/UR5E/ur_robotiq.usd",

        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        #Disable
        #articulation_props=sim_utils.ArticulationRootPropertiesCfg(
        #    enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        #),        
        #activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": -1.57,
            "elbow_joint": 1.57,
            "wrist_1_joint": -1.57,
            "wrist_2_joint": -1.57,
            "wrist_3_joint": 0,
            "hande_finger_distance": 0.0,
            "hande_left_finger_joint": 0.024,
            "hande_right_finger_joint": 0.024,

        },
    ),
    actuators={
        "shoulder_pan": ImplicitActuatorCfg(
            joint_names_expr=["shoulder_pan_joint"],
            velocity_limit=1.0,
            effort_limit=87.0,
            stiffness=50.0,
            damping=10.0,
        ),
        "shoulder_lift": ImplicitActuatorCfg(
            joint_names_expr=["shoulder_lift_joint"],
            velocity_limit=1.0,
            effort_limit=87.0,
            stiffness=70.0,
            damping=30.0,
        ),
        "elbow_joint": ImplicitActuatorCfg(
            joint_names_expr=["elbow_joint"],
            velocity_limit=1.0,
            effort_limit=87.0,
            stiffness=50.0,
            damping=10.0,
        ),
        "wrist_1": ImplicitActuatorCfg(
            joint_names_expr=["wrist_1_joint"],
            velocity_limit=1.0,
            effort_limit=87.0,
            stiffness=10.0,
            damping=1.0,
        ),
        "wrist_2": ImplicitActuatorCfg(
            joint_names_expr=["wrist_2_joint"],
            velocity_limit=1.0,
            effort_limit=100.0,
            stiffness=100.0,
            damping=40.0,
        ),
        "wrist_3": ImplicitActuatorCfg(
            joint_names_expr=["wrist_3_joint"],
            velocity_limit=1.0,
            effort_limit=100.0,
            stiffness=10.0,
            damping=0.10,
        ),
        "hande_finger": ImplicitActuatorCfg(
            joint_names_expr=["hande_finger_distance"],
            effort_limit=12.0,
            velocity_limit=1,
            stiffness=20,
            damping=2,
        ),             
        "hande_left": ImplicitActuatorCfg(
            joint_names_expr=["hande_left_finger_joint"],
            effort_limit=100.0,
            velocity_limit=1,
            stiffness=10,
            damping=1,
        ),       
        "hande_right": ImplicitActuatorCfg(
            joint_names_expr=["hande_right_finger_joint"],
            effort_limit=100.0,
            velocity_limit=1,
            stiffness=10,
            damping=1,
        ),         

    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of UR-10 arm using implicit actuator models."""


UR5E_ROBOTIQ_HIGH_PD_CFG = UR5E_ROBOTIQ_CFG.copy()
UR5E_ROBOTIQ_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
UR5E_ROBOTIQ_HIGH_PD_CFG.actuators["shoulder_pan"].stiffness = 400.0
UR5E_ROBOTIQ_HIGH_PD_CFG.actuators["shoulder_lift"].damping = 80.0
UR5E_ROBOTIQ_HIGH_PD_CFG.actuators["elbow_joint"].stiffness = 400.0
UR5E_ROBOTIQ_HIGH_PD_CFG.actuators["wrist_1"].damping = 80.0
