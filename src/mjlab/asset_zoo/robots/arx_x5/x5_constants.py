"""ARX-X5 constants."""

from pathlib import Path

import mujoco

from mjlab import MJLAB_SRC_PATH
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.actuator import (
    ElectricActuator,
    reflect_rotary_to_linear,
)
from mjlab.utils.os import update_assets
from mjlab.utils.spec_config import CollisionCfg

##
# MJCF and assets.
##

X5_XML: Path = (
    MJLAB_SRC_PATH / "asset_zoo" / "robots" / "arx_x5" / "xmls" / "X5.xml"
)
assert X5_XML.exists()


def get_assets(meshdir: str) -> dict[str, bytes]:
    assets: dict[str, bytes] = {}
    update_assets(assets, X5_XML.parent / "assets", meshdir)
    return assets


def get_spec() -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(X5_XML))
    spec.assets = get_assets(spec.meshdir)
    return spec


ARMATURE_EC_A4310 = 0.033  # N=36, Rotor inertia=25.5kg*mm²
ARMATURE_DM_J4310 = 0.0018

NATURAL_FREQ = 10 * 2.0 * 3.1415926535  # 10Hz
DAMPING_RATIO = 2.0

EC_A4310 = ElectricActuator(
    reflected_inertia=ARMATURE_EC_A4310,
    velocity_limit=10.0,
    effort_limit=36.0,  # TODO: Check this
)

#  yam default pd:
#  kp = np.array([80, 80, 80, 40, 10, 10, 10])
#  kd = np.array([5, 5, 5, 1.5, 1.5, 1.5, 0.3])

STIFFNESS_EC_A4310 = ARMATURE_EC_A4310 * NATURAL_FREQ**2
DAMPING_EC_A4310 = 2.0 * DAMPING_RATIO * ARMATURE_EC_A4310 * NATURAL_FREQ

ACTUATOR_EC_A4310 = BuiltinPositionActuatorCfg(
    joint_names_expr=("joint1", "joint2", "joint3"),
    stiffness=STIFFNESS_EC_A4310,
    damping=DAMPING_EC_A4310,
    effort_limit=EC_A4310.effort_limit,
    armature=EC_A4310.reflected_inertia,
)

DM_J4310 = ElectricActuator(
    reflected_inertia=ARMATURE_DM_J4310,
    velocity_limit=30.0,
    effort_limit=10.0,
)

STIFFNESS_DM_J4310 = ARMATURE_DM_J4310 * NATURAL_FREQ**2
DAMPING_DM_J4310 = 2.0 * DAMPING_RATIO * ARMATURE_DM_J4310 * NATURAL_FREQ

ACTUATOR_DM_J4310 = BuiltinPositionActuatorCfg(
    joint_names_expr=("joint4", "joint5", "joint6"),
    stiffness=STIFFNESS_DM_J4310,
    damping=DAMPING_DM_J4310,
    effort_limit=DM_J4310.effort_limit,
    armature=DM_J4310.reflected_inertia,
)

##
# Gripper transmission parameters.
##

GRIPPER_MOTOR_STROKE_CRANK = 3.14  # [rad]: operational motor range (from limits)
GRIPPER_LINEAR_STROKE_CRANK = 0.088  # [m]: design stroke (full mechanism range)
GRIPPER_TRANSMISSION_RATIO_CRANK = (
    GRIPPER_LINEAR_STROKE_CRANK / GRIPPER_MOTOR_STROKE_CRANK
)

# Reflect motor properties to linear gripper joint.
(
    ARMATURE_DM_J4310_LINEAR_CRANK,
    VELOCITY_LIMIT_DM_J4310_LINEAR_CRANK,
    EFFORT_LIMIT_DM_J4310_LINEAR_CRANK,
) = reflect_rotary_to_linear(
    armature_rotary=ARMATURE_DM_J4310,
    velocity_limit_rotary=DM_J4310.velocity_limit,
    effort_limit_rotary=DM_J4310.effort_limit,
    transmission_ratio=GRIPPER_TRANSMISSION_RATIO_CRANK,
)

# PD controller gains.
NATURAL_FREQ_GRIPPER = 2 * 2.0 * 3.1415926535  # 2Hz
STIFFNESS_DM_J4310_LINEAR_CRANK = ARMATURE_DM_J4310_LINEAR_CRANK * NATURAL_FREQ_GRIPPER**2
DAMPING_DM_J4310_LINEAR_CRANK = (
    2.0 * DAMPING_RATIO * ARMATURE_DM_J4310_LINEAR_CRANK * NATURAL_FREQ_GRIPPER
)

# Artificially limit gripper force for sim stability (must also be done on hardware).
EFFORT_LIMIT_DM_J4310_LINEAR_CRANK_SAFE = EFFORT_LIMIT_DM_J4310_LINEAR_CRANK * 0.1

# Only actuate left_finger; right_finger is coupled via equality constraint.
ACTUATOR_DM_J4310_LINEAR_CRANK = BuiltinPositionActuatorCfg(
    joint_names_expr=("left_finger",),
    stiffness=STIFFNESS_DM_J4310_LINEAR_CRANK,
    damping=DAMPING_DM_J4310_LINEAR_CRANK,
    effort_limit=EFFORT_LIMIT_DM_J4310_LINEAR_CRANK_SAFE,
    armature=ARMATURE_DM_J4310_LINEAR_CRANK,
)

##
# Keyframe config.
##
# [0.0, 0.948, 0.858, -0.573, 0.0, 0.0, 0.0]
HOME_KEYFRAME = EntityCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.01),
    joint_pos={
        "joint2": 0.948,
        "joint3": 0.858,
        "joint4": -0.573,
    },
    joint_vel={".*": 0.0},
)

##
# Collision config.
##

GRIPPER_ONLY_COLLISION = CollisionCfg(
    geom_names_expr=(".*_collision",),
    contype={
        "(link6|[lr]f)_.*_collision": 1,
        ".*_collision": 0,
    },
    conaffinity={
        "(link6|[lr]f)_.*_collision": 1,
        ".*_collision": 0,
    },
    condim={
        "[lr]f_down(6|7|8|9|10|11)_collision": 6,
        ".*_collision": 3,
    },
    friction={
        "[lr]f_down(6|7|8|9|10|11)_collision": (1, 5e-3, 5e-4),
        ".*_collision": (0.6,),
    },
    solref={
        "[lr]f_down(6|7|8|9|10|11)_collision": (0.01, 1),
    },
    priority={
        "[lr]f_down(6|7|8|9|10|11)_collision": 1,
    },
)

##
# Final config.
##

# Note: Actuators are defined in X5.xml with real robot PD values
# Do not add Python actuators to avoid duplication
ARTICULATION = EntityArticulationInfoCfg(
    actuators=(),  # Empty - using XML actuators only
    soft_joint_pos_limit_factor=0.9,
)


def get_x5_robot_cfg() -> EntityCfg:
    return EntityCfg(
        init_state=HOME_KEYFRAME,
        collisions=(GRIPPER_ONLY_COLLISION,),
        spec_fn=get_spec,
        articulation=ARTICULATION,
    )


# Action scale based on XML actuator PD values (matching real robot)
# kp values from X5.xml: [80, 80, 80, 40, 10, 10, 5]
# effort_limit estimated from real robot

# X5_ACTION_SCALE: dict[str, float] = {}
# for a in ARTICULATION.actuators:
#     assert isinstance(a, BuiltinPositionActuatorCfg)
#     e = a.effort_limit
#     s = a.stiffness
#     names = a.joint_names_expr
#     assert e is not None
#     for n in names:
#         X5_ACTION_SCALE[n] = 0.25 * e / s

X5_ACTION_SCALE: dict[str, float] = {
    "joint1": 0.25 * 36.0 / 80.0,   # ~0.1125
    "joint2": 0.25 * 36.0 / 80.0,   # ~0.1125
    "joint3": 0.25 * 36.0 / 80.0,   # ~0.1125
    "joint4": 0.25 * 10.0 / 40.0,   # ~0.0625
    "joint5": 0.25 * 10.0 / 10.0,   # ~0.25
    "joint6": 0.25 * 10.0 / 10.0,   # ~0.25
    "left_finger": 0.25 * 1.0 / 5.0,  # ~0.05
}

if __name__ == "__main__":
    import mujoco
    import mujoco.viewer as viewer

    from mjlab.entity import Entity

    robot = Entity(get_x5_robot_cfg())
    model = robot.spec.compile()

    # Print actuator kp and kd values
    print("\n=== X5 Actuator PD Gains ===")
    print(f"{'Actuator':<20} {'Joint':<15} {'kp':<12} {'kd':<12}")
    print("-" * 60)
    for i in range(model.nu):
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        # Get the joint this actuator controls
        trnid = model.actuator_trnid[i, 0]
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, trnid)
        
        # For position actuators: gainprm[0] = kp, biasprm[2] = -kd (or kv)
        kp = model.actuator_gainprm[i, 0]
        # biasprm layout: [bias_term, -kp, -kd]
        kd = -model.actuator_biasprm[i, 2] if model.actuator_biasprm[i, 2] != 0 else 0
        
        print(f"{act_name:<20} {joint_name:<15} {kp:<12.4f} {kd:<12.4f}")
    
    print("\n=== Joint Ranges ===")
    print(f"{'Joint':<15} {'Lower':<12} {'Upper':<12}")
    print("-" * 40)
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if model.jnt_limited[i]:
            lower, upper = model.jnt_range[i]
            print(f"{joint_name:<15} {lower:<12.4f} {upper:<12.4f}")
    
    print()
    viewer.launch(model)
