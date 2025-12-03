from mjlab.tasks.registry import register_mjlab_task

from .env_cfgs import arx5_lift_cube_env_cfg
from .rl_cfg import arx5_lift_cube_ppo_runner_cfg

register_mjlab_task(
  task_id="Mjlab-Lift-Cube-Arx5",
  env_cfg=arx5_lift_cube_env_cfg(),
  play_env_cfg=arx5_lift_cube_env_cfg(play=True),
  rl_cfg=arx5_lift_cube_ppo_runner_cfg(),
)
