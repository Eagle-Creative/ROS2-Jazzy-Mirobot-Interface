# mirobot_moveit_config

MoveIt 2 planning configuration for the WLKATA Mirobot (ROS 2 Jazzy). **Planning only**: trajectory
execution is disabled because the Mirobot has no `ros2_control` interface; see the repository
README ("MoveIt 2 Motion Planning") for executing plans through the G-code driver and for the
geometry measured on a real unit.

```bash
ros2 launch mirobot_moveit_config move_group.launch.py robot_name:=mirobot_01 tool_length:=0.035
```

| File | Content |
|---|---|
| `urdf/mirobot_moveit.urdf.xacro` | `mirobot_description/urdf/mirobot_urdf_2.urdf` + `mirobot_tcp` (argument `tool_length`) |
| `srdf/mirobot.srdf` | group `mirobot_arm` (`base_link` → `mirobot_tcp`), state `home`, disabled collision pairs |
| `config/kinematics.yaml` | KDL |
| `config/joint_limits.yaml` | velocity/acceleration limits (shape the planned timing only) |
| `config/pilz_*.yaml`, `config/ompl_planning.yaml` | planning pipelines (Pilz `PTP` is the default) |
| `launch/move_group.launch.py` | `robot_state_publisher` + `move_group`, joint states from `/<robot_name>/joint_states` |
| `tools/collision_census.py` | samples random states against a running `move_group` and lists link pairs that always/never collide |

`tool_length` is measured from the firmware's Cartesian point (8 mm beyond `Link6` = the J6
flange face where the tool is bolted on) to the tool tip; the default 0.035 is the WLKATA suction
cup assembly. (Before 2026-10-26 the default was 0.0575, 21 mm too long - see the main README.)

Self-collision: only adjacent links are disabled. A census of 1500 random states found no
collision at home and only `base_link` against `Link4`/`Link5`/`Link6`/`ee_link` in folded poses,
so no further pairs are disabled.
