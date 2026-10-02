# Repository Guidelines

## Project Structure & Module Organization

This is a ROS 1/ROS 2 OpenVINS workspace source tree. `ov_core/` contains camera models, feature tracking, and shared utilities; `ov_init/` handles initialization; `ov_msckf/` contains the estimator, ROS nodes, and launch files; `ov_eval/` provides evaluation tools. Dataset calibration and estimator YAML files live in `config/<dataset>/`. `ov_data/` holds reference trajectories and simulation data. Put new C++ files beside the component they extend; keep dataset-specific settings in `config/` rather than embedding them in estimator code.

## Build, Test, and Development Commands

From the workspace root (`/home/he/open_vins_ltv_ws`), run:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to ov_msckf
source install/setup.bash
ros2 launch ov_msckf subscribe.launch.py rviz_enable:=true
```

The build includes `ov_msckf` and its package dependencies. The launch command runs the configured EuRoC bag; override the local default with `bag_path:=/path/to/sequence_db`, or use `bag_play:=false` for live topics. Set `dosave:=true` to write an estimated trajectory.

## Coding Style & Naming Conventions

C++ targets require C++14. Format changed C++ files with `clang-format -style=file`; `.clang-format` specifies two-space indentation and a 140-column limit. Follow nearby naming: `CamelCase` types, `snake_case` functions and variables, and frame-aware geometry names such as `R_ItoC` and `p_IinG`. Keep launch arguments and YAML keys descriptive and consistent with the estimator's parameter names.

## Testing Guidelines

Tests are standalone `test_*.cpp` executables under package `src/` directories; they are not registered as a comprehensive CTest suite. Build the affected package, then run the relevant executable with a configuration path, for example `ros2 run ov_msckf test_sim_repeat /absolute/path/to/config/rpng_sim/estimator_config.yaml`. For ROS or launch changes, replay a short dataset and confirm initialization plus the expected topics or output files. Report the dataset, command, and result.

## Commit & Pull Request Guidelines

The visible history has one import commit, so it establishes no commit-message convention. Use a concise imperative subject naming the affected component. In pull requests, explain the behavior change, link any relevant issue, list build and runtime checks, and include RViz screenshots only when visualization changes. Keep generated build files, logs, and large bags out of commits.
