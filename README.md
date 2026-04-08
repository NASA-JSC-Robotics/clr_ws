# CLR Demonstration Workspace

This workspace includes open source resources for using the ChonkUR L Rail-E robot system,
part of the [iMETRO Facility](https://ntrs.nasa.gov/citations/20240013956) at NASA's Johnson Space Center.

The system consists of a UR10e robot mounted on top of an Ewellix column lift and Vention rail.
Peripherals include a Robotiq Hand-E gripper with custom printed fingers and a wrist mounted Realsense camera.
Descriptions of commonly used environmental components are also included, such as hatches, storage benches, or Merlin Freezes.
The mock-ups here can be used by anyone to develop or test robot applications for space logistics in our hardware environment.

![alt text](./docs/imetro_sim_real.png "iMETRO Environment and Simulation")

This workspace bundles all required git submodules into a Docker containerized workflow that is identical to that which we run on hardware.
While not required, we recommend using our Dockerfiles for consistent environment setup.
Alternatively, individual packages and submodules can be added or extracted from the `src` directory as needed by the user.

In addition to the base packages, it adds multiple submodules for running demonstrations with the CLR system, both on hardware an with the dynamic MuJoCo simulation.
For more information, refer to the documentation in [clr_sim_demos](https://github.com/NASA-JSC-Robotics/clr_sim_demos).

This workflow has been tested against the `jazzy` ROS distro.
Note the `2`! As this is intended to be isolated from your system.

## Quick Development Setup

1) [Install Docker](https://docs.docker.com/engine/install/ubuntu/)
    - Don't worry about Docker Desktop
    - For Ubuntu recommend using the [utility script](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script)
2) Fork or copy the contents of this repository as needed.
NASA internal users should refer to confluence for how to setup authentication to GitHub.

    **_NOTE:_**  This repository uses LFS for mesh file storage, be sure it is installed with:

    ```bash
    sudo apt-get install git-lfs
    ```

    Then,

    ```bash
    # Clone with submodules
    git clone --recursive https://github.com/NASA-JSC-Robotics/clr_ws.git

    # Or initialize them from the repo's root
    cd clr_ws
    git submodule update --init
    ```

3) Set your user information for the project build
    - We recommend just putting this in your `~/.bashrc`:

      ```bash
      export USER_UID=$(id -u $USER)
      export USER_GID=$(id -g $USER)
      ```

    - Alternatively, copy `.env.default` file in the root of this repo to a new file named just `.env` and update each line with your information
        - `USER_UID` and `USER_GID`
    - Alternatively, copy `.env.default` file in the root of this repo to a new file named just `.env` and update each line with your information
        - `USER_UID` and `USER_GID`
        - found using `id -u` and `id -g` respectively

       ```bash
      cp .env.default .env
      ```   

Then follow the instructions below to build and run the application.

## Using the Demo Image

The demo image is based of pre-built images that are pushed to our [Docker Hub](https://hub.docker.com/r/nasajscrobotics/clr_ws).

These images contain the fully compiled workspace and can be run out of the box.

To build and launch the demo image, from the workspace root run:

```bash
# Compile (pull) the image
docker compose build

# Start the demo service in the background
docker compose up -d demo

# Run the bash shell on the demo service container
docker compose exec demo bash
```

Once inside the demo container you will be able to run applications our of the clr_ws.

Launch files for our supported simulation environments are included:

- CLR kinematic sim
    ```bash
    # In one terminal launch the kinematic simulation environment
    ros2 launch clr_deploy clr_sim.launch.py

    # then open another instance of bash inside the demo container and run MoveIt
    ros2 launch clr_moveit_config clr_moveit.launch.py
    ```

- MuJoCo CLR dynamic sim
    ```bash
    # In one terminal launch the MuJoCo dynamic simulation environment and CLR controllers
    ros2 launch clr_mujoco_config clr_mujoco.launch.py

    # next open another instance of bash inside the demo container and run MoveIt
    ros2 launch clr_moveit_config clr_moveit.launch.py include_mockups_in_description:=true use_sim_time:=true
    ```

## Using the Development Image

The development image is built locally starting from a baseline `ros:${ROS2-DISTRO}` image.

This image is not setup to run once built.

Instead, the user's local workspace is mounted to the container and must be built at runtime.

To build and launch the development image, from the workspace root run:

```bash
# Compile the image
docker compose build dev

# Start it
docker compose up dev -d

# Connect to the console shell
docker compose exec dev bash
```

Once you're attached to the container, you can use it as a regular colcon workspace.
The contents of the `src/` directory will be mounted into `/home/er4-user/ws/src`.

For example:

```bash
cd ${HOME}/ws
colcon build
source install/setup.bash
```

Once the workspace is built and sourced within the container, ROS 2 executables and launch files can be ran.

## Using the Hardware

Running the system on hardware is more involved than running simulations.

Please see [HARDWARE.md](HARDWARE.md) for details.

## The Pixi Workflow

We also provide a [pixi/robostack](https://prefix.dev) build for compiling on baremetal in consistent, isolated environments.
Be sure to install the latest (after 0.65.0) release of the tool.
The build relies on the [pixi-build-ros](https://prefix-dev.github.io/pixi-build-backends/backends/pixi-build-ros/) backend for compatibility with our ROS projects.

This is an experimental workflow that is not as tested as the Docker build methods.
For more information on pixi refer to the [instructions](./docs/USING_PIXI.md).

To install and run with pixi:

```bash
# Install the frozen environment and configure colcon
pixi install --frozen
pixi run setup-colcon

# Build and test
pixi run build
pixi run test
```

Alternatively, launch an interactive shell and do things "normally":

```bash
# Launch the shell and compile the workspace
pixi shell
colcon build

# Source the workspace and launch an application
source install/setup.bash
ros2 launch clr_mujoco_config clr_mujoco.launch.py
```

Note that any package we are building from source must be included in [pixi.toml](./pixi.toml).

## Important Notes

- Build logs, compiled artifaces, and the `.ccache` are also mounted in the workspace/user home.
This ensure artifacts are persisted even when restarting or recreating the container.

- The `.bash` folder gets mounted into your workspace, and the environment variable `HISTFILE` is set in the docker compose file.
This points the bash to keep the history in this folder, which will persist between docker container sessions so that your history is kept.

- Your host's DDS configuration (either cyclone or fastrtps) will be mounted into the image if set in your environment.
For more information refer to the [compose specification](docker-compose.yaml).

- Defaults for `colcon build` are set for the user. To change or modify, refer to the [defaults file](config/colcon-defaults.yaml).

- We use [MuJoCo](https://mujoco.readthedocs.io/en/stable/XMLreference.html) for many of our dynamic simulations, so we include installing in the [Dockerfile](./Dockerfile).

- If you have an NVIDIA or other graphics card, you will have to complete additional configuration steps to use the docker container.
Please refer to the [troubleshooting guide](./docs/TROUBLESHOOTING.md#slow-rendering) for more information.

## Troubleshooting

Common pitfalls and troubleshooting tips are documented in the [troubleshooting guide](./docs/TROUBLESHOOTING.md).

## Citation

This project falls under the purview of the iMETRO project.
If you use this in your own work, please cite the following paper:

```bibtex
@INPROCEEDINGS{imetro-facility-2025,
  author={Dunkelberger, Nathan and Sheetz, Emily and Rainen, Connor and Graf, Jodi and Hart, Nikki and Zemler, Emma and Azimi, Shaun},
  booktitle={2025 22nd International Conference on Ubiquitous Robots (UR)},
  title={Design of the iMETRO Facility: A Platform for Intravehicular Space Robotics Research},
  year={2025},
  volume={},
  number={},
  pages={390-397},
  keywords={NASA;Moon;Seals;Maintenance engineering;Maintenance;Robots;Standards;Open source software;Testing;Logistics},
  doi={10.1109/UR65550.2025.11077983}}
```
