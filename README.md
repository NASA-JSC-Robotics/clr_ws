# ChonkUR L Rail-E Workspace

This workspace includes open source resources for using the ChonkUR L Rail-E robot system,
part of the [iMETRO Facility](https://ntrs.nasa.gov/citations/20240013956) at NASA's Johnson Space Center.

The system consists of a UR10e robot mounted on top of an Ewellix column lift and Vention rail.
Peripherals include a Robotiq Hand-E gripper with custom printed fingers and a wrist mounted Realsense D435 camera.
Descriptions of commonly used environmental components are also included, such as hatches, storage benches, or Merlin Freezes.
The mock-ups here can be used by anyone to develop or test robot applications for space logistics in our hardware environment.

![alt text](./docs/imetro_sim_real.png "iMETRO Environment and Simulation")

This workspace bundles all required git submodules into a Docker containerized workflow that is identical to that which we run on hardware.
While not required, we recommend using our Dockerfiles for consistent environment setup.
Alternatively, individual packages and submodules can be added or extracted from the `src` directory as needed by the user.

## Quick Development Setup

1) [Install Docker](https://docs.docker.com/engine/install/ubuntu/)
    - Don't worry about Docker Desktop
    - For Ubuntu recommend using the [utility script](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script)
2) Fork or copy the contents of this repository as needed.
NASA internal users should refer to confluence for how to setup authentication to GitHub.

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

    - Alternatively, open the `.env` file in the root of this repo and update each line with your information
        - `USER_UID` and `USER_GID`
            - found using `id -u` and `id -g` respectively

Then follow the instructions below to build and run the application.

## Using the Development Image

Build the base images using the compose specification.

To build the development image from the repo root, and then launch it

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

A basic kinematic simulation with the description files, as well as a MoveIt configuration is included.
To start those applications from inside the container,

```bash
# In one terminal launch the simulated environment
ros2 launch clr_deploy clr_sim.launch.py

# In another shell launch the moveit interface and move group nodes
ros2 launch clr_moveit_config clr_moveit.launch.py
```

Additionally, a dynamic simulation of the CLR and mockups environment built with MuJoCo is available.
To run,

```bash
# Start the mujoco ros2 control-based simulation
ros2 launch clr_mujoco_config clr_mujoco.launch.py

# In another shell launch the moveit interface with sim parameters set
ros2 launch clr_moveit_config clr_moveit.launch.py include_mockups_in_description:=true use_sim_time:=true
```

More information about the dynamic simulation is available in the [project](https://github.com/NASA-JSC-Robotics/chonkur_l_raile) and [drivers](https://github.com/NASA-JSC-Robotics/mujoco_ros2_simulation) packages.

## Using the Hardware Image

The [compose file](docker-compose.yml) includes one additional runtime target, `hw`, for running on the physical robot.
This service extends the `dev` service by adding necessary configuration for interacting with ChonkUR's hardware.
It is built and run identically to the `dev` target,

```bash
# Compile the image
docker compose build hw

# Start it
docker compose up hw -d

# Connect to the console shell
docker compose exec hw bash
```

Then use it in the same way as the development image, with the added hardware connections.
When running ChonkUR or all of CLR, launching the hardware is a two step process to ensure that the UR pendant is running fully remotely:

```bash
# Start the dashboard client and other UR tools prior to launching the ROS 2 HW drivers
ros2 launch chonkur_deploy ur_tools.launch.py

# Then start the relevant hardware interface
ros2 launch clr_deploy clr_hw.launch.py
```

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
