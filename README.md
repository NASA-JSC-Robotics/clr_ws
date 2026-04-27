# Sample Containerized Workspace

This empty workspace can be used a starting point for a Docker-enabled workspace using Git submodules.
Users can use this as a starting point for containerized ROS 2 applications.
Start by forking or copying the contents of this project, and modifying as needed!

The contents of the `src` directory should be treated similarly to a "normal" ROS workspace.
That is, source code can be added, submoduled, or imported as needed to `src/`, then be built and run inside of an isolated, ROS enabled environment.

This workflow supports the `jazzy` ROS distro.

## Quick Development Setup

> [!WARNING]
> When forking this repo be sure to update the default prefix and image names.
> This includes what is in `.env.default` and at the top of the Dockerfile.
> The workspace is configured to pull demo images from our internal GitLab by default,
> but this is likely not what every workspace wants!

> [!WARNING] These warnings should not be in forks!
> If you see them then you did a bad merge and you should double check your workspace.

1) [Install Docker](https://docs.docker.com/engine/install/ubuntu/)
    - Don't worry about Docker Desktop
    - For Ubuntu recommend using the [utility script](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script)
2) Fork or copy the contents of this repository as needed

3) Copy `.env.default` in the root of this repo to a new file named just `.env`

    ```bash
    cp .env.default .env
    ```

4) Set your user information for the project build
    - We recommend just putting this in your `~/.bashrc`:
    - `USER_UID` and `USER_GID` (found using `id -u` and `id -g` respectively)

      ```bash
      export USER_UID=$(id -u $USER)
      export USER_GID=$(id -g $USER)
      ```

    Alternatively, edit the contents of the newly created `.env`.

Then follow the instructions below to build and run the application.

## Using the Demo Image

The demo image is based of pre-built images that are pushed to [DockerHub](https://hub.docker.com/r/nasajscrobotics/).

These images contain the fully compiled workspace and can be run out of the box.

To build and launch the demo image, from the workspace root run:

```bash
# Compile (pull) the image
docker compose build

# Start the demo service in the background
docker compose up -d demo

# Launch a bash session in the container
docker compose exec demo bash
```

## Using the Development Image

The development image is built locally starting from a baseline `ros:jazzy` image.

This image is not setup to run once built.

Instead, the user's local workspace is mounted into the container and must be compiled manually.

To build and launch the development image, from the workspace root run:

```bash
# Compile the image
docker compose build dev

# Start it
docker compose up -d dev

# Connect to the console shell
docker compose exec dev bash
```

Once attached to the container, it is usable as a regular colcon workspace.
The contents of the `src/` directory will be mounted into `/home/er4-user/ws/src`.

For example:

```bash
cd ${HOME}/ws
colcon build
source install/setup.bash
```

Once the workspace is built and sourced within the container, ROS 2 executables and launch files can be run.

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

# Or launch an interactive shell and do things "normally"
pixi shell
colcon build
```

Note that any package we are building from source must be included in [pixi.toml](./pixi.toml).

## Other Things to Note

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
