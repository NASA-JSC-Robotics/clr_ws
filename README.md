# Sample Containerized Workspace

This empty workspace can be used a starting point for a Docker-enabled workspace using Git submodules.
The contents of the `src` directory should be treated similarly to a "normal" ROS workspace.
That is, source code can be imported and added as needed to `src/`, then be built and run inside of an isolated, ROS enabled environment.

This workflow has been tested against the `jazzy` ROS distro.
To change ROS versions, update the `ROS2_DISTRO` variable in your environment.
Note the `2`! As this is intended to be isolated from your system.

## Quick Development Setup

1) [Install Docker](https://docs.docker.com/engine/install/ubuntu/)
    - Don't worry about Docker Desktop
    - For Ubuntu recommend using the [utility script](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script)
2) Fork or copy the contents of this repository as needed
3) Setup your source code for the `src/` directory
    - Either with git submodules (`git submodule add ...`)
    - Or with a repos file and vcs tool  (`vcs import ...`)
4) Set your user information for the project build
    - We recommend just putting this in your `~/.bashrc`:

      ```bash
      export USER_UID=$(id -u $USER)
      export USER_GID=$(id -g $USER)
      ```

    - Alternatively, open the `.env` file in the root of this repo and update each line with your information
        - `USER_UID` and `USER_GID`
            - found using `id -u` and `id -g` respectively

## Using the Images

Build the base images using the compose specification.

To build the development image from the repo root, and then launch it

```bash
# Compile the image
docker compose build

# Start it
docker compose up dev -d

# Connect to the console
docker compose exec dev bash
```

Once you're attached to the container, you can use it as a regular colcon workspace.
The contents of the `src/` directory will be mounted into `/home/er4-user/ws/src`.

## The Pixi Workflow

We also provide a [pixi/robostack](https://prefix.dev) build for compiling on baremetal in consistent, isolated environments.
Be sure to install the latest (after 0.65.0) release of the tool.
The build relies on the [pixi-build-ros](https://prefix-dev.github.io/pixi-build-backends/backends/pixi-build-ros/) backend for compatibility with our ROS projects.

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
This is a bit annoying, but it can be updated using a script to find all `package.xmls` and name them accordingly:

```bash
# Be sure to ignore env files or otherwise. We only want package.xmls from
# packages compiled in the workspace.
find src/ -name package.xml | grep -v '\.pixi/' | while read f; do
  pkg=$(grep -oP '(?<=<name>)[^<]+' "$f")
  pkg_key="ros-jazzy-$(echo "$pkg" | tr '_' '-')"
  echo "$pkg_key = { path = \"./$f\" }"
done | sort
```

## Other Things to Note

- Build logs, compiled artifaces, and the `.ccache` are also mounted in the workspace/user home.
This ensure artifacts are persisted even when restarting or recreating the container.

- The `.bash` folder gets mounted into your workspace, and the environment variable `HISTFILE` is set in the docker compose file.
This points the bash to keep the history in this folder, which will persist between docker container sessions so that your history is kept.

- Your host's DDS configuration (either cyclone or fastrtps) will be mounted into the image if set in your environment.
For more information refer to the [compose specification](docker-compose.yaml).

- Defaults for `colcon build` are set for the user. To change or modify, refer to the [defaults file](config/colcon-defaults.yaml).

- We use [MuJoCo](https://mujoco.readthedocs.io/en/stable/XMLreference.html) for many of our dynamic simulations, so we include installing in the [Dockerfile](./Dockerfile).

## Troubleshooting

Common pitfalls and troubleshooting tips are documented in the [troubleshooting guide](./docs/TROUBLESHOOTING.md).
