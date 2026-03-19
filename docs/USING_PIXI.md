# The Pixi Workflow

This guide covers the [pixi/robostack](https://prefix.dev) workflow for building our ROS 2 workspaces.

## Why Pixi?

Dependency management in the ROS 2 ecosystem can be annoying.
While the "single source of truth" in a `package.xml` is convenient, using apt/rosdep necessitates installing packages at the _system level_ with minimal capabilities for conflict resolution or version pinning.
That is, as packages are developed and snapshots updated, there is no guarantee of stability or consistency between installs.
To get around this, we general rely on Docker containers using tagged images that provide an eternal, stable snapshot of a complete system.
However, containers have their own issues, graphics acceleration, host access permissions, etc.

Rather than relying on Docker for isolation, pixi provides consistent, reproducible environments directly on the host machine.
Pixi uses [conda environments](https://docs.conda.io/projects/conda/en/latest/user-guide/getting-started.html) under the hood to create isolated, reproducible builds _without_ a container.
Critically, all installed packages are isolated in a virtual environment, which means no apt dependencies are installed at the system level.

Using Pixi with [robostack](https://robostack.github.io/), which packages ROS distributions as conda packages, we can create fully self-contained ROS workspaces that do not touch the host at the _system_ level.
Additionally, the [pixi-build-ros](https://prefix-dev.github.io/pixi-build-backends/backends/pixi-build-ros/) backend supports parsing `package.xml` for dependencies, giving us similar behavior to `rosdep`.
This means installing different versions of ROS, or rosdeps, or python-pip packages becomes possible on a single machine, without requiring root access.

## Configuring Pixi

We require pixi version **0.65.0 or later**, as we rely on features that are not available in prior versions.

All specifications for the pixi workflow live in the [pixi.toml](../pixi.toml).
This file, along with the [pixi.lock](../pixi.lock) file are the single sources of truth for the environment.
We provide a brief overview of the relevant sections, for more information refer to the [documentation](https://pixi.prefix.dev/latest/tutorials/ros2/).

* `[workspace]`

  Contains high level workspace metadata such as authorship and version information.
  Most important here is the `channels` tag, which defines the package repositories that pixi will search from.
  This project includes:

  ```toml
  channels = [
    "https://prefix.dev/conda-forge",
    "https://prefix.dev/robostack-jazzy",
  ]
  ```

  Which are the base `conda-forge` repo for "standard" dependencies, and the `robostack-jazzy` repo, which provides ROS 2 jazzy versions of package dependencies.
  A matrix of the available packages for each major ROS distro is available on [robostack's website](https://robostack.github.io/jazzy.html).
  While some things are missing, we have found it to be a relatively painless process to add additional packages.
  Note that this workflow _only_ supports Jazzy, but it is possible to support multiple ROS distributions using environments, though that is not covered here.

  We also note the `preview = ["pixi-build"]` line, which is required for the experimental `pixi-build-ros` backend.

* `[activation]`

  This section includes any scripts that should be called when activating the pixi environment.
  By default, this includes an [entrypoint](../scripts/entrypoint_pixi.sh) that will source the installed workspace, if present.
  This is essentially the same logic that exists for the Docker workflow.

* `[activation.env]`

  Defines environment variables for the pixi environment.
  Most of what is here is necessary for compiling and linking using the virtual environment's location on disk.

* `[tasks]`

  Contains pre-defined tasks that are runnable with the `pixi` command.
  In the basic workflow these include:

  | Task | Command | Description |
  |------|---------|-------------|
  | `setup-colcon` | `pixi run setup-colcon` | Pull colcon mixins from the upstream repository |
  | `build` | `pixi run build` | Build all packages with `colcon build` |
  | `test` | `pixi run test` | Run tests with `colcon test` |
  | `test-result` | `pixi run test-result` | Print verbose test results |
  | `clean` | `pixi run clean` | Removes content from the `build/`, `install/`, and `log/` directories |

  These are useful for users (but mostly for CI).
  Reminder that an interactive shell session can be started with `pixi shell`, which provides a "normal" ROS 2 enabled shell experience.

* `[dependencies]`

  All build tools and non-Robostack available packages should be defined in this section.
  These are resolved locally and locked by pixi.
  Additional ROS tooling can be specified here, as well.
  For instance, note that we install the `ros-jazzy-ros-base` package as a dependency, as the base ROS version is typically not specified by any particular `package.xml`.

* `[dev]`

  Used by the `pixi-build-ros` backend to understand the dependencies of source packages in the workspace.
  Any ROS package built from source in the workspace must be listed here as a path dependency pointing to its `package.xml`.
  This tells pixi-build-ros to resolve that package's ROS dependencies from the conda channels and make them available at build time.

  This can be annoying if the workspace has many packages installed.
  More information is provided in the next section.

## Adding Source Dependencies

As noted above, all source dependencies must have their `package.xml` files included in the `[dev]` section of `pixi.toml`.
This is a bit annoying, but it can be updated using a script:

```bash
# Be sure to ignore env files or otherwise. We only want package.xmls from
# packages compiled in the workspace.
find src/ -name package.xml | grep -v '\.pixi/' | while read f; do
  pkg=$(grep -oP '(?<=<name>)[^<]+' "$f")
  pkg_key="ros-jazzy-$(echo "$pkg" | tr '_' '-')"
  echo "$pkg_key = { path = \"./$f\" }"
done | sort
```

This finds all `package.xml` files under `src/`, extracts the package names, converts them to the robostack naming convention (`ros-jazzy-<package-name>` with underscores replaced by hyphens), and prints the TOML entries required.
Copy the output into the `[dev]` section of `pixi.toml`.

## Adding or Updating Sources

As noted in the [README](../README.md), source code is managed in the `src/` directory, typically with submodules.
When updating sources or modifying dependencies, pixi must be updated as well.

Pixi manages two key files: `pixi.toml` (what you want) and `pixi.lock` (what you have).
The lockfile is a complete, resolved snapshot of every package version in the environment.
How you install determines whether that snapshot is maintained or re-resolved:

* `pixi install --frozen`

  installs _exactly_ the packages listed in `pixi.lock`.
  No packages will be updated or re-resolved.
  This is generally what users will want to do when setting up workspaces as it guarantees consistency with the cloned repo.

  Running `pixi install` without the flag will trigger a re-resolve, and may update dependencies listed in the lockfile.
  However, if nothing has changed, it will be equivalent to using `--frozen`.

* `pixi update`

  Resolves and updates all dependencies in the workspace from scratch.
  This will also bump versions of all packages, staying within the confines defined by `pixi.toml`.
  This should only be used intentionlly when needing to update the workspace.

Note that the `pixi.lock` file _SHOULD_ be committed to version control.
Some developers can be put off by the large set of changes, but this is what ensures consistency across machines.
When making changes, if `git` reports a difference in the lockfile, then the changes represent modified dependencies and committing them should be an _intentional choice_.
This is good!

## Troubleshooting

* **Weird conflicts at build or runtime**

  We DO NOT recommend mixing a system installed ROS and the pixi workflow.
  Having ROS installed in `/opt/ros/...` and sourcing that before sourcing a pixi install can break things horribly.
  This workflow exists so that users need not install ROS at the system level!

* **`pixi install` fails to resolve**

  Ensure pixi is on version `0.65.0` or later.
  Older versions do not support the `pixi-build` preview feature.

* **Missing shared libraries at runtime**

  Check that `LD_LIBRARY_PATH` includes `${CONDA_PREFIX}/lib`.
  The activation block in `pixi.toml` should handle this, but if running without `pixi shell` or `pixi run`, the environment will not be activated.
