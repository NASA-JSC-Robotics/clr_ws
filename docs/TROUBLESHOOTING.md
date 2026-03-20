# Troubleshooting

A semi-complete guide for troubleshooting this containerized workflow.

## Slow Rendering

If you have an NVIDIA or other graphics card, you will have to modify the compose file to ensure that graphics drivers are enabled in the container.
Uncomment the lines noted in the [docker-compose](../docker-compose.yml) file:

```yaml
    # If using an nvidia driver uncomment these lines
    # runtime: nvidia
    # deploy:
    #   resources:
    #     reservations:
    #       devices:
    #         - capabilities: [gpu]
    #           count: all
```

When starting the container, you may see the following error:

```bash
docker: Error response from daemon: unknown or invalid runtime name: nvidia
```

On your host, you must be sure to have the [nvidia container toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) installed.
The following commands can then be used to configure docker to use nvidia:

```bash
# Refer to the setup documents to be sure these are correct.
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
sudo nvidia-ctk runtime configure --runtime=containerd
sudo systemctl restart containerd
```

## X Permissions Errors

X authentication objects and files are bind mounted into the container in the [compose file](../docker-compose.yml).
If RViz or other graphical applications give errors such as:

```bash
[rviz2-2] Authorization required, but no authorization protocol specified
[rviz2-2]
[rviz2-2] qt.qpa.xcb: could not connect to display :0
[rviz2-2] qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in "" even though it was found.
[rviz2-2] This application failed to start because no Qt platform plugin could be initialized. Reinstalling the application may fix this problem.
[rviz2-2]
[rviz2-2] Available platform plugins are: eglfs, linuxfb, minimal, minimalegl, offscreen, vnc, wayland-egl, wayland, wayland-xcomposite-egl, wayland-xcomposite-glx, xcb.
```

Firstly, double check that user information (`UID` and `GID`) on the host match what's in the container.
Run the `id` both inside and outside the container and make sure that the `UID` and `GID` of both users match.
If not, refer to the [README](./../README.md) setup section for setting those host variables.

If the user information matches, try running `xhost +local:docker` and restarting the container.
This will give docker broader X permissions, but would have to be run once per login, if required.

## Build Changes Not Visible

Changing the workflow or workspace then running `docker compose build` will recreate the image.
However, if you have an existing container `docker compose up` may just restart a previously stopped service.

To ensure you are using the latest and greatest image add `--force-recreate` to remove any old images and restart from scratch:

```bash
docker compose up dev -d --force-recreate
```

## Permissions Errors

Depending on your environment you may run into build or folder creation issues.
For example:

```bash
er4-user@hostname(docker):~/ws$ colcon build
Traceback (most recent call last):
  File "/usr/bin/colcon", line 33, in <module>
    sys.exit(load_entry_point('colcon-core==0.19.0', 'console_scripts', 'colcon')())
  File "/usr/lib/python3/dist-packages/colcon_core/command.py", line 130, in main
    return _main(
  File "/usr/lib/python3/dist-packages/colcon_core/command.py", line 219, in _main
    create_log_path(args.verb_name)
  File "/usr/lib/python3/dist-packages/colcon_core/location.py", line 186, in create_log_path
    os.makedirs(str(path))
  File "/usr/lib/python3.10/os.py", line 225, in makedirs
    mkdir(name, mode)
PermissionError: [Errno 13] Permission denied: 'log/build_2025-07-08_18-40-19'
```

As noted in the [README](../README.md), when the container is started build artifacts from [build](../build/), [install](../install/), [log](../log/), and [.ccache](../.cache) are bind mounted into the workspace.

### Missing User Information

To ensure that the container has appropriate permissions to modify the contents of these directories, the container's user's UID and GID are setup to match the host's.
Before building, ensure the `USER_UID` and `USER_GID` environment variables are set so that the created user's info matches!

This can always be checked with `id` and by making sure the directories in the `ws` are owned by `er4-user`.
If you something like the following, then your user info was not properly set:

```bash
# Note the missing user information in the UID/GID
er4-user@hostname(docker):~/ws$ ls -l
total 16
drwxrwxr-x 2 1001 1001 4096 Jul  8 18:48 build
drwxrwxr-x 2 1001 1001 4096 Jul  8 18:48 install
drwxrwxr-x 2 1001 1001 4096 Jul  8 18:48 log
drwxrwxr-x 4 1001 1001 4096 Jul  8 18:20 src
```

It should be:

```bash
er4-user@hostname(docker):~/ws$ ls -l
total 16
drwxrwxr-x 2 er4-user er4-user 4096 Jul  8 18:48 build
drwxrwxr-x 2 er4-user er4-user 4096 Jul  8 18:48 install
drwxrwxr-x 2 er4-user er4-user 4096 Jul  8 18:48 log
drwxrwxr-x 4 er4-user er4-user 4096 Jul  8 18:20 src
```

### Removed Artifact Directories

Be careful using `colcon clean`!
If any of the bind mounted directories are removed, then docker will create the directories when starting the container.
If that happens then the directories will be owned by `root` and you may see something like,

```bash
er4-user@hostname(docker):~/ws$ ls -l
total 16
drwxrwxr-x 2 root root 4096 Jul  8 18:48 build
drwxrwxr-x 2 root root 4096 Jul  8 18:48 install
drwxrwxr-x 2 root root 4096 Jul  8 18:48 log
drwxrwxr-x 4 er4-user er4-user 4096 Jul  8 18:20 src
```

Rather than `colcon clean`, we recommend clearing the build artifacts manually with `rm -r build/* install/* log/*`.
