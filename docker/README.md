# Docker and DDS Startup Configuration

## Fast DDS profile

The Ubuntu launcher defaults to `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` and uses
`configs/fastdds.xml` from the workspace. The profile enables UDPv4 and shared
memory (SHM), with a 16 MiB SHM segment. This configuration addresses data loss
observed with a 512 KiB segment when publishing EuRoC stereo images in succession.

Both environment variables point to the same file inside the container:

```text
FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/gemini336-orbslam3/configs/fastdds.xml
FASTDDS_DEFAULT_PROFILES_FILE=/workspaces/gemini336-orbslam3/configs/fastdds.xml
```

The profile and `docker/entrypoint.sh` are provided through the existing workspace
bind mount, so these changes do not require an image rebuild. The launcher checks
that the profile is readable on the host. The entrypoint then checks that the
profile inside the container is a readable, nonempty file, that both variables
match, and that the XML is well formed. It then runs the ROS base image's
`/ros_entrypoint.sh`. The XML check covers syntax only; it does not replace
Fast DDS semantic validation.

## Starting and recreating containers

Run from the project root:

```bash
bash docker/run_ubuntu.sh
```

**Exit an existing container and recreate it with the launcher** to update the
environment variables and entrypoint stored by Docker. Neither `docker restart`
nor `docker exec` updates those settings in an existing container. These changes
do not automatically stop a running development container.

Use the same DDS environment for the camera driver, SlamNode, rosbag player,
and validation tools. To open and configure another shell:

```bash
docker exec -it gemini336-orbslam3-dev bash
source /opt/ros/humble/setup.bash
source /workspaces/gemini336-orbslam3/install/setup.bash
printenv RMW_IMPLEMENTATION FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
```

The WSL launcher retains its CycloneDDS default. The Fast DDS profile does not
affect CycloneDDS. To explicitly use Fast DDS on WSL:

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp bash docker/run_wsl.sh
```

The launchers also accept command arguments for noninteractive tests without a
TTY. Set `CONTAINER_NAME` to choose a container name and `CONTAINER_HOME` to choose
the host directory mounted as the container's home, keeping test and development
home directories separate.

## Validation with the project configuration

On 2026-09-16, this startup flow completed the MH01 sequence: all 3,682 stereo
pairs were received, synchronized, and processed by tracking, with no gaps and
a normal process exit. See `docs/Phase3C_formal_dds_validation.md` for the detailed
local record.

The following command uses the DDS environment set by the launcher directly,
without a temporary profile under `/tmp`:

```bash
CONTAINER_NAME=gemini336-dds-validation \
CONTAINER_HOME=/tmp/gemini336-dds-home \
ROS_DOMAIN_ID=183 \
bash docker/run_ubuntu.sh bash -c '
  source install/setup.bash &&
  python3 src/gemini336_orbslam3/scripts/validate_stereo_ros.py \
    --mav0 datasets/EuRoC/MH01/mav0 \
    --settings external/ORB_SLAM3/Examples/Stereo/EuRoC.yaml \
    --vocabulary external/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    --output docs/validation/dds_formal
'
```

Choose a new output directory for each run. Results are saved in the host
workspace, and the validation container is automatically removed when the command
finishes. The project's existing Git ignore rules exclude `docs/`, so preserve
validation evidence separately.

The 16 MiB size was validated for this sequence; it is not a universal minimum
for every camera mode. Before end-to-end validation with a Gemini336 rosbag,
confirm the camera settings, image dimensions, frame rate, and rectification.
