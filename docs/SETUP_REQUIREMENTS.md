# Setup requirements and download sources

> Publication update: follow the root README for the current candidate install commands. Script-relative FoundationPose imports are now implemented. The tables below retain the observed environment, including its original package conflicts; they are not the cleaned requirements list. ArUco/RViz behavior limitations remain unchanged.

Initial inventory dated 2026-10-15, scoped to `foundationpose_ros_zed2_v1_5.py` and `foundationpose_ros_realsense_v1_1.py`. This document is a setup baseline, not a verified clean-install recipe. Existing application paths and build scripts need the fixes listed in [the analysis](INITIAL_ANALYSIS.md).

## Observed baseline

| Component | Observed version |
| --- | --- |
| Ubuntu / ROS installation | 22.04.5 / Humble directory present |
| Conda environment | `foundationpose_ros` |
| Python | 3.10.16 |
| torch / torchvision | 2.1.0+cu121 / 0.16.0+cu121 |
| NumPy / SciPy | 1.26.4 / 1.14.1 |
| Ultralytics | 8.3.31 |
| trimesh / Open3D | 4.4.7 / 0.18.0 |
| PyTorch3D / warp-lang | 0.7.8 / 1.3.1 |
| OmegaConf / Kornia | 2.3.0 / 0.5.10 |
| OpenCV packages | python 4.10.0.82; contrib and headless 4.10.0.84 |

The full [inventory](environment-inventory.txt) includes transitive and ROS distributions. Do not feed it directly to pip: local builds, CUDA, ROS ABI compatibility, and package origins are not captured by name/version pairs. Installed versions alone do not prove runtime health.

## Installation layers

1. Use Ubuntu 22.04 and Python 3.10 for this observed ROS Humble baseline. Configure ROS repositories using the [official ROS Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). Its content was blocked by the site's browser challenge during this review.
2. Install ROS Humble and its `rclpy`, `sensor_msgs`, `geometry_msgs`, `cv_bridge`, and `tf2_ros` packages through ROS/apt. Typical packages include `ros-humble-desktop`, `ros-humble-cv-bridge`, `ros-humble-tf2-ros`, `ros-humble-sensor-msgs`, and `ros-humble-geometry-msgs`. Source `/opt/ros/humble/setup.bash` in every runtime shell.
3. Provide an NVIDIA driver and CUDA development toolkit compatible with the selected GPU and PyTorch build. PyTorch's CUDA wheel does not replace `nvcc` for compiling extensions. The observed torch build targets CUDA 12.1; the installed driver/toolkit and GPU were not validated here. Use [NVIDIA CUDA installation guidance](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/) and [PyTorch's archived install commands](https://pytorch.org/get-started/previous-versions/). Do not automatically replace working drivers.
4. Create/activate a Python 3.10 Conda environment. Install torch/torchvision first, then compatible Python dependencies and compiled extensions. The archived PyTorch baseline is `python -m pip install torch==2.1.0 torchvision==0.16.0 torchaudio==2.1.0 --index-url https://download.pytorch.org/whl/cu121`. Hardware compatibility must be confirmed before adopting it.
5. Supply build tools (Git, CMake, C/C++ compiler, Ninja, Eigen and pybind11), GUI support (Tkinter and a display-capable OpenCV build), and FoundationPose dependencies. The old build script targets Eigen 3.4.0 and pybind11 2.10.0. Review upstream instructions at the chosen FoundationPose commit rather than mixing the latest recipe with old sources.
6. Resolve OpenCV package ownership in a fresh test environment: all three installed variants share `cv2`; the interactive scripts need GUI support. Validate NumPy/ROS cv_bridge compatibility. Use `warp-lang` for NVIDIA Warp; do not assume similarly named `warp` is interchangeable.

## External sources and expected locations

| Dependency / data | Source | Local destination or role |
| --- | --- | --- |
| FoundationPose source and build instructions | [NVlabs/FoundationPose](https://github.com/NVlabs/FoundationPose) | `FoundationPose/`; retain the recorded commit and required local patch |
| FoundationPose network weights | Official README's **Data prepare → network weights** link above | `FoundationPose/weights/2023-10-28-18-33-37/` (refiner) and `FoundationPose/weights/2024-01-11-20-02-45/` (scorer); preserve complete folder contents |
| SAM 2.1 base checkpoint | [Ultralytics SAM 2 model table](https://docs.ultralytics.com/models/sam-2/) → `sam2.1_b.pt` | Repository root; use the Ultralytics checkpoint expected by `SAM(...)` |
| Differentiable rasterizer | [NVlabs/nvdiffrast](https://github.com/NVlabs/nvdiffrast) | Build/install into the project Python environment |
| PyTorch3D | [facebookresearch/pytorch3d](https://github.com/facebookresearch/pytorch3d) | Build compatible with selected torch/CUDA |
| ZED SDK | [Stereolabs downloads](https://www.stereolabs.com/developers/release) | Host camera runtime; choose matching Ubuntu/CUDA build |
| ZED ROS 2 wrapper | [stereolabs/zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) | Separate ROS workspace; select release compatible with SDK and Humble |
| RealSense ROS driver / SDK instructions | [realsenseai/realsense-ros](https://github.com/realsenseai/realsense-ros) | Camera driver and librealsense; follow a compatible release's instructions |
| Generic demo models | FoundationPose README's demo-data link | `demo_data/<object>/...`; retain associated texture/material files |

Upstream FoundationPose and Ultralytics pages were checked for the weight directory names and SAM filename. Downloads were not performed. The existing root build script also contains separate Google Drive links for both weight folders, but their download availability was not tested. No public source was established for the user's custom meshes or ZED recording; upload these separately only if sharing them is intended. Record asset units, licenses, checksums, and download destinations in a future asset manifest.

## Camera-specific configuration

| Entry point | Color / depth / camera info | Optical frame |
| --- | --- | --- |
| ZED v1.5 | `/zed/zed_node/rgb/image_rect_color`, `/zed/zed_node/depth/depth_registered`, `/zed/zed_node/rgb/camera_info` | `zed_left_camera_optical_frame` |
| RealSense v1.1 | `/camera/camera/color/image_raw`, `/camera/camera/aligned_depth_to_color/image_raw`, `/camera/camera/color/camera_info` | `camera_color_optical_frame` |

ZED code requests `32FC1` depth. RealSense handles `32FC1` and `16UC1`, converting the latter to metres. Check driver namespace, encoding, alignment, frame IDs, mesh scale, and calibration against the actual installation. These scripts consume ROS topics, so `pyzed` and `pyrealsense2` are not direct script dependencies.

After portability and installation issues are fixed, run from the repository root in a graphical session:

```bash
conda activate foundationpose_ros
source /opt/ros/humble/setup.bash
# Start the matching camera driver in another sourced terminal first.
python foundationpose_ros_zed2_v1_5.py
# Or, for the RealSense camera:
python foundationpose_ros_realsense_v1_1.py
```

Before declaring the setup reproducible, verify imports (`rclpy`, `cv_bridge`, `torch`, `nvdiffrast.torch`, `estimater`, `ultralytics`, `tkinter`), CUDA availability, model loading, GUI operation, and a short live run on each camera. Repair the missing `typeguard` dependency reported by `pip check`, then record a tested environment and extension build revisions.

## Calibration and RViz helper scripts

Include `arucomarker_v4.py` and `rviz_marker_v3.py` alongside the two pose entry points. Additional ROS dependencies are `visualization_msgs`, `builtin_interfaces`, and RViz2. With ROS apt repositories configured:

```bash
sudo apt install ros-humble-visualization-msgs ros-humble-builtin-interfaces ros-humble-rviz2
```

ArUco additionally uses NumPy, SciPy, `cv_bridge`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`, and a GUI-capable OpenCV installation exposing `cv2.aruco.detectMarkers` and `estimatePoseSingleMarkers`. Validate those APIs in the chosen OpenCV version; the source only handles version differences for `DetectorParameters`. Do not install several competing OpenCV wheel variants to satisfy this requirement. RViz marker publishing requires no additional pip package beyond the ROS Python installation; `os`, `re`, and `glob` are standard-library modules.

`arucomarker_v4.py` defaults to a 0.24 m marker, ID 7, dictionary `DICT_4X4_1000`. Supply a printed marker from that dictionary with accurately measured side length. Before using its calibration, replace the hard-coded known marker pose relative to the robot base with the measured pose for your installation. Source ROS and activate the environment in each terminal, then use:

```bash
python arucomarker_v4.py --ros-args -p marker_length:=0.24 -p marker_id:=7 -p camera_frame:=zed_left_camera_optical_frame -p base_frame:=base_link
python rviz_marker_v3.py
rviz2
```

Run these as separate processes from the repository root, after starting the ZED camera. The ArUco helper subscribes to the ZED topics listed above; changing `camera_frame` alone does not change camera topics. For RealSense, remap both image and camera-info topics and supply its optical frame, after checking image calibration conventions.

In RViz, choose a connected fixed frame (typically `base_link`), add a TF display and Marker displays for `/visualization_marker` and `/aruco_marker_labels`. Start RViz before the current one-shot ArUco label publication. Meshes must exist under `demo_data/` even though that folder is excluded from Git. See the analysis for the static-TF delivery issue and selection limitations that need repair before release.
