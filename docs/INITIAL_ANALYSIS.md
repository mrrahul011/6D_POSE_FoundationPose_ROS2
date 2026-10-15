# GitHub readiness — initial analysis

> Publication preparation update: the two pose entry points now resolve FoundationPose relative to their script location. The README has been replaced with setup/run instructions and a pipeline diagram. Direct duplicate OpenCV/Warp/PyYAML entries and pip `cv_bridge` were removed from requirements; transitive OpenCV conflicts still need clean-environment testing. The original inspection below records the state before these edits. Destination: `mrrahul011/FoundationPose_ROS2_Zed`. Only the four selected application scripts, requirements, documentation, ignore rules, and license notices are being prepared in a separate checkout of that repository.

Inspected 2026-10-15. This is an assessment and documentation pass, not a tested release. No commit, push, environment installation, history rewrite, or application-code change was performed. Following the requested exclusions, the previously tracked `assets/` files and `Dockerfile` were removed from the Git index (staged deletions), with local copies preserved.

## Intended entry points

Include `foundationpose_ros_zed2_v1_5.py` (24,189 bytes) and `foundationpose_ros_realsense_v1_1.py` (22,401 bytes). Both exist, are untracked, and are not excluded by the new `.gitignore`. They use FoundationPose, Ultralytics SAM 2.1, ROS image topics, Tkinter selection, and OpenCV GUI windows. Neither directly imports a camera SDK; separate camera ROS drivers provide images.

## Upload scope and large files

| Local item | Size | Recommendation |
| --- | ---: | --- |
| Each of two cube recording copies in `originaldataset/` | 4,124.3 MiB | External download only |
| `zed2_demo/zed2_demo_0.db3` | 781.8 MiB | External download only |
| FoundationPose scorer / refiner weights | 181.4 / 65.1 MiB | Download from upstream |
| `sam2.1_b.pt` | 154.4 MiB | Download from Ultralytics |
| Headset mesh in `originaldataset/` | 102.9 MiB | External asset |
| `demo_data/Cube/mesh/rubiks_cube_scaled.obj` | 12.0 MiB | External model library |

The added `.gitignore` excludes dependency checkouts, local model libraries, weights, recordings, generated TF diagrams, caches, and new media files. It does not remove files already tracked or enforce a general size limit. Keep small explicitly reviewed sample meshes if needed; they will require an explicit exception or forced add.

Previously tracked media includes videos of 14.9 and 9.1 MiB and GIFs of 10.5 and 3.5 MiB. These assets are now staged for removal from tracking. The largest blob found across local Git refs is an older video of approximately 18.3 MiB. Deleting or ignoring media now does not shrink existing history. For a genuinely small first publication, decide between preserving this history and creating a separate clean export; do not rewrite the existing checkout casually.

The entire `assets/`, `originaldataset/`, `zed2_demo/` (and alternate spelling `zed2demo/`), `docker/`, and `.devcontainer/` directories are ignored, along with root Dockerfiles and Docker Compose YAML files. README references to `assets/` images must be replaced with hosted links or removed before publishing the resulting source tree.

## Additional script review

Both `arucomarker_v4.py` and `rviz_marker_v3.py` should accompany the requested pose scripts if calibration and RViz visualization are part of the release. Neither is ignored. See [their setup requirements](SETUP_REQUIREMENTS.md#calibration-and-rviz-helper-scripts).

### `arucomarker_v4.py`

- Detects dictionary `DICT_4X4_1000`, marker ID 7, side length 0.24 m by default. Reads ZED rectified RGB and camera info; estimates a single marker pose and publishes static transforms for the camera and markers.
- The known marker position and quaternion relative to `base_link` are hard-coded, specific to this installation. Expose them as parameters and measure them for each setup. Quaternion order is x, y, z, w.
- Calibration stops after the first successful detection. There is no multi-frame averaging or reprojection-error gate; camera and reference marker must remain stationary afterward.
- The transform composition is consistent with `T_base_camera = T_base_marker @ inverse(T_camera_marker)`, provided the supplied marker pose is correct. Check for an existing driver TF parent for the optical frame before publishing another parent.
- The installed Humble `StaticTransformBroadcaster.sendTransform()` publishes only the supplied transforms, with transient-local depth 1. Three separate calls therefore retain only the final message for late subscribers. Publish all three transforms together in one call.
- The text label is published once with ordinary volatile QoS, so RViz started later may miss it. Republish periodically or use suitable transient-local QoS on both ends.
- It uses rectified images but takes `K` and distortion from `CameraInfo`. Verify these describe the rectified image; use appropriate rectified intrinsics and distortion treatment where required. Match subscription QoS to the camera publisher.

### `rviz_marker_v3.py`

- Scans TF every second for names matching `<mesh>_frame_base`, finds a matching OBJ/STL under `demo_data/`, and publishes one red mesh marker on `visualization_marker`.
- `matches[-1]` is merely the final regex match in YAML text, not an established newest/current object. Parse frame keys and select the active object explicitly; the regex can also match values and mishandle names containing spaces or hyphens.
- Deletes and republishes the existing marker every second even when selection is unchanged, creating needless traffic and possible flicker. If all matching frames disappear, it returns without deleting the old marker.
- Mesh lookup uses the first basename match, so duplicate filenames are ambiguous. `demo_data` depends on the working directory.
- The mesh URI is a local absolute `file://` path. RViz must have access to that same path; remote RViz machines need shared assets or a portable package resource strategy. Scale is fixed at 1, requiring model units and origins to match the pose-estimation pipeline.

These findings are from source inspection and the installed ROS broadcaster implementation. Both helper scripts pass syntax parsing; live calibration and RViz behavior have not been tested. No application code was changed in this review.

`origin` currently points to `https://github.com/ammar-n-abbas/FoundationPoseROS2.git`. The intended destination/fork has not been established. Four tracked original OBJ files are already deleted in the working tree, and `foundationpose_ros_multi.py` already has local edits. Preserve and review these separately; avoid `git add .`.

## Release blockers

1. Both requested scripts append `/home/rahul/Desktop/Robin/FoundationPoseROS2/...` to `sys.path`. Replace these with paths based on `__file__` before sharing as runnable code.
2. Both resolve `demo_data` and `sam2.1_b.pt` relative to the working directory. Document running from the repository root or parameterize paths.
3. The root build script contains `cd /nvdiffrast` after cloning inside `FoundationPose`; this resolves to the wrong directory. It also has unconditional clones, destructive build cleanup, system-wide installs, and no fail-fast behavior.
4. The README contains a stray trailing `~` in CUDA PATH commands and blanket NVIDIA-driver removal instructions. Replace the installation section with a tested, machine-appropriate procedure.
5. Existing `requirements.txt` installs three conflicting OpenCV distributions, repeats PyYAML, lists multiple Warp names, and mixes ROS packages with pip dependencies. Preserve the observed versions as evidence, then validate a curated environment separately.
6. The local FoundationPose checkout is at `e3d597b8c6b851d053094ebd6fa240191c5238f8`, with a local C++14-to-C++17 change in `bundlesdf/mycuda/setup.py`. Preserve this modification as a documented patch if that build path is retained. Upstream's current setup differs from this old checkout.
7. The root Dockerfile invokes missing `main.py` and lacks a complete FoundationPose/CUDA build. It is not a validated deployment path for these scripts.

## Environment findings and validation

No `VIRTUAL_ENV` or `CONDA_PREFIX` was active, and `python` was unavailable on the shell PATH. The likely project environment was inspected explicitly at `/home/rahul/miniconda3/envs/foundationpose_ros/bin/python`; it is not proven to be the IDE-selected environment. Host OS is Ubuntu 22.04.5; `/opt/ros/humble` exists.

See [setup requirements](SETUP_REQUIREMENTS.md) and [full observed distribution inventory](environment-inventory.txt). `pip check` reports `generate-parameter-library-py 0.7.5` requires missing `typeguard`. Package metadata does not show distributions named `nvdiffrast` or `mycuda`; source-path imports/local extensions may still provide them, so this is not proof they are unavailable.

The two entry points pass Python syntax parsing. No camera, GPU inference, full import test, or clean installation was executed. Before publishing: repair portability/build issues, test each camera path, confirm RGB/depth alignment and TF frames, choose the GitHub destination and media/history policy, then stage only reviewed files.
