# ORBSLAM3-grid-mapper

**Version:** v1.0 | **Release:** July 2025 | **Author:** Harishma Prakash, Chennai Institute of Technology

---

## Overview

`ORBSLAM3-grid-mapper` is an extended version of the original [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) framework.

This repository generates a **2D occupancy grid map** from the sparse map output of ORB-SLAM3 by running a background occupancy-grid generation thread alongside the SLAM process.

The generated outputs include:
- `.pgm` — occupancy grid map
- `.yaml` — occupancy metadata file

making the output directly compatible with **ROS2 navigation** workflows.

---

## Citation

If you use this work, please star this repository and cite the original ORB-SLAM3:

```bibtex
@article{ORBSLAM3_TRO,
  title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map {SLAM}},
  author={Campos, Carlos AND Elvira, Richard AND G\'omez, Juan J. AND Montiel, Jos\'e M. M. AND Tard\'os, Juan D.},
  journal={IEEE Transactions on Robotics},
  volume={37},
  number={6},
  pages={1874-1890},
  year={2021}
}
```

---

## Pre-requisites

Follow the original [ORB-SLAM3 repository](https://github.com/UZ-SLAMLab/ORB_SLAM3) for installation of all required dependencies and datasets.

---

## Tested Environments

### Original ORB-SLAM3 (Officially Tested)

| Component | Version |
|-----------|---------|
| OS | Ubuntu 16.04 / 18.04 |
| Compiler | C++11 / C++0x |
| OpenCV | 3.2.0 |
| Pangolin | 2019–2020 era |
| Eigen | Eigen3 |
| ROS | Melodic (optional) |
| Python | 2.7 |
| g2o | Bundled in Thirdparty |
| DBoW2 | Bundled in Thirdparty |

### ORBSLAM3-grid-mapper (This Repository)

| Component | Version |
|-----------|---------|
| OS | Ubuntu 20.04.6 LTS |
| GCC | 9.4.0 |
| CMake | 3.16.3 |
| OpenCV | 4.3.0 |
| Eigen | 3.3.7 |
| Pangolin | Manually built (modern) |
| C++ | C++14 / C++17 |
| g2o | Bundled in Thirdparty |
| DBoW2 | Bundled in Thirdparty |
| ROS | Melodic (optional) |

---

## Building

```bash
git clone https://github.com/Harishma356/ORBSLAM3-grid-mapper.git
cd ORBSLAM3-grid-mapper
chmod +x build.sh
./build.sh
```

---

## Running

Execution commands are identical to the original ORB-SLAM3. Example — Monocular TUM:

```bash
./Examples/Monocular/mono_tum \
  Vocabulary/ORBvoc.txt \
  Examples/Monocular/TUM1.yaml \
  PATH_TO_SEQUENCE
```

Supported modes: Monocular · Stereo · RGB-D · Monocular-Inertial · Stereo-Inertial

---

## Output

The mapper automatically generates during execution:

| File | Description |
|------|-------------|
| `global_occupancy_grid.pgm` | 2D occupancy grid image |
| `map.yaml` | ROS-compatible map metadata |

---

## Example Results

Occupancy grid generation during SLAM execution (KITTI dataset):

![Occupancy Grid Generation](docs/kittiexample.gif)

Comparison between direct sparse point flattening (general method) and the background thread approach used in this repository:

**General method output:**

![General Method](docs/direct_sparse_2_occupancy.jpeg)

**ORBSLAM3-grid-mapper output:**

![Grid Mapper Output](docs/orbslam3_map.gif)


## Acknowledgements

This repository is a modified version of [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) 
developed by Carlos Campos, Richard Elvira, Juan J. Gómez, José M. M. Montiel, 
and Juan D. Tardós at the University of Zaragoza, extended with a 2D occupancy 
grid generation thread that converts sparse map points into an occupancy grid map.
