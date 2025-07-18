# ORB-SLAM3 with Occupancy Grid Mapper Extension

This repository is a fork and extension of [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3), enhanced with an integrated **Occupancy Grid Map converter** that generates ROS 2-compatible `.yaml` and `.pgm`/`.png` files from sparse SLAM maps.

### Version: 1.0 (July 18th, 2025)
**Author:** Harishma Prakash  
**Affiliation:** Chennai Institute of Technology, India

### Key Features

- Added **OccupancyGridBuilder** to generate 2D occupancy grids from 3D sparse maps
- Automatically generates `.png`/`.pgm` image and `.yaml` description file for the map
- Compatible with **ROS 2** Navigation Stack
- Code is modular and runs as a background thread after first successful tracking
- Ready for further integration in real-world navigation tasks
- No external command needed – conversion runs seamlessly with SLAM execution.

---

# 1. License

This repository is distributed under the **[GNU GPLv3 license](https://www.gnu.org/licenses/gpl-3.0.html)**, in accordance with the original ORB-SLAM3 license.

If you use ORB-SLAM3-grid-mapper in an academic work, please cite:
> Harishma Prakash,  
> Undergraduate, Chennai Institute of Technology, India  
> GitHub: [github.com/Harishma356/ORBSLAM3-grid-mapper](https://github.com/Harishma356/ORBSLAM3-grid-mapper)


Please also cite the original authors of ORB-SLAM3. The credits for the original version of ORBSLAM3 goes to the respective authors:
  
    @article{ORBSLAM3_TRO,
      title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial 
               and Multi-Map {SLAM}},
      author={Campos, Carlos AND Elvira, Richard AND G\´omez, Juan J. AND Montiel, 
              Jos\'e M. M. AND Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics}, 
      volume={37},
      number={6},
      pages={1874-1890},
      year={2021}
     }

# 2. Prerequisites
We modified and tested the library in **Ubuntu 20.04**.

The pre-requisites of the original ORBSLAM3 is mentioned in the official git repository. Here the modifications in certain requisites that we used are specified. The rest is as followed as in the official repo.


## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Download and install instructions can be found at: http://opencv.org. **We tested with OpenCV 4.3**.


# 3. Building ORB-SLAM3 library and examples

Clone the repository:
```
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
```

Please make sure you have installed all required dependencies. Execute to build:
```
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```
**Follow the official repo of ORBSLAM3 for instructions of execution of executable files. Once you execute the ORBSLAM3 as per the instructions mentioned there, you will find the occupancy grid getting built. There is no specific external execution commands to extract occupancy grid, since we have integrated the converter part in the ORBSLAM3 as an extension. You will get 'global_occupancy_grid' in both .png and .pgm format and a 'map.yaml' file saved in the ORBSLAM3 root directory.**

You can modify the resolution, voxel sizes, minDepth, maxDepth in the 'OccupancyGridBuilder.h' or 'OccupancyGridBuilder.cc' as per your requirement or preference, it doesn't affect the occupancy building ability rather customize it to your preference.

**You can find the tested examples in the repo directory under 'docs/'**

# 4. Community and ROS 2 Contribution

This extension has been shared with the ROS 2 community. You can find the official contribution post and community discussion here:

[ORB-SLAM3 Extension: Real-Time Sparse Map to Occupancy Grid (.yaml/.pgm) for ROS 2 Navigation](https://discourse.openrobotics.org/t/orb-slam3-extension-real-time-sparse-map-to-occupancy-grid-yaml-pgm-for-ros-2-navigation/49158?u=harishma356)

Feel free to contribute, raise issues, or suggest improvements!


