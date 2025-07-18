/**
 * @file converter.cc
 * @brief Converts sparse 3D map from ORB-SLAM3 into a 2D occupancy grid (.png/.pgm) and generates a ROS-compatible .yaml map file.
 *
 * @author Harishma Prakash
 * Chennai Institute of Technology, India
 *
 * This is an original extension of the ORB-SLAM3 project.
 * It is distributed under the terms of the GNU General Public License v3,
 * as specified by the original ORB-SLAM3 repository.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 *
 * Original ORB-SLAM3 repository: https://github.com/UZ-SLAMLab/ORB_SLAM3
 */


#ifndef OCCUPANCY_GRID_BUILDER_H
#define OCCUPANCY_GRID_BUILDER_H

#include "Map.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <mutex>
#include <thread>
#include <Eigen/Core>  // for Eigen::Vector3f

namespace ORB_SLAM3 {

class OccupancyGridBuilder {
public:
    OccupancyGridBuilder(Map* pMap);
    void Run();
    void RequestFinish();
    bool CheckFinish();
    cv::Mat GetOccupancyGrid(); // ← Needs to be public for external access
    
    /*struct VoxelRegion {
        Eigen::Vector3f centroid;
        Eigen::Vector3f leftMost;
        Eigen::Vector3f rightMost;
        Eigen::Vector3f topMost;
        Eigen::Vector3f bottomMost;
    };*/

    // Handle mouse panning events
    void HandleMouse(int event, int x, int y, int flags);
    std::vector<Eigen::Vector3f> VoxelGridFilter(const std::vector<Eigen::Vector3f>& points, float voxelSize);
    void BuildOccupancyGrid();
    //void RaycastAndMarkFree(cv::Mat& grid, int x0, int y0, int x1, int y1);
    void RaycastAndMarkFreeAndOccupied(cv::Mat& grid, const Eigen::Vector3f& camPos, const Eigen::Vector3f& voxelPos);
    //std::vector<VoxelRegion> VoxelGridFilter(const std::vector<Eigen::Vector3f>& points, float voxelSize);
    
    

private:
    
    Map* mpMap;
    cv::Mat mGlobalOccupancyGrid;

    // Map parameters
    float mResolution = 0.02f;     // meters per cell
    int mWidth = 4000;             // number of columns
    int mHeight = 4000;            // number of rows
    float mOriginX = - (mWidth * mResolution) / 2.0f;       // meters (bottom-left corner)
    float mOriginY = - (mHeight * mResolution) / 2.0f;

    // Viewport window over the occupancy grid
    cv::Rect mViewWindow;
    cv::Point mDragStart;
    bool mIsDragging = false;

    std::mutex mGridMutex;         // Thread safety
    bool mbFinishRequested;
    
    std::vector<std::pair<float, float>> computeConvexHull2D(std::vector<std::pair<float, float>> points);
    std::vector<Eigen::Vector3f> fillPolygonAndExtrude(
        const std::vector<std::pair<float, float>>& hull2D, 
        float minZ, float maxZ, float voxelSize);
    bool isPointInPolygon(float x, float y, 
        const std::vector<std::pair<float, float>>& polygon);
    
};

}  // namespace ORB_SLAM3

#endif // OCCUPANCY_GRID_BUILDER_H

