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

#include "OccupancyGridBuilder.h"
#include "MapPoint.h"
#include "Map.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <chrono>
#include <cmath>  // for floor()
#include <unordered_map>
#include <mutex>
#include <thread>
#include <Eigen/Core>
#include <algorithm>

namespace ORB_SLAM3 {

// Clamp template to handle min/max safely with any type

template <typename T>
T clamp(const T& value, const T& low, const T& high) {
    return std::max(low, std::min(value, high));
}

// this is for cv display
cv::Mat globalDisplayGrid;

// this is to show last camera position initially at -1,1
cv::Point lastCamCircle(-1, -1);

// initialised constructor with data structures and gui
OccupancyGridBuilder::OccupancyGridBuilder(Map* pMap)
    : mpMap(pMap), mbFinishRequested(false) {
    mGlobalOccupancyGrid = cv::Mat(mHeight, mWidth, CV_8SC1, cv::Scalar(-1));

    mViewWindow = cv::Rect(0, 0, 4000, 4000);
    cv::namedWindow("Occupancy Grid", cv::WINDOW_NORMAL);
    cv::resizeWindow("Occupancy Grid", 4000, 4000);
}

// declared flag
void OccupancyGridBuilder::RequestFinish() {
    mbFinishRequested = true;
}

// declared another flag like a signal to stop
bool OccupancyGridBuilder::CheckFinish() {
    return mbFinishRequested;
}

// run function to run the occupancygridbuilder in while loop
void OccupancyGridBuilder::Run() {
    std::cout << "[OccupancyGridBuilder] Started." << std::endl;
    while (!CheckFinish()) {
        BuildOccupancyGrid();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "[OccupancyGridBuilder] Finished." << std::endl;
}

// initialising voxel filtering
std::vector<Eigen::Vector3f> OccupancyGridBuilder::VoxelGridFilter(const std::vector<Eigen::Vector3f>& points, float voxelSize) {
    struct VoxelKey {
        int x, y, z;
        bool operator==(const VoxelKey& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };
    // hash for the custom unorderedmap
    struct VoxelKeyHash {
        std::size_t operator()(const VoxelKey& k) const {
            return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^ (std::hash<int>()(k.z) << 1);
        }
    };
    
    // Map voxel -> vector of points in that voxel
    std::unordered_map<VoxelKey, std::vector<Eigen::Vector3f>, VoxelKeyHash> voxelMap;
    
    // Group points by voxel
    for (const auto& pt : points) {
        VoxelKey key = {
            static_cast<int>(std::floor(pt.x() / voxelSize)),
            static_cast<int>(std::floor(pt.y() / voxelSize)),
            static_cast<int>(std::floor(pt.z() / voxelSize))
        };
        voxelMap[key].push_back(pt);
    }
    
    std::vector<Eigen::Vector3f> filtered;	// return variable
    
    // Process each voxel
    for (const auto& kv : voxelMap) {
        const VoxelKey& voxelKey = kv.first;
        const std::vector<Eigen::Vector3f>& voxelPoints = kv.second;	//voxelPoints - points mapped to voxelkey
        
        if (voxelPoints.empty()) continue;
        
         // NEW CONDITION: Only process voxels with more than 20 points
        if (voxelPoints.size() <= 20) {
            // Skip this voxel - do not add any points to filtered result
            continue;
        }
        
        // Find Z range for this voxel
        float minZ = voxelPoints[0].z();	//initialise min and max z points
        float maxZ = voxelPoints[0].z();
        for (const auto& pt : voxelPoints) {
            minZ = std::min(minZ, pt.z());	//compare minZ and the iterated point find min
            maxZ = std::max(maxZ, pt.z());	//compare maxZ and the iterated point find max
        }
        
        // Convert 3D points to 2D for hull computation (project to XY plane)
        std::vector<std::pair<float, float>> points2D;	// initialise 2D points
        for (const auto& pt : voxelPoints) {
            points2D.emplace_back(pt.x(), pt.y());
        }
        
        // Compute 2D convex hull using Graham scan
        auto convexHull2D = computeConvexHull2D(points2D);	//function is called
        
        // Fill the 2D polygon and extrude in Z direction
        auto filledPoints = fillPolygonAndExtrude(convexHull2D, minZ, maxZ, voxelSize); //function called
        
        // Add all filled points to result
        filtered.insert(filtered.end(), filledPoints.begin(), filledPoints.end());
    }
    
    return filtered;
}

// Helper function to compute 2D convex hull using Graham scan
std::vector<std::pair<float, float>> OccupancyGridBuilder::computeConvexHull2D(std::vector<std::pair<float, float>> points) {
    if (points.size() <= 3) return points; // Return all points if too few
    //points2D passed in points
    
    // Find bottom-most point (and leftmost in case of tie)
    auto bottom = std::min_element(points.begin(), points.end(), 
        [](const auto& a, const auto& b) {
            return a.second < b.second || (a.second == b.second && a.first < b.first);
        });
    std::swap(points[0], *bottom);
    
    auto pivot = points[0];
    
    // Sort points by polar angle with respect to pivot
    std::sort(points.begin() + 1, points.end(), 
        [&pivot](const auto& a, const auto& b) {
            float cross = (a.first - pivot.first) * (b.second - pivot.second) - 
                         (a.second - pivot.second) * (b.first - pivot.first); //cross product of vectors
            if (std::abs(cross) < 1e-9) {
                // Collinear points - choose closer one to pivot
                float distA = (a.first - pivot.first) * (a.first - pivot.first) + 
                             (a.second - pivot.second) * (a.second - pivot.second);
                float distB = (b.first - pivot.first) * (b.first - pivot.first) + 
                             (b.second - pivot.second) * (b.second - pivot.second);
                return distA < distB;
            }	//used squared Euclidean distance to find the distance
            return cross > 0;
        });
    
    // Graham scan
    std::vector<std::pair<float, float>> hull;
    for (const auto& pt : points) {
        while (hull.size() >= 2) {
            auto& p1 = hull[hull.size()-2];
            auto& p2 = hull[hull.size()-1];
            float cross = (p2.first - p1.first) * (pt.second - p1.second) - 
                         (p2.second - p1.second) * (pt.first - p1.first); //cross product of vectors
            if (cross <= 0) {
                hull.pop_back();
            } else {
                break;
            }
        }
        hull.push_back(pt);
    }
    
    return hull;	// to hull polygon returned
}

// --------------------- extra ---------------------------------
float polygonArea(const std::vector<std::pair<float, float>>& pts) {
    float area = 0.0f;
    int n = pts.size();
    for (int i = 0; i < n; ++i) {
        auto [x1, y1] = pts[i];
        auto [x2, y2] = pts[(i + 1) % n];
        area += (x1 * y2 - x2 * y1);
    }
    return std::fabs(area) * 0.5f;
}
    // -------------------------------------------------------------------  

// Helper function to fill polygon and extrude in Z
std::vector<Eigen::Vector3f> OccupancyGridBuilder::fillPolygonAndExtrude(
    const std::vector<std::pair<float, float>>& hull2D, 
    float minZ, float maxZ, float voxelSize) {
    
    std::vector<Eigen::Vector3f> result;
    
    
    
    if (hull2D.size() < 3) {
        // Not enough points for a polygon, just return a point at average Z
        //if (!hull2D.empty()) {
        //    float avgZ = (minZ + maxZ) / 2.0f;
        //    result.emplace_back(hull2D[0].first, hull2D[0].second, avgZ);
        //}
        return result;
    }
    
    //========extra call
    float area = polygonArea(hull2D);
    if (area < 0.05f)  //  Adjust this threshold depending on map scale
        return result;
     //===================
    // Find bounding box of 2D hull
    float minX = hull2D[0].first, maxX = hull2D[0].first;
    float minY = hull2D[0].second, maxY = hull2D[0].second;
    
    for (const auto& pt : hull2D) {
        minX = std::min(minX, pt.first);
        maxX = std::max(maxX, pt.first);
        minY = std::min(minY, pt.second);
        maxY = std::max(maxY, pt.second);
    }
    
    // Sample points within bounding box and test if inside polygon
    float step = voxelSize * 0.3f; // Sample at smaller resolution for better filling
    
    for (float x = minX; x <= maxX; x += step) {
        for (float y = minY; y <= maxY; y += step) {
            if (isPointInPolygon(x, y, hull2D)) {
                // Point is inside 2D polygon, extrude in Z direction
                for (float z = minZ; z <= maxZ; z += step) {
                    result.emplace_back(x, y, z);
                }
            }
        }
    }
    
    // If no points were generated (very small polygon), add center point
    //if (result.empty()) {
       // float centerX = (minX + maxX) / 2.0f;
       // float centerY = (minY + maxY) / 2.0f;
      //  float centerZ = (minZ + maxZ) / 2.0f;
      //  result.emplace_back(centerX, centerY, centerZ);
    //}
    
    return result;
}

// Helper function to test if point is inside 2D polygon using ray casting
bool OccupancyGridBuilder::isPointInPolygon(float x, float y, 
    const std::vector<std::pair<float, float>>& polygon) {
    
    int n = polygon.size();
    bool inside = false;
    
    for (int i = 0, j = n - 1; i < n; j = i++) {
        float xi = polygon[i].first, yi = polygon[i].second;
        float xj = polygon[j].first, yj = polygon[j].second;
        
        if (((yi > y) != (yj > y)) && 
            (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }	//raycasting algorithm
    }
    
    return inside;
}

void OccupancyGridBuilder::RaycastAndMarkFreeAndOccupied(cv::Mat& grid, const Eigen::Vector3f& camPos, const Eigen::Vector3f& voxelPos) {
    int x0 = static_cast<int>((camPos[0] - mOriginX) / mResolution);
    int y0 = mHeight - static_cast<int>((camPos[2] - mOriginY) / mResolution) - 1;

    int x1 = static_cast<int>((voxelPos[0] - mOriginX) / mResolution);
    int y1 = mHeight - static_cast<int>((voxelPos[2] - mOriginY) / mResolution) - 1;

    x1 = clamp(x1, 0, grid.cols - 1);
    y1 = clamp(y1, 0, grid.rows - 1);

    int dx = abs(x1 - x0);
    int dy = -abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    int x = x0;
    int y = y0;
    
    //variables for bresenham's algorithm

    while (true) {
        if (x >= 0 && x < grid.cols && y >= 0 && y < grid.rows) {
            if (grid.at<schar>(y, x) == -1) {
                grid.at<schar>(y, x) = 0;	//marked as free
            }
        }
        if (x == x1 && y == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) {	//bresenham's algorithm
            err += dy;
            x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y += sy;
        }
    }

    if (x1 >= 0 && x1 < grid.cols && y1 >= 0 && y1 < grid.rows) {
        grid.at<schar>(y1, x1) = 100;	//marking as occupied
    }
}

void OccupancyGridBuilder::BuildOccupancyGrid() {
    if (!mpMap) return;

    std::vector<MapPoint*> mps = mpMap->GetAllMapPoints();
    KeyFrame* pKF = mpMap->GetLastKeyFrame();
    if (!pKF) return;

    Sophus::SE3f Tcw = pKF->GetPose();            
    Sophus::SE3f Twc = Tcw.inverse();             
    Eigen::Vector3f camPos = Twc.translation(); 

    std::vector<Eigen::Vector3f> allPoints;

    float fx = 718.856f;
    float fy = 718.856f;
    float cx_intrinsics = 607.1928f;
    float cy_intrinsics = 185.2157f;
    int   imageWidth  = 1241;
    int   imageHeight = 376;

    float minDepth = 0.9f;
    float maxDepth = 8.0f;
    float distanceThreshold = 3.0f;

    Eigen::Matrix3f Rcw = Tcw.rotationMatrix();
    Eigen::Vector3f tcw = Tcw.translation();

    for (MapPoint* pMP : mps) {
        if (!pMP || pMP->isBad()) continue;

        Eigen::Vector3f Pw = pMP->GetWorldPos();
        Eigen::Vector3f Pc = Rcw * Pw + tcw;

        if (Pc[2] <= minDepth || Pc[2] >= maxDepth) continue;

        float u = fx * (Pc[0] / Pc[2]) + cx_intrinsics;
        float v = fy * (Pc[1] / Pc[2]) + cy_intrinsics;

        if (u < 0 || u >= imageWidth || v < 0 || v >= imageHeight) continue;

        float distance = (Pw - camPos).norm();
        if (distance > distanceThreshold) continue;

        allPoints.push_back(Pw);
    }

    if (allPoints.size() < 50) return;

    std::srand(0);
    int maxInliers = 0;
    Eigen::Vector3f bestNormal(0, 1, 0);
    float bestD = 0;
    float threshold = 0.05f;
    int iterations = 100;

    for (int i = 0; i < iterations; ++i) {
        int idx1 = rand() % allPoints.size();
        int idx2 = rand() % allPoints.size();
        int idx3 = rand() % allPoints.size();
        if (idx1 == idx2 || idx2 == idx3 || idx1 == idx3) continue;

        Eigen::Vector3f p1 = allPoints[idx1];
        Eigen::Vector3f p2 = allPoints[idx2];
        Eigen::Vector3f p3 = allPoints[idx3];

        Eigen::Vector3f v1 = p2 - p1;
        Eigen::Vector3f v2 = p3 - p1;
        Eigen::Vector3f normal = v1.cross(v2).normalized();
        if (std::isnan(normal[0]) || std::isnan(normal[1]) || std::isnan(normal[2])) continue;

        float d = -normal.dot(p1);
        int inliers = 0;
        for (const auto& pt : allPoints) {
            float dist = std::abs(normal.dot(pt) + d);
            if (dist < threshold) ++inliers;
        }

        if (inliers > maxInliers) {
            maxInliers = inliers;
            bestNormal = normal;
            bestD = d;
        }
    }

    float voxelSize = 0.9f;
    std::vector<Eigen::Vector3f> filteredPoints = VoxelGridFilter(allPoints, voxelSize);

    std::lock_guard<std::mutex> lock(mGridMutex);

    for (const auto& pos : filteredPoints) {
        float distToPlane = std::abs(bestNormal.dot(pos) + bestD);
        if (distToPlane < 0.04f) continue;

        int x0 = static_cast<int>((camPos[0] - mOriginX) / mResolution);
        int y0 = mHeight - static_cast<int>((camPos[2] - mOriginY) / mResolution) - 1;

        int x1 = static_cast<int>((pos[0] - mOriginX) / mResolution);
        int y1 = mHeight - static_cast<int>((pos[2] - mOriginY) / mResolution) - 1;

        if (x1 < 0 || x1 >= mWidth || y1 < 0 || y1 >= mHeight) continue;

        RaycastAndMarkFreeAndOccupied(mGlobalOccupancyGrid, camPos, pos);
    }

    cv::Mat displayGrid(mGlobalOccupancyGrid.size(), CV_8UC1);
    for (int y = 0; y < mGlobalOccupancyGrid.rows; ++y) {
        for (int x = 0; x < mGlobalOccupancyGrid.cols; ++x) {
            schar val = mGlobalOccupancyGrid.at<schar>(y, x);
            displayGrid.at<uchar>(y, x) = (val == -1) ? 127 : (val == 0 ? 255 : 0);
        }
    }

    globalDisplayGrid = displayGrid.clone();

    int camGridX = static_cast<int>((camPos[0] - mOriginX) / mResolution);
    int camGridY = mHeight - static_cast<int>((camPos[2] - mOriginY) / mResolution) - 1;
    lastCamCircle = cv::Point(camGridX, camGridY);

    if (camGridX >= 0 && camGridX < displayGrid.cols && camGridY >= 0 && camGridY < displayGrid.rows) {
        cv::circle(globalDisplayGrid, lastCamCircle, 5, cv::Scalar(200), 2);
    }

    cv::imshow("Occupancy Grid", globalDisplayGrid);
    cv::waitKey(1);
    cv::imwrite("global_occupancy_grid.png", displayGrid);	//create png
    cv::imwrite("global_occupancy_grid.pgm", displayGrid);	//create pgm
    
    // write yaml file for pgm
    std::ofstream yamlFile("map.yaml");
if (yamlFile.is_open()) {
    yamlFile << "image: global_occupancy_grid.pgm\n";
    yamlFile << "resolution: " << mResolution << "\n";
    yamlFile << "origin: [" << mOriginX << ", " << mOriginY << ", 0.0]\n";
    yamlFile << "negate: 0\n";
    yamlFile << "occupied_thresh: 0.65\n";
    yamlFile << "free_thresh: 0.196\n";
    yamlFile.close();
    
} else {
    std::cerr << "[OccupancyGridBuilder] Failed to write map.yaml." << std::endl;
}

}

} // namespace ORB_SLAM3
