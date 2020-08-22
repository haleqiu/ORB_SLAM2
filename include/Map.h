/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>
#include <Eigen/Dense>

#include <mutex>



namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;

typedef std::vector<cv::Mat> humanposes;
class Map
{
public:
    int body1[13] = {1, 1, 2, 3, 5, 6, 1, 8, 9, 1, 11, 12, 1 };
    int body2[13] = {2, 5, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 0};

    std::vector<Eigen::Vector3f> colormap{ // = {0,0,0,0,0,0,1,1,1,1,1,1,2};
    Eigen::Vector3f(0,0,230)/255.0,
    Eigen::Vector3f(0,0,230)/255.0,
    Eigen::Vector3f(0,0,230)/255.0,
    Eigen::Vector3f(0,0,230)/255.0,
    Eigen::Vector3f(0,0,230)/255.0,
    Eigen::Vector3f(0,0,230)/255.0,
    Eigen::Vector3f(0,230,0)/255.0,
    Eigen::Vector3f(0,230,0)/255.0,
    Eigen::Vector3f(0,230,0)/255.0,
    Eigen::Vector3f(0,230,0)/255.0,
    Eigen::Vector3f(0,230,0)/255.0,
    Eigen::Vector3f(0,230,0)/255.0,
    Eigen::Vector3f(230,0,230)/255.0};

    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    //MapHumanPose
    void AddMapHumanPose(std::vector<cv::Mat>* v);

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();
    std::vector<std::vector<cv::Mat>*> GetAllHumanPoses();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;
    std::vector<std::vector<cv::Mat>*> msvHumanPose;// TODO consider the human pose in class and pointer in the future


protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;
};

} //namespace ORB_SLAM

#endif // MAP_H
