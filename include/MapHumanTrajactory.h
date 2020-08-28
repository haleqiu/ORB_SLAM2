

#ifndef MAPHUMANTRAJACTORY_H
#define MAPHUMANTRAJACTORY_H

#include<opencv2/core/core.hpp>

#include<mutex>
#include <map>


namespace ORB_SLAM2
{

  class MapPoint;

struct Rigidbody{//means one constraint one line //how to measure the edge
  MapPoint* first_point;
  MapPoint* second_point;
  int mnId;//the id reffered to the distance vertex
};

// The trajactory of the human pose includes time, motion and the distance of each part.
// This will be add and associated with the Vertex created in g2o, ex. Vertex Distance.
class MapHumanTrajactory
{
public:
    MapHumanTrajactory();

    // std::map<KeyFrame*,size_t> GetObservations();
    void AddMapHumanPose(MapHumanPose* pHumanPose);

    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx);

    bool isBad();

public:
  long unsigned int mnId;
  static long unsigned int nNextId;

  std::vector<MapHumanPose*> mvAllHumanTrajactory;// Sequential
  std::vector<cv::Mat> mvPoseHumanTrajactory; // Maybe the hip bone to constraint the motion


protected:

    // Keyframes observing the point and associated index in keyframe
    // std::map<KeyFrame*,size_t> mObservations;

    // Reference KeyFrame
    std::mutex mMutexHumanTrajactory;
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
