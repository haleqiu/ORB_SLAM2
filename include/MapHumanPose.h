

#ifndef MAPHUMANPOSE_H
#define MAPHUMANPOSE_H

#include<opencv2/core/core.hpp>

#include<mutex>
#include <map>

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;

// the struct to store human pose consider for a new class or not
typedef std::pair<int, int> posekeypoints;
struct human_pose{
  int human_idx;
  std::vector<posekeypoints> vHumanKeyPoints;
  std::vector<posekeypoints> vHumanKeyPointsRight;
  std::vector<float> vKeysConfidence;
  std::vector<float> vKeysConfidenceRight;
  std::vector<float> vHumanKeyDepths;
  std::vector<bool> vbIsBad;
};

struct MapHumanKey{
  int mnId;
  cv::Mat WorldPos;
  bool bIsBad;
};

class MapHumanPose
{
public:
    MapHumanPose(const std::vector<cv::Mat> &vPos, KeyFrame* pRefKF, Map* pMap);
    MapHumanPose(const std::vector<cv::Mat> &vPos, const std::vector<bool> &visbad, KeyFrame* pRefKF, Map* pMap);

    cv::Mat GetHumanKeyPos(int nkey);
    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx);

    bool isBad(int nkey);

public:
  long unsigned int mnId;
  static long unsigned int nNextId;
  long int mnFirstKFid;
  long int mnFirstFrame;

  long unsigned int mnTrackId = -1;

protected:

    std::vector<MapHumanKey*> mvHumanKeysPos;
    std::vector<cv::Mat> vHumanKeysPos;// For lazy reason
    std::vector<bool> vbIsBad;

    // Keyframes observing the human pose and associated index in keyframe
    std::map<KeyFrame*,size_t> mObservations;

    // Reference KeyFrame // TODO what if all the viewed frame.
    KeyFrame* mpRefKF;

    Map* mpMap;

    std::mutex mMutexHumanPose;
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
