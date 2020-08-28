

#include "MapHumanPose.h"
#include "ORBmatcher.h"
#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<mutex>

namespace ORB_SLAM2
{

  long unsigned int MapHumanPose::nNextId=0;

  MapHumanPose::MapHumanPose(const std::vector<cv::Mat> &vPos, const std::vector<bool> &visbad, KeyFrame *pRefKF, Map* pMap):
  mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), mpRefKF(pRefKF), mpMap(pMap)
  {
    mnId = nNextId++;
    vHumanKeysPos = vPos;
    int count = 0;// so ugly
    for (std::vector<cv::Mat>::const_iterator itMat = vPos.begin(); itMat != vPos.end(); itMat++){
        MapHumanKey* pMapKey = new MapHumanKey;
        pMapKey->mnId = nNextId++;
        pMapKey->WorldPos = *itMat; // is this a deep copy?
        pMapKey->bIsBad = visbad[count]; count++;
        mvHumanKeysPos.push_back(pMapKey);
    }
    vbIsBad = visbad;

    // MapHumanPose can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
  }

  cv::Mat MapHumanPose::GetHumanKeyPos(int nkey)
  {
    unique_lock<mutex> lock(mMutexHumanPose);
    return vHumanKeysPos.at(nkey).clone();
  }


  bool MapHumanPose::isBad(int nkey)
  {
    unique_lock<mutex> lock(mMutexHumanPose);
    return vbIsBad.at(nkey);
  }



} //namespace ORB_SLAM
