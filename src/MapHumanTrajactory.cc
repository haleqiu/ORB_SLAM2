

#include "MapHumanPose.h"
#include "MapHumanTrajactory.h"
#include "MapPoint.h"
#include "ORBmatcher.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{
  long unsigned int MapHumanTrajactory::nNextId=0;

  MapHumanTrajactory::MapHumanTrajactory(){
      mnId = nNextId++;
  }

  void MapHumanTrajactory::AddMapHumanPose(MapHumanPose* pHumanPose){
      mvAllHumanTrajactory.push_back(pHumanPose);
      // for (int itr = 0; itr<13; itr++){
      //
      // }
      //ToDo Time stamp

      // Add the Rigid Body constraint

  }

} //namespace ORB_SLAM
