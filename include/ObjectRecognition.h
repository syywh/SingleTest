/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OBJECTRECOGNITION_H
#define OBJECTRECOGNITION_H

extern "C"{
#include "quickshift.h"
#include "generic.h"
}
#include "KeyFrame.h"
#include "ModelFrame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"
#include <boost/thread.hpp>
#include "LoopClosing.h"

#include "KeyFrameDatabase.h"
#include "ModelDataBase.h"
#include "FramePublisher.h"


#include "MapPoint.h"

namespace ORB_SLAM
{

class Tracking;
class KeyFrame;
class ModelDataBase;
class LocalMapping;
class LoopClosing;
class FramePublisher;
class MapPoint;

class ObjectRecognition
{
public:
  
  ObjectRecognition(ModelDataBase* mdatabase,ORBVocabulary *pVoc,FramePublisher* mrFramePublisher);
  ObjectRecognition(ModelDataBase* mdatabase,ORBVocabulary *pVoc);

  void Run();

  void SetLoopClosing(LoopClosing* pLoopClosing);
  void SetTracker(Tracking* pTracker);
  void SetLocalMapping(LocalMapping* pLocalMapping);
  
  void InsertKeyFrame(KeyFrame *pKF);	//可以在这里处理队列长度		××××××××在LocalMapping里面调用，LocalMapping判断是否加入新的关键帧
  
    //聚类当前keyframe的特征点
  void Segment(KeyFrame* pKF);
  
  cv::Mat ShowObject(KeyFrame* pKF, int num_of_Object);
  void findBounding(KeyFrame* pKF, int num, float& left,  float& right, float& up, float& down,vector<cv::KeyPoint>& keypoints);	//找特征点的边界方框
  
  void findBounding(KeyFrame* pKF, int num, float& left,  float& right, float& up, float& down);	//找特征点的边界方框

//   void AssignDescriptors(KeyFrame* pKF);
  
//   KeyFrame mrCKF;//当前处理的关键帧 
  KeyFrame* mrCurrentKF;//当前处理的关键帧 
  
  vector<string> index_ModelName;
  
  //维护检测到地图点的map
  map<MapPoint*, vector<long unsigned int> > CandidateForConfine;//vector的长度够三个，就确定 map.find()
  
  void CombineRect(KeyFrame* pKF);
  


public:
  Tracking* mrTracker;
  LoopClosing* mrLoopCloser;
  LocalMapping* mrLocalMapping;
  
  ModelDataBase* mrModelDB;
  ORBVocabulary* mrORBVocabulary;
 
  FramePublisher* mrFramePublisher;
  vector<int> root;
  VlQS* quickShift;
  
  cv::Mat copy_out;
  
  std::list<KeyFrame* > mrlpLoopKeyFrameQueue;//未处理关键帧队列，可以维持一个固定长度，或者超过一定长度了，delect掉一些
  boost::mutex mrMutexReadKF;
  boost::mutex mrMutexLoopQueue;
  
  bool CheckNewKeyFrames();
  
  int DetectObjects(KeyFrame* pKF);
  
  int findChildren(int root_index,int root, vector<int>& all ,vector<int >& part,int root_downcount);
  
  bool IoU(vector< cv::Rect >& rects, cv::Rect rect1,vector<map<float, ModelFrame* > > score_list, map<float, ModelFrame* > map_score_list);	//判断是否合并
  
  bool IoUOnlyRect(vector<cv::Rect>& rects, cv::Rect rect1,vector<vector<int > > &tempSegment, vector<int > vsegment);	//只通过面积判断是否合并区域
  
  bool FirstCheck;

};

} //namespace ORB_SLAM

#endif // OBJECTRECOGNITION_H
