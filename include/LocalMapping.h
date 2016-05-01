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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include <boost/thread.hpp>
#include "KeyFrameDatabase.h"
#include "ObjectRecognition.h"
#include "ModelPoint.h"

namespace ORB_SLAM
{

class Tracking;
class LoopClosing;
class ObjectRecognition;
class Map;
class ModelPoint;

class LocalMapping
{
public:
    LocalMapping(Map* pMap);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);
    
    void SetObjectRecognizer(ObjectRecognition* pObjectRecognizer);

    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    void RequestStop();
    void RequestReset();

    void Stop();

    void Release();

    bool isStopped();

    bool stopRequested();

    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);

    void InterruptBA();
    
    void CreateNewMapPointsByObjectDetection(vector<ModelPoint*> mModelPoints, KeyFrame* pKF ,vector<cv::DMatch> matches);
    
    ///////

    void InsertModelPoints(vector<vector<cv::Point3f> >  maddModelPoints);

protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();

    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    void ResetIfRequested();
    bool mbResetRequested;
    boost::mutex mMutexReset;

    Map* mpMap;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;
    ObjectRecognition * mpObjectRecognizer;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    boost::mutex mMutexNewKFs;    //这个类里的，在检查新关键帧，插入关键帧，处理关键帧的时候锁上，让这个线程不运行？

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    boost::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    boost::mutex mMutexAccept;
    
    list<vector<cv::Point3f> > mlvModelPoints3f;
    bool mbMpInserted;
    boost::mutex mMutexModelPoints;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
