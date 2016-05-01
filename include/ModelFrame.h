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

#ifndef MODELFRAME_H
#define MODELFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "ModelPoint.h"


#include <opencv2/opencv.hpp>

namespace ORB_SLAM
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class tracking;
class MapPoint;
class KeyFrame;
class KeyFrameDatabase;
class ModelPoint;

class ModelFrame
{
public:
    ModelFrame();
    ModelFrame(const ModelFrame &Modelframe);	//必须是引用
//     Frame(cv::Mat &im, const double &timeStamp, ORBextractor* extractor, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef);
    ModelFrame(long unsigned int Model_Id, long unsigned int Model_Frame_Id,cv::Mat descriptor, cv::Mat keypoints, cv::Mat image,const ORBVocabulary* voc);
    
    
   const  ORBVocabulary* mmORBvocabulary;
//     // Frame image
//     cv::Mat im;

    // Number of KeyPoints
    int mN;
    
    //图像编号
   long unsigned int mModel_Id;
    long unsigned int mModel_Frame_Id;
//     // Vector of keypoints (original for visualization) and undistorted (actually used by the system)
//     std::vector<cv::KeyPoint> mvKeys;
//     std::vector<cv::KeyPoint> mvKeysUn;

    //搜索时使用，判断是否被搜索过
    long unsigned int mmRecQuery_Mid;//第几个Keyframe
    int mmRecQuery_Mid_id;//Keyframe里面的第几个块
    int mmRecWords;
    float mmRecScore;
    
    // Bag of Words Vector structures
    DBoW2::BowVector mmBowVec;
    DBoW2::FeatureVector mmFeatVec;

    // ORB descriptor, each row associated to a keypoint
    cv::Mat mmDescriptors;
    cv::Mat mmKeyPoints;
    cv::Mat image;
    vector<cv::KeyPoint> mvKeyPoins; 
    std::vector<ModelPoint*> mvpModelPoints;

//     // Camera Pose
//     cv::Mat mTcw;

    void ComputeBoW();



};

}// namespace ORB_SLAM

#endif // MODELFRAME_H
