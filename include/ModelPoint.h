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

#ifndef MODELPOINT_H
#define MODELPOINT_H

#include<opencv2/core/core.hpp>
#include"KeyFrame.h"
#include"Map.h"
#include "ModelFrame.h"
#include "ORBVocabulary.h"

#include<boost/thread.hpp>
#include <boost/iterator/iterator_concepts.hpp>


namespace ORB_SLAM
{

class ImageFeature;
class KeyFrame;
class Map;
class ModelFrame;


class ModelPoint
{
public:
   ModelPoint();
   ModelPoint(string &s, long unsigned int mnId, long unsigned int mnModelId_Id, const ORBVocabulary* voc);//s——Position
   long unsigned int mnModelId;
   long unsigned int mnModelId_Id;
   vector<int > modelframe_list;
   
   const ORBVocabulary* mmpORBVocabulary;

   cv::Point3f mWorldPos;	//一行一个坐标，x，y，z
   cv::Mat mWorldPosMat;
   std::map<ModelFrame*,size_t> mObservations;
   
   cv::Mat mDescriptors;
   vector<cv::Mat > mvDistinctDiscriptors;
   map<DBoW2::WordId, cv::Mat> mWord_Descriptor;
   void ComputeDistinctiveDescriptors();
   void ComputeDistinctiveDescriptorsForWord();
   
   cv::Mat GetDistinctiveDescriptors(vector<cv::Mat> vDescriptors );
   
   void addDescriptors(vector<cv::Mat> & vDescs);

};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
