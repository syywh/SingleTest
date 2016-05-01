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

#ifndef MODEL_H
#define MODEL_H

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


class tracking;
class MapPoint;
class KeyFrame;
class KeyFrameDatabase;
class ModelPoint;

class Model
{
public:
    Model();
    Model(string& name);
    vector<ModelPoint* > vModelPoints;
    
    // Bag of Words Vector structures
    DBoW2::BowVector mmBowVec;
    DBoW2::FeatureVector mmFeatVec;
    string m_name;
    



};

}// namespace ORB_SLAM

#endif // MODELFRAME_H
