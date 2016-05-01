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

#include "ModelFrame.h"
#include "Converter.h"

#include <ros/ros.h>
using namespace std;
namespace ORB_SLAM
{

ModelFrame::ModelFrame()
{
}


//Copy Constructor
ModelFrame::ModelFrame(const ModelFrame& modelframe):
mmORBvocabulary(modelframe.mmORBvocabulary),mmBowVec(modelframe.mmBowVec),mmFeatVec(modelframe.mmFeatVec),mModel_Id(modelframe.mModel_Id),
mModel_Frame_Id(modelframe.mModel_Frame_Id),mmRecQuery_Mid(0),mmRecQuery_Mid_id(0)
{

}


ModelFrame::ModelFrame(long unsigned int Model_Id,  long unsigned int Model_Frame_Id,cv::Mat descriptor, cv::Mat keypoints, cv::Mat image,const ORBVocabulary* voc)
    :mmORBvocabulary(voc),mmDescriptors(descriptor),mmKeyPoints(keypoints),mModel_Id(Model_Id),mModel_Frame_Id(Model_Frame_Id),image(image)
{
    // Exctract ORB  
//     (*mpORBextractor)(im,cv::Mat(),mvKeys,mDescriptors);

    mN = mmDescriptors.rows;
    ComputeBoW();
    mmRecQuery_Mid = 0;
    mmRecQuery_Mid_id = 0;
    mvpModelPoints.resize(keypoints.rows);
    
//     cout<<"mmBowVec:"<<mmBowVec.<<"mmFeatVec: "<<mmFeatVec<<endl;

}

void ModelFrame::ComputeBoW()
{
      if(mmBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mmDescriptors);
        mmORBvocabulary->transform(vCurrentDesc,mmBowVec,mmFeatVec,4);
    }

}


} //namespace ORB_SLAM
