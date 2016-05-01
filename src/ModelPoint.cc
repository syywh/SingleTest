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

#include "ModelPoint.h"
#include "ORBmatcher.h"

#include "ros/ros.h"

namespace ORB_SLAM
{
  ModelPoint::ModelPoint()
  {
    
  }
  
ModelPoint::ModelPoint(string &s, long unsigned int mnId, long unsigned int mnId_id,const ORBVocabulary* voc):
mnModelId(mnId),mnModelId_Id(mnId_id),mmpORBVocabulary(voc)
{
  mWorldPosMat = cv::Mat::zeros(3,1,CV_32F);
  
  stringstream ss;
  ss<<s;

  
  double pos;
  ss>>pos;
  mWorldPos.x = (pos);
  mWorldPosMat.at<float>(0,0) = pos;
  ss>>pos;
  mWorldPos.y = (pos);
  mWorldPosMat.at<float>(1,0) = pos;
  ss>>pos;
  mWorldPos.z = (pos);	//test OK
  mWorldPosMat.at<float>(2,0) = pos;
  

}

void ModelPoint::ComputeDistinctiveDescriptors()
{
  vector<cv::Mat> vDescriptors;
  map<ModelFrame*,size_t> observations;
  observations = mObservations;
  
  vDescriptors.reserve(observations.size());
  
    for(map<ModelFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        ModelFrame* pKF = mit->first;
        vDescriptors.push_back(pKF->mmDescriptors.row(mit->second));
    }
    
    const size_t N = vDescriptors.size();
    
    float Distances[N][N];
    
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }
    
    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);//傻不傻，是一行的吧
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }
    
    mDescriptors = vDescriptors[BestIdx].clone();       


}

cv::Mat ModelPoint::GetDistinctiveDescriptors(vector< cv::Mat > vDescriptors)
{
   const size_t N = vDescriptors.size();
    
    float Distances[N][N];
    
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }
    
    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);//傻不傻，是一行的吧
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }
    
    return vDescriptors[BestIdx].clone();     
  

}

void ModelPoint::ComputeDistinctiveDescriptorsForWord()
{
//   vector<cv::Mat> vDescriptors;
  map<ModelFrame*,size_t> observations;
  observations = mObservations;
  
  map<DBoW2::WordId, vector<cv::Mat > > mWord_Discs; 
  map<DBoW2::WordId, vector<cv::Mat > >::iterator it_mWord_Discs;
  


    for(map<ModelFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        ModelFrame* pKF = mit->first;
//         vDescriptors.push_back(pKF->mmDescriptors.row(mit->second));
	DBoW2::WordId temp_word = mmpORBVocabulary->transform(pKF->mmDescriptors.row(mit->second));
	it_mWord_Discs = mWord_Discs.lower_bound(temp_word);
	
	if(it_mWord_Discs == mWord_Discs.end() || (mWord_Discs.key_comp()(temp_word,it_mWord_Discs->first)))
	{
	  vector<cv::Mat> temp_vector;
	  temp_vector.push_back(pKF->mmDescriptors.row(mit->second));
	  mWord_Discs.insert(it_mWord_Discs,map<DBoW2::WordId, vector<cv::Mat > >::value_type(temp_word,temp_vector) );
	}
	
	else
	{
	  it_mWord_Discs->second.push_back(pKF->mmDescriptors.row(mit->second));
	}
	
    }
//     vDescriptors.reserve(observations.size());	//这么多个描述子,多reserve一点

    for(it_mWord_Discs = mWord_Discs.begin(); it_mWord_Discs != mWord_Discs.end(); it_mWord_Discs++)
    {
      pair<DBoW2::WordId, cv::Mat> pair_word_mat;
      pair_word_mat.first = it_mWord_Discs->first;
      pair_word_mat.second = GetDistinctiveDescriptors(it_mWord_Discs->second);
      mWord_Descriptor.insert(pair_word_mat);
      mvDistinctDiscriptors.push_back(pair_word_mat.second);
    }
    cout<<"Descriptor size："<<observations.size()<<"WordSize："<<mWord_Discs.size()<<" dd:"<<mvDistinctDiscriptors.size()<<endl;
}

void ModelPoint::addDescriptors(vector< cv::Mat >& vDescs)
{
  for(size_t i = 0; i<mvDistinctDiscriptors.size(); i++)
  {
    vDescs.push_back(mvDistinctDiscriptors[i]);
  }

}



} //namespace ORB_SLAM
