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

extern "C"{
#include "quickshift.h"
#include "generic.h"
}
#include "LocalMapping.h"

#include "ObjectRecognition.h"

#include "LoopClosing.h"

#include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"

#include <ros/ros.h>

#include "Thirdparty/g2o/g2o/types/sim3/types_seven_dof_expmap.h"

#include "math.h"

using namespace cv;
namespace ORB_SLAM
{
ObjectRecognition::ObjectRecognition(ModelDataBase* mdatabase,ORBVocabulary *pVoc,FramePublisher* mrFramePublisher):
mrModelDB(mdatabase),mrORBVocabulary(pVoc),mrFramePublisher(mrFramePublisher)
{
//     rroot.resize(1000);
  root.reserve(3000*sizeof(int));
  index_ModelName.clear();
  index_ModelName = mdatabase->vmodelName;
  FirstCheck = false;
  
//   index_ModelName.push_back("cap");
//   index_ModelName.push_back("frog");
//   index_ModelName.push_back("Cup");
//   index_ModelName.push_back("book");
//   index_ModelName.push_back("comic_cup");
}

ObjectRecognition::ObjectRecognition(ModelDataBase* mdatabase,ORBVocabulary *pVoc):
mrModelDB(mdatabase),mrORBVocabulary(pVoc)
{
  root.reserve(3000*sizeof(int));
  index_ModelName.clear();
  index_ModelName = mdatabase->vmodelName;
  FirstCheck = false;
}

void ObjectRecognition::SetLoopClosing(LoopClosing* pLoopClosing)
{
  mrLoopCloser = pLoopClosing;


}

void ObjectRecognition::SetTracker(Tracking* pTracker)
{
  mrTracker = pTracker;
}

void ObjectRecognition::SetLocalMapping(LocalMapping* pLocalMapping)
{
  mrLocalMapping = pLocalMapping;

}


void ObjectRecognition::Run()
{
  ros::Rate r(200);
  while(ros::ok())
  {

    //检测等候被检测的关键帧队列是否有关键帧
    if(CheckNewKeyFrames())
    {
      //检测关键帧中物体模型
//       cout<<"NewKeyframe"<<endl;		Done
//       int patches = DetectObjects();
//        if( patches)
//        {
// // 	 mrFramePublisher->Updata(this,mrTracker,mrCurrentKF->patch_bestMatch);
// // 	 cout<<"Recognized: "<<(mrCurrentKF->Refine_Match_map_result.size())<<endl;
// 	 
//       }
      mrCurrentKF->SetErase();
    }
  }

}

bool ObjectRecognition::CheckNewKeyFrames()
{
    boost::mutex::scoped_lock lock(mrMutexLoopQueue);
    return(!mrlpLoopKeyFrameQueue.empty());
}

void ObjectRecognition::InsertKeyFrame(KeyFrame* pKF)	//可以在这里处理队列长度
{

  if(!FirstCheck)
    pKF->GetImage().copyTo(copy_out);
  
  
      boost::mutex::scoped_lock lock(mrMutexLoopQueue);
      if(pKF->mnId!=0)
      {
	if(mrlpLoopKeyFrameQueue.size()>4)
	{
// 	  mrlpLoopKeyFrameQueue.pop_front();
	  cout<<"######  好长好长处理不来啦   ######"<<endl;
	}
	mrlpLoopKeyFrameQueue.push_back(pKF);
      }
	  

}

void ObjectRecognition::Segment(KeyFrame* pKF)
{
  pKF->tsegment.clear();
  vector<KeyPoint> keypoints = pKF->GetKeyPoints(); 

  KeyPoint keypoint;

  int count = 0;
  double *image;		//怎么释放？
  image = new double[2*((pKF->GetDescriptors()).rows)];
     for(int i = 0; i<keypoints.size(); i++)
     {
       keypoint = keypoints[i];
       image[keypoints.size()*0+i] = keypoint.pt.x;
       image[keypoints.size()+i] = keypoint.pt.y;
    }
    
// Create a new quickshift instance using the image, the height and width of the// image as well as the number of channels.
  quickShift = vl_quickshift_new(image, keypoints.size(),1, 2);//(image, height, width, channels)
  vl_quickshift_set_kernel_size(quickShift,40);
  vl_quickshift_set_max_dist(quickShift,40);
  
  delete[] image;
  
  // Run Quick Shift.
  vl_quickshift_process(quickShift);
  int* parents;
  parents = vl_quickshift_get_parents(quickShift);
//   cout<<"parents: "<<parents[0]<<"  "<<parents[1]<<endl;
  
    for(int i = 0;i < keypoints.size(); i++)
  {

    pKF->all[i] = parents[i];

    if(parents[i]==i)
    {

      count++;

      pKF->all[i] = -count;
      root.push_back(i);	//root的index
    }
  }
//   delete parents;
   int root_count=0;
    vector<int > part;
   
    for(int i = 0; i<count; i++)
  {     
   
    root_count = 0;
    root_count=findChildren((i+1),root[i],pKF->all,part,root_count);
    if(part.size()>20)
    {
//       cout<<"!!!!"<<endl;
      pKF->tsegment.push_back(part);
    }
      
    part.clear();
  }

  //***************
//   pKF->mroot_num = pKF->tsegment.size();
  
  vl_quickshift_delete(quickShift);
  root.clear();

}

int ObjectRecognition::findChildren(int root_index, int root, vector< int >& all, vector< int >& part, int root_downcount)
{
   vector<int> children;
   root_downcount++;
   part.push_back(root);
   
   for(int i=0;i<all.size();i++)
   {
     if(all[i]==root)
     {
       children.push_back(i);	//记录了index
       all[i]=-root_index;	//被读取了的数置为-i
    }
  }
  if(children.size()==0)
  {
//     cout<<"leaves"<<endl;
    return root_downcount;	//这个root_downcount因为一直被传递，并且最后被返回了，所以是不是指针也无所谓
  }else
  {
//     cout<<"have children, the number is: "<<children.size()<<endl;
    for(int j=0;j<children.size();j++)
    {
     root_downcount= findChildren(root_index,children[j],all,part,root_downcount);
    }
      
  }
  return root_downcount;

}



int ObjectRecognition::DetectObjects(KeyFrame* pKF)
{
//       {
//         boost::mutex::scoped_lock lock(mrMutexLoopQueue);
// 	
// 	mrCurrentKF= (mrlpLoopKeyFrameQueue.front());
//         mrlpLoopKeyFrameQueue.pop_front();
// 	
//         // Avoid that a keyframe can be erased while it is being process by this thread
//         mrCurrentKF->SetNotErase();	//关键帧如果不满足一些条件，可能被删除
//     }
    

    mrCurrentKF = pKF;
    Segment(mrCurrentKF);	//分割成patches
    
    CombineRect(mrCurrentKF);	//融合IoU大于一定阈值的patches
    
/*    
    //得到每一个patch的候选模型图像列表，取得分排序的前十——score还是有用的
    cout<<"~~~~~~~~~~~~~~~1  Detect Model Candidates!  ~~~~~~~~~~~~~~~~"<<endl;
    int DetectPatches= mrModelDB->DetectFrameCandidates(mrCurrentKF);
    
//     int FundamentalCheck = mrModelDB->ChectFundamental(mrCurrentKF);
    
    //对列表计算PnP，得到patch的唯一匹配
    cout<<"~~~~~~~~~~~~~~~2  Check PnP!       ~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl;
    int PnPCheck = mrModelDB->CheckPnP(mrCurrentKF);

    cv::Mat out;
    if(PnPCheck!=0)
    {
      //vector<vector<cv::Point3f> > temp=mrModelDB->ModelPointsToBeInserted;
    //mrLocalMapping->InsertModelPoints(temp);

      mrLocalMapping->InsertModelPoints(mrModelDB->ModelPointsToBeInserted);
      
    out = ShowObject(mrCurrentKF,mrCurrentKF->Refine_Match_map_result.size());
    imshow("Detected Result",out);
    
    out.copyTo(copy_out);
    waitKey(2);
    
    }else{
      imshow("Detected Result",copy_out);
    }
    cout<<"after PnPCheck，该关键帧上所含的物体数量: "<<PnPCheck<<endl;*/

//     return mrCurrentKF->mroot_num;
//     return mrCurrentKF->Refine_Match_map_result.size();
    return mrCurrentKF->mroot_num;

}

bool ObjectRecognition::IoUOnlyRect(vector< Rect >& rects, Rect rect1, vector<vector<int > > &tempSegment, vector<int > vsegment)
{
  float MIN = 0.8;
   for(int i = 0; i < rects.size(); i++)
   {
      int a = (rect1.area() < rects[i].area()) ? 0:1;	//算IoU的时候面积不一样
      int insect = 0;
      float iou;
      if(a == 0)
      {
	for(int mrow = rect1.x; mrow<(rect1.width+rect1.x); mrow++)
	  for(int mcol = rect1.y; mcol<(rect1.height+rect1.y); mcol++)
	  {
	    if(rects[i].contains(Point2d(mrow,mcol)))
	    {
	      insect++;
	    }
	  }
	  iou = 1.0*insect/rect1.area();
	if((iou > MIN))
	{
	  int left = (rect1.x<rects[i].x) ? rect1.x:rects[i].x;
	  int up = (rect1.y<rects[i].y) ? rect1.y:rects[i].y;
	  int right = ((rect1.x+rect1.width)>(rects[i].x+rects[i].width))?(rect1.x+rect1.width):(rects[i].x+rects[i].width);
	  int down = ((rect1.y+rect1.height)>(rects[i].y+rects[i].height))?(rect1.y+rect1.height):(rects[i].y+rects[i].height);
	  rects[i] = cvRect(left,up,(right-left),(down-up));	
	  tempSegment[i].reserve(tempSegment[i].size()+vsegment.size());
	  for(int j = 0; j<vsegment.size(); j++)
	  {
	    tempSegment[i].push_back(vsegment[j]);
	  }
	  
	  return true;
      }
  }else{
    	for(int mrow = rects[i].x; mrow<(rects[i].width+rects[i].x); mrow++)
	  for(int mcol = rects[i].y; mcol<(rects[i].height+rects[i].y); mcol++)
	  {
	    if(rect1.contains(Point2d(mrow,mcol)))
	    {
	      insect++;
	    }
	  }
	  iou = 1.0*insect/rects[i].area();
	if((iou > MIN))
	{
	  int left = (rects[i].x<rect1.x) ? rects[i].x:rect1.x;
	  int up = (rects[i].y<rect1.y) ? rects[i].y:rect1.y;
	  int right = ((rects[i].x+rects[i].width)>(rect1.x+rect1.width))?(rects[i].x+rects[i].width):(rect1.x+rect1.width);
	  int down = ((rects[i].y+rects[i].height)>(rect1.y+rect1.height))?(rects[i].y+rects[i].height):(rect1.y+rect1.height);
	  rects[i] = cvRect(left,up,(right-left),(down-up));	
	  tempSegment[i].reserve(tempSegment[i].size()+vsegment.size());
	  for(int j = 0; j<vsegment.size(); j++)
	  {
	    tempSegment[i].push_back(vsegment[j]);
	  }
	  return true;
	}
  }
  
   }
   return false;
}


void ObjectRecognition::CombineRect(KeyFrame* pKF)
{
  
  vector<cv::Rect> vRect;	//用于判断是否融合
  
  for(int i = 0; i<(pKF->tsegment.size()); i++)
  {
    
    float left, right, up, down;
    left = 100000000;
    right = -100000000;
    up = 10000000;
    down = -100000000;
    
    findBounding(pKF,i,left,right,up,down);
    
    cv::Rect rec = cvRect(left, up, (right-left), (down-up));
    
    if(!IoUOnlyRect(vRect,rec,pKF->segment,pKF->tsegment[i]))
    {
      vRect.push_back(rec);
      pKF->segment.push_back(pKF->tsegment[i]);
    }
  }
  
  pKF->mroot_num = pKF->segment.size();

}



cv::Mat ObjectRecognition::ShowObject(KeyFrame* pKF, int num_of_Object)
{
  
  cv::Mat out;
  stringstream s;
  vector<cv::Rect > rects;
  vector<long unsigned int > rect_Index;
  vector<float > scores;
  vector<map<float, ModelFrame* > > score_list;
  
  int baseline=0;
  
  s<<"Detected Object Number: "<<num_of_Object;
  cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);
  
  {
    cv::Mat im;
    cv::Mat color;
    pKF->GetImage().copyTo(im);
    cvtColor(pKF->GetImage(), color, CV_GRAY2RGB);
    out = cv::Mat(im.rows+textSize.height+10,im.cols,color.type());
    color.copyTo(out.rowRange(0,im.rows).colRange(0,im.cols));
    out.rowRange(im.rows,out.rows) = cv::Mat::zeros(textSize.height+10, im.cols, im.type());
    cv::putText(out,s.str(),cv::Point(5,out.rows-5-textSize.height),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);
  }

  
  for(int i = 0;i<num_of_Object; i++)
  {
    
    float left, right, up, down;
    left = out.cols;
    right = -out.cols;
    up = out.rows;
    down = -out.rows;
    vector<cv::KeyPoint> keypoints;
    
    int inum = pKF->Refine_Match_map_result[i].first;
    findBounding(pKF,inum,left,right,up,down,keypoints);
    cv::drawKeypoints(pKF->GetImage(),keypoints,out,cv::Scalar(0,0,255));
    cv::Rect rec = cvRect(left, up, (right-left), (down-up));
    if(rects.size() == 0)
    {
      rects.push_back(rec);
      score_list.push_back(pKF->Refine_Match_map_result[i].second);
//       rect_Index.push_back(pKF->Refined_Object_Model[i].second);
//       scores.push_back(pKF->Refined_Object_Model_match_score[i].second);
    }else
    {
      if(!(IoU(rects, rec,score_list,pKF->Refine_Match_map_result[i].second)))		//返回true，表示是在Scope内,在IoU函数内实现了合并，还得判断是否属于同一物体。。。
      {//不满足合并条件
	rects.push_back(rec);
	score_list.push_back(pKF->Refine_Match_map_result[i].second);
	
      }
    }
   
  }
  
  for(int i = 0; i<(int)(rects.size()); i++)
  {
    
    rectangle(out,Point(rects[i].x,rects[i].y),Point((rects[i].x+rects[i].width),rects[i].y+rects[i].height), Scalar(255,0,0),2);
    map<float, ModelFrame* >::iterator it;
    int count = 0;
    it = score_list[i].end();
    it--;
    if(score_list[i].size()==1)
    {
       stringstream ss;
       ss<<(index_ModelName[(int)(it->second->mModel_Id)])<<" si: " << it->first;
       cv::putText(out,ss.str(),cv::Point(rects[i].x,(rects[i].y+5+textSize.height*(++count))),cv::FONT_HERSHEY_PLAIN,1.2,cv::Scalar(0,255,0),2,8);
    }else{
	for(; it!=score_list[i].begin(); it--)
	{
	  stringstream ss;
	  ss<<(index_ModelName[(int)(it->second->mModel_Id)])<<" si: " << it->first;
	  cv::putText(out,ss.str(),cv::Point(rects[i].x,(rects[i].y+5+textSize.height*(++count))),cv::FONT_HERSHEY_PLAIN,1.2,cv::Scalar(0,255,0),2,8);
    }
    }
    

    
   
  }
  return out;
}

//发现第i个patch对应的区域边界
void ObjectRecognition::findBounding(KeyFrame* pKF, int i, float& left, float& right, float& up, float& down,vector<cv::KeyPoint>& mkeypoints)
{
  
  vector<cv::KeyPoint> keypoints = pKF->GetKeyPoints();
  int num = i;
  
  
  for(int i = 0; i<(int)(pKF->segment[num].size()); i++)
  {
    
    mkeypoints.push_back(keypoints[pKF->segment[num][i]]);
    if(keypoints[pKF->segment[num][i]].pt.x  < left)
      left = keypoints[pKF->segment[num][i]].pt.x;
    if(keypoints[pKF->segment[num][i]].pt.x  > right)
      right = keypoints[pKF->segment[num][i]].pt.x;
    if(keypoints[pKF->segment[num][i]].pt.y  < up)
      up = keypoints[pKF->segment[num][i]].pt.y;
    if(keypoints[pKF->segment[num][i]].pt.y  > down)
      down = keypoints[pKF->segment[num][i]].pt.y;

  }
  
}

void ObjectRecognition::findBounding(KeyFrame* pKF, int num, float& left, float& right, float& up, float& down)
{
  vector<cv::KeyPoint> keypoints = pKF->GetKeyPoints();
  for(int i = 0; i<(int)(pKF->tsegment[num].size()); i++)
  {
    
    if(keypoints[pKF->tsegment[num][i]].pt.x  < left)
      left = keypoints[pKF->tsegment[num][i]].pt.x;
    if(keypoints[pKF->tsegment[num][i]].pt.x  > right)
      right = keypoints[pKF->tsegment[num][i]].pt.x;
    if(keypoints[pKF->tsegment[num][i]].pt.y  < up)
      up = keypoints[pKF->tsegment[num][i]].pt.y;
    if(keypoints[pKF->tsegment[num][i]].pt.y  > down)
      down = keypoints[pKF->tsegment[num][i]].pt.y;

  }
}


bool ObjectRecognition::IoU(vector< Rect >& rects, Rect rect1,vector<map<float, ModelFrame* > > score_list, map<float, ModelFrame* > map_score_list)
{
   for(int i = 0; i < rects.size(); i++)
    {
      int a = (rect1.area() < rects[i].area()) ? 0:1;
      int insect = 0;
      float iou;
      if(a == 0)
      {
	for(int mrow = rect1.x; mrow<(rect1.width+rect1.x); mrow++)
	  for(int mcol = rect1.y; mcol<(rect1.height+rect1.y); mcol++)
	  {
	    if(rects[i].contains(Point2d(mrow,mcol)))
	    {
	      insect++;
	    }
	  }
	  iou = 1.0*insect/rect1.area();
	if((iou > 0.7))
	{
	  int left = (rect1.x<rects[i].x) ? rect1.x:rects[i].x;
	  int up = (rect1.y<rects[i].y) ? rect1.y:rects[i].y;
	  int right = ((rect1.x+rect1.width)>(rects[i].x+rects[i].width))?(rect1.x+rect1.width):(rects[i].x+rects[i].width);
	  int down = ((rect1.y+rect1.height)>(rects[i].y+rects[i].height))?(rect1.y+rect1.height):(rects[i].y+rects[i].height);
	  rects[i] = cvRect(left,up,(right-left),(down-up));	
	  
	  //比较score，合并
	  map<float, ModelFrame* >::iterator m1 = score_list[i].begin();
	  map<float, ModelFrame* >::iterator m2 = map_score_list.begin();
	  map<float, ModelFrame*> temp_score_list;
	  while((m1 != score_list[i].end()) && (m2 != map_score_list.end()))
	  {
	    if(m1->first == m2->first)
	    {
	      temp_score_list.insert(pair<float, ModelFrame*>(m1->first, m1->second));
	      m1++;
	      m2++;
	    }else{
	      if(m1->first > m2->first)
	      {
		temp_score_list.insert(pair<float, ModelFrame*>(m2->first, m2->second));
		m2++;
	      }else{
		temp_score_list.insert(pair<float, ModelFrame*>(m1->first, m1->second));
		m1++;
	      }
	    }
	  }
	  score_list[i] = temp_score_list;
	  
	  return true;
	}
      }else{
	for(int mrow = rects[i].x; mrow<(rects[i].width+rects[i].x); mrow++)
	  for(int mcol = rects[i].y; mcol<(rects[i].height+rects[i].y); mcol++)
	  {
	    if(rect1.contains(Point2d(mrow,mcol)))
	    {
	      insect++;
	    }
	  }
	  iou = 1.0*insect/rects[i].area();
	if((iou > 0.7))
	{
	  int left = (rects[i].x<rect1.x) ? rects[i].x:rect1.x;
	  int up = (rects[i].y<rect1.y) ? rects[i].y:rect1.y;
	  int right = ((rects[i].x+rects[i].width)>(rect1.x+rect1.width))?(rects[i].x+rects[i].width):(rect1.x+rect1.width);
	  int down = ((rects[i].y+rects[i].height)>(rect1.y+rect1.height))?(rects[i].y+rects[i].height):(rect1.y+rect1.height);
	  rects[i] = cvRect(left,up,(right-left),(down-up));	
	  
	  map<float, ModelFrame* >::iterator m1 = score_list[i].begin();
	  map<float, ModelFrame* >::iterator m2 = map_score_list.begin();
	  map<float, ModelFrame*> temp_score_list;
	  while((m1 != score_list[i].end()) && (m2 != map_score_list.end()))
	  {
	    if(m1->first == m2->first)
	    {
	      temp_score_list.insert(pair<float, ModelFrame*>(m1->first, m1->second));
	      m1++;
	      m2++;
	    }else{
	      if(m1->first > m2->first)
	      {
		temp_score_list.insert(pair<float, ModelFrame*>(m2->first, m2->second));
		m2++;
	      }else{
		temp_score_list.insert(pair<float, ModelFrame*>(m1->first, m1->second));
		m1++;
	      }
	    }
	  }
	  score_list[i] = temp_score_list;
	  
	  return true;
	}
	
      }
      
     }
     return false;

}






} //namespace ORB_SLAM
