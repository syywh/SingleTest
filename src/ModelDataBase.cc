extern "C"{
#include "quickshift.h"
#include "generic.h"
}
#include <ModelDataBase.h>
#include <ModelFrame.h>
#include <LocalMapping.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include<ros/ros.h>
#include<ros/package.h>
#include "Converter.h"
#include "ModelPoint.h"
#include "ORBmatcher.h"
#include "Model.h"
#include </home/dxq/openMVG/openMVG/src/third_party/eigen/Eigen/src/Core/products/GeneralBlockPanelKernel.h>


using namespace cv;
using namespace std;

namespace ORB_SLAM
{
ModelDataBase::ModelDataBase(const ORBVocabulary &voc):
mdbpORBVocabulary(&voc)
{
  mdbInvertedFile.resize(voc.size());		//mdbInvertedFile——————std::vector<std::list<cv::KeyFrame*> >，resize使得vector的长度与voc中的word的长度相同
  mdbInvertedFile4Model.resize(voc.size());
  cout<<voc.size()<<endl;
  segment_Bow.reserve(1000);	///1000*sizeof(cv::Mat(1,32,))
  vCurrentDesc.reserve(1000*sizeof(cv::Mat(1,32,CV_32F)));
  
  K<<485.8898, 0, 347.4504, 0,486.0202,246.3170,0,0,1;

//   MatK.resize(3);
  Mat mm(3,3,CV_32F);
  mm.at<float>(0,0) = 485.8898;
  mm.at<float>(0,1) = 0;
  mm.at<float>(0,2) = 347.4504;
  mm.at<float>(1,0) = 0;
  mm.at<float>(1,1) = 486.0202;
  mm.at<float>(1,2) = 246.3170;
  mm.at<float>(2,0) = 0;
  mm.at<float>(2,1) = 0;
  mm.at<float>(2,2) = 1;
  mm.copyTo(MatK);
  
  Mat distCoeffs(1,4,CV_32FC1);
  distCoeffs.at<float>(0,0) = 0.1476;
  distCoeffs.at<float>(0,1) = -0.1238;
  distCoeffs.at<float>(0,2) = 0.0043;
  distCoeffs.at<float>(0,3) = -0.0043;
  distCoeffs.copyTo(MdistCoeffs);
  
}
 
void ModelDataBase::SetLocalMapper(LocalMapping* LocalMapper)
{
  mrLocalMapping = LocalMapper;

}


void ModelDataBase::Construct()
{
  string listName = "/home/dxq/ORB_SLAM/Model/model_list.txt";
  ifstream fileIn;
  fileIn.open(listName.c_str());
while(true)
{
//读入模型list，格式是模型——图片数量
  string s;
   if(getline(fileIn,s)){
     string name;
     int num;
     stringstream temp;
     temp<<s;
     temp>>name;
     temp>>num;
     
     Model* tempModel = new Model(name);
     vModel.push_back(tempModel);
     
     model_num.push_back(num);
     vmodelName.push_back(name);
     cout<<name<<"  "<<num<<endl;//注意文本最后一行不能回车
//      fsModelXml<<name.c_str()<<num;
   }else
   {
     fileIn.close();
      break;
//       fsModelXml.release();
      
  }
    
}
  vModel_BowVector.resize(model_num.size());
  vModel_FeatureVector.resize(model_num.size());
  
  string modelFile = ros::package::getPath("ORB_SLAM")+"/Model/";
//    string strXmlFile = modelFile +"model_list.xml";
//   FileStorage fxml(strXmlFile.c_str(), FileStorage::READ);
   
//   model_num.push_back(fxml["cap"]);			//model_num:  模型种类的vector，每个元素表明该模型有多少张图片
//   model_num.push_back(fxml["frog"]);
//   model_num.push_back(fxml["cup"]);
//   model_num.push_back(fxml["book"]);
//   model_num.push_back(fxml["comic_cup"]);
  int count_keypoint = 0;
  vvbundlerxml.reserve((int)(model_num.size()));
  for (int  i = 0; i < model_num.size(); i++)
  {
    vector<ModelFrame*> temp_list;
    Mat *desc = new Mat[model_num[i]]; 
    Mat *keyp = new Mat[model_num[i]];
    for(int j = 0; j<(int)(model_num[i]); j++)
    {
      stringstream jj;
      jj<<j;
      string strKeyDescFile = modelFile+vmodelName[i]+"/"+jj.str()+".xml";
      string strImageName = modelFile+vmodelName[i]+"/"+jj.str()+".jpg";
//       cout<<strKeyDescFile<<endl;
      FileStorage  fdesc(strKeyDescFile.c_str(),FileStorage::READ);
      fdesc["descriptor"]>>desc[j] ;
      fdesc["keypoint"]>>keyp[j];
      count_keypoint+=desc[j].rows;
//       ModelFrame(i, j, desc[j], &voc);
//       cout<<"1"<<endl;
      Mat image = imread(strImageName.c_str());
      ModelFrame *modelframe = new ModelFrame(i, j, desc[j],keyp[j],image, mdbpORBVocabulary);
      
      
      temp_list.push_back(modelframe);
      add(modelframe);
    }
    mdbModelFrames.push_back(temp_list);
   }
    int count = 0;
    for(int i=0;i<mdbInvertedFile.size();i++) //mdbInvertedFile
    {
      if(mdbInvertedFile[i].size()!=0)
      {
	count+=mdbInvertedFile[i].size();
      }
      
    }
    cout<<count_keypoint<<endl;
    cout<<count<<endl;
    cout<<"所有modelFrame的数量mdbModelFrames.size()："<<mdbModelFrames.size()<<endl;
  

}

void ModelDataBase::add4Model(int i, DBoW2::BowVector bv)
{
  for(DBoW2::BowVector::const_iterator vit = bv.begin(), vend = bv.end(); vit != vend; vit++)
  {
    mdbInvertedFile4Model[vit->first].push_back(i);
  }

}


void ModelDataBase::TestWordDistribution()
{
  string listName = "/home/dxq/ORB_SLAM/Model/model_list.txt";
  ifstream fileIn;
  fileIn.open(listName.c_str());
while(true)
{
//读入模型list，格式是模型——图片数量
  string s;
   if(getline(fileIn,s)){
     string name;
     int num;
     stringstream temp;
     temp<<s;
     temp>>name;
     temp>>num;
     model_num.push_back(num);
     vmodelName.push_back(name);
     cout<<name<<"  "<<num<<endl;//注意文本最后一行不能回车
//      fsModelXml<<name.c_str()<<num;
   }else
   {
     fileIn.close();
      break;
//       fsModelXml.release();
      
  }
    
}
  
  
  string modelFile = ros::package::getPath("ORB_SLAM")+"/Model/";

  vector<vector<int > > Histogram;
  Histogram.resize(model_num.size());
  cout<<"Histogram Model size: "<<model_num.size()<<endl<<endl;


//   vvbundlerxml.resize((int)(model_num.size()));
  //模型种类
  for (int  i = 0; i < model_num.size(); i++)
  {

    Mat *desc = new Mat[model_num[i]]; 
//该模型种类里面的图片数量
    Histogram[i].resize(mdbpORBVocabulary->size());
    for(int j = 0; j<(int)(model_num[i]); j++)
    {
      stringstream jj;
      jj<<j;
      string strKeyDescFile = modelFile+vmodelName[i]+"/"+jj.str()+".xml";

      FileStorage  fdesc(strKeyDescFile.c_str(),FileStorage::READ);
      fdesc["descriptor"]>>desc[j] ;


      
      for(size_t row = 0; row < desc[j].rows; row++)
      {
	Mat temp_desc = desc[j].row(row);
	DBoW2::WordId id = mdbpORBVocabulary->transform(temp_desc);
	Histogram[i][id] ++;
      }
      cout<<"Histogram"<<i<<".size()"<<Histogram[i].size()<<endl;

    }

   }
   FileStorage Hisfs("Histogram.xml",FileStorage::WRITE);
   Hisfs<<"Distribution"<<Histogram;
   Hisfs.release();
  
}



void ModelDataBase::MatchPointFrame()
{
  
  string modelFile = ros::package::getPath("ORB_SLAM")+"/Model/";
  vector<vector<ModelFrame* > >::iterator vvit = mdbModelFrames.begin();
  vector<ModelFrame* >::iterator vit = vvit->begin();
  vvRT.reserve((int)(vmodelName.size()));
  vvfk.reserve((int)(vmodelName.size()));
  vvModelPoints.reserve((int)(vmodelName.size()));

  
  for(int i = 0; i<(int)(vmodelName.size()); i++)	//按类别
  {
    string rd = modelFile+vmodelName[i]+"/"+"list_xml2.rd.txt";	//里面存储的是number，序号
    string bundler = modelFile+vmodelName[i]+"/"+"bundle.rd.out";
    ifstream fileIn;
    fileIn.open(rd.c_str());
    
    
    //读取list_xml2.rd.txt
    vector<string> temp;
    while(true)
    {
      string s;
      if(getline(fileIn,s))
      {
// 	xmllist.push_back(s);
// 	vvbundlerxml[i].push_back(s);
	temp.push_back(s);
      }else{
	fileIn.close();
	break;
      }
    }
    vvbundlerxml.push_back(temp);
    
    
    //读取相机位姿和3D点
    ifstream fileBundler;
    fileBundler.open(bundler.c_str());
    
    string s;    
    getline(fileBundler,s);
    
    string num;
    getline(fileBundler,num);
    
    stringstream numC_P;
    numC_P<<num;
    int camera,Points;
    numC_P>>camera>>Points;
    
    
    // 相机位姿
    vector<string > temp_fk;
    vector<Mat* > temp_rt;
    
    for(int c = 0; c<camera; c++)
    {
      string s;
      getline(fileBundler,s);

      temp_fk.push_back(s);
      cv::Mat *RT = new Mat(4,3,CV_32F);	//new了                 
      for(int row = 0; row<4; row++)
      {
	getline(fileBundler,s);
	stringstream ss;
	ss<<s;
	ss>>RT->at<float>(row,0)>>RT->at<float>(row,1)>>RT->at<float>(row,2);
      }
//       vvRT[i].push_back(RT);
	temp_rt.push_back(RT);
    }
    vvfk.push_back(temp_fk);
    vvRT.push_back(temp_rt);
    
    vector<ModelPoint*> allModelPoints2ComputeDes;
    
    
    //模型点
    int p_num = 0;
    vector<ModelPoint*> temp_mp;
    for(int p = 0; p<Points; p++)
    {
      string ps;
      getline(fileBundler,ps);//坐标
      ModelPoint* modelPoint = new ModelPoint(ps,i,p_num++,mdbpORBVocabulary);
//       vvModelPoints[i].push_back(modelPoint);
      temp_mp.push_back(modelPoint);
      
//       vModel[i]->vModelPoints.push_back(modelPoint);	//给模型push入模型点
      
      getline(fileBundler,ps); //颜色
      getline(fileBundler,ps);//长长的
      int frame_num;	//有几个modelframe看到当前3D点
      stringstream longframe;
      longframe<<ps;
      longframe>>frame_num;
      
      for(int f = 0; f<frame_num; f++)
      {
	int index,key_index;
	double px,py;
	longframe>>index>>key_index>>px>>py;
	stringstream exchange;
	exchange<<vvbundlerxml[i][index];
	int ModelIdId;
	exchange>>ModelIdId;	//string2num
	mdbModelFrames[i][ModelIdId]->mvpModelPoints[key_index] = modelPoint;     //关联图像frame和其观测的物体模型点  
// 	modelPoint->modelframe_list.push_back(ModelIdId);	
	modelPoint->mObservations.insert(pair<ModelFrame*,size_t>(mdbModelFrames[i][ModelIdId],key_index));
	
      }
    }
    vvModelPoints.push_back(temp_mp);

  }
  
  for(int i = 0; i<vvModelPoints.size(); i++)
  {
//     vModel[i]->vModelPoints = vvModelPoints[i];
    vector<cv::Mat> vtempDescs;
    for(int j = 0; j<vvModelPoints[i].size();j++)
    {
//       vvModelPoints[i][j]->ComputeDistinctiveDescriptors();
      vvModelPoints[i][j]->ComputeDistinctiveDescriptorsForWord();
      vvModelPoints[i][j]->addDescriptors(vtempDescs);
    }
    mdbpORBVocabulary->transform(vtempDescs,vModel_BowVector[i],vModel_FeatureVector[i],1);
    add4Model(i,vModel_BowVector[i]);
  }

}


void ModelDataBase::add(ModelFrame * pMF)
{
//     boost::mutex::scoped_lock lock(mdbMutex);
    for(DBoW2::BowVector::const_iterator vit= pMF->mmBowVec.begin(), vend=pMF->mmBowVec.end(); vit!=vend; vit++)
        mdbInvertedFile[vit->first].push_back(pMF);
}



void ModelDataBase::AssignDescriptors(KeyFrame* pKF, int i)
{

       cv::Mat temp_descriptor;
       for(int j=0;j<(int)(pKF->segment[i].size()); j++)
       {
	 if((pKF->segment[i][j])<0)
	 {
	   cout<<"xiao yu 0->"<<(pKF->segment[i][j])<<"i "<<i<<" j "<<j<<endl;
	   continue;
	}
	 temp_descriptor.push_back(pKF->GetDescriptor(pKF->segment[i][j]));
      }
      
      vCurrentDesc = Converter::toDescriptorVector(temp_descriptor);

      bowv.clear();
      mdbpORBVocabulary->transform(vCurrentDesc,bowv,feav,1);
      vCurrentDesc.clear();

  
}



int ModelDataBase::DetectFrameCandidates(KeyFrame* pKF)
{
  //也可以用ConnectedKeyFrames获取连接的KeyFrame与模型之间的一致性

     int root_num;
     root_num = pKF->mroot_num;	//分割的patch的数量
     ModelFrame temp_patch;
     float KFall_Highest_Score = 0;

//****************
     float MINSCORE = 0.015;
     
     cout<<"经过聚类后分块的数量segment.size()： "<<pKF->mroot_num<<endl;
	 
	 //计算每一个patch的匹配结果
     
	 for (int num = 0; num<pKF->mroot_num; num++)
	 {
	     AssignDescriptors(pKF,num);	//得到这个patch对应的BoW Vector
	     
	     list<ModelFrame*> lKFsSharingWords;
	     
	    for(DBoW2::BowVector::const_iterator vit=bowv.begin(), vend=bowv.end(); vit != vend; vit++)//这个patch里面的word
	    {
	      //BowVector————map<WorldId, WordValue>
	       list<ModelFrame*> &lMFs =   mdbInvertedFile[vit->first];	//得到该wordId对应的modelFrames的list
	       
	      for(list<ModelFrame*>::iterator lit=lMFs.begin(), lend= lMFs.end(); lit!=lend; lit++)
	      {
		
		ModelFrame* pMFi=*lit;
		
		if(pMFi->mmRecQuery_Mid != pKF->mnId)	//该modelframe没有被该keyframe搜索过，这个物体第一次在该keyframe中被搜索
		{
		  pMFi->mmRecQuery_Mid = pKF->mnId;	//Mid——keyframe的Id
		  pMFi->mmRecQuery_Mid_id = num;		//Mid_id——patch的Id
		  pMFi->mmRecWords = 1;		//该ModelFrame和当前帧的当前patch，share的word数量
		  lKFsSharingWords.push_back(pMFi);		//该keyframe有share 共同word的modelframe
		}else{
		  if(pMFi->mmRecQuery_Mid_id != num)	//该物体在当前帧中被搜索过，但不是被当前patch
		  {
		    pMFi->mmRecQuery_Mid_id = num;	
		    pMFi->mmRecWords = 1;
		    lKFsSharingWords.push_back(pMFi);		//与当前帧share共同word的frame数++
		  }else{
		    pMFi->mmRecWords++;	//该frame和当前patchshare的word数++
		  }	  
		}
		
	      }
	    }
	    
	    
	      if(lKFsSharingWords.empty())	//没有与这个patch share相同word的modelframe
		  continue;
	      
	      
	    int maxCommonWords=0;
	    for(list<ModelFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
	    {
		if((*lit)->mmRecWords>maxCommonWords)
		    maxCommonWords=(*lit)->mmRecWords;
	    }
	    
	  int minCommonWords = maxCommonWords*0.85f;
	  vector<pair<float,ModelFrame*> > lScoreAndMatch;
// 	  map<float, long unsigned int > mScoreAndMatch;
	  map<float, ModelFrame* > mScoreAndMatch;
	  map<float, ModelFrame* >::iterator mScoreAndMatch_it;
     
	  int nscores=0;//share common words大于最小值的modelframe
	  float maxscores = 0.0;
	  float minscores = 100000;
	  long unsigned int bestmatch;	//得分最高的模型

	  // Compute similarity score.
	  for(list<ModelFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
	  {
	      ModelFrame* pMFi = *lit;

	      if(pMFi->mmRecWords>minCommonWords)
	      {
		  nscores++;
		  float si = mdbpORBVocabulary->score(bowv,pMFi->mmBowVec);

		  if(si > MINSCORE)	//全局最小得分**********************
		  {
		    pMFi->mmRecScore=si;

		    if(si>maxscores)
		    {
		      maxscores = si;
// 		      cout<<"si  "<<si<<endl;
		      bestmatch = pMFi->mModel_Id;	//得分最高的模型mModel_Id，表示是什么物体
		    }
	      
// 		    lScoreAndMatch.push_back(make_pair(si,pMFi));
		    mScoreAndMatch_it = mScoreAndMatch.lower_bound(si);
		    
		    mScoreAndMatch.insert(mScoreAndMatch_it,map<float, ModelFrame* >::value_type(si,pMFi));
		    
		    if(mScoreAndMatch.size()>9)		//留一个，后面！=begin()
		    {
		      mScoreAndMatch_it = mScoreAndMatch.begin();

		      mScoreAndMatch.erase(mScoreAndMatch_it);
		    }
		    
		  }

		}
    }
        if(mScoreAndMatch.empty())		//没有大于全局最低分的
	    continue;

	if(maxscores > KFall_Highest_Score)	//整张keyframe的最高分
	    KFall_Highest_Score = maxscores;

	    pKF->Match_map_result.push_back(pair<int,  map<float, ModelFrame*> >(num, mScoreAndMatch));
	}
    cout<<"满足大于全局最低分的patch的数量Match_map_result.size(): "<<pKF->Match_map_result.size()<<endl;
    
    //按照分块的得分筛一筛
    float KFallmin = 0.0*KFall_Highest_Score;
    for(int i = 0; i<(int)(pKF->Match_map_result.size()); i++)
    {
      
      if((--(pKF->Match_map_result[i].second.end()))->first>KFallmin)	//最后一个，最高得分
      {

	pKF->Refine_Match_map_result.push_back(pKF->Match_map_result[i]);
	
	map<float,ModelFrame*>::iterator it;
	it = pKF->Match_map_result[i].second.begin();
	
	for(;it!=pKF->Match_map_result[i].second.end();it++)
	{
	  pKF->hasObject.insert(it->second);	//set，判断当前keyframe有哪些物体
	}

      }

    }
  

       cout<<"按最高的patch得分筛选patch的结果Refine_Match_map_result.size(): "<<pKF->Refine_Match_map_result.size()<<endl;
    
    return pKF->Refine_Match_map_result.size();



}

int ModelDataBase::DetectModelCandidates(KeyFrame* pKF)
{
     int root_num;
     root_num = pKF->mroot_num;	//分割的patch的数量
     
     for (int num = 0; num<pKF->mroot_num; num++)	//第num个patch
     {
       
       AssignDescriptors(pKF,num);	//得到这个patch对应的BoW Vector
       
       map<int,int > SharingWordsModels;	//<ModelIdx, shareWords_Number>
       map<int,int >::iterator it_SharingWordsModels;
       
       for(DBoW2::BowVector::const_iterator vit=bowv.begin(), vend=bowv.end(); vit != vend; vit++)//这个patch里面的word
       {
	list<int> &lModellist =  mdbInvertedFile4Model[vit->first];
	for(list<int>::iterator lit = lModellist.begin(),lend = lModellist.end(); lit != lend; lit++)
	{
	  it_SharingWordsModels = SharingWordsModels.lower_bound(*lit);
	  if(it_SharingWordsModels == SharingWordsModels.end() || (SharingWordsModels.key_comp()(*lit,it_SharingWordsModels->first)))
	  {
	      SharingWordsModels.insert(it_SharingWordsModels,map<int,int>::value_type(*lit,1));
	  }
	  else
	  {
	    it_SharingWordsModels->second++;
	  }
	}
      }
      if(SharingWordsModels.empty())
	continue;
      
      //计算相似度分数
      map<float , int> mModelScore;
      map<float, int>::iterator it_mModelScore; 
      for(map<int,int>::iterator lit = SharingWordsModels.begin(),lend = SharingWordsModels.end(); lit!=lend;lit++)
      {
	float si = mdbpORBVocabulary->score(bowv,vModel_BowVector[lit->first]);
	
	it_mModelScore = mModelScore.lower_bound(si);//从小到大
	
	mModelScore.insert(it_mModelScore,map<float,int>::value_type(si,lit->first));
// 	if(it_mModelScore == mModelScore.end() || (mModelScore.key_comp()(si,it_mModelScore->first)))
// 	  mModelScore.insert(it_mModelScore,map<float,int>::value_type(si,lit->first));
	
	
      }
      
       
    }
}

int ModelDataBase::CheckCovisibility(KeyFrame* pKF)
{
  vector<KeyFrame*> vpCovisibilityKFs;
  vpCovisibilityKFs.reserve(3);
  vpCovisibilityKFs = pKF->GetBestCovisibilityKeyFrames(2);
  if(vpCovisibilityKFs.empty())	//没有Covisibility
    return 0;
  
  int nKFs = vpCovisibilityKFs.size();
  ORBmatcher orbmatcher(0.75,true);
  vector<vector<MapPoint*> > vvpMapPointMatches;
  for(int i=0; i<nKFs;i++)
  {
    int nmatches = orbmatcher.SearchByBoW(pKF,vpCovisibilityKFs[i],vvpMapPointMatches[i]);//vvpMapPointMatches是1的长度，第i个对应存储了2的mappoint
  }
  
  vector<MapPoint* > MapPointMatches = pKF->GetMapPointMatches();
  vector<vector<int > > vIsMapPoints;
  vector<int> IsMapPointCheck;	//第几个num块有对应的地图点
  
  for(int i = 0;i<(pKF->Refine_Match_map_result.size()); i++)//每一块
  {
    int num = pKF->Refine_Match_map_result[i].first;
    vector<int> temp_isMapPoints;
    for(int j = 0; j<pKF->segment[num].size(); j++)
    {
      if(MapPointMatches[pKF->segment[num][j]])
      {
	temp_isMapPoints.push_back(pKF->segment[num][j]);	//第几个特征点有对应的地图点
      }
    }
    if(!(temp_isMapPoints.empty()))	//不是空的
    {
      vIsMapPoints.push_back(temp_isMapPoints);
      IsMapPointCheck.push_back(i);
    }
  }
  if(IsMapPointCheck.empty())
    return 0;	//都没配上地图点
    
    
//   vector<int> ObjectList;
//   set<int> ObjectName;
  set<ModelFrame*>::iterator it = pKF->hasObject.begin();
  for(;it!=pKF->hasObject.end();it++)
  {
//     pKF->ObjectName.insert((int)(it->mModel_Id));
  }
  
  map<int, int> CheckedObjectList;	//确定当前帧有哪些物体
  for(set<int>::iterator j = pKF->ObjectName.begin();j!=pKF->ObjectName.end();j++)
  {
    CheckedObjectList.insert(pair<int , int>(*j,0));
  }
  
//   CheckedObjectList.resize(pKF->ObjectName.size());
  for(set<int>::iterator j = pKF->ObjectName.begin();j!=pKF->ObjectName.end();j++)
  {
      for(int i=0;i<nKFs;i++)
    {
     set<int>::iterator itt=  vpCovisibilityKFs[i]->ObjectName.find(*j);
     if(itt!=vpCovisibilityKFs[i]->ObjectName.end())
     {
       map<int ,int>::iterator mmm = CheckedObjectList.find(*itt);
       mmm->second++;
    }
    }
  }

}

void ModelDataBase::matches2points(const vector<cv::KeyPoint>& train, const vector<cv::KeyPoint>& query,
        const std::vector<cv::DMatch>& matches, std::vector<cv::Point2f>& pts_train,
        std::vector<cv::Point2f>& pts_query)
{
	pts_train.clear();
        pts_query.clear();
        pts_train.reserve(matches.size());
        pts_query.reserve(matches.size());

        size_t i = 0;

        for (; i < matches.size(); i++)
        {

            const DMatch & dmatch = matches[i];

            pts_query.push_back(query[dmatch.queryIdx].pt);
            pts_train.push_back(train[dmatch.trainIdx].pt);

        }
}

void ModelDataBase::Mat2KeyPoints(Mat mKeyPoints, vector< KeyPoint >& vKeyPoints)
{
  for(int i = 0; i < mKeyPoints.rows;  i++)
  {
    KeyPoint keyp;
    keyp.pt.x = mKeyPoints.at<float>(i,1) ;
    keyp.pt.y = mKeyPoints.at<float>(i,0) ;
    keyp.size = mKeyPoints.at<float>(i,2);
    keyp.angle = mKeyPoints.at<float>(i,3);
    vKeyPoints.push_back(keyp);
  }

}

 
void ModelDataBase::EssentialFromFundamental(const Mat3 &F,
                              const Mat3 &K1,
                              const Mat3 &K2,
                              Mat3 *E) {
  *E = K2.transpose() * F * K1;
}

void ModelDataBase::MotionFromEssential(const Mat3 &E,
                         std::vector<Mat3> *Rs,
                         std::vector<Vec3> *ts) {
  Eigen::	JacobiSVD<Mat3> USV(E, Eigen::ComputeFullU|Eigen::ComputeFullV);
  Mat3 U =  USV.matrixU();
  // Vec3 d =  USV.singularValues();
  Mat3 Vt = USV.matrixV().transpose();

  // Last column of U is undetermined since d = (a a 0).
  if (U.determinant() < 0) {
    U.col(2) *= -1;
  }
  // Last row of Vt is undetermined since d = (a a 0).
  if (Vt.determinant() < 0) {
    Vt.row(2) *= -1;
  }

  Mat3 W;
  W << 0, -1,  0,
       1,  0,  0,
       0,  0,  1;

  Mat3 U_W_Vt = U * W * Vt;
  Mat3 U_Wt_Vt = U * W.transpose() * Vt;

  Rs->resize(4);
  ts->resize(4);
  (*Rs)[0] = U_W_Vt;  (*ts)[0] =  U.col(2);
  (*Rs)[1] = U_W_Vt;  (*ts)[1] = -U.col(2);
  (*Rs)[2] = U_Wt_Vt; (*ts)[2] =  U.col(2);
  (*Rs)[3] = U_Wt_Vt; (*ts)[3] = -U.col(2);
}
  template <typename TMat, typename TVec>
  double ModelDataBase::Nullspace(TMat *A, TVec *nullspace) {
    if (A->rows() >= A->cols()) {
      Eigen::JacobiSVD<TMat> svd(*A, Eigen::ComputeFullV);
      (*nullspace) = svd.matrixV().col(A->cols()-1);
      return svd.singularValues()(A->cols()-1);
    }
        // Extend A with rows of zeros to make it square. It's a hack, but is
    // necessary until Eigen supports SVD with more columns than rows.
    TMat A_extended(A->cols(), A->cols());
    A_extended.block(A->rows(), 0, A->cols() - A->rows(), A->cols()).setZero();
    A_extended.block(0,0, A->rows(), A->cols()) = (*A);
    return Nullspace(&A_extended, nullspace);
  }

  void ModelDataBase::HomogeneousToEuclidean(const Vec4 &H, Vec3 *X) {
  double w = H(3);
  *X << H(0) / w, H(1) / w, H(2) / w;
}



  
// HZ 12.2 pag.312
void ModelDataBase::TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
                    const Mat34 &P2, const Vec2 &x2,
                    Vec4 *X_homogeneous) {
  Mat4 design;
  for (int i = 0; i < 4; ++i) {
    design(0,i) = x1[0] * P1(2,i) - P1(0,i);
    design(1,i) = x1[1] * P1(2,i) - P1(1,i);
    design(2,i) = x2[0] * P2(2,i) - P2(0,i);
    design(3,i) = x2[1] * P2(2,i) - P2(1,i);
  }
  Nullspace(&design, X_homogeneous);
}


void ModelDataBase::TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
                    const Mat34 &P2, const Vec2 &x2,
                    Vec3 *X_euclidean) {
  Vec4 X_homogeneous;
  TriangulateDLT(P1, x1, P2, x2, &X_homogeneous);
  HomogeneousToEuclidean(X_homogeneous, X_euclidean);
}

Mat34  ModelDataBase::HStack ( const Mat3 & lhs, const Vec3 & rhs) {
      Mat34 res;
      res.resize(lhs.rows(), lhs.cols()+rhs.cols());
      res << lhs, rhs;
      return res;
  }

void ModelDataBase::P_From_KRt(
  const Mat3 &K,  const Mat3 &R,  const Vec3 &t, Mat34 *P) {
  *P = K * HStack(R,t);
}


double ModelDataBase::Depth(const Mat3 &R, const Vec3 &t, const Vec3 &X) {
  return (R*X)[2] + t[2];
}


Vec2 ModelDataBase::Project(const Mat34 &P, const Vec3 &X) {
  Vec4 HX;
  HX << X, 1.0;
  Vec3 hx = P * HX;
  return hx.head<2>() / hx(2);
}

void ModelDataBase::Project(const Mat34 &P, const Mat4X &X, Mat2X *x) {
  x->resize(2, X.cols());
  for (Mat4X::Index c = 0; c < X.cols(); ++c) {
    Vec3 hx = P * X.col(c);
    x->col(c) = hx.head<2>() / hx(2);
  }
}
void ModelDataBase::Project(const Mat34 &P, const Mat3X &X, Mat2X *x) {
  x->resize(2, X.cols());
  for (size_t c = 0; c < static_cast<size_t>(X.cols()); ++c) {
    x->col(c) = Project(P, Vec3(X.col(c)));
  }
}
    

int ModelDataBase::ChectFundamental(KeyFrame* pKF)
{ 
  int count = 0;//符合Fundamental检验的patches个数
  vector<pair<int,  map<float, ModelFrame*> > > temp_Refine_Match_map_result;
//   BFMatcher matcher;	//匹配算法
  for(int num = 0; num<pKF->Refine_Match_map_result.size(); num++)
  {
    map<float, ModelFrame*>::iterator it_Refine_Match_map_result = pKF->Refine_Match_map_result[num].second.begin();	//当前patch对应的model的map的迭代器
    int patchNum = pKF->Refine_Match_map_result[num].first;	//当前patch的num
    //提取当前patch的特征点和描述子
    vector<KeyPoint > srcKeyPoints;
    Mat srcDescriptor;
    for(int j = 0; j<pKF->segment[patchNum].size(); j++)
    {
	    srcKeyPoints.push_back(pKF->GetKeyPointUn(pKF->segment[patchNum][j]));
	    srcDescriptor.push_back(pKF->GetDescriptor(pKF->segment[patchNum][j]));
    }
    //遍历当前patch对应的map，搜索最优匹配结果
    for(;it_Refine_Match_map_result!=pKF->Refine_Match_map_result[num].second.end(); it_Refine_Match_map_result++)
    {
      //～计算匹配点
      ///提取特征点，描述子
	  ModelFrame* cModelFrame = it_Refine_Match_map_result->second;
	  Mat2KeyPoints(cModelFrame->mmKeyPoints,cModelFrame->mvKeyPoins);
      ///计算匹配matches
	  vector<DMatch> matches;
	  matcher.match(cModelFrame->mmDescriptors,srcDescriptor,matches);	//注意模型的descriptor在前

	  
      //～计算基础矩阵fundamental fundaMatrix
      ///将keypoint转为points
	  vector< Point2f > srcPoints;
	  vector< Point2f > objPoints;
	  matches2points(srcKeyPoints,cModelFrame->mvKeyPoins,matches,srcPoints,objPoints);
      ///将points转为Mat2
	  int size = srcPoints.size();
	  Mat2X x1,x2;
	  x1.resize(2,size);
	  x2.resize(2,size);
	  for(int i = 0; i < srcPoints.size(); i++)
	  {
	    Vec2 pt1,pt2;
	    pt1[0] = objPoints[i].x;
	    pt1[1] = objPoints[i].y;
	    pt2[0] = srcPoints[i].x;
	    pt2[1] = srcPoints[i].y;
	    x1.col(i) = pt1;
	    x2.col(i) = pt2;
	  }
      ///计算fundamental matrix
	  vector<unsigned char> ma;
	  Mat fundamental = findFundamentalMat(objPoints,srcPoints,CV_FM_RANSAC,1,0.99,ma);
	  if(countNonZero(ma)<5)//////5？
	    break;
	  else{
		F<< fundamental.at<double>(0,0),fundamental.at<double>(0,1),fundamental.at<double>(0,2),
		       fundamental.at<double>(1,0),fundamental.at<double>(1,1),fundamental.at<double>(1,2),
		       fundamental.at<double>(2,0),fundamental.at<double>(2,1),fundamental.at<double>(2,2);
	  ///提取inlier，index
		vector<size_t> inliers;
		for(int q = 0; q<ma.size(); q++)
		{
		  if(ma[q])
		  {
		    inliers.push_back(q);
		  }
	      }
         
	  //~分解Essential Matrix，得到R，t
	  ///计算Essential Matrix
	    Mat3 Essential;
	    EssentialFromFundamental(F,K,K,&Essential);
	  ///分解Essential Matrix，会得到四组解
	    vector<Mat3> R;	//四个解
	    vector<Vec3> t;	//四个解
	    MotionFromEssential(Essential,&R,&t);
	///筛选得到唯一解——点在图像前，且在的点最多——对应的解
	    Mat34 P1, P2;		//K*[R|t]
	    Mat3 R1 = Mat3::Identity();
	    Vec3 t1 = Vec3::Zero();
	    std::vector<size_t> f(4, 0);//四个元素，的count，分别代表了1,2,3,4
	    P_From_KRt(K, R1, t1, &P1);    //模型匹配图像的P
	///筛一遍四个解
	   for (unsigned int i = 0; i < 4; ++i)
	  {
	      const Mat3 &R2 = R[i];
	      const Vec3 &t2 = t[i];
	      P_From_KRt(K, R2, t2, &P2);
	      Vec3 X;

	      for (size_t k = 0; k < inliers.size(); ++k)
	      {
		  const Vec2 & x1_ = x1.col(inliers[k]),	//内点index
				      &x2_ = x2.col(inliers[k]);
		TriangulateDLT(P1, x1_, P2, x2_, &X);
	    // Test if point is front to the two cameras.
		if (Depth(R1, t1, X) > 0 && Depth(R2, t2, X) > 0)
		{
		  ++f[i];
		}
	      }
	  }
	/// Check the solution:
	const std::vector<size_t>::iterator iter = max_element(f.begin(), f.end());
	if (*iter == 0)
	{
	  ///计算不出R，t
	  break;
	}
	const size_t index = std::distance(f.begin(), iter);//最优解的index
	
	//~ 得到模型图像对应的模型3D点，然后投影到当前帧，计算匹配的数量
	vector<ModelPoint*> mvpModelPoints = cModelFrame->mvpModelPoints;
	
	
    }//到else，计算fundamental matrix的点大于5个

         
    }
    
    
    
    
    

    }

    
  
  pKF->Refine_Match_map_result.clear();
  pKF->Refine_Match_map_result = temp_Refine_Match_map_result;

}

void ModelDataBase::KeyPoint2Points(vector< KeyPoint > keypoints, vector< Point2f >& points)
{
  for(int i = 0; i<keypoints.size(); i++)
  {
    Point2f point;
    point.x = keypoints[i].pt.x;
    point.y = keypoints[i].pt.y;
    points.push_back(point);
  }
}

bool ModelDataBase::Matches2Points2Points3(vector< KeyPoint > srcKeyPoints,std::vector< ModelPoint* > objModelPoints, vector< DMatch > matches, vector< Point2f >& srcPoints, 
					   vector< Point3f >& objModelPoint3f,  vector<DMatch> &afterModelPointMatches, vector<ModelPoint* >& modelPointsAfterMatches
)
{
  int cnt = 0;
  for(int i = 0; i<matches.size(); i++)
  {
    DMatch imatch = matches[i];
    
    if(objModelPoints[imatch.trainIdx]!=NULL)
    {
      
	srcPoints.push_back(Point2f(srcKeyPoints[imatch.queryIdx].pt.x,srcKeyPoints[imatch.queryIdx].pt.y));
	objModelPoint3f.push_back(objModelPoints[imatch.trainIdx]->mWorldPos);
	afterModelPointMatches.push_back(imatch);
	modelPointsAfterMatches.push_back(objModelPoints[imatch.trainIdx]);
	cnt++;
    }
    
  }
  if(cnt<3)
    return false;
  else
  {
    cout<<"该匹配上的模型图片包含的物体三维点："<<cnt<<endl;
    return true;
  }
}

void ModelDataBase::ModelPoints23fPoints(std::vector< ModelPoint* > ModelPoints, vector< Point3f >& CPoints3f)
{
  for(size_t t = 0; t<ModelPoints.size(); t++)
  {
    if(ModelPoints[t] != NULL)
    {
      CPoints3f.push_back(ModelPoints[t]->mWorldPos);
    }
  }

}

//将3D点坐标转化为Mat3X
void ModelDataBase::Points3f2Mat(vector< Point3f > CPoints3f, Mat4X& Mat4XPoints)
{
  
  Mat4XPoints.resize(4,CPoints3f.size());
  for(size_t t = 0; t<CPoints3f.size(); t++)
  {
    Vec4 tempV;
    tempV[0] = CPoints3f[t].x;
    tempV[1] = CPoints3f[t].y;
    tempV[2] = CPoints3f[t].z;
    tempV[3] = 1;
    Mat4XPoints.col(t) = tempV;
  }

}


void ModelDataBase::Mat2X2Point2f(Mat2X MatPoints, vector< KeyPoint >& vKeyPoints)
{
 int cols =  MatPoints.cols();
 for(int  i = 0;i<cols; i++)
 {
   KeyPoint kp;
   Vec2 temp = MatPoints.col(i);

   kp.pt.x = temp[0];
   kp.pt.y = temp[1];
   vKeyPoints.push_back(kp);
}

}

Mat ModelDataBase::GetModelPointsDescriptors(vector< ModelPoint* > mp)
{
  Mat desc;
  for(int i = 0; i<mp.size(); i++)
  {
    if(mp[i] != NULL)
    {
      desc.push_back(mp[i]->mDescriptors);
    }
  }

  return desc;
}

Mat ModelDataBase::GetMapPointsDescriptors(std::vector< MapPoint* > mp)
{
  Mat desc;
  for(int i = 0 ; i<mp.size() ; i++)
  {
    if(mp[i] != NULL)
    {
      desc.push_back(mp[i]->GetDescriptor());
    }
  }
  return desc;

}

Mat ModelDataBase::GetModelPointsDescriptorsFromCurrentFrame(ModelFrame* mcurrentModelFrame)
{
  Mat desc;
  vector<ModelPoint* > mModelPoints = mcurrentModelFrame->mvpModelPoints;
  Mat descriptor = mcurrentModelFrame->mmDescriptors; 
  
  for(size_t i = 0; i<mModelPoints.size(); i++)
  {
    if(mModelPoints[i] != NULL)
    {
      desc.push_back(descriptor.row(i));
    }
  }
  
  return desc;
  
}


vector< KeyPoint > ModelDataBase::GetKeyPointsByModelPoints(ModelFrame* mf)
{
  vector<KeyPoint > keypoints;
  vector<ModelPoint* > mpModelPoints= mf->mvpModelPoints;
  vector<KeyPoint> mfKeyPoints = mf->mvKeyPoins;
  
  
  for(int  i = 0; i<(mpModelPoints.size()); i++)
  {
    if(mpModelPoints[i] != NULL)
    {
      keypoints.push_back(mfKeyPoints[i]);
    }
  
  }
  return keypoints;

}

vector< Point2f > ModelDataBase::GetKeyPointsByMatches(vector< KeyPoint > mfKeyPoints, vector< DMatch > matches)
{
  vector<Point2f> mfvkeypoints;
  for(int i = 0; i<matches.size(); i++)
  {
    KeyPoint kp = mfKeyPoints[matches[i].trainIdx];
    mfvkeypoints.push_back(Point2f(kp.pt.x,kp.pt.y));
  }

  return  mfvkeypoints;
}

Mat3 ModelDataBase::Mat2Mat3(Mat Rodri)
{
  Mat3 R;
  
  	    R<<Rodri.at<double>(0,0),Rodri.at<double>(0,1),Rodri.at<double>(0,2)
		  ,Rodri.at<double>(1,0),Rodri.at<double>(1,1),Rodri.at<double>(1,2)
		  ,Rodri.at<double>(2,0),Rodri.at<double>(2,1),Rodri.at<double>(2,2);
		  
  return R;

}

Vec3 ModelDataBase::Mat2Vec3(Mat tvec)
{
  Vec3 t;
  
  	    t[0] = tvec.at<double>(0,0);
	    t[1] = tvec.at<double>(1,0);
	    t[2] = tvec.at<double>(2,0);

	    return t;
}

vector< DMatch > ModelDataBase::ComputeReprojectInliers(Mat rvec, Mat tvec, Mat4X AllModelPointsMat, KeyFrame* pKF,Mat mpModelDesc )
{
  
  Mat Rodri;
  Rodrigues(rvec,Rodri);
  
  
  Mat3 R;
  Vec3 t;
	    
  R = Mat2Mat3(Rodri);
  t = Mat2Vec3(tvec);

  Mat34 P;
  P_From_KRt(K,R,t,&P);
  
  Mat2X Project2P;	//投影到当前帧上的物体2D点
  Project(P,AllModelPointsMat,&Project2P);
  
   vector<KeyPoint> ProjectKeyPoints;	//用地图点的坐标，转换成keypoints，与地图点的vector对齐的
   Mat2X2Point2f(Project2P,ProjectKeyPoints);
   
   Mat mask = windowedMatchingMask(ProjectKeyPoints,pKF->GetKeyPoints(), 7, 7);
   
   vector<DMatch> mmatches;
   matcher.match(mpModelDesc,pKF->GetDescriptors(),mmatches,mask);	//all modelpoints——all KeyPoints
   
   vector<DMatch> good_matches;
    int MIN_DISTANCE = 70;
    for(int i = 0;i<mmatches.size(); i++)
    {
	DMatch iimatch = mmatches[i];
	int dist = ORBmatcher::DescriptorDistance(mpModelDesc.row(iimatch.queryIdx),pKF->GetDescriptors().row(iimatch.trainIdx));

	if(dist<MIN_DISTANCE)
	  good_matches.push_back(iimatch);
    }
   
  return good_matches;
}

std::vector< ModelPoint* > ModelDataBase::GetModelPointsByCheckNULL(vector<ModelPoint* > pModelPoints)
{
  vector<ModelPoint* > mModelPoints;
  for(size_t i  = 0; i<pModelPoints.size(); i++)
  {
    if(pModelPoints[i] != NULL)
    {
      mModelPoints.push_back(pModelPoints[i]);
    }
  }
  return mModelPoints;

}

void ModelDataBase::TranslateO2C(const Mat3& R, const Vec3 &t,  const Mat4X& X, vector< Point3f >& newX)
{
  Mat34 RT = HStack(R,t);
  for(Mat4X::Index c=0; c<X.cols(); ++c)
  {
    Vec3 hx = RT * X.col(c);
    newX.push_back(Point3f(hx(0),hx(1),hx(2)));
  }

}

void ModelDataBase::ModelPoints2Mat4X(std::vector< ModelPoint* > ModelPoints, Mat4X& Mat4XModelPoints)
{
  vector<Point3f> mModelPoints3f;
  ModelPoints23fPoints(ModelPoints,mModelPoints3f);
  Points3f2Mat(mModelPoints3f,Mat4XModelPoints);

}


int ModelDataBase::CheckPnP(KeyFrame* pKF)
{
  
  ModelPointsToBeInserted.clear();
  
  int count = 0;//符合PnP检验的patches个数
  vector<pair<int,  map<float, ModelFrame*> > > temp_Refine_Match_map_result;	//最后替代pKF里的Refine_Match_map_result
//   BFMatcher matcher;	//匹配算法
  
  OrbDescriptorExtractor orbextractor;
  
///////////////patch 导向
  for(int num = 0; num<pKF->Refine_Match_map_result.size(); num++)
  {

    int patchNum = pKF->Refine_Match_map_result[num].first;	//当前patch的num
    
    
    //提取当前patch的特征点和描述子
    vector<KeyPoint > srcKeyPoints;
    Mat srcDescriptor;
    
    
    for(int j = 0; j<pKF->segment[patchNum].size(); j++)
    {
	    srcKeyPoints.push_back(pKF->GetKeyPointUn(pKF->segment[patchNum][j]));
	    srcDescriptor.push_back(pKF->GetDescriptor(pKF->segment[patchNum][j]));
    }
    
    
    ///keypoint 转为points
    vector<Point2f> srcPoints;
    KeyPoint2Points(srcKeyPoints,srcPoints);
    
    int inlier_count = 0;
    map<float, ModelFrame*>::iterator it_temp_Refine_Match_map_result; 

    vector<DMatch> showMatches;
    vector<DMatch> showProjectMatches;
    vector<DMatch> showAllMatches;
    vector<DMatch> showSecondProjectMatches;
    
    Mat3 O2CR;
    Vec3  O2Ct;

    
    //遍历当前patch对应的map，搜索最优匹配结果
    map<float, ModelFrame*>::iterator it_Refine_Match_map_result = pKF->Refine_Match_map_result[num].second.begin();	//当前patch对应的model的map的迭代器
    for(;it_Refine_Match_map_result!=pKF->Refine_Match_map_result[num].second.end(); it_Refine_Match_map_result++)
    {
      //～计算PnP匹配
      ///提取特征点，描述子
	  ModelFrame* cModelFrame = it_Refine_Match_map_result->second;
	  Mat2KeyPoints(cModelFrame->mmKeyPoints,cModelFrame->mvKeyPoins);
	  
       ///～计算匹配点
      ///计算匹配matches
	  vector<DMatch> firstmatches;
	  vector<DMatch> matches;
	  vector<DMatch> afterModelPointMatches;////是srcKeyPoints和cModelFrame->mvKeyPoins的
	  
	  //patch的descriptor配对模型frame的整体descriptor
	  matcher.match(srcDescriptor,cModelFrame->mmDescriptors,firstmatches);	//注意当前帧的descriptor在前，后面query和train要分开

	  double max_dis = 0, min_dis = 9000;
	 
	  for(size_t i = 0; i < firstmatches.size(); i++)
	  {
	    if(firstmatches[i].distance<min_dis)
	      min_dis = firstmatches[i].distance;
	    else
	    {
	      if(firstmatches[i].distance>max_dis)
		max_dis = firstmatches[i].distance;
	    }
	  }
	  
	  for(size_t i = 0; i<firstmatches.size(); i++)
	  {
	    DMatch imatch = firstmatches[i];
	    if(imatch.distance < 3*min_dis)
	      matches.push_back(imatch);
	  }
	  
	  
     ///从模型图片提取对应模型3D点
	  vector<ModelPoint*> mvpModelPoints;	//当前modelframe对应的所有3D模型点
	  mvpModelPoints = cModelFrame->mvpModelPoints;
	  
	  vector<Point3f> cModelPoint3f;	//存储模型中当前图像中对应的3D地图点
	  vector< Point2f > srcPoints;		//存储对准后的src的2D点
	  
	  vector<KeyPoint> srckeypoint;
// 	  vector<Point2f> modelPoints;
	  
	  vector<ModelPoint* > modelPointsAfterMatches;
	  
	  //根据matches的结果和特征点-地图点关联性做对准，并且检验匹配上的图片是否包含足够的模型3D点
	   bool enoughModelPoints = Matches2Points2Points3(srcKeyPoints,mvpModelPoints,matches,   srcPoints,cModelPoint3f,afterModelPointMatches,modelPointsAfterMatches);
// 	   modelPoints  = GetKeyPointsByMatches(cModelFrame->mvKeyPoins,afterModelPointMatches);
	   
	  if(!enoughModelPoints)
	    break;
	  

	  Mat rvec;
	  Mat tvec;
	  vector<unsigned char> ma;

	  solvePnPRansac(cModelPoint3f,srcPoints,MatK,MdistCoeffs,rvec,tvec,false,120,3,100,ma,CV_ITERATIVE);
	  
	 

	  int icount = countNonZero(ma);
	  cout<<"solvePnPRansac后的内点数量："<<icount<<endl;
	  
	  if(icount<3)
	    break;  
	  else
	  {
	    
	   vector<DMatch> afterPnPMatches;	//PnP筛选后的内点
	   for(int i = 0; i<cModelPoint3f.size(); i++)
	   {
	     if((ma[i]))
	     {
	       afterPnPMatches.push_back(afterModelPointMatches[i]);
	    }
	  }
	     
// 	   Mat mpModelDesc = GetModelPointsDescriptors(mvpModelPoints);	//all modelpoints, 不是空指针的
	  Mat mpModelDesc = GetModelPointsDescriptorsFromCurrentFrame(cModelFrame);
	    

    
	    ///将模型点投影到当前帧，计算match上的点		不考虑遮挡！！！
	    vector<Point3f> AllModelPoints;	//当前模型图片对应的3D点的Point3f表示
	    Mat4X AllModelPointsMat;
	    
	    ModelPoints23fPoints(mvpModelPoints,AllModelPoints);	//得到所有当前模型图片对应的非空模型点
	    cout<<"ModelPoints："<<AllModelPoints.size()<<"       KeyPoints："<<cModelFrame->mvKeyPoins.size()<<endl;
	    Points3f2Mat(AllModelPoints,AllModelPointsMat);

	    //计算重投影的内点
	    vector<DMatch> good_matches;
	    good_matches = ComputeReprojectInliers(rvec, tvec, AllModelPointsMat,pKF,mpModelDesc);      
	    cout<<"7*7区域内匹配点数量："<<good_matches.size()<<endl;
// 	      icount = good_matches.size();
// 	      cout<<"投影点数量："<<des.rows<<"   关键帧点数量："<<pKF->GetDescriptors().rows<<"    匹配点数量："<<mmatches.size()<<endl;
 
	      
	      if(good_matches.size()<3)
		break;
	      else{
		vector<Point3f> SecondModelPoints;
		vector<Point2f> SecondsrcKeyPoints;
		
		vector<KeyPoint> pKFKeyPoints = pKF->GetKeyPoints();
		
		for(size_t i = 0; i<good_matches.size(); i++)
		{
		  Point3f tempPoints = AllModelPoints[good_matches[i].queryIdx];
		  KeyPoint tempPoints2 = pKFKeyPoints[good_matches[i].trainIdx];
		  
		  SecondModelPoints.push_back(Point3f(tempPoints.x,tempPoints.y,tempPoints.z));
		  SecondsrcKeyPoints.push_back(Point2f(tempPoints2.pt.x, tempPoints2.pt.y));	  
		}
		
		Mat rvec2,tvec2;
		ma.clear();
		solvePnPRansac(SecondModelPoints,SecondsrcKeyPoints,MatK,MdistCoeffs,rvec2,tvec2,false,100,8,100,ma,CV_ITERATIVE);
		
		cout<<"Input number: SecondModelPoints："<<SecondModelPoints.size()<<"  内点数："<<countNonZero(ma)<<endl<<endl;
// 		icount = countNonZero(ma);
		vector<DMatch> SecondGoodMatches;
		SecondGoodMatches = ComputeReprojectInliers(rvec2, tvec2,AllModelPointsMat, pKF, mpModelDesc);
		cout<<"第二次重投影的内点数"<<SecondGoodMatches.size()<<endl<<endl;
// 		icount = (SecondGoodMatches.size()-good_matches.size());//~~~~~~~~~~~~
		icount = SecondGoodMatches.size();

	      
    
	    
	    
	    if(icount>inlier_count)//找候选中的PnP计算出的内点最多的frame
	    {
	      inlier_count = icount;
	      it_temp_Refine_Match_map_result = it_Refine_Match_map_result;
	      showMatches.clear();
	      showMatches=afterPnPMatches;
	      showProjectMatches.clear();
	      showProjectMatches = good_matches;
	      showAllMatches=matches;
	      showSecondProjectMatches = SecondGoodMatches;
	      
	      Mat Rod;
	      Rodrigues(rvec2,Rod);
	      O2CR = Mat2Mat3(Rod);
	      O2Ct = Mat2Vec3(tvec2);
// 	      showHomography = afterModelPointMatches;
	      
// 	      homomodel = modelPoints;
// 	      homosrc = srcPoints;
	    }	    
	      }//第一次重投影后的else
	  }//solvePnPRansac后的else

    }//到map的迭代搜索
    if(inlier_count != 0)
    {
       map<float, ModelFrame*> tempmap;
       tempmap.insert(pair<float, ModelFrame*>(it_temp_Refine_Match_map_result->first,it_temp_Refine_Match_map_result->second));
       
       vector<KeyPoint> ModelKeyPoint = GetKeyPointsByModelPoints(it_temp_Refine_Match_map_result->second);	//得到不是3D点不是空指针的特征点
       temp_Refine_Match_map_result.push_back(pair<int,  map<float, ModelFrame*> >(num,tempmap));
       count++;
       
       Mat out,outProject,origin, outSecondProject;
       drawMatches(pKF->GetImage(),srcKeyPoints,tempmap.begin()->second->image,tempmap.begin()->second->mvKeyPoins,showMatches,out);
       drawMatches(tempmap.begin()->second->image,ModelKeyPoint,pKF->GetImage(),pKF->GetKeyPoints(), showProjectMatches, outProject);
       drawMatches(pKF->GetImage(),srcKeyPoints,tempmap.begin()->second->image,tempmap.begin()->second->mvKeyPoins,showAllMatches,origin);
       drawMatches(tempmap.begin()->second->image,ModelKeyPoint,pKF->GetImage(),pKF->GetKeyPoints(), showSecondProjectMatches, outSecondProject);       
       
       stringstream name;
	name<<count;
	
	stringstream number;
	number<<showSecondProjectMatches.size();
	
	imshow(name.str(),out);
	imshow(name.str()+"Project",outProject);
	imshow(name.str()+"Origin",origin);
	imshow(name.str()+"SecondProject  ", outSecondProject);
	
	if(inlier_count>30)
	{
	  vector<ModelPoint*> addModelPoints = GetModelPointsByCheckNULL(tempmap.begin()->second->mvpModelPoints);
	  Mat4X addMatModelPoints;
	  ModelPoints2Mat4X(addModelPoints,addMatModelPoints);
	  vector<Point3f> PointsToAdd;
	  TranslateO2C(O2CR,O2Ct,addMatModelPoints,PointsToAdd);

// 	  LocalMapper->InsertModelPoints(PointsToAdd);
	  ModelPointsToBeInserted.push_back(PointsToAdd);
	  cout<<"——————检测到物体"<<tempmap.begin()->second->mModel_Id<<"~~~~~~~"<<endl<<endl<<endl;
	  imshow("Detected Object",outSecondProject);
// 	  waitKey(0);
	}

	
        waitKey(5); 
       
    }

}
  pKF->Refine_Match_map_result.clear();
  pKF->Refine_Match_map_result = temp_Refine_Match_map_result;
  return count;

}

}//namespace ORB_SLAM