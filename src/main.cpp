#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include "include/ORBVocabulary.h"
#include "include/ModelDataBase.h"
// #include "include/ModelFrame.h"
#include "include/ORBextractor.h"
#include "include/ObjectRecognition.h"
#include "include/KeyFrame.h"

#include "random.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    int nFeatures = 1000;
    float fScaleFactor = 1.2;
    int nLevels = 8;
    int fastTh = 20;    
    int Score = 1;
    
    ORB_SLAM::ORBextractor* mORBextractor;
    mORBextractor = new ORB_SLAM::ORBextractor(nFeatures,fScaleFactor,nLevels,Score,fastTh);
    
    vector<KeyPoint> keypoints;
    Mat descriptors;

  
  Mat in = imread("11.jpg",CV_8UC1 );
  
  (*mORBextractor)(in,Mat(),keypoints,descriptors);
    
    
    Mat outt;
    vector<Scalar> color;
//     srand(255);
    for(int i = 0; i<7; i++)
    {
      cout<<rand()<<endl;
      
    }
    
    drawKeypoints(in,keypoints,outt);
    imshow("out",outt);
    waitKey();
    
  string strVocFile_Model = "/home/dxq/TestSingle/Data/small_voc.yml.gz"; 
  
  ORB_SLAM::ORBVocabulary Vocabulary_Model(strVocFile_Model);
  cout<<Vocabulary_Model<<endl;
  
  ORB_SLAM::ModelDataBase ModelDatabase(Vocabulary_Model);

    ModelDatabase.Construct();	//读取图像信息，构造模型库
    ModelDatabase.MatchPointFrame();
    
//     ORB_SLAM::KeyFrame srcKF(in,keypoints,descriptors);
    ORB_SLAM::KeyFrame *srcKF = new ORB_SLAM::KeyFrame(in,keypoints, descriptors);
    cout<<endl<<srcKF->GetImage().rows<<"  "<<srcKF->GetKeyPoints().size()<<"   "<<srcKF->GetDescriptors().rows<<endl;
//     imshow("in",srcKF->GetImage());
    
    ORB_SLAM::ObjectRecognition ObjectRecognizer(&ModelDatabase,&Vocabulary_Model);	//&&&&&   MARK  !!!   这里跟ORB_SLAM不一样
    int segnum = ObjectRecognizer.DetectObjects(srcKF);
    cout<<segnum<<endl;
    
    
    vector<Scalar> vScalar;
    
    Mat out;
    for(int i = 0; i<segnum; i++)
    {
      vector<cv::KeyPoint> keypoints;
      for(size_t j = 0; j<srcKF->segment[i].size(); j++)
      {
	keypoints.push_back(srcKF->GetKeyPointUn(srcKF->segment[i][j]));
      }
      
//       drawKeypoints(srcKF->GetImage(),keypoints,out);
    }
    
    
    
    
    
  
  waitKey();
  return 0;
}