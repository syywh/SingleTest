#ifndef MODELDATABASE_H
#define MODELDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "ORBVocabulary.h"
#include <opencv2/core/core.hpp>
#include<iostream>
#include<fstream>
#include "ModelFrame.h"
#include "KeyFrame.h"


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "ModelPoint.h"
#include "Model.h"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SparseCore>
#include <Eigen/SVD>
#include <Eigen/StdVector>

  typedef Eigen::Matrix<double, 3, 3> Mat3;
  typedef Eigen::Vector3d Vec3;
  typedef Eigen::Matrix<double, 3, 4> Mat34;
  typedef Eigen::Vector2d Vec2;
  typedef Eigen::Vector4d Vec4;
  typedef Eigen::Matrix<double, 4, 4> Mat4;
  typedef Eigen::Matrix<double, 2, Eigen::Dynamic> Mat2X;
  typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Mat3X;
  typedef Eigen::Matrix<double, 4, Eigen::Dynamic> Mat4X;


namespace ORB_SLAM
{
  class Frame;
  class ModelFrame;
  class KeyFrame;
  class LocalMapping;
  class ModelDataBase{
  public:
    ModelDataBase(const ORBVocabulary &voc);
    void Construct();
    void TestWordDistribution();
    void add(ModelFrame * pMF);
    void add4Model(int i, DBoW2::BowVector bv);
    
    // Loop Detection
    int DetectFrameCandidates(KeyFrame* pKF);
    int DetectModelCandidates(KeyFrame* pKF);
    void MatchPointFrame();
    int CheckCovisibility(KeyFrame* pKF);
    int ChectFundamental(KeyFrame* pKF);
    int CheckPnP(KeyFrame* pKF);
    
    void SetLocalMapper(LocalMapping* LocalMapper);
    
    vector<cv::KeyPoint > GetKeyPointsByModelPoints(ModelFrame* mf);
    vector<cv::Point2f > GetKeyPointsByMatches(vector<cv::KeyPoint> mfKeyPoints, vector<cv::DMatch> matches);
    
    LocalMapping* mrLocalMapping;
    
    
    //segment current KeyFrame
    long unsigned int nmId;
    int mroot_num;
    vector<int > all;
    vector<vector<int > > segment;
    vector<DBoW2::BowVector > segment_Bow; 
    vector<DBoW2::FeatureVector> segment_FeaV;
    DBoW2::FeatureVector feav;
    DBoW2::BowVector bowv;
    
    cv::BFMatcher matcher;	//匹配算法
//     cv::FlannBasedMatcher matcher;
    
//     vector<pair<long unsigned int, list<pair<float,ModelFrame*> > > > patch_Model;
//     vector<pair<cv::KeyPoint,long unsigned int > > patch_bestMatch;	//可能可以换成指针
    
    cv::Mat GetModelPointsDescriptors(vector<ModelPoint* > mp);
    cv::Mat GetMapPointsDescriptors(vector<MapPoint* > mp);
    cv::Mat GetModelPointsDescriptorsFromCurrentFrame(ModelFrame* mcurrentModelFrame);
    vector<ModelPoint*> GetModelPointsByCheckNULL(vector<ModelPoint*> pModelPoints);
    
   vector< vector<cv::Point3f> > ModelPointsToBeInserted;
    

  
  Mat3 K,F;
  cv::Mat MatK;
  cv::Mat MdistCoeffs;
    
    vector<cv::Mat> vCurrentDesc;
    vector<string > vmodelName;
    vector<vector<string> > vvbundlerxml;//每个模型里面list_xml，图像序列的名字
    vector<vector<cv::Mat*> > vvRT;
    vector<vector<string> > vvfk;
    vector<vector<ModelPoint*> > vvModelPoints;
    
    vector<Model* > vModel;
    vector<DBoW2::BowVector> vModel_BowVector;
    vector<DBoW2::FeatureVector> vModel_FeatureVector;
    
    
  protected:
    const ORBVocabulary* mdbpORBVocabulary;
    vector<int > model_num;	//模型的种类数目，里面每一个元素存储了第i个模型有多少图片——-——换成vector，可以push back
    //     vector<cv::Mat *> Descriptors;	//所有模型的descriptor
    vector<vector<ModelFrame*> > mdbModelFrames;
    std::vector<list<ModelFrame*> > mdbInvertedFile;
    std::vector<list<int> > mdbInvertedFile4Model;
    
    void Mat2KeyPoints(cv::Mat mKeyPoints, vector<cv::KeyPoint> &vKeyPoints);
    Mat3 Mat2Mat3(cv::Mat Rodri);
    Vec3 Mat2Vec3(cv::Mat tvec);
    
    vector<cv::DMatch> ComputeReprojectInliers(cv::Mat rvec, cv::Mat tvec, Mat4X AllModelPointsMat,  KeyFrame* pKF, cv::Mat mpModelDesc );

    // Mutex
    boost::mutex mdbMutex;
    void AssignDescriptors(KeyFrame* pKF, int i);
     void matches2points(const vector<cv::KeyPoint>& train, const vector<cv::KeyPoint>& query,
        const std::vector<cv::DMatch>& matches, std::vector<cv::Point2f>& pts_train,
        std::vector<cv::Point2f>& pts_query);
     ////分解Essential Matrix
     void EssentialFromFundamental(const Mat3 &F,
                              const Mat3 &K1,
                              const Mat3 &K2,
                              Mat3 *E) ;
    void MotionFromEssential(const Mat3 &E,
                         std::vector<Mat3> *Rs,
                         std::vector<Vec3> *ts);
    template <typename TMat, typename TVec>
    double Nullspace(TMat *A, TVec *nullspace);
    void HomogeneousToEuclidean(const Vec4 &H, Vec3 *X);
    void TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
                    const Mat34 &P2, const Vec2 &x2,
                    Vec4 *X_homogeneous);
    void TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
                    const Mat34 &P2, const Vec2 &x2,
                    Vec3 *X_euclidean) ;
    Mat34    HStack ( const Mat3 & lhs, const Vec3 & rhs); 
    void P_From_KRt(
    const Mat3 &K,  const Mat3 &R,  const Vec3 &t, Mat34 *P);

    double Depth(const Mat3 &R, const Vec3 &t, const Vec3 &X);
    Vec2 Project(const Mat34 &P, const Vec3 &X) ;
    void Project(const Mat34 &P, const Mat4X &X, Mat2X *x);
    void Project(const Mat34 &P, const Mat3X &X, Mat2X *x);
    void KeyPoint2Points(vector<cv::KeyPoint> keypoints, vector<cv::Point2f>& points);
    
    void TranslateO2C(const Mat3 &R, const Vec3 &t, const Mat4X &X , vector<cv::Point3f> &newX);
    
    bool Matches2Points2Points3(vector<cv::KeyPoint> srcKeyPoints,
				vector<ModelPoint*> objModelPoints, vector<cv::DMatch> matches, vector<cv::Point2f> &srcPoints, vector<cv::Point3f> &objModelPoint3f,vector<cv::DMatch>& afterModelPointMatches
    , vector<ModelPoint* >& modelPointsAfterMatches
      
    );
    void ModelPoints23fPoints(vector<ModelPoint*> ModelPoints, vector<cv::Point3f>& CPoints3f);
    void ModelPoints2Mat4X(vector<ModelPoint*> ModelPoints, Mat4X &Mat4XModelPoints);
    void Points3f2Mat(vector<cv::Point3f> CPoints3f, Mat4X & Mat3XPoints);
    void Mat2X2Point2f(Mat2X MatPoints, vector<cv::KeyPoint> &vKeyPoints);
    
    void AlignDescriptores();
  };
}


#endif//KEYPOINTCLUSTER