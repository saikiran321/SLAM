#include <iostream>
#include<opencv2/opencv.hpp>
#include<string>
#include<istream>
#include<sstream>
#include<ostream>
#include<vector>
#include<eigen3/Eigen/Eigen>
#include "pointcloud.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/poisson.h>
#include <pcl/common/common.h>

using namespace std;
#define MAX_FRAME 1000
//#define VISUAL_DEBUG 1


const string datapath="/home/sai/data/rgbd_dataset_freiburg1_xyz/";

Eigen::Matrix3f intrinsicMatrix;


struct Point3D {

    cv::Point3d pt;
    std::map<const int,int> idxImage; // for bundle adjustment
    std::map<const int,cv::Point2d> pt2D; // for bundle adjustment

};

void get_data(std::vector<cv::Mat> &depthImages,std::vector<cv::Mat> &rgbImages,std::vector<Eigen::Matrix4f> &poses)
{


    int i=0;
    //intrinsic matrix
    intrinsicMatrix <<  525.0f, 0.0f, 319.5f,
                        0.0f, 525.0f, 239.5f,
                        0.0f, 0.0f, 1.0f;

    float q1, q2, q3, q4, translation1, translation2, translation3;
    float junkstamp;
    std::string depthname; std::string rgbname;

    std::fstream associationfile;
    std::vector<std::pair<Eigen::Matrix3d,Eigen::Vector3d> > poses_from_assfile;
    std::vector<std::vector<std::string> > depthNames;
    std::vector<std::vector<std::string> > rgbNames;
    std::vector<std::string> &depthNamesLast = depthNames.back();

    associationfile.open(datapath+"associate.txt",std::ios::in);



    if(!associationfile)
    {
        std::cout << "hell" << std::endl;
        std::cerr << "Unable to open association file" << std::endl;
        exit(1);
    }

    std::string temp("");
    while(getline(associationfile,temp))
    {
        i++;

        if(i%5!=0)
        {
            continue;
        }

        std::cout << i << std::endl;

        std::stringstream stream(temp);
        stream >> junkstamp;
        stream >> rgbname;
        stream >> junkstamp;
        stream >> depthname;
        stream >> junkstamp;
        stream >> translation1; stream >> translation2; stream >> translation3;
        stream >> q1; stream >> q2; stream >> q3; stream >> q4;


        cv::Mat depthImg = cv::imread(datapath+depthname,cv::IMREAD_ANYDEPTH);
        cv::Mat rgbImg = cv::imread(datapath+rgbname,cv::IMREAD_COLOR);

        //pushing depth and rgb
        depthImages.emplace_back(depthImg);
        rgbImages.emplace_back(rgbImg);

        //setting pose here
        Eigen::Matrix4f extrinsics;
        extrinsics.setIdentity();
        Eigen::Matrix3d rotation = Eigen::Quaterniond(q4,q1,q2,q3).toRotationMatrix();
        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++){
                extrinsics(i,j) = rotation(i,j);
            }
        }
        extrinsics(0,3) = translation1;
        extrinsics(1,3) = translation2;
        extrinsics(2,3) = translation3;

        //pushing pose
        poses.emplace_back(extrinsics);





    }

    //closing associate file
    associationfile.close();

}

void alignedPoints(const std::vector<cv::Point2d>& queryImg,const std::vector<cv::Point2d>& trainImg,const std::vector<cv::DMatch>& match, std::vector<cv::Point2d>& alignedL, std::vector<cv::Point2d>& alignedR){


      //align left and right point sets
      for(unsigned int i=0;i<match.size();i++){

        alignedL.push_back(queryImg[match[i].queryIdx]);
        alignedR.push_back(trainImg[match[i].trainIdx]);

      }


}

void keyPointsToPoints(const std::vector<cv::KeyPoint> &keypoints,std::vector<cv::Point2d> &points2d)
{
    points2d.clear();
    for(const cv::KeyPoint& kp: keypoints){
           points2d.push_back(kp.pt);
     }
}

void getPrunedMatchesHomography( const std::vector<cv::DMatch> &matches , const std::vector<cv::Point2d> &queryPoint,const std::vector<cv::Point2d> &trainPoint,  std::vector<cv::DMatch> &prunedMatches)
{



     const double ransac_thresh = 2.5;
     cv::Mat inliers_mask, homography;
     if(matches.size()<4)
     {
         std::cerr<< "could not find enoug points for estimating homography" << std::endl;
         exit(1);
     }

    homography = cv::findHomography(queryPoint,trainPoint,CV_RANSAC,ransac_thresh,inliers_mask);


    for(unsigned i = 0; i < inliers_mask.rows; i++) {
          if(inliers_mask.at<uchar>(i)) {
              prunedMatches.push_back(matches[i]);
          }
      }

//    std::cout << "[homographicInliers]: " << (float)prunedMatches.size()/(float)matches.size() << std::endl;

}


double determinante(cv::Mat& relativeRotationCam){

  Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,
                                  Eigen::Dynamic,
                                  Eigen::RowMajor> > eigenMatrix((double *)relativeRotationCam.data,3,3);

  Eigen::FullPivLU<Eigen::Matrix<double, Eigen::Dynamic,
                                         Eigen::Dynamic,
                                         Eigen::RowMajor>> eigenMatrixV2(eigenMatrix);

  double det = eigenMatrixV2.determinant();
  return det;
}

bool isValidRotation(cv::Mat &R)
{
    if(fabsf(determinante(R))-1.0 > 1e-07) {

        std::cout << "det(R) != +-1.0, this is not a rotation matrix" << std::endl;
        return false;
    }
    return true;
}


void cameraPose(const cv::Mat &intrinsic, const std::vector<cv::DMatch> &matches , const std::vector<cv::Point2d> &queryPoint,const std::vector<cv::Point2d> &trainPoint,cv::Matx34d &Pleft,cv::Matx34d &Pright)
{

    std:vector<cv::DMatch> prunedMatches;
    std::vector<cv::Point2d> alignedLeft,alignedRight;
    alignedPoints(queryPoint,trainPoint,matches,alignedLeft,alignedRight);

    getPrunedMatchesHomography(matches,alignedLeft,alignedRight,prunedMatches);

    std::cout << "pruned matches:" << prunedMatches.size() << std::endl;

    cv::Mat mask;
    cv::Mat cam_matrix = (cv::Mat_<double>(3,3) << 525.0f, 0.0f, 319.5f, 0.0f, 525.0f, 239.5f,0.0f, 0.0f, 1.0f);


    cv::Mat E = cv::findEssentialMat(alignedLeft, alignedRight,cam_matrix,CV_RANSAC,0.999, 1.0,mask);


    std::cout << "Essential matrix:\n" << E << std::endl;

     cv::Mat R,T;

     double fx = cam_matrix.at<double>(0,0);
     double cx = cam_matrix.at<double>(0,2);
     double cy = cam_matrix.at<double>(1,2);
     cv::Point2d pp = cv::Point2d(cx,cy);

     cv::recoverPose(E,alignedLeft, alignedRight,R,T,fx,pp,mask);

     bool success = isValidRotation(R);


     std::cout << "R:\n" << R << std::endl;
     std::cout << "T:\n" << T << std::endl;

     if(not success){

         std::cerr << "Bad rotation." << std::endl;
         exit(1);
     }



     Pright = cv::Matx34d(R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),T.at<double>(0),
                          R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),T.at<double>(1),
                          R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),T.at<double>(2));

     Pleft  = cv::Matx34d::eye();

//     std::cout << "Pright:\n" << Pright << std::endl;


}

void traingulateView(const int viewnum, const std::vector<cv::Point2d> &queryPoint,const std::vector<cv::Point2d> &trainPoint,std::vector<cv::DMatch> &matches,const cv::Mat &intrinsic,cv::Matx34d &Pleft,cv::Matx34d &Pright,std::vector<cv::Point3d> &pointcloud)
{
    std::cout << "-----------------------------------------------" << std::endl;
    std::cout << "Triangulating image:" << viewnum << " and image:" << viewnum+1 << std::endl;
    std::cout << "** IMAGE COORDINATE - CAMERA COORDINATE CONVERTION **" << std::endl;

    //clearing pointlcoud container
    pointcloud.clear();


    std::vector<cv::Point2d> alignedLeft,alignedRight;
    alignedPoints(queryPoint,trainPoint,matches,alignedLeft,alignedRight);

    cv::Mat cameraDistCoeffs = (cv::Mat_<float>(1,5)<< 0,0,0,0,0);

    cv::Mat normalizedLeftPts,normalizedRightPts;

    cv::undistortPoints(alignedLeft,normalizedLeftPts,intrinsic,cameraDistCoeffs);
    cv::undistortPoints(alignedRight,normalizedRightPts,intrinsic,cameraDistCoeffs);

    std::cout << "Triangulating points..." << std::endl;
    cv::Mat pts3dHomogeneous;

    cv::triangulatePoints(Pleft,Pright,normalizedLeftPts,normalizedRightPts,pts3dHomogeneous);

    std::cout << "** CAMERA COORDINATE - WORLD COORDINATE CONVERTION **" << std::endl;

    std::cout << "Converting points to world coordinate..." << std::endl;
    cv::Mat pts3d;
    cv::convertPointsFromHomogeneous(pts3dHomogeneous.t(),pts3d);



    cv::Mat rvecLeft;
    cv::Rodrigues(Pleft.get_minor<3,3>(0,0),rvecLeft);
    cv::Mat tvecLeft(Pright.get_minor<3,1>(0,3).t());
    std::vector<cv::Point2d> projectedLeft(alignedLeft.size());
    cv::projectPoints(pts3d,rvecLeft,tvecLeft,intrinsic,cameraDistCoeffs,projectedLeft);


    cv::Mat rvecRight;
    cv::Rodrigues(Pright.get_minor<3,3>(0,0),rvecRight);
    cv::Mat tvecRight(Pright.get_minor<3,1>(0,3).t());
    std::vector<cv::Point2d> projectedRight(alignedRight.size());
    cv::projectPoints(pts3d,rvecRight,tvecRight,intrinsic,cameraDistCoeffs,projectedRight);


    std::cout << "Creating a pointcloud vector..." << std::endl;
    const float MIN_REPROJECTION_ERROR = 6.0; //Maximum 10-pixel allowed re-projection error

    for(int i = 0; i < pts3d.rows; i++){

        const float queryError = cv::norm(projectedLeft[i]  - alignedLeft[i]);
        const float trainError = cv::norm(projectedRight[i] - alignedRight[i]);

        if(MIN_REPROJECTION_ERROR < queryError or
                MIN_REPROJECTION_ERROR < trainError) continue;


        cv::Point3d p = cv::Point3d(pts3d.at<double>(i, 0),
                                    pts3d.at<double>(i, 1),
                                    pts3d.at<double>(i, 2));


        pointcloud.push_back(p);



    }

    std::cout <<  viewnum <<"-" << pointcloud.size()<<std::endl;







}

int main()
{

    std::vector<cv::Mat>depthImages;
    std::vector<cv::Mat>rgbImages;
    std::vector<Eigen::Matrix4f> poses;

    std::vector<std::vector<cv::Point2d>> allPoints2d;

    get_data(depthImages,rgbImages,poses);


    std::cout << depthImages.size() << std::endl;

    std::cout << rgbImages.size() << std::endl;

    std::cout << poses.size() << std::endl;


    std::vector<std::vector<cv::KeyPoint>> allKeypoints;
    std::vector<cv::Mat> allDescriptors;
    std::vector<std::vector<std::vector<cv::DMatch>>> allMatches;
    std::vector<std::vector<cv::DMatch>> allgoodMatches;


    //orb feature detector
    cv::Ptr<cv::ORB> orbDetector = cv::ORB::create(2000);
    //Brisk feature descriptor
    cv::Ptr<cv::BRISK> briskDetector = cv::BRISK::create();

    cv::BFMatcher matcher(cv::NORM_HAMMING);
    int nearestNeigbour = 2;
    const float ratio = 0.60;


    for(int i=0;i<rgbImages.size();i++)
    {
        //
        std::cout << i << std::endl;
        //extracring features and descriptors here
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        orbDetector->detect(rgbImages.at(i),keypoints);
        briskDetector->compute(rgbImages.at(i),keypoints,descriptors);
        allKeypoints.emplace_back(keypoints);
        allDescriptors.emplace_back(descriptors);
        //converting keypoints to points
        std::vector<cv::Point2d> points2d;
        keyPointsToPoints(keypoints,points2d);
//        std::cout << "[points2d]" << points2d.size() <<std::endl;


        allPoints2d.push_back(points2d);

#ifdef VISUAL_DEBUG
        cv::Mat image=rgbImages[i].clone();
        cv::Mat imageKps;
        cv::drawKeypoints(image,keypoints,imageKps,cv::Scalar::all(-1),0);
        cv::imshow("Keypoint Debugger",imageKps);
        cv::waitKey(10);
#endif

        if(i>0)
        {
            std::vector<std::vector<cv::DMatch>> matches;
            matcher.knnMatch(allDescriptors[i-1],allDescriptors[i],matches,nearestNeigbour);
            allMatches.emplace_back(matches);

//            std::vector<cv::DMatch> goodMatches;

//            for(int x = 0; x < matches.size(); i++) {
//                 if(matches[x][0].distance <= 0.7 * matches[x][1].distance) {
//                     goodMatches.push_back(matches[x][0]);
//                 }
//             }

//            allgoodMatches.push_back(goodMatches);





        }
    }


    for(int i=0;i<allMatches.size();i++)
    {
        std::vector<cv::DMatch> goodMatches;
        for(int j=0;j<allMatches[i].size();j++)
        {
            if(allMatches[i][j][0].distance<0.7*allMatches[i][j][1].distance)
            {
                 std::cout << i <<"--"<< j <<std::endl;
                 goodMatches.push_back(allMatches[i][j][0]);
            }
        }
        allgoodMatches.push_back(goodMatches);
    }


    std::vector<PointCloud> allPcl;

    /*
    
    for(int i=0;i<rgbImages.size();i++)
    {
        Eigen::Matrix4f extrinsics;

        extrinsics = Eigen::MatrixX4f::Identity();

        PointCloud pcl =PointCloud(intrinsicMatrix,extrinsics);

        allPcl.push_back(pcl);

    }
    
    for(int i=0;i<rgbImages.size()-2;i++)
    {
        std::vector<std::vector<cv::DMatch>> matchFirstThirdFrame;

        matcher.knnMatch(allDescriptors[i],allDescriptors[i+2],matchFirstThirdFrame,nearestNeigbour);


        std::map<int, int> good_matches, good_matches_prev;

        //for every image pair find outlier image
        for(int j=0;j<allMatches[i].size();j++)
        {

            // from David lowe paper
            if(allMatches[i][j][0].distance < 0.7*allMatches[i][j][1].distance)
            {
                //checking samepoint in the second image pair to  make sure that it is not noise
                //[future]improve to set the min num of cams a point to be seen to know if it is inlier
                if(allMatches[i+1][allMatches[i][j][0].trainIdx][0].distance<0.7 * allMatches[i+1][allMatches[i][j][0].trainIdx][1].distance)
                {

                    if (allMatches[i + 1][allMatches[i][j][0].trainIdx][0].trainIdx == matchFirstThirdFrame[j][0].trainIdx) {

                        cv::Point2f point2d = allKeypoints[i][allMatches[i][j][0].queryIdx].pt;
                        cv::Point2f point2d_next_frame = allKeypoints[i + 1][allMatches[i][j][0].trainIdx].pt;




                        good_matches[allMatches[i][j][0].queryIdx] = allMatches[i][j][0].trainIdx;
                        good_matches_prev[allMatches[i][j][0].trainIdx] = allMatches[i][j][0].queryIdx;



                    }



                }

            }





        }




    }


*/

    //not doing recursive search for image, cause here we have sequential images;
    // if images are not in sequence please find best matches using  matchingBestPair


// cameramarix (intrinsic)
    cv::Mat cam_matrix = (cv::Mat_<double>(3,3) << 525.0f, 0.0f, 319.5f, 0.0f, 525.0f, 239.5f,0.0f, 0.0f, 1.0f);

//     matchedPointsLeft, matchePointsRight
// to find inlier ratio
     for(int i_=0;i_<rgbImages.size()-1;i_++)
     {
        //checking for net immideate image

         int j_=i_+1;
//         std::cout << "[Aligning]"<<i_ << std::endl;
         std::vector<cv::Point2d> alignedLeft,alignedRight;
         alignedPoints(allPoints2d[i_],allPoints2d[j_],allgoodMatches[i_],alignedLeft,alignedRight);
         cv::Mat mask;
         cv::Mat E = cv::findEssentialMat(alignedLeft, alignedRight,
                                          cam_matrix,CV_RANSAC,0.999, 1.0,mask);

         //refine inlier based on EssentialMatrix
         std::vector<cv::DMatch> prunedMatch;
         for(int x_=0;x_<mask.rows;x_++)
         {
             if(mask.at<uchar>(x_))
             {
                 prunedMatch.push_back(allgoodMatches[i_][x_]);
             }
         }
         float poseInliersRatio = (float)prunedMatch.size() / (float)allgoodMatches[i_].size();
         std::cout << " pose inliers ratio. - "  <<poseInliersRatio << std::endl;

     }


    std::vector<std::vector<cv::Point3d>> pclCloud;

    for(int i=0;i<rgbImages.size()-1;i++)
    {
        cv::Matx34d Pleft  = cv::Matx34d::eye();
        cv::Matx34d Pright = cv::Matx34d::eye();
        cameraPose(cam_matrix,allgoodMatches[i],allPoints2d[i],allPoints2d[i+1],Pleft,Pright);
        std::vector<cv::Point3d> pointcloud;
        //triangulate views to get Pointcloud
//input -> imagePoints_img1,imagePoints_img2
        traingulateView(i,allPoints2d[i],allPoints2d[i+1],allgoodMatches[i],cam_matrix,Pleft,Pright,pointcloud);
        pclCloud.push_back(pointcloud);
    }



    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCL;

    cloudPCL.reset(new pcl::PointCloud<pcl::PointXYZ> ());
    for(size_t i = 0; i < pclCloud.size(); i++){
        for(size_t j=0;j<pclCloud[i].size();j++)
        {
            pcl::PointXYZ pclp;
            cv::Point3d pt3d = pclCloud[i][j];
            pclp.x  = pt3d.x;
            pclp.y  = pt3d.y;
            pclp.z  = pt3d.z;
            cloudPCL->push_back(pclp);
        }

    }
    cloudPCL->width = (uint32_t) cloudPCL->points.size(); // number of points
    cloudPCL->height = 1;	// a list, one row of data
    cloudPCL->header.frame_id ="map";
    cloudPCL->is_dense = false;
    pcl::visualization::CloudViewer viewer("MAP3D");
    viewer.showCloud(cloudPCL,"cloudSFM");

    std::cout << cloudPCL->points.size()  << std::endl;
    std::cout << "Press q to quit" << std::endl;
    while(!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    }


    pcl::io::savePLYFile("MAP3D.ply",*cloudPCL);
    std::cout << "ended" << std::endl;

    return 0;

}




void matchingBestPair(const std::vector<cv::Mat> &rgbImages,const std::vector<std::vector<cv::KeyPoint>> &allKeypoints,const std::vector<std::vector<cv::DMatch>> &allMatches)
{
    //Matching best pair

    std::map<float,std::pair<int,int>> numInliers;

    for(int i_=0;i_<rgbImages.size()-1;i_++)
    {
        for(int j_=i_+1;j_<rgbImages.size();j_++)
        {

#ifdef VISUAL_DEBUG
            cv::Mat matchImage;
            cv::drawMatches(rgbImages[i_],allKeypoints[i_],rgbImages[j_],
                                    allKeypoints[j_],allMatches[i_],matchImage,
                                    cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 255));
            cv::imshow("Matching pairs", matchImage);
            cv::waitKey(100);
#endif
            std::cout << allMatches[i_].size() << std::endl;


        }
    }

}
