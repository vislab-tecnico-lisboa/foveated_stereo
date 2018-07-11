#ifndef Stereo_H
#define Stereo_H
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/calib3d.hpp"

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cmath>
#include <vector>
//#include "opencv2/contrib/contrib.hpp"
#include <iostream>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "opencv2/core/eigen.hpp"
#include "stereo_sensors/StereoSensor.h"
#include "opencv2/calib3d.hpp"
#ifdef __cplusplus


/// GPU STUFF
// includes CUDA Runtime
//#include "opencv2/gpu/gpu.hpp"
#define EPSILON 0.01

class Stereo
{
protected:
    int number_of_disparities;
    int ignore_border_left;
    //Disparity Method
    cv::Ptr<cv::StereoSGBM> SBM;

    std::string ego_frame;
public:
    Stereo(const cv::Mat & cameraMatrix1_,
           const cv::Mat & cameraMatrix2_,
           const int _w,
           const int _h,
           const std::string & ego_frame_,
           const double & scaling_factor_,
           const int number_of_disparities_,
           const int pre_filter_cap,
           const int sad_window_size,
           const int P1_,
           const int P2_,
           const int min_disparity,
           const int uniqueness_ratio,
           const int speckle_window_size,
           const int speckle_range,
           const int disp_12_max_diff,
           const bool full_dp,
           const int ignore_border_left_);

    void stereoRectify(const cv::Mat & left_image,
                       const cv::Mat & right_image,
                       const cv::Mat & R1,
                       const cv::Mat & R2,
                       const cv::Mat & P1,
                       const cv::Mat & P2,
                       cv::Mat & rectified_left_image,
                       cv::Mat & rectified_right_image);


    cv::Mat stereo_rectification_map1_left, stereo_rectification_map2_left, stereo_rectification_map1_right, stereo_rectification_map2_right;
    virtual void get3DpointCloud(const cv::Mat & disparity_map,
                                 cv::Mat & point_cloud,
                                 const cv::Mat &transf_,
                                 const cv::Mat & Q) = 0;
    cv::Mat rectifiedCameraMatrix1, rectifiedCameraMatrix2;

    cv::Mat trans1, trans2;
    cv::Mat H1, H2;

    cv::Mat makeProjectionMatrix(const cv::Mat & P2, double x_scale, double y_scale)
    {
        double cx = P2.at<double>(0,2) / x_scale;
        double cy = P2.at<double>(1,2) / y_scale;

        double fx = P2.at<double>(0,0) / x_scale;
        double fy = P2.at<double>(1,1) / y_scale;

        double tx = (P2.at<double>(0,3) / x_scale) / fx;

        cv::Mat Q=cv::Mat::zeros(4,4,CV_64F);
        Q.at<double>(0,0)=fy * tx;
        Q.at<double>(1,1)=fx * tx;
        Q.at<double>(0,3)=-fy * cx * tx;
        Q.at<double>(1,3)=-fx * cy * tx;
        Q.at<double>(2,3)= fx * fy * tx;
        Q.at<double>(3,2)=-fy;
        Q.at<double>(3,3)= fy * (cx - cx);

        return Q;
    }


protected:

    void closestPointsLines(const double & x_origin,
                            const cv::Vec3d & camera_1_point,
                            const cv::Vec3d & camera_2_point,
                            cv::Vec3d & resulting_point);

    ///////////////////////////
    // Rectify images method //
    ///////////////////////////

    void getCartesianDisparityMap(const cv::Mat left_image,
                                  const cv::Mat right_image,
                                  cv::Mat & disparity_image,
                                  cv::Mat & disparity_values);

    int M,N;

    double scaling_factor;                         // scaling factor

    cv::Mat cameraMatrix1, cameraMatrix2;

    cv::Mat P1, P2;

    //cv::Mat R1, R2;

    cv::Mat disparity32F, disparity16S;


    std::vector<std::vector<cv::Mat> > cov_3d;
    cv::Mat mean3d;

    cv::Mat rot1, rot2;

    double center_x,center_y;





    /*void callKernel(const cv::gpu::GpuMat &src, cv::gpu::GpuMat &dst)
    {
        float* p = (float*)src.data;
        float* p2 =(float*) dst.data;
        func(p,p2,src.step,dst.step,src.cols,src.rows);
    }*/


    cv::Vec3d IterativeLinearLSTriangulation(cv::Point3d u,    //homogenous image point (u,v,1)
                                             cv::Matx34d P,          //camera 1 matrix
                                             cv::Point3d u1,         //homogenous image point in 2nd camera
                                             cv::Matx34d P1          //camera 2 matrix
                                             ) {

        double wi = 1, wi1 = 1;
        cv::Mat_<double> X(4, 1);


        for (int i = 0; i < 10; i++) { //Hartley suggests 10 iterations at most
            cv::Mat_<double> X_ = LinearLSTriangulation(u, P, u1, P1);
            X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
            //recalculate weights

            double p2x = cv::Mat_<double>(cv::Mat_<double>(P).row(2)*X)(0);
            double p2x1 = cv::Mat_<double>(cv::Mat_<double>(P1).row(2)*X)(0);

            //breaking point
            if (fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;

            wi = p2x;
            wi1 = p2x1;

            //reweight equations and solve
            cv::Matx43d A((u.x*P(2, 0) - P(0, 0)) / wi, (u.x*P(2, 1) - P(0, 1)) / wi, (u.x*P(2, 2) - P(0, 2)) / wi,
                          (u.y*P(2, 0) - P(1, 0)) / wi, (u.y*P(2, 1) - P(1, 1)) / wi, (u.y*P(2, 2) - P(1, 2)) / wi,
                          (u1.x*P1(2, 0) - P1(0, 0)) / wi1, (u1.x*P1(2, 1) - P1(0, 1)) / wi1, (u1.x*P1(2, 2) - P1(0, 2)) / wi1,
                          (u1.y*P1(2, 0) - P1(1, 0)) / wi1, (u1.y*P1(2, 1) - P1(1, 1)) / wi1, (u1.y*P1(2, 2) - P1(1, 2)) / wi1
                          );
            cv::Mat_<double> B = (cv::Mat_<double>(4, 1) << -(u.x*P(2, 3) - P(0, 3)) / wi,
                                  -(u.y*P(2, 3) - P(1, 3)) / wi,
                                  -(u1.x*P1(2, 3) - P1(0, 3)) / wi1,
                                  -(u1.y*P1(2, 3) - P1(1, 3)) / wi1
                                  );

            cv::solve(A, B, X_, cv::DECOMP_SVD);
            X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
        }
        cv::Vec3d threed_point;
        threed_point[0]= X(0) ;
        threed_point[1]= X(1) ;
        threed_point[2]= X(2) ;

        return threed_point;
    }

    cv::Mat_<double> LinearLSTriangulation(cv::Point3d u,       //homogenous image point (u,v,1)
                                           cv::Matx34d P,       //camera 1 matrix
                                           cv::Point3d u1,      //homogenous image point in 2nd camera
                                           cv::Matx34d P1       //camera 2 matrix
                                           )
    {
        //build matrix A for homogenous equation system Ax = 0
        //assume X = (x,y,z,1), for Linear-LS method
        //which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
        cv::Matx43d A(u.x*P(2, 0) - P(0, 0), u.x*P(2, 1) - P(0, 1), u.x*P(2, 2) - P(0, 2),
                      u.y*P(2, 0) - P(1, 0), u.y*P(2, 1) - P(1, 1), u.y*P(2, 2) - P(1, 2),
                      u1.x*P1(2, 0) - P1(0, 0), u1.x*P1(2, 1) - P1(0, 1), u1.x*P1(2, 2) - P1(0, 2),
                      u1.y*P1(2, 0) - P1(1, 0), u1.y*P1(2, 1) - P1(1, 1), u1.y*P1(2, 2) - P1(1, 2)
                      );

        cv::Mat_<double> B = (cv::Mat_<double>(4, 1) << -(u.x*P(2, 3) - P(0, 3)),
                              -(u.y*P(2, 3) - P(1, 3)),
                              -(u1.x*P1(2, 3) - P1(0, 3)),
                              -(u1.y*P1(2, 3) - P1(1, 3)));

        cv::Mat_<double> X;
        solve(A, B, X, cv::DECOMP_SVD);

        return X;
    }


};

#endif // Stereo_H

#endif
