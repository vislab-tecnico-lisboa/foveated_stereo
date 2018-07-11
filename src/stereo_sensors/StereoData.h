#ifndef STEREODATA_H
#define STEREODATA_H
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
//#include "opencv2/gpu/gpu.hpp"

#include <cmath>
#include <vector>
//#include "opencv2/contrib/contrib.hpp"
#include <iostream>
#include <vector>
//#include <stereo_calib_lib.h> // stereo_disparity_data
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "opencv2/core/eigen.hpp"
#include <cmath>
class StereoData
{
    cv::Mat covariances_determinants;
    cv::Mat informations_determinants;
    unsigned int columns;
    unsigned int rows;
public:
    // OpenCV
    cv::Mat disparity_image;
    cv::Mat disparity_values;

    cv::Mat point_cloud_cartesian;
    cv::Mat point_cloud_rgb;

    std::vector<cv::Mat> sigma_clouds_mats;
    cv::Mat mean_3d;

    std::vector<std::vector<cv::Mat> > cov_3d;
    std::vector<std::vector<cv::Mat> > information_3d;

    cv::Mat left_retinal_image;
    cv::Mat left_cortical_image;
    cv::Mat right_retinal_image;
    cv::Mat right_cortical_image;

    // PCL
    pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
    pcl::PointCloud<pcl::PointXYZI> point_cloud_uncertainty;
    pcl::PointCloud<pcl::PointXYZI> point_cloud_uncertainty_viz;

    pcl::PointCloud<pcl::PointXYZRGB> mean_point_cloud;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB> > sigma_point_clouds;

    StereoData(){}

    StereoData(const int & rows_,
               const int & columns_,
               const std::string & ego_frame);

    cv::Mat get3Dpoints();
    cv::Mat getLeftImage();

    cv::Mat getMean();

    std::vector<std::vector<cv::Mat> > getCovariances();
    std::vector<std::vector<cv::Mat> > getInformations();

    cv::Mat getCovariancesDeterminants();
    cv::Mat getInformationsDeterminants();
    void transformPointCloudsOpenCVToPCL();
};

#endif // STEREODATA_H
