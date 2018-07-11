#include "Stereo.h"


Stereo::Stereo(const cv::Mat & cameraMatrix1_,
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
               const int ignore_border_left_)
    :
      cameraMatrix1(cameraMatrix1_),
      cameraMatrix2(cameraMatrix2_),
      N(_h),
      M(_w),
      center_y(_h/2),
      center_x(_w/2),
      ego_frame(ego_frame_),
      scaling_factor(scaling_factor_),
      number_of_disparities(number_of_disparities_),

      ignore_border_left(ignore_border_left_)
{
    SBM=cv::StereoSGBM::create(min_disparity,
                           number_of_disparities_,
                           sad_window_size,
                           P1_,
                           P2_,
                           disp_12_max_diff,
                           pre_filter_cap,
                           uniqueness_ratio,
                           speckle_window_size,
                           speckle_range,
                           full_dp);

    /*SBM->create(min_disparity,
                number_of_disparities_,
                sad_window_size,
                P1_,
                P2_,
                disp_12_max_diff,
                pre_filter_cap,
                uniqueness_ratio,
                speckle_window_size,
                speckle_range,
                full_dp);*/



    cameraMatrix1=cameraMatrix1_.clone();
    cameraMatrix2=cameraMatrix2_.clone();

    std::cout << "init global variables" << std::endl;


    std::cout << "SBM->numberOfDisparities:" << SBM->getNumDisparities() << std::endl;
    std::cout << "SBM->min_disparity:"<<SBM->getMinDisparity() << std::endl;
//    std::cout << "SBM->SADWindowSize:"<<SBM->SADWindowSize << std::endl;
    std::cout << "SBM->P1 :"<< SBM->getP1()  << std::endl;
    std::cout << "SBM->P2 :"<< SBM->getP2()  << std::endl;
    std::cout << "SBM->uniquenessRatio:" <<SBM->getUniquenessRatio()  << std::endl;
    //std::cout << "SBM->disp12MaxDiff:" <<SBM->disp12MaxDiff  << std::endl;
    //std::cout << "SBM->speckleRange:" <<SBM->speckleRange  << std::endl;
    //std::cout << "SBM->speckleWindowSize:" <<SBM->speckleWindowSize  << std::endl;
    //std::cout << "SBM->fullDP:" <<SBM->fullDP  << std::endl;

    std::cout << "done" << std::endl;
}

void Stereo::stereoRectify(const cv::Mat & left_image,
                           const cv::Mat & right_image,
                           const cv::Mat & R1,
                           const cv::Mat & R2,
                           const cv::Mat & P1,
                           const cv::Mat & P2,
                           cv::Mat & rectified_left_image,
                           cv::Mat & rectified_right_image)
{


    cv::initUndistortRectifyMap(cameraMatrix1,
                                cv::Mat::zeros(5,1,CV_64F),
                                R1,
                                P1,
                                cv::Size(left_image.cols,
                                     left_image.rows),
                                CV_16SC2,
                                stereo_rectification_map1_left,
                                stereo_rectification_map2_left);

    cv::initUndistortRectifyMap(cameraMatrix2,
                                cv::Mat::zeros(5,1,CV_64F),
                                R2,
                                P2,
                                cv::Size(right_image.cols,
                                     right_image.rows),
                                CV_16SC2,
                                stereo_rectification_map1_right,
                                stereo_rectification_map2_right);

    cv::remap(left_image, rectified_left_image, stereo_rectification_map1_left, stereo_rectification_map2_left, cv::INTER_LINEAR);
    cv::remap(right_image, rectified_right_image, stereo_rectification_map1_right, stereo_rectification_map2_right, cv::INTER_LINEAR);

    decomposeProjectionMatrix(P1,rectifiedCameraMatrix1,rot1,trans1);
    decomposeProjectionMatrix(P2,rectifiedCameraMatrix2,rot2,trans2);

    H1=cv::Mat::eye(4,4,CV_64F);
    H1.colRange(0,3).rowRange(0,3)=R1*cameraMatrix1.inv();
    H2=cv::Mat::eye(4,4,CV_64F);
    H2.colRange(0,3).rowRange(0,3)=R2*cameraMatrix2.inv();

    //H1=R1*cameraMatrix1.inv();
    //H2=R2*cameraMatrix2.inv();
    return;
}

// Receives rectified cartesian images and calibration matrices
void Stereo::getCartesianDisparityMap(const cv::Mat left_image,
                                      const cv::Mat right_image,
                                      cv::Mat & disparity_image,
                                      cv::Mat & disparity_values)
{
    SBM->compute(left_image, right_image, disparity16S);
    for(int r=0; r<disparity16S.rows; ++r)
    {
        for (int c=0; c<ignore_border_left; ++c)
        {
            disparity16S.at<short>(r,c)=-16;
        }
    }
    disparity16S.convertTo(disparity_image,CV_8U, (255/(SBM->getNumDisparities()*16.0)));
    disparity16S.convertTo(disparity32F, CV_32F, 1.0/16.0);
    disparity32F.convertTo(disparity_values,CV_64F);
}








