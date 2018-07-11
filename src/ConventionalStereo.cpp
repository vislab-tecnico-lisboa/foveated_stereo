#include "ConventionalStereo.h"

ConventionalStereo::ConventionalStereo(const cv::Mat & cameraMatrix1_,
                                       const cv::Mat & cameraMatrix2_,
                                       const int & _w,
                                       const int & _h,
                                       const cv::Point2i _center,
                                       const int _R,
                                       const double _ro0,
                                       const int _interp,
                                       const int _full,
                                       const int _s,
                                       const int _sp,
                                       const int & fovea_rows_,
                                       const int & fovea_columns_,
                                       const std::string & ego_frame_,
                                       const double & L_,
                                       const double & alpha_,
                                       const double & ki_,
                                       const double & beta_,
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
                                       const int ignore_border_left_) :
    Stereo(cameraMatrix1_,
           cameraMatrix2_,
           _w,
           _h,
           ego_frame_,
           scaling_factor_,
           number_of_disparities_,
           pre_filter_cap,
           sad_window_size,
           P1_,
           P2_,
           min_disparity,
           uniqueness_ratio,
           speckle_window_size,
           speckle_range,
           disp_12_max_diff,
           full_dp,
           ignore_border_left_),
    cartesian_sensor( _w,
                      _h,
                      scaling_factor_,
                      number_of_disparities_,
                      min_disparity,
                      L_,
                      alpha_,
                      ki_,
                      beta_,
                      ego_frame_)
{

    std::cout << "done" << std::endl;
}

// Receives cartesian images and calibration matrices
StereoData ConventionalStereo::computeStereo(const cv::Mat & left_image,
                                             const cv::Mat & right_image,
                                             const cv::Mat & R1,
                                             const cv::Mat & R2,
                                             const cv::Mat & P1_,
                                             const cv::Mat & P2_,
                                             const cv::Mat & left_to_center)
{
    cv::Mat Q=makeProjectionMatrix(P2_,1.0,1.0);
    std::cout << "Q:" << Q << std::endl;

    cv::Mat rectified_left_image;
    cv::Mat rectified_right_image;

    stereoRectify(left_image,
                  right_image,
                  R1,
                  R2,
                  P1_,
                  P2_,
                  rectified_left_image,
                  rectified_right_image);

    getCartesianDisparityMap(rectified_left_image,
                             rectified_right_image,
                             cartesian_sensor.stereo_data.disparity_image,
                             cartesian_sensor.stereo_data.disparity_values);
    cv::Mat R1_ = cv::Mat::eye(4,4,CV_64F);
    R1.copyTo(R1_(cv::Range(0,3), cv::Range(0,3)));
    //cv::Mat transf_=left_to_center*R1_.t();
    cv::Mat transf_=R1_.t();

    get3DpointCloud(disparity32F,cartesian_sensor.stereo_data.point_cloud_cartesian,transf_,Q);

    cartesian_sensor.stereo_data.point_cloud_rgb=rectified_left_image;

    ///////////////////////////
    // Propagate uncertainty //
    ///////////////////////////

    std::cout << "propagate uncertainty" << std::endl;

    cartesian_sensor.computeUncertainty(cartesian_sensor.stereo_data.disparity_values,
                                        H1,
                                        H2,
                                        stereo_rectification_map1_left,
                                        stereo_rectification_map2_left,
                                        stereo_rectification_map1_right,
                                        stereo_rectification_map2_right,
                                        trans2.at<double>(0,0));

    return cartesian_sensor.stereo_data;
}

void ConventionalStereo::get3DpointCloud(const cv::Mat & disparity_map,
                                         cv::Mat & point_cloud,
                                         const cv::Mat &transf_,
                                         const cv::Mat & Q)
{
    cv::Mat Q_=transf_*Q;

    cv::Mat point_cloud_cartesian;
    cv::reprojectImageTo3D(disparity_map, point_cloud_cartesian, Q_,true);
    point_cloud_cartesian.convertTo(point_cloud, CV_64FC3); // UNCERTAINTY USES THIS!!! CAREFULL

    double squared_radius_max_aux=pow(cv::min(M-1,N-1)/2.0,2.0);
    for (int c=0; c<M; ++c)
    {
        for(int r=0; r<N; ++r)
        {
            if( ((r-center_y)*(r-center_y)+(c-center_x)*(c-center_x))>squared_radius_max_aux||point_cloud.at<cv::Vec3d>(r,c)[2]>100)
            {
                point_cloud.at<cv::Vec3d>(r,c)=cv::Vec3d(NAN,NAN,NAN);
            }
        }
    }
}





