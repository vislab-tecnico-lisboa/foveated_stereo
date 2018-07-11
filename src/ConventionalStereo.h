#ifndef CONVENTIONALSTEREO_H
#define CONVENTIONALSTEREO_H

#include "Stereo.h"
#include "stereo_sensors/Cartesian.h"

class ConventionalStereo : public Stereo
{
public:
    ConventionalStereo(const cv::Mat & cameraMatrix1_,
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
                       const int ignore_border_left_);

    StereoData computeStereo(const cv::Mat & left_image,
                             const cv::Mat & right_image,
                             const cv::Mat & R1,
                             const cv::Mat & R2,
                             const cv::Mat & P1_,
                             const cv::Mat & P2_,
                             const cv::Mat & left_to_center);

    void get3DpointCloud(const cv::Mat & disparity_map,
                         cv::Mat & point_cloud,
                         const cv::Mat & transf_,
                         const cv::Mat & Q);

    Cartesian cartesian_sensor;

};

#endif // CONVENTIONALSTEREO_H

