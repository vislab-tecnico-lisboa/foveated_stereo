#ifndef CARTESIAN_H
#define CARTESIAN_H

#include "StereoSensor.h"
#include "Uniform.h"

class Cartesian : public StereoSensor, public Uniform
{
public:
    Cartesian(const int & width_,
              const int & height_,
              const double & scaling_factor_,
              const int & number_of_disparities_,
              const int & min_disparity_,
              const double & L_,
              const double & alpha_,
              const double & ki_,
              const double & beta_,
              const std::string & reference_frame);

    void computeSigmaPoints();
    void rectifySigmaPoints(const cv::Mat & H1,
                            const cv::Mat & H2,
                            const cv::Mat & stereo_rectification_map1_left,
                            const cv::Mat & stereo_rectification_map2_left,
                            const cv::Mat & stereo_rectification_map1_right,
                            const cv::Mat & stereo_rectification_map2_right);
    void findWarpedCorrespondences(const cv::Mat & disparities_, cv::Mat & correspondences_map_1, cv::Mat & correspondences_map_2);

};
#endif // CARTESIAN_H
