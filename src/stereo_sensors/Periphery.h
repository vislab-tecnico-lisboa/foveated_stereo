#ifndef PERIPHERY_H
#define PERIPHERY_H

#include "StereoSensor.h"
#include "logpolar_contrib.hpp"


class Periphery : public StereoSensor, public cv::LogPolar_Interp
{
    std::vector<double> Wsr;
    int ind1;
public:
    Periphery(const int & width_,
              const int & height_,
              const double & scaling_factor_,
              const int & number_of_disparities_,
              const int & min_disparity_,
              const double & L_,
              const double & alpha_,
              const double & ki_,
              const double & beta_,
              const cv::Point2i & _center,
              const int & _R,
              const double & _ro0,
              const int & _interp,
              const int & _full,
              const int & _s,
              const int & _sp,
              const std::string & reference_frame
              );

    void computeSigmaPoints();
    void rectifySigmaPoints(const cv::Mat & H1,
                            const cv::Mat & H2,
                            const cv::Mat & stereo_rectification_map1_left,
                            const cv::Mat & stereo_rectification_map2_left,
                            const cv::Mat & stereo_rectification_map1_right,
                            const cv::Mat & stereo_rectification_map2_right);

    void findWarpedCorrespondences(const cv::Mat & disparities_, cv::Mat & correspondences_map_1, cv::Mat & correspondences_map_2);
    //void computeUncertainty(const cv::Mat & disparities_);
    ///////////////////////////////////////
    // cortical cortical disparity warps //
    ///////////////////////////////////////

    std::vector<cv::Mat> eta_cortical_warp;
    std::vector<cv::Mat> rho_cortical_warp;

    std::vector<cv::Mat> delta_eta_cortical_disparity_map;
    std::vector<cv::Mat> delta_rho_cortical_disparity_map;
    std::vector<double> disparities;

    void createDisparityWarpMaps();
};

#endif // PERIPHERY_H
