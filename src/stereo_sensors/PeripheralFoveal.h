#ifndef PERIPHERALFOVEAL_H
#define PERIPHERALFOVEAL_H

#include "StereoSensor.h"
#include "Uniform.h"
#include "logpolar_contrib.hpp"

class PeripheralFoveal : public StereoSensor, public Uniform, public cv::LogPolar_Interp
{
    std::vector<double> Wsr;
    int ind1;
public:
    PeripheralFoveal(const int & width_,
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
                     const int & fovea_rows_,
                     const int & foveal_columns_,
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

    ///////////////////////////////////////
    // cortical cortical disparity warps //
    ///////////////////////////////////////

    std::vector<cv::Mat> column_disparity_warp;
    std::vector<cv::Mat> row_disparity_warp;

    std::vector<cv::Mat> delta_column_disparity_warp;
    std::vector<cv::Mat> delta_row_disparity_warp;
    std::vector<double> disparities;

    void createDisparityWarpMaps();

    void create_map(const int & input_rows,
                    const int & input_columns,
                    const int & output_rows_,
                    const int & output_columns_,
                    const int & _R,
                    const int & _s,
                    const double & _ro0);

    const cv::Mat to_cortical(const cv::Mat &source);
    //const cv::Mat to_cartesian(const cv::Mat &source);

    std::vector<int> Rsr;
    std::vector<int> Csr;
    struct kernel
    {
        kernel() { w = 0; }
        std::vector<double> weights;
        int w;
    };

    std::vector<kernel> w_ker_2D;

};

#endif // PERIPHERALFOVEAL_H

