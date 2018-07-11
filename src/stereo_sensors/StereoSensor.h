#ifndef STEREOSENSOR_H
#define STEREOSENSOR_H
#include "UnscentedTransform.h"
#include "StereoData.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>



class StereoSensor :  public UnscentedTransform<cv::Mat>
{
protected:
    std::vector<cv::Mat> diff;
    virtual void computeSigmaPoints() = 0;
    virtual void rectifySigmaPoints(const cv::Mat & H1,
                                    const cv::Mat & H2,
                                    const cv::Mat & stereo_rectification_map1_left,
                                    const cv::Mat & stereo_rectification_map2_left,
                                    const cv::Mat & stereo_rectification_map1_right,
                                    const cv::Mat & stereo_rectification_map2_right) = 0;
    virtual void findWarpedCorrespondences(const cv::Mat & disparities_, cv::Mat & correspondences_map_1, cv::Mat & correspondences_map_2) = 0;
private:
    void triangulate(const cv::Mat & correspondences_map_1,
                     const cv::Mat & correspondences_map_2,
                     const double & translation);

    void closestPointsLines(const double & base_line,
                            const cv::Vec3d & camera_1_point,
                            const cv::Vec3d & camera_2_point,
                            cv::Vec3d & resulting_point);
public:

    StereoSensor(const int & width_,
                 const int & height_,
                 const int & number_of_disparities_,
                 const int & min_disparity_,
                 const double & L_,
                 const double & alpha_,
                 const double & ki_,
                 const double & beta_,
                 const double & scaling_factor_,
                 const std::string & reference_frame
                 );



    void computeUncertainty(const cv::Mat & disparities_,
                            const cv::Mat & H1,
                            const cv::Mat & H2,
                            const cv::Mat & stereo_rectification_map1_left,
                            const cv::Mat & stereo_rectification_map2_left,
                            const cv::Mat & stereo_rectification_map1_right,
                            const cv::Mat & stereo_rectification_map2_right,
                            const double & translation);



    StereoData stereo_data;


protected:

    double center_height;      // cartesian x center of the sensor
    double center_width;       // cartesian y cennter of the center

    int number_of_disparities;
    int min_disparity;
};

#endif // STEREOSENSOR_H
