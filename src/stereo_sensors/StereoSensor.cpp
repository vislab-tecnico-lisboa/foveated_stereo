#include "StereoSensor.h"

StereoSensor::StereoSensor(const int & width_,
                           const int & height_,
                           const int & number_of_disparities_,
                           const int & min_disparity_,
                           const double & L_,
                           const double & alpha_,
                           const double & ki_,
                           const double & beta_,
                           const double & scaling_factor_,
                           const std::string & reference_frame
                           ):
    UnscentedTransform(L_,alpha_,beta_,ki_,scaling_factor_),
    number_of_disparities(number_of_disparities_),
    min_disparity(min_disparity_),
    center_height((height_-1)/2.0),
    center_width((width_-1)/2.0)
{
    diff.resize(2*L+1);
}


void StereoSensor::triangulate(const cv::Mat & correspondences_map_1,
                               const cv::Mat & correspondences_map_2,
                               const double & translation)
{
    for(int i=0; i<stereo_data.sigma_clouds_mats.size();++i)
    {
        stereo_data.sigma_clouds_mats[i].setTo(NAN);
    }

    //int nthreads;int tid;
    int u,v;
    std::cout << "ui"<<std::endl;
    for(v=0; v<correspondences_map_1.rows; ++v)
    {
        //for(u=ignore_border_left; u<correspondences_map_1.cols; ++u)
        for(u=0; u<correspondences_map_1.cols; ++u)
        {
            if(correspondences_map_1.at<double>(v,u)<0||correspondences_map_2.at<double>(v,u)<0)
                continue;

            closestPointsLines(translation,
                               sigma_points_1_rectified[0].at<cv::Vec3d>(v,u),
                    sigma_points_2_rectified[0].at<cv::Vec3d>(correspondences_map_1.at<double>(v,u),correspondences_map_2.at<double>(v,u)),
                    stereo_data.sigma_clouds_mats[0].at<cv::Vec3d>(v,u));

            for(int i=1; i<5;++i)
            {
                closestPointsLines(translation,
                                   sigma_points_1_rectified[i].at<cv::Vec3d>(v,u),
                                   sigma_points_2_rectified[0].at<cv::Vec3d>(correspondences_map_1.at<double>(v,u),correspondences_map_2.at<double>(v,u)),
                        stereo_data.sigma_clouds_mats[i].at<cv::Vec3d>(v,u));

                closestPointsLines(translation,
                                   sigma_points_1_rectified[0].at<cv::Vec3d>(v,u),
                        sigma_points_2_rectified[i].at<cv::Vec3d>(correspondences_map_1.at<double>(v,u),correspondences_map_2.at<double>(v,u)),
                        stereo_data.sigma_clouds_mats[i+4].at<cv::Vec3d>(v,u));
            }
        }
    }
    std::cout << "ui2"<<std::endl;

    // recompute covariance matrices in 3D space
    std::cout << "  compute 3D mean matrices" << std::endl;
    stereo_data.mean_3d.setTo(cv::Scalar(0));

    int i;
#pragma omp parallel for private(v,u)
    for(i=0; i<weights_mean.size();++i)
    {
        for(v=0; v<correspondences_map_1.rows; ++v)
        {
            //for(u=ignore_border_left; u<correspondences_map_1.cols; ++u)
            for(u=0; u<correspondences_map_1.cols; ++u)
            {
                stereo_data.mean_3d.at<cv::Vec3d>(v,u)+=weights_mean[i]*stereo_data.sigma_clouds_mats[i].at<cv::Vec3d>(v,u);
            }
        }
    }

#pragma omp parallel for private(u)
    for(v=0; v<stereo_data.cov_3d.size(); ++v)
    {
        //for(u=ignore_border_left; u<correspondences_map_1.cols; ++u)
        for(u=0; u<stereo_data.cov_3d[0].size(); ++u)
        {
            stereo_data.cov_3d[v][u].setTo(cv::Scalar(0));
        }
    }
    std::cout << "  compute 3D covariance matrices" << std::endl;


#pragma omp parallel for private(v,u)
    for(i=0; i<stereo_data.sigma_clouds_mats.size();++i)
    {
        diff[i]=stereo_data.sigma_clouds_mats[i]-stereo_data.mean_3d;
        for(v=0; v<correspondences_map_1.rows; ++v)
        {
            //for(u=ignore_border_left; u<correspondences_map_1.cols; ++u)
            for(u=0; u<correspondences_map_1.cols; ++u)
            {
                if(correspondences_map_1.at<double>(v,u)<0||correspondences_map_2.at<double>(v,u)<0)
                    continue;

                stereo_data.cov_3d[v][u]+=weights_cov[i]*cv::Mat(diff[i].at<cv::Vec3d>(v,u))*cv::Mat(diff[i].at<cv::Vec3d>(v,u)).t();
            }
        }
    }

    //#pragma omp parallel for private(u)
    for(v=0; v<correspondences_map_1.rows; ++v)
    {
        //for(u=ignore_border_left; u<correspondences_map_1.cols; ++u)
        for(u=0; u<correspondences_map_1.cols; ++u)
        {
            cv::Mat aux=((stereo_data.cov_3d[v][u]+stereo_data.cov_3d[v][u].t())/2).inv();
            aux.copyTo(stereo_data.information_3d[v][u]);
        }
    }

    for(v=0; v<correspondences_map_1.rows; ++v)
    {
        //for(u=ignore_border_left; u<correspondences_map_1.cols; ++u)
        for(u=0; u<correspondences_map_1.cols; ++u)
        {
            if(correspondences_map_1.at<double>(v,u)<0||correspondences_map_2.at<double>(v,u)<0)
            {
                stereo_data.mean_3d.at<cv::Vec3d>(v,u)=cv::Vec3d(NAN,NAN,NAN);
                stereo_data.cov_3d[v][u].setTo(cv::Scalar(NAN));
                stereo_data.information_3d[v][u].setTo(cv::Scalar(NAN));
            }
        }
    }
}


// http://geomalgorithms.com/a07-_distance.html
void StereoSensor::closestPointsLines(const double & base_line,
                                      const cv::Vec3d & camera_1_point,
                                      const cv::Vec3d & camera_2_point,
                                      cv::Vec3d & resulting_point)
{

    cv::Vec3d u=camera_1_point;
    normalize(u,u,1);
    cv::Vec3d v=camera_2_point;
    normalize(v,v,1);

    double a=u.dot(u);
    double b=u.dot(v);
    double c=v.dot(v);
    double d=-u(0)*base_line;
    double e=-v(0)*base_line;

    double denominator=(a*c)-(b*b);

    double numerator_1=(b*e)-(c*d);
    double numerator_2=(a*e)-(b*d);

    double sc=numerator_1/denominator;
    double tc=numerator_2/denominator;

    cv::Vec3d point_1=sc*u;
    cv::Vec3d point_2=cv::Vec3d(base_line,0,0)+tc*v;

    // SHOULD RETURN ONLY ONE POINT  (THE MIDDLE POINT)

    /*if(denominator<0.00001)
    {
        point_1=NAN;
        point_2=NAN;
        return;
    }*/
    // compute the line parameters of the two closest points
    /*if (denominator < 0.00001) {          // the lines are almost parallel
        sc = 0.0;
        if(b>c)
        {
            tc=d/b;
        }
        else
        {
            tc=e/c;
        }
        tc = (b>c ? d/b : e/c);    // use the largest denominator
        //std::cout << "YAH" << std::endl;
    }//*/

    resulting_point=(point_1+point_2)/2.0;

}


void StereoSensor::computeUncertainty(const cv::Mat & disparities_,
                                      const cv::Mat & H1,
                                      const cv::Mat & H2,
                                      const cv::Mat & stereo_rectification_map1_left,
                                      const cv::Mat & stereo_rectification_map2_left,
                                      const cv::Mat & stereo_rectification_map1_right,
                                      const cv::Mat & stereo_rectification_map2_right,
                                      const double & translation)
{

    std::cout << "  find warped correspondences" << std::endl;
    cv::Mat correspondences_map_1;
    cv::Mat correspondences_map_2;
    findWarpedCorrespondences(disparities_,
                              correspondences_map_1,
                              correspondences_map_2);

    std::cout << "  rectify sigma points" << std::endl;
    rectifySigmaPoints(H1,
                       H2,
                       stereo_rectification_map1_left,
                       stereo_rectification_map2_left,
                       stereo_rectification_map1_right,
                       stereo_rectification_map2_right);
    std::cout << "  triangulate" << std::endl;

    triangulate(correspondences_map_1,
                correspondences_map_2,
                translation);
    std::cout << "transform pcl"<< std::endl;
    stereo_data.transformPointCloudsOpenCVToPCL();
}


