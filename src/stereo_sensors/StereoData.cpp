#include "StereoData.h"

StereoData::StereoData(const int & rows_,
                       const int & columns_,
                       const std::string & ego_frame) :
    rows(rows_),
    columns(columns_)
{

    point_cloud.reserve(columns*rows);
    point_cloud.header.frame_id=ego_frame;

    point_cloud_uncertainty.reserve(columns*rows);
    point_cloud_uncertainty.header.frame_id=ego_frame;

    point_cloud_uncertainty_viz.reserve(columns*rows);
    point_cloud_uncertainty_viz.header.frame_id=ego_frame;

    mean_point_cloud.reserve(columns*rows);
    mean_point_cloud.header.frame_id=ego_frame;

    sigma_clouds_mats.resize(9);
    sigma_point_clouds.resize(9);
    for(int i=0; i<sigma_clouds_mats.size(); ++i)
    {
        sigma_point_clouds[i].reserve(columns*rows);
        sigma_point_clouds[i].header.frame_id=ego_frame;
        sigma_point_clouds[i].width=columns;
        sigma_point_clouds[i].height=rows;
        sigma_clouds_mats[i]=cv::Mat(rows,columns,CV_64FC3);
    }

    mean_3d=cv::Mat::zeros(rows,columns, CV_64FC3);

    covariances_determinants=cv::Mat(rows,columns, CV_64F,NAN);
    informations_determinants=cv::Mat(rows,columns, CV_64F,NAN);

    cov_3d.resize(rows);
    information_3d.resize(rows);
    for(int r=0;r<rows;++r)
    {
        cov_3d[r].resize(columns);
        information_3d[r].resize(columns);

        for(int c=0;c<columns;++c)
        {
            cov_3d[r][c]=cv::Mat(3,3, CV_64F,cv::Scalar(NAN));
            information_3d[r][c]=cv::Mat(3,3, CV_64F,cv::Scalar(NAN));
        }
    }
}

cv::Mat StereoData::getMean()
{
    return mean_3d;
}

cv::Mat StereoData::get3Dpoints()
{
    return point_cloud_cartesian;
}

cv::Mat StereoData::getLeftImage()
{
    return point_cloud_rgb;
}

std::vector<std::vector<cv::Mat> > StereoData::getCovariances()
{
    return cov_3d;
}

std::vector<std::vector<cv::Mat> > StereoData::getInformations()
{
    return information_3d;
}

cv::Mat StereoData::getCovariancesDeterminants()
{
    covariances_determinants.setTo(cv::Scalar(0));
    Eigen::Matrix<double,3,3> covariance_eigen;

    for(int c=0;c<columns;++c)
    {
        for(int r=0;r<rows;++r)
        {
            cv::cv2eigen(cov_3d[r][c],covariance_eigen);
            covariances_determinants.at<double>(r,c)=covariance_eigen.determinant(); // DETERMINANT (VOLUME)
        }
    }

    return covariances_determinants;
}

cv::Mat StereoData::getInformationsDeterminants()
{
    informations_determinants.setTo(cv::Scalar(0));
    Eigen::Matrix<double,3,3> information_eigen;
    for(int r=0;r<rows;++r)
    {
        for(int c=0;c<columns;++c)
        {
            cv::cv2eigen(information_3d[r][c],information_eigen);
            informations_determinants.at<double>(r,c)=information_eigen.determinant(); // DETERMINANT (VOLUME)
        }
    }

    return informations_determinants;
}

void StereoData::transformPointCloudsOpenCVToPCL()
{
    point_cloud.clear();
    mean_point_cloud.clear();
    point_cloud_uncertainty.clear();
    point_cloud_uncertainty_viz.clear();
    cv::Mat informations_determinants=getInformationsDeterminants();

    for(int r=0; r<point_cloud_cartesian.rows; ++r)
    {
        for (int c=0; c<point_cloud_cartesian.cols; ++c)
        {


            //if(std::isnan(informations_determinants.at<double>(r,c))||std::isnan(log(informations_determinants.at<double>(r,c)))||log(informations_determinants.at<double>(r,c))<information_lower_bound)

            if(std::isnan(informations_determinants.at<double>(r,c))||std::isnan(log(informations_determinants.at<double>(r,c))))
                continue;
            // Point cloud cartesian
            pcl::PointXYZRGB point;
            //point.data[0] = stereo_data.mean_3d.at<cv::Vec3d>(r,c)[0];
            //point.data[1] = stereo_data.mean_3d.at<cv::Vec3d>(r,c)[1];
            //point.data[2] = stereo_data.mean_3d.at<cv::Vec3d>(r,c)[2];
            point.data[0] = point_cloud_cartesian.at<cv::Vec3d>(r,c)[0];
            point.data[1] = point_cloud_cartesian.at<cv::Vec3d>(r,c)[1];
            point.data[2] = point_cloud_cartesian.at<cv::Vec3d>(r,c)[2];

            point.r=point_cloud_rgb.at<cv::Vec3b>(r,c)[2];
            point.g=point_cloud_rgb.at<cv::Vec3b>(r,c)[1];
            point.b=point_cloud_rgb.at<cv::Vec3b>(r,c)[0];

            point_cloud.points.push_back(point);

            // Point cloud mean
            point.data[0] = mean_3d.at<cv::Vec3d>(r,c)[0];
            point.data[1] = mean_3d.at<cv::Vec3d>(r,c)[1];
            point.data[2] = mean_3d.at<cv::Vec3d>(r,c)[2];

            mean_point_cloud.points.push_back(point);

            // Point cloud uncertainty
            pcl::PointXYZI point_uncertainty;
            point_uncertainty.intensity=informations_determinants.at<double>(r,c);

            point_uncertainty.data[0] = point_cloud_cartesian.at<cv::Vec3d>(r,c)[0];
            point_uncertainty.data[1] = point_cloud_cartesian.at<cv::Vec3d>(r,c)[1];
            point_uncertainty.data[2] = point_cloud_cartesian.at<cv::Vec3d>(r,c)[2];

            point_cloud_uncertainty.push_back(point_uncertainty);
            point_uncertainty.intensity=log(informations_determinants.at<double>(r,c));
            point_cloud_uncertainty_viz.push_back(point_uncertainty);
        }
    }
}

