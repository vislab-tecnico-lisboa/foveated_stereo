#include "Cartesian.h"

Cartesian::Cartesian(const int & width_,
                     const int & height_,
                     const double & scaling_factor_,
                     const int & number_of_disparities_,
                     const int & min_disparity_,
                     const double & L_,
                     const double & alpha_,
                     const double & ki_,
                     const double & beta_,
                     const std::string & reference_frame):
    StereoSensor(width_,
                 height_,
                 number_of_disparities_,
                 min_disparity_,
                 L_,
                 alpha_,
                 ki_,
                 beta_,
                 scaling_factor_,
                 reference_frame),
    Uniform(height_,
            width_,
            height_,
            width_,
            std::min(height_,width_))
{
    std::cout << "init cartesian..." << std::endl;
    stereo_data=StereoData(rows,columns,reference_frame);

    for(int i=0;i<L+1;++i)
    {
        sigma_points[i]=cv::Mat(rows,columns, CV_64FC3);
        sigma_points_1_rectified[i]=cv::Mat(rows,columns,CV_64FC3);
        sigma_points_2_rectified[i]=cv::Mat(rows,columns, CV_64FC3);
    }

    computeSigmaPoints();

    for(int i=0;i<diff.size();++i)
        diff[i]=cv::Mat(rows,columns, CV_64FC3);

    std::cout << "done" << std::endl;
}

void Cartesian::computeSigmaPoints()
{
    for(int r=0; r<rows; ++r)
    {
        for(int c=0; c<columns; ++c)
        {
            //double distance_to_fovea=(pow(r-jc,2.0) + pow(c-ic,2.0)); // variance! (std deviation increases linearly, then this is squared)
            //double distance_to_fovea=1.0;
            if(sqrt(pow(r+0.5-center_height,2.0) + pow(c+0.5-center_width,2.0))>(rows/2))
                continue;

            double sigma=scaling_factor*0.5/3.0;
            cv::Mat cov_mat=pow(sigma,2.0)*cv::Mat::eye(2,2,CV_64F);
            cv::Vec3d mean_mat;
            mean_mat(0)=(double)c+0.5;
            mean_mat(1)=(double)r+0.5;
            mean_mat(2)=(double)1.0;

            cv::Mat chol = cov_mat.clone();

            if (Cholesky(chol.ptr<double>(), chol.step, chol.cols, 0, 0, 0))
            {
                cv::Mat diagElem = chol.diag();
                for (int e = 0; e < diagElem.rows; ++e)
                {
                    double elem = diagElem.at<double>(e);
                    chol.row(e) *= elem;
                    chol.at<double>(e,e) = 1.0f / elem;
                }
            }

            cv::Mat A_=const_*chol;
            cv::Mat A=cv::Mat(3,3,CV_64F,cv::Scalar(0));
            A_.copyTo(A.colRange(0,2).rowRange(0,2));
            sigma_points[0].at<cv::Vec3d>(r,c)=(mean_mat);
            sigma_points[1].at<cv::Vec3d>(r,c)=((mean_mat)+cv::Vec3d(A.col(0)));
            sigma_points[2].at<cv::Vec3d>(r,c)=((mean_mat)+cv::Vec3d(A.col(1)));
            sigma_points[3].at<cv::Vec3d>(r,c)=((mean_mat)-cv::Vec3d(A.col(0)));
            sigma_points[4].at<cv::Vec3d>(r,c)=((mean_mat)-cv::Vec3d(A.col(1)));
        }
    }
}

void Cartesian::rectifySigmaPoints(const cv::Mat & H1,
                                   const cv::Mat & H2,
                                   const cv::Mat & stereo_rectification_map1_left,
                                   const cv::Mat & stereo_rectification_map2_left,
                                   const cv::Mat & stereo_rectification_map1_right,
                                   const cv::Mat & stereo_rectification_map2_right)
{
    for(int i=0; i<sigma_points.size(); ++i)
    {
        cv::Mat aux1;
        cv::Mat aux2;
        // Transform sigma points from unrectified left image to rotated camera coordinates
        perspectiveTransform(sigma_points[i], aux1, H1);
        // Transform sigma points from unrectified right image to rotated camera coordinates
        perspectiveTransform(sigma_points[i], aux2, H2);

        cv::remap(aux1, sigma_points_1_rectified[i], stereo_rectification_map1_left, stereo_rectification_map2_left, cv::INTER_LINEAR);
        cv::remap(aux2, sigma_points_2_rectified[i], stereo_rectification_map1_right, stereo_rectification_map2_right, cv::INTER_LINEAR);
    }
}



void Cartesian::findWarpedCorrespondences(const cv::Mat & disparities_, cv::Mat & correspondences_map_1, cv::Mat & correspondences_map_2)
{
    // FIND CORRESPONDENCES
    correspondences_map_1=cv::Mat(rows,columns,CV_64F);
    correspondences_map_1.setTo(cv::Scalar(-1));
    correspondences_map_2=cv::Mat(rows,columns,CV_64F);
    correspondences_map_2.setTo(cv::Scalar(-1));

    for (int c=0; c<rows; ++c)
    {
        for(int r=0; r<columns; ++r)
        {
            if(disparities_.at<double>(r,c)<0) // Outliers
            {
                continue;
            }

            // ROUND SENSOR
            if(sqrt((r+0.5-center_height)*(r+0.5-center_height)+(c+0.5-center_width)*(c+0.5-center_width))>(std::min(rows,columns)/2.0))
            {
                continue;
            }

            correspondences_map_1.at<double>(r,c)=r;
            correspondences_map_2.at<double>(r,c)=c-round(disparities_.at<double>(r,c));
        }
    }
}
