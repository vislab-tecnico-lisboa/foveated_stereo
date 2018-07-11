#include "Periphery.h"

Periphery::Periphery(const int & width_,
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
                     ):
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
    cv::LogPolar_Interp(width_,
                        height_,
                        _center,
                        _R,
                        _ro0,
                        _interp,
                        _full,
                        _s,
                        _sp)
{
    std::cout << "init periphery..." << std::endl;
    stereo_data=StereoData(_s,_R,reference_frame);

    //romax=width_/2.0;
    //a=exp(log(center_width/ro0)/R);
    Wsr.resize(R);

    bool done=false;

    for(int i=0; i<R; i++)
    {
        Wsr[i]=ro0*(a-1)*pow(a,i-1);
        if((Wsr[i]>1)&&(done==false))
        {
            ind1=i;
            done =true;
        }
    }

    for(int i=0;i<L+1;++i)
    {
        sigma_points[i]=cv::Mat(S,R, CV_64FC3);
        sigma_points_1_rectified[i]=cv::Mat(S,R,CV_64FC3);
        sigma_points_2_rectified[i]=cv::Mat(S,R,CV_64FC3);
    }

    computeSigmaPoints();

    createDisparityWarpMaps();

    for(int i=0;i<diff.size();++i)
        diff[i]=cv::Mat(_s,_R, CV_64FC3);
    std::cout << "done" << std::endl;
}

void Periphery::computeSigmaPoints()
{
    ///////////////
    // Periphery //
    ///////////////

    for(int v=0; v<S; ++v)
    {
        for(int u=0; u<R; ++u)
        {

            double x=(double)(ro0*pow(a,u)*cos(v/q)+center_width);
            double y=(double)(ro0*pow(a,u)*sin(v/q)+center_height);

            double sigma=scaling_factor*Wsr[u]/3.0;
            cv::Mat cov_mat=pow(sigma,2.0)*cv::Mat::eye(2,2,CV_64F);

            cv::Vec3d mean_mat;
            mean_mat(0)=(double)x;
            mean_mat(1)=(double)y;
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
            sigma_points[0].at<cv::Vec3d>(v,u)=(mean_mat);
            sigma_points[1].at<cv::Vec3d>(v,u)=((mean_mat)+cv::Vec3d(A.col(0)));
            sigma_points[2].at<cv::Vec3d>(v,u)=((mean_mat)+cv::Vec3d(A.col(1)));
            sigma_points[3].at<cv::Vec3d>(v,u)=((mean_mat)-cv::Vec3d(A.col(0)));
            sigma_points[4].at<cv::Vec3d>(v,u)=((mean_mat)-cv::Vec3d(A.col(1)));
            //std::cout << "x: "<<x << " y:" << y << " " << sigma_points[0].at<cv::Vec3d>(v,u) << << std::endl;
        }
    }
}

void Periphery::createDisparityWarpMaps()
{
    disparities.reserve(number_of_disparities);
    for(int i=min_disparity;i<number_of_disparities;++i)
    {
        disparities.push_back(i);
    }

    eta_cortical_warp.resize(disparities.size());
    rho_cortical_warp.resize(disparities.size());

    delta_eta_cortical_disparity_map.resize(disparities.size());
    delta_rho_cortical_disparity_map.resize(disparities.size());

    for(int d=0; d<disparities.size();++d)
    {
        eta_cortical_warp[d]=cv::Mat::zeros(S,R,CV_64FC1);
        rho_cortical_warp[d]=cv::Mat::zeros(S,R,CV_64FC1);

        delta_eta_cortical_disparity_map[d]=cv::Mat::zeros(S,R,CV_64FC1);
        delta_rho_cortical_disparity_map[d]=cv::Mat::zeros(S,R,CV_64FC1);
    }

    for(int s=0; s<S; ++s)
    {
        for(int r=0; r<R; ++r)
        {
            // Get x and y for each logpolar
            double x=(double)(ro0*pow(a,r)*cos(s/q)+center_width);
            double y=(double)(ro0*pow(a,r)*sin(s/q)+center_height);

            for(int d=0; d<disparities.size(); ++d)
            {
                double dx=disparities[d];
                double x_final=x-dx; // Check the sign...
                double y_final=y;
                // get eta, rho for each x disparity
                double eta;
                if(x_final>=center_width)
                    eta=atan((double)(y_final-center_height)/(double)(x_final-center_width));
                else
                    eta=atan((double)(y_final-center_height)/(double)(x_final-center_width))+CV_PI;

                if(eta<0)
                    eta+=2*CV_PI;
                eta_cortical_warp[d].at<double>(s,r)=(double)(q*eta);
                double ro2=(y_final-center_height)*(y_final-center_height)+(x_final-center_width)*(x_final-center_width);
                rho_cortical_warp[d].at<double>(s,r)=(double)(0.5*log(ro2/(ro0*ro0))/log(a));

                delta_eta_cortical_disparity_map[d].at<double>(s,r)=eta_cortical_warp[d].at<double>(s,r)-s;
                delta_rho_cortical_disparity_map[d].at<double>(s,r)=rho_cortical_warp[d].at<double>(s,r)-r;
            }
        }
    }

    return;
}

void Periphery::findWarpedCorrespondences(const cv::Mat & disparities_, cv::Mat & correspondences_map_1, cv::Mat & correspondences_map_2)
{
    // FIND CORRESPONDENCES
    correspondences_map_1=cv::Mat(S,R,CV_64F);
    correspondences_map_1.setTo(cv::Scalar(-1));
    correspondences_map_2=cv::Mat(S,R,CV_64F);
    correspondences_map_2.setTo(cv::Scalar(-1));

    for(int s=0; s<S; ++s)
    {
        for(int r=0; r<R; ++r)
        {
            int input_row=round((ro0*pow(a,r)*sin(s/q)+center_height));
            int input_column=round((ro0*pow(a,r)*cos(s/q)+center_width));

            /*if(input_column>disparities_.cols||input_row>disparities_.rows)
            {
                std::cout << "Bolas disparities_.cols:"<<disparities_.cols << " disparities_.rows:"<<disparities_.rows << " x: "<<x << " y:"<<y <<  std::endl;
                continue;
            }*/
            if(disparities_.at<double>(input_row,input_column)<0) // Outliers?
            {
                //std::cout << "OUTLIEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEERS?"<<  std::endl;
                continue;
            }
            int d=round(disparities_.at<double>(input_row,input_column)+fabs(min_disparity));

            if(input_column-d<0)
            {
                continue;
            }

            if(eta_cortical_warp[d].at<double>(s,r)>=S || rho_cortical_warp[d].at<double>(s,r)>=R)
                continue;

            correspondences_map_1.at<double>(s,r)=floor(eta_cortical_warp[d].at<double>(s,r));
            correspondences_map_2.at<double>(s,r)=floor(rho_cortical_warp[d].at<double>(s,r));
        }
    }
}

void Periphery::rectifySigmaPoints(const cv::Mat & H1,
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
        // Convert to cartesian

        cv::logPolar(
          sigma_points_1_rectified[i],
          aux1,
          cv::Point2f(aux1.cols*0.5f, aux1.rows*0.5f),
          M,
          cv::INTER_LINEAR | cv::WARP_INVERSE_MAP
        );

        cv::logPolar(
          sigma_points_1_rectified[i],
          aux2,
          cv::Point2f(aux1.cols*0.5f, aux1.rows*0.5f),
          M,
          cv::INTER_LINEAR | cv::WARP_INVERSE_MAP
        );

        //sigma_points_1_rectified[i]=to_cartesian(aux1);
        //sigma_points_2_rectified[i]=to_cartesian(aux2);

        // Rectify
        cv::remap(sigma_points_1_rectified[i], aux1, stereo_rectification_map1_left, stereo_rectification_map2_left, cv::INTER_LINEAR);
        cv::remap(sigma_points_2_rectified[i], aux2, stereo_rectification_map1_right, stereo_rectification_map2_right, cv::INTER_LINEAR);

        // Convert back to log polar

        cv::logPolar(
          sigma_points_1_rectified[i],
          aux1,
          cv::Point2f(aux1.cols*0.5f, aux1.rows*0.5f),
          M,
          cv::INTER_LINEAR | cv::WARP_FILL_OUTLIERS
        );

        cv::logPolar(
          sigma_points_1_rectified[i],
          aux2,
          cv::Point2f(aux1.cols*0.5f, aux1.rows*0.5f),
          M,
          cv::INTER_LINEAR | cv::WARP_FILL_OUTLIERS
        );


        //sigma_points_1_rectified[i]=to_cortical(aux1);
        //sigma_points_2_rectified[i]=to_cortical(aux2);
    }
}


