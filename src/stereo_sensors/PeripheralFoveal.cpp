#include "PeripheralFoveal.h"

PeripheralFoveal::PeripheralFoveal(const int & width_,
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
                                   const int & fovea_columns_,
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
    Uniform(fovea_rows_,
            fovea_columns_,
            height_,
            width_,
            _ro0),
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
    std::cout << "init peripheral foveal sensor..." << std::endl;
    stereo_data=StereoData(S+rows,R+columns,reference_frame);

    romax=(width_-1)/2.0;
    a=exp(log(center_width/ro0)/R);
    create_map(input_rows, input_columns, rows, columns, _R, _s, _ro0);

    for(int i=0;i<L+1;++i)
    {
        sigma_points[i]=cv::Mat(S+rows,R+columns, CV_64FC3);
        sigma_points_1_rectified[i]=cv::Mat(S+rows,R+columns,CV_64FC3);
        sigma_points_2_rectified[i]=cv::Mat(S+rows,R+columns,CV_64FC3);
    }

    computeSigmaPoints();
    createDisparityWarpMaps();

    for(int i=0;i<diff.size();++i)
        diff[i]=cv::Mat(rows,columns, CV_64FC3);
    std::cout << "done" << std::endl;
}

void PeripheralFoveal::computeSigmaPoints()
{
    ///////////////
    // Periphery //
    ///////////////

    // Compute sigma points for all peripherical RFs
    for(int v=0; v<S; ++v)
    {
        for(int u=0; u<R; ++u)
        {

            double x=(double)(ro0*pow(a,u+0.5)*cos((v+0.5)/q)+center_width);
            double y=(double)(ro0*pow(a,u+0.5)*sin((v+0.5)/q)+center_height);

            double sigma=scaling_factor*Wsr[u]*0.5/3.0;

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
        }
    }

    ///////////
    // FOVEA //
    ///////////

    // Compute sigma points for all foveal RFs
    for(int r=0; r<rows; ++r)
    {
        for(int c=0; c<columns; ++c)
        {
            double output_row=r;
            double output_column=c; // Check the sign...
            double input_row;
            double input_column;

            getInputSensorCoordinates(output_row,
                                      output_column,
                                      input_row,
                                      input_column);

            cv::Vec3d mean_mat;
            mean_mat(0)=(double)(input_column+0.5);
            mean_mat(1)=(double)(input_row+0.5);
            mean_mat(2)=(double)1.0;

            double sigma=scaling_factor*0.5*Wsr[0]/3.0;
            cv::Mat cov_mat=pow(sigma,2.0)*cv::Mat::eye(2,2,CV_64F);

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
            sigma_points[0].at<cv::Vec3d>(S+r,R+c)=(mean_mat);
            sigma_points[1].at<cv::Vec3d>(S+r,R+c)=((mean_mat)+cv::Vec3d(A.col(0)));
            sigma_points[2].at<cv::Vec3d>(S+r,R+c)=((mean_mat)+cv::Vec3d(A.col(1)));
            sigma_points[3].at<cv::Vec3d>(S+r,R+c)=((mean_mat)-cv::Vec3d(A.col(0)));
            sigma_points[4].at<cv::Vec3d>(S+r,R+c)=((mean_mat)-cv::Vec3d(A.col(1)));
        }
    }
}

void PeripheralFoveal::createDisparityWarpMaps()
{
    disparities.reserve(number_of_disparities);
    for(int i=min_disparity;i<number_of_disparities;++i)
    {
        disparities.push_back(i);
    }

    column_disparity_warp.resize(disparities.size());
    row_disparity_warp.resize(disparities.size());

    delta_column_disparity_warp.resize(disparities.size());
    delta_row_disparity_warp.resize(disparities.size());

    for(int d=0; d<disparities.size();++d)
    {
        column_disparity_warp[d]=cv::Mat::zeros(S+rows,R+columns,CV_64FC1);
        row_disparity_warp[d]=cv::Mat::zeros(S+rows,R+columns,CV_64FC1);

        delta_column_disparity_warp[d]=cv::Mat::zeros(S+rows,R+columns,CV_64FC1);
        delta_row_disparity_warp[d]=cv::Mat::zeros(S+rows,R+columns,CV_64FC1);
    }

    // Compute disparity warp maps for all peripherical RFs
    for(int s=0; s<S; ++s)
    {
        for(int r=0; r<R; ++r)
        {
            // Get input x and y coordinates for each logpolar RF
            double input_column=(double)(ro0*pow(a,r+0.5)*cos((s+0.5)/q)+center_width);
            double input_row=(double)(ro0*pow(a,r)*sin(s/q)+center_height);

            for(int d=0; d<disparities.size(); ++d)
            {
                // Get input sensor coordinates after disparity
                double input_dx=disparities[d];
                double input_row_final=input_row+0.5;
                double input_column_final=input_column+0.5-input_dx;

                // Get output cartesian sensor coordinates
                if(checkInsideSensor(input_row_final,input_column_final))
                {
                    // CASE MATCHES A FOVEAL RF
                    getOutputSensorCoordinates(input_row_final,
                                               input_column_final,
                                               row_disparity_warp[d].at<double>(s,r),
                                               column_disparity_warp[d].at<double>(s,r));
                }
                else
                {
                    // CASE MATCHES A PERIPHERICAL RF
                    // get eta, rho for each x disparity
                    double eta;
                    if(input_column_final>=center_width)
                        eta=atan((double)(input_row_final-center_height)/(double)(input_column_final-center_width));
                    else
                        eta=atan((double)(input_row_final-center_height)/(double)(input_column_final-center_width))+CV_PI;

                    if(eta<0)
                        eta+=2*CV_PI;

                    row_disparity_warp[d].at<double>(s,r)=(double)(q*eta);
                    double ro2=(input_row_final-center_height)*(input_row_final-center_height)+(input_column_final-center_width)*(input_column_final-center_width);
                    column_disparity_warp[d].at<double>(s,r)=(double)(0.5*log(ro2/(ro0*ro0))/log(a));

                    delta_row_disparity_warp[d].at<double>(s,r)=column_disparity_warp[d].at<double>(s,r)-s;
                    delta_column_disparity_warp[d].at<double>(s,r)=row_disparity_warp[d].at<double>(s,r)-r;
                }
            }
        }
    }

    // Compute disparity warp maps for all foveal RFs
    for(int r=0; r<rows; ++r)
    {
        for(int c=0; c<columns; ++c)
        {
            for(int d=0; d<disparities.size(); ++d)
            {
                // Get output sensor coordinates after disparity
                double input_dx=disparities[d];
                double output_dx=input_dx/column_scale;
                double output_row_final=r+0.5;
                double output_column_final=c+0.5-output_dx; // Check the sign...
                double input_row_final;
                double input_column_final;

                getInputSensorCoordinates(output_row_final,
                                          output_column_final,
                                          input_row_final,
                                          input_column_final);

                if(checkInsideSensor(input_row_final,input_column_final))
                {
                    // CASE MATCHES A FOVEAL RF
                    row_disparity_warp[d].at<double>(S+r,R+c)=output_row_final;
                    column_disparity_warp[d].at<double>(S+r,R+c)=output_column_final;
                }
                else
                {
                    // CASE MATCHES A PERIPHERICAL RF

                    // get eta, rho for each x disparity
                    double eta;
                    if(input_column_final>=center_width)
                        eta=atan((double)(input_row_final-center_height)/(double)(input_column_final-center_width));
                    else
                        eta=atan((double)(input_row_final-center_height)/(double)(input_column_final-center_width))+CV_PI;

                    if(eta<0)
                        eta+=2*CV_PI;

                    row_disparity_warp[d].at<double>(S+r,R+c)=(double)(q*eta);
                    double ro2=(input_row_final-center_height)*(input_row_final-center_height)+(input_column_final-center_width)*(input_column_final-center_width);
                    column_disparity_warp[d].at<double>(S+r,R+c)=(double)(0.5*log(ro2/(ro0*ro0))/log(a));

                    //delta_column_disparity_warp[d].at<double>(s,r)=column_disparity_warp[d].at<double>(s,r)-s;
                    //delta_row_disparity_warp[d].at<double>(s,r)=row_disparity_warp[d].at<double>(s,r)-r;
                }
            }
        }
    }

    return;
}

void PeripheralFoveal::findWarpedCorrespondences(const cv::Mat & disparities_, cv::Mat & correspondences_map_1, cv::Mat & correspondences_map_2)
{
    // FIND CORRESPONDENCES
    correspondences_map_1=cv::Mat(S+rows,R+columns,CV_64F);
    correspondences_map_1.setTo(cv::Scalar(-1));
    correspondences_map_2=cv::Mat(S+rows,R+columns,CV_64F);
    correspondences_map_2.setTo(cv::Scalar(-1));

    for(int s=0; s<S; ++s)
    {
        for(int r=0; r<R; ++r)
        {
            // Get input sensor cartesian coordinates
            int input_row=round((ro0*pow(a,r+0.5)*sin((s+0.5)/q)+center_height));
            int input_column=round((ro0*pow(a,r+0.5)*cos((s+0.5)/q)+center_width));
            /*if(input_column>disparities_.cols||input_row>disparities_.rows)
            {
                std::cout << "Bolas disparities_.cols:"<<disparities_.cols << " disparities_.rows:"<<disparities_.rows << " x: "<<x << " y:"<<y <<  std::endl;
                continue;
            }*/

            if(disparities_.at<double>(input_row,input_column)<0) // Outliers
            {
                continue;
            }

            int d=round(disparities_.at<double>(input_row,input_column)+fabs(min_disparity));

            if(input_column-d<0) // Means correspondence is out of image... impossible
            {
                continue;
            }

            if(row_disparity_warp[d].at<double>(s,r)>=S || column_disparity_warp[d].at<double>(s,r)>=R)
                continue;

            correspondences_map_1.at<double>(s,r)=floor(row_disparity_warp[d].at<double>(s,r));
            correspondences_map_2.at<double>(s,r)=floor(column_disparity_warp[d].at<double>(s,r));
        }
    }

    for(int r=0; r<rows; ++r)
    {
        for(int c=0; c<columns; ++c)
        {
            // Get input sensor cartesian coordinates
            double output_row=(double)r+0.5; // Check the sign...
            double output_column=(double)c+0.5;
            double input_row;
            double input_column;

            getInputSensorCoordinates(output_row,
                                      output_column,
                                      input_row,
                                      input_column);

            if(disparities_.at<double>(input_row,input_column)<0) // Outliers
            {
                continue;
            }

            int d=round(disparities_.at<double>(input_row,input_column)+fabs(min_disparity));

            if(input_column-d<0) // Means correspondence is out of image... impossible
            {
                std::cout << "WHAAAAAAAAAAAAAAAAAAA?"<<  std::endl;
                continue;
            }

            correspondences_map_1.at<double>(S+r,R+c)=floor(column_disparity_warp[d].at<double>(S+r,R+c));
            correspondences_map_2.at<double>(S+r,R+c)=floor(row_disparity_warp[d].at<double>(S+r,R+c));
        }
    }


}

void PeripheralFoveal::rectifySigmaPoints(const cv::Mat & H1,
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
        sigma_points_1_rectified[i]=to_cartesian(aux1);
        sigma_points_2_rectified[i]=to_cartesian(aux2);

        // Rectify
        cv::remap(sigma_points_1_rectified[i], aux1, stereo_rectification_map1_left, stereo_rectification_map2_left, cv::INTER_LINEAR);
        cv::remap(sigma_points_2_rectified[i], aux2, stereo_rectification_map1_right, stereo_rectification_map2_right, cv::INTER_LINEAR);

        // Convert back to log polar
        sigma_points_1_rectified[i]=to_cortical(aux1);
        sigma_points_2_rectified[i]=to_cortical(aux2);
    }
}



void PeripheralFoveal::create_map(const int & input_rows,
                                  const int & input_columns,
                                  const int & output_rows_,
                                  const int & output_columns_,
                                  const int & _R,
                                  const int & _s,
                                  const double & _ro0)
{
    R=_R;
    S=_s;
    ro0=_ro0;
    ind1=0;

    romax=std::min(center_height, center_width);
    a=exp(log((double)romax/(double)ro0)/(double)R);
    q=((double)S)/(2*CV_PI);

    Rsri = cv::Mat::zeros(S+output_rows_,R+output_columns_,CV_32FC1);
    Csri = cv::Mat::zeros(S+output_rows_,R+output_columns_,CV_32FC1);
    ETAyx = cv::Mat::zeros(input_rows,input_columns,CV_32FC1);
    CSIyx = cv::Mat::zeros(input_rows,input_columns,CV_32FC1);


    Rsr.resize(R*S);
    Csr.resize(R*S);
    Wsr.resize(R);
    w_ker_2D.resize(R*S);

    // Peripherical RFs
    for(int v=0; v<S; v++)
    {
        for(int u=0; u<R; u++)
        {
            Rsri.at<float>(v,u)=(float)(ro0*pow(a,u)*sin(v/q)+center_height);
            Csri.at<float>(v,u)=(float)(ro0*pow(a,u)*cos(v/q)+center_width);
            Rsr[v*R+u]=(int)floor(Rsri.at<float>(v,u));
            Csr[v*R+u]=(int)floor(Csri.at<float>(v,u));
        }
    }

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

    for(int r=0; r<rows; ++r)
    {
        for(int c=0; c<columns; ++c)
        {
            // Get output sensor coordinates
            double output_row=(double)r+0.5;
            double output_column=(double)c+0.5;
            double input_row;
            double input_column;

            // Foveal RFs
            getInputSensorCoordinates(output_row,
                                      output_column,
                                      input_row,
                                      input_column);

            //if(checkInsideSensor(input_row,input_column))
            {
                // CASE MATCHES A FOVEAL RF
                Rsri.at<float>(S+r,R+c)=(float)input_row;
                Csri.at<float>(S+r,R+c)=(float)input_column;
            }

        }
    }

    for(int r=0; r<input_rows; ++r)
    {
        for(int c=0; c<input_columns; ++c)
        {
            if(checkInsideSensor(r,c))
            {
                double output_row;
                double output_column;
                getOutputSensorCoordinates(r,
                                           c,
                                           output_row,
                                           output_column);

                ETAyx.at<float>(r,c)=(float)output_row+(float)S;
                CSIyx.at<float>(r,c)=(float)output_column+(float)R;
            }
            else
            {
                double ro2=(r+0.5-center_height)*(r+0.5-center_height)+(c+0.5-center_width)*(c+0.5-center_width);

                double theta;
                if(c>=center_width)
                    theta=atan((double)(r-center_height)/(double)(c-center_width));
                else
                    theta=atan((double)(r-center_height)/(double)(c-center_width))+CV_PI;

                if(theta<0)
                    theta+=2*CV_PI;

                ETAyx.at<float>(r,c)=(float)(q*theta);


                CSIyx.at<float>(r,c)=(float)(0.5*log(ro2/(ro0*ro0))/log(a));
            }
        }
    }

    // Initialize averaging mask
    for(int v=0; v<S; v++)
        for(int u=ind1; u<R; u++)
        {
            //double sigma=Wsr[u]/2.0;
            double sigma=Wsr[u]/3.0;//modf
            int w=(int) floor(3*sigma+0.5);
            w_ker_2D[v*R+u].w=w;
            w_ker_2D[v*R+u].weights.resize((2*w+1)*(2*w+1));
            double dx=Csri.at<float>(v,u)-Csr[v*R+u];
            double dy=Rsri.at<float>(v,u)-Rsr[v*R+u];
            double tot=0;
            for(int j=0; j<2*w+1; j++)
                for(int i=0; i<2*w+1; i++)
                {
                    (w_ker_2D[v*R+u].weights)[j*(2*w+1)+i]=exp(-(pow(i-w-dx, 2)+pow(j-w-dy, 2))/(2*sigma*sigma));
                    tot+=(w_ker_2D[v*R+u].weights)[j*(2*w+1)+i];
                }
            for(int j=0; j<(2*w+1); j++)
                for(int i=0; i<(2*w+1); i++)
                    (w_ker_2D[v*R+u].weights)[j*(2*w+1)+i]/=tot;
        }

}


const cv::Mat PeripheralFoveal::to_cortical(const cv::Mat &source)
{
    cv::Mat out(S,R,CV_8UC3,cv::Scalar(0));

    cv::Mat source_border;
    copyMakeBorder(source,source_border,top,bottom,left,right,cv::BORDER_CONSTANT,cv::Scalar(0));

    remap(source_border,out,Csri,Rsri,cv::INTER_LINEAR);

    /*int wm=w_ker_2D[R-1].w;
    std::vector<int> IMG((input_columns+2*wm+1)*(input_rows+2*wm+1), 0);

    for(int j=0; j<input_rows; j++)
        for(int i=0; i<input_columns; i++)
            IMG[(input_columns+2*wm+1)*(j+wm)+i+wm]=source_border.at<uchar>(j,i);

    std::cout << "ind1:"<< ind1 << std::endl;

    for(int v=0; v<S; v++)
        for(int u=ind1; u<R; u++)
        {
            int w=w_ker_2D[v*R+u].w;
            double tmp=0;
            for(int rf=0; rf<(2*w+1); rf++)
            {
                for(int cf=0; cf<(2*w+1); cf++)
                {
                    double weight=(w_ker_2D[v*R+u]).weights[rf*(2*w+1)+cf];
                    tmp+=IMG[(input_columns+2*wm+1)*((rf-w)+Rsr[v*R+u]+wm)+((cf-w)+Csr[v*R+u]+wm)]*weight;
                }
            }
            out.at<uchar>(v,u)=(uchar) floor(tmp+0.5);
        }*/

    return out;
}
/*
const cv::Mat PeripheralFoveal::to_cartesian(const cv::Mat &source)
{
    cv::Mat out(N,M,CV_8UC1,cv::Scalar(0));

    cv::Mat source_border;
    copyMakeBorder(source,source_border,0,1,0,0,cv::BORDER_CONSTANT,cv::Scalar(0));
    cv::Mat rowS = source_border.row(S);
    source_border.row(0).copyTo(rowS);
    remap(source_border,out,CSIyx,ETAyx,cv::INTER_LINEAR);

    int wm=w_ker_2D[R-1].w;

    std::vector<double> IMG((N+2*wm+1)*(M+2*wm+1), 0.);
    std::vector<double> NOR((N+2*wm+1)*(M+2*wm+1), 0.);

    for(int v=0; v<S; v++)
        for(int u=ind1; u<R; u++)
        {
            int w=w_ker_2D[v*R+u].w;
            for(int j=0; j<(2*w+1); j++)
            {
                for(int i=0; i<(2*w+1); i++)
                {
                    int ind=(M+2*wm+1)*((j-w)+Rsr[v*R+u]+wm)+(i-w)+Csr[v*R+u]+wm;
                    IMG[ind]+=((w_ker_2D[v*R+u]).weights[j*(2*w+1)+i])*source.at<uchar>(v, u);
                    NOR[ind]+=((w_ker_2D[v*R+u]).weights[j*(2*w+1)+i]);
                }
            }
        }

    for(int i=0; i<((N+2*wm+1)*(M+2*wm+1)); i++)
        IMG[i]/=NOR[i];

    //int xc=M/2-1, yc=N/2-1;

    for(int j=wm; j<N+wm; j++)
        for(int i=wm; i<M+wm; i++)
        {

            //int ro=(int)floor(sqrt((double)((j-wm-yc)*(j-wm-yc)+(i-wm-xc)*(i-wm-xc))));
            int csi=(int) floor(CSIyx.at<float>(j-wm,i-wm));

            if((csi>=(ind1-(w_ker_2D[ind1]).w))&&(csi<R))
                out.at<uchar>(j-wm,i-wm)=(uchar) floor(IMG[(M+2*wm+1)*j+i]+0.5);
        }

    cv::Mat out_cropped=out(cv::Range(top,N-1-bottom),cv::Range(left,M-1-right));
    return out_cropped;
}
*/
