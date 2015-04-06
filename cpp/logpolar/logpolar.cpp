/*M///////////////////////////////////////////////////////////////////////////////////////
 * //
 * // IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 * //
 * // By downloading, copying, installing or using the software you agree to this license.
 * // If you do not agree to this license, do not download, install,
 * // copy or use the software.
 * //
 * //
 * // License Agreement
 * // For Open Source Computer Vision Library
 * //
 * // Copyright (C) 2012, Willow Garage Inc., all rights reserved.
 * // Third party copyrights are property of their respective owners.
 * //
 * // Redistribution and use in source and binary forms, with or without modification,
 * // are permitted provided that the following conditions are met:
 * //
 * // * Redistribution's of source code must retain the above copyright notice,
 * // this list of conditions and the following disclaimer.
 * //
 * // * Redistribution's in binary form must reproduce the above copyright notice,
 * // this list of conditions and the following disclaimer in the documentation
 * // and/or other materials provided with the distribution.
 * //
 * // * The names of the copyright holders may not be used to endorse or promote products
 * // derived from this software without specific prior written permission.
 * //
 * // This software is provided by the copyright holders and contributors "as is" and
 * // any express or implied warranties, including, but not limited to, the implied
 * // warranties of merchantability and fitness for a particular purpose are disclaimed.
 * // In no event shall the Intel Corporation or contributors be liable for any direct,
 * // indirect, incidental, special, exemplary, or consequential damages
 * // (including, but not limited to, procurement of substitute goods or services;
 * // loss of use, data, or profits; or business interruption) however caused
 * // and on any theory of liability, whether in contract, strict liability,
 * // or tort (including negligence or otherwise) arising in any way out of
 * // the use of this software, even if advised of the possibility of such damage.
 * //
 * //M*/
/*******************************************************************************************
 * The LogPolar Blind Spot Model code has been contributed by Fabio Solari and Manuela Chessa.
 * More details can be found in:
 * M. Chessa, S. P. Sabatini, F. Solari and F. Tatti (2011)
 * A Quantitative Comparison of Speed and Reliability for Log-Polar Mapping Techniques,
 * Computer Vision Systems - 8th International Conference,
 * ICVS 2011, Sophia Antipolis, France, September 20-22, 2011
 * (http://dx.doi.org/10.1007/978-3-642-23968-7_5)
 ********************************************************************************************/
#include "logpolar.hpp"


using namespace cv;

LogPolar::LogPolar(int w, int h, Point2i center, int _R, double _ro0, int _interp, int full, int _s, int sp, std::vector<double> & _disparities, float & _sigma, float & _q_like, float & _pole, bool _high_pass): 
        cv::LogPolar_Interp(w, h, center, _R, _ro0, _interp, full, _s, sp),
        disparities(_disparities), 
        sigma(_sigma), 
        q_like(_q_like),
        pole(_pole),
        high_pass(_high_pass)
{
    createLogpolarWarpMaps(disparities);
    createCartesianLogpolarWarpMaps(disparities);
    /*for(int i=0; i< disparities.size() ; ++i)
    {
        
        std::cout << "disparity "<<i<<": " <<disparities[i] << std::endl;
    }*/
    float M=256.0;
    
    occlusion_likelihood=q_like*disparities.size()/(M-q_like*M);
    //std::cout << "disparities size:" << disparities.size() << std::endl;
    //std::cout <<"occlusion_likelihood:"<<occlusion_likelihood<< std::endl;
    // disparity 0 = Uniform
    likelihood_images.resize(disparities.size()+1);
    likelihood_images[0]=cv::Mat(S,R, CV_32FC1);    
    likelihood_images[0].setTo(cv::Scalar(occlusion_likelihood));
    
    for(int d=1; d<likelihood_images.size(); ++d)
    {
        // Allocate
        likelihood_images[d]=Mat::zeros(S,R, CV_32FC1); 
    }
}

void LogPolar::createLogpolarWarpMaps(const std::vector<double> &disparities)
{
    eta_cortical_warp.resize(disparities.size());
    rho_cortical_warp.resize(disparities.size());
    
    delta_eta_cortical_disparity_map.resize(disparities.size());
    delta_rho_cortical_disparity_map.resize(disparities.size());
    
    for(int d=0; d<disparities.size();++d)
    {
        eta_cortical_warp[d]=Mat::zeros(S,R,CV_32FC1);
        rho_cortical_warp[d]=Mat::zeros(S,R,CV_32FC1);
        
        delta_eta_cortical_disparity_map[d]=Mat::zeros(S,R,CV_32FC1);
        delta_rho_cortical_disparity_map[d]=Mat::zeros(S,R,CV_32FC1);
    }
    
    int jc=N/2-1, ic=M/2-1;
    for(int s=0; s<S; ++s)
    {
        for(int r=0; r<R; ++r)
        {
            // Get x and y for each logpolar
            double y=(float)(ro0*pow(a,r)*sin(s/q)+jc);
            double x=(float)(ro0*pow(a,r)*cos(s/q)+ic);
            
            for(int d=0; d<disparities.size(); ++d)
            {
                double dx=disparities[d];
                double x_final=x+dx; // Check the sign...
                double y_final=y;
                // get eta, rho for each x disparity
                double theta;
                if(x_final>=ic)
                    theta=atan((double)(y_final-jc)/(double)(x_final-ic));
                else
                    theta=atan((double)(y_final-jc)/(double)(x_final-ic))+CV_PI;
                if(theta<0)
                    theta+=2*CV_PI;
                eta_cortical_warp[d].at<float>(s,r)=(float)(q*theta);
                double ro2=(y_final-jc)*(y_final-jc)+(x_final-ic)*(x_final-ic);
                rho_cortical_warp[d].at<float>(s,r)=(float)(0.5*log(ro2/(ro0*ro0))/log(a));
                
                delta_eta_cortical_disparity_map[d].at<float>(s,r)=eta_cortical_warp[d].at<float>(s,r)-s;
                delta_rho_cortical_disparity_map[d].at<float>(s,r)=rho_cortical_warp[d].at<float>(s,r)-r;
            }
        }
    }
    
    return;
}

void LogPolar::createCartesianLogpolarWarpMaps(const std::vector<double> &disparities)
{
    x_foveal_to_cortical_warp.resize(disparities.size());
    y_foveal_to_cortical_warp.resize(disparities.size());
    
    for(int d=0; d<disparities.size();++d)
    {
        x_foveal_to_cortical_warp[d]=Mat::zeros(S,R,CV_32FC1);
        y_foveal_to_cortical_warp[d]=Mat::zeros(S,R,CV_32FC1);
    }
    
    int jc=N/2-1, ic=M/2-1;
    
    // cartesian to cortical map
    for(int d=0; d<disparities.size(); ++d)
    {
        double dx=disparities[d];
        
        for(int v=0; v<S; v++)
        {
            for(int u=0; u<R; u++)
            {
                x_foveal_to_cortical_warp[d].at<float>(v,u)=(float)(ro0*pow(a,u)*cos(v/q)+ic+dx);
                y_foveal_to_cortical_warp[d].at<float>(v,u)=(float)(ro0*pow(a,u)*sin(v/q)+jc);
            }
        }
    }
    
    return;
}

// Both source and destiny in LogPolar coordinates
void LogPolar::warpLogPolar(const Mat &source, Mat & destiny, const int &disparity_index)
{
    // remap image according to disparity index
    destiny=Mat(S,R,CV_8UC1,Scalar(0));
    remap(source, destiny, rho_cortical_warp[disparity_index],eta_cortical_warp[disparity_index],interp);
}

// Source cartesian, destiny logpolar
void LogPolar::warpCartesianLogPolar(const Mat &source, Mat & destiny, const int &disparity_index)
{
    // remap image according to disparity index
    destiny=Mat(S,R,CV_8UC1,Scalar(0));
    Mat source_border;
    copyMakeBorder(source,source_border,top,bottom,left,right,BORDER_CONSTANT,Scalar(0));
    
    remap(source_border, destiny, x_foveal_to_cortical_warp[disparity_index], y_foveal_to_cortical_warp[disparity_index],interp);
}

// ATTENTION: sources are logpolar
Mat LogPolar::computeDisparityMap(const Mat &source_left, const Mat &source_right)
{
    ////////////////////////////////////////
    // Compute cortical likelihood images //
    ////////////////////////////////////////
    
    likelihood_images[0].setTo(cv::Scalar(occlusion_likelihood));
    
    for(int d=1; d<likelihood_images.size(); ++d)
    {
        // Reset
        likelihood_images[d].setTo(cv::Scalar(0));

        // remap image according to disparity index
        Mat right_image_transf=Mat::zeros(S,R,CV_8UC1);
        warpLogPolar(source_right,right_image_transf ,d-1);
        
        // subtract one image from the other
        Mat disparity_map(S,R,CV_32FC1);
        Mat left_image_double;
        Mat right_image_transf_double;
        source_left.convertTo(left_image_double, CV_32FC1);
        right_image_transf.convertTo(right_image_transf_double, CV_32FC1);
        
        disparity_map=left_image_double-right_image_transf_double;
        
        // Compute likelihood image
        for(int s=0; s<S; ++s)
        {
            for(int r=0;r<R; ++r)
            {
                //float pdf_gaussian = ( 1.0 / ( sigma * sqrt(2*M_PI) ) ) * exp( -0.5 * pow( (disparity_map.at<float>(s,r))/sigma, 2.0 ) );
                float pdf_gaussian = ( 1.0 / ( sigma * sqrt(2*M_PI) ) ) * exp( -0.5 * pow( (disparity_map.at<float>(s,r))/sigma, 2.0 ) );
                //float disparity_value=disparity_map.at< cv::Vec<float,disparities.size()> >(i,j)[d];
                
                likelihood_images[d].at<float>(s,r)=pdf_gaussian;
                //std::cout << "intensity:" << disparity_map.at<float>(i,j) << "  prob:" << pdf_gaussian << std::endl;
            }
        }
    }

    //////////////////////////////////////
    // Apply first order spatial filter //
    //////////////////////////////////////
    //std::cout << "POLE:" << pole << std::endl;
    // Compute facilitated likelihood images
    for(int d=0; d < likelihood_images.size(); ++d)
    {
        Mat out;
        PolarFiltFilt2(likelihood_images[d], out, pole, 2, 2);
        likelihood_images[d]=out;
    }
    
    ///////////////////////////////////////////////////
    // Apply high pass filter on disparity dimension //
    //////////////////////////////////////////////////
    std::cout << "high_pass:"<< high_pass << std::endl;
    if(high_pass)
    {
        std::cout << "HIGH PASS FILTER!!" << std::endl;
        disparityFilter(likelihood_images);
    }
    
    ///////////////
    // Normalize //
    ///////////////
    
    /*for(int s=0; s<S; ++s)
    {
        for(int r=0;r<R; ++r)
        {
            float sum=0.0;

            for(int d=0; d < likelihood_images.size(); ++d)
            {
            	sum+=likelihood_images[d].at<float>(s,r);
            }
            
            //std::cout << "sum before:" << sum << " occ like:" << occlusion_likelihood<< std::endl;
            float sum_after=0.0;
            for(int d=0; d < likelihood_images.size(); ++d)
            {
            	likelihood_images[d].at<float>(s,r)/=sum;
                //sum_after+=likelihood_images[d].at<float>(s,r);
            }
            //std::cout << "sum after:" << sum_after << std::endl;
        }
    }//*/
    //std::cout << "likelihood_images[0]:"<< likelihood_images[0].at<float>(0,0) << std::endl;
    

    //////////////////////////////////////////////////////
    // Compute cortical disparity activation images map //
    //////////////////////////////////////////////////////
    
    Mat disparity_map=Mat::zeros(S,R,CV_32FC1);

    for(int s=0; s<S;++s)
    {
        for(int r=0;r<R;++r)
        {
            // Get disparity with maximum likelihood for pixel i,j
            
            //Start with occlusion probability
            float max_likelihood=likelihood_images[0].at<float>(s,r);
            disparity_map.at<float>(s,r)=1000.0;
            
            for(int d=1; d<likelihood_images.size(); ++d)
            {
                if((float)likelihood_images[d].at<float>(s,r)>max_likelihood)
                {
                    max_likelihood=(float)likelihood_images[d].at<float>(s,r);
                    disparity_map.at<float>(s,r)=disparities[d-1];
                }
            }
        }
    }

    //std::cout << "disp_map[0]:"<< disparity_map.at<float>(0,0) << std::endl;

    //std::cout << "disparity map: " << disparity_map << std::endl;
    return disparity_map;
}

std::vector<Mat> LogPolar::getLogPolarEtaWarpMaps()
{
    return eta_cortical_warp;
    
}

std::vector<Mat> LogPolar::getLogPolarRhoWarpMaps()
{
    return rho_cortical_warp;
    
}

std::vector<Mat> LogPolar::getDeltaLogPolarEtaWarpMaps()
{
    return delta_eta_cortical_disparity_map;
}

std::vector<Mat> LogPolar::getDeltaLogPolarRhoWarpMaps()
{
    return delta_rho_cortical_disparity_map;
}

LogPolar::~LogPolar()
{
}

// Log polar!!
void LogPolar::PolarFiltFilt2(const Mat &source, Mat &output, float pole, int edge, int wrap)
{
    float a = pole;
    float b = 1-pole;
    float state;
    
    ///////////////////////
    // Filter per column //
    ///////////////////////
    
    // Wrap border to avoid angular discontinuities
    Mat source_border;
    copyMakeBorder(source, source_border, wrap, wrap, 0, 0, BORDER_WRAP, Scalar(0));

    for(int u=0; u<R; ++u)
    { 
        //forward filtering
        state = source_border.at<float>(0,u);
        for(int v=0; v<S+2*wrap; ++v)
        {
            state=b*source_border.at<float>(v,u)+a*state;
            source_border.at<float>(v,u) = state;
        }
        
        //backward filtering
        state = source_border.at<float>(S-1+2*wrap,u);
        for(int v = S-1+2*wrap; v >= 0; --v)
        {
            state=b*source_border.at<float>(v,u)+a*state;
            source_border.at<float>(v,u) = state;
        }
    }

    Rect lines_roi = Rect(0, wrap, source.cols, source.rows);
    Mat lines_filtered = source_border(lines_roi);
 
    /////////////////////
    // Filter per line //
    /////////////////////
      
    // Wrap border to avoid angular discontinuities
    Mat source_border_cols;
    copyMakeBorder(lines_filtered, source_border_cols, 0, 0, edge, edge, BORDER_REFLECT, Scalar(0));
    for(int v=0; v<S; ++v)
    {
        //forward filtering
        state = source_border_cols.at<float>(v,0);
        for(int u=0; u<R+2*edge; ++u)
        { 
            state=b*source_border_cols.at<float>(v,u)+a*state;
            source_border_cols.at<float>(v,u) = state;
        }
        
        //backward filtering
        state = source_border_cols.at<float>(v,R-1+2*edge);
        for(int u = R-1+2*edge; u >= 0; --u)
        {
            state=b*source_border_cols.at<float>(v,u)+a*state;
            source_border_cols.at<float>(v,u) = state;
        }
    }
    
    Rect cols_roi = Rect(edge, 0, source.cols, source.rows);
    Mat cols_filtered = source_border_cols(cols_roi);
    
    output=cols_filtered;

    return;
        
}

void LogPolar::disparityFilter(std::vector<Mat> & disp)
{

    // Wrap border to avoid angular discontinuities
    std::vector<float> kernel;
    kernel.push_back( -1.0);
    kernel.push_back( -1.0);
    kernel.push_back( 4.0);
    kernel.push_back( -1.0);
    kernel.push_back( -1.0);

    ////////////////////////////
    // sobel high pass filter //
    ////////////////////////////
    
    int border=2;
    for(int v=0; v<S; ++v)
    {
        for(int u=0; u<R; ++u)
        { 
            // ACCOUNT TO BORDER
            for (int d = border; d < disp.size()-border; ++d) 
            {
                disp[d].at<float>(v,u)=kernel[0]*disp[d-2].at<float>(v,u)+
                                       kernel[1]*disp[d-1].at<float>(v,u)+
                                       kernel[2]*disp[d]  .at<float>(v,u)+
                                       kernel[3]*disp[d+1].at<float>(v,u);
                                       kernel[4]*disp[d+2].at<float>(v,u);

            }
        }
    }
    
    return;
}

