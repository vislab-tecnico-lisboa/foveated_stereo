/*M///////////////////////////////////////////////////////////////////////////////////////
 * //
 * //  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 * //
 * //  By downloading, copying, installing or using the software you agree to this license.
 * //  If you do not agree to this license, do not download, install,
 * //  copy or use the software.
 * //
 * //
 * //                           License Agreement
 * //                For Open Source Computer Vision Library
 * //
 * // Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
 * // Copyright (C) 2009, Willow Garage Inc., all rights reserved.
 * // Third party copyrights are property of their respective owners.
 * //
 * // Redistribution and use in source and binary forms, with or without modification,
 * // are permitted provided that the following conditions are met:
 * //
 * //   * Redistribution's of source code must retain the above copyright notice,
 * //     this list of conditions and the following disclaimer.
 * //
 * //   * Redistribution's in binary form must reproduce the above copyright notice,
 * //     this list of conditions and the following disclaimer in the documentation
 * //     and/or other materials provided with the distribution.
 * //
 * //   * The name of the copyright holders may not be used to endorse or promote products
 * //     derived from this software without specific prior written permission.
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

#ifndef __LOG_POLAR_STEREO__
#define __LOG_POLAR_STEREO__

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include <cmath>
#include <vector>
#include "opencv2/contrib/contrib.hpp"

#include <iostream>
#include <vector>
#ifdef __cplusplus
using namespace cv;

/**
 *Overlapping circular receptive fields technique
 *
 *The Cartesian plane is divided in two regions: the fovea and the periphery.
 *The fovea (oversampling) is handled by using the bilinear interpolation technique described above, whereas in
 *the periphery we use the overlapping Gaussian circular RFs.
 *
 *More details can be found in http://dx.doi.org/10.1007/978-3-642-23968-7_5
 */
class LogPolar : public cv::LogPolar_Interp
{
public:
    
    
    LogPolar() {}
            
    LogPolar(int w, int h, Point2i center, int _R, double _ro0, 
            int _interp, int full, int _s, int sp, std::vector<double> & _disparities, float & _sigma, float & _q_like, float & _pole, bool _high_pass=true);

    void createLogpolarWarpMaps(const std::vector<double> &disparities);
    void createCartesianLogpolarWarpMaps(const std::vector<double> &disparities);
   
   	void warpLogPolar(const Mat & source, Mat & destiny, const int &disparity_index);
    void warpCartesianLogPolar(const Mat &source, Mat & destiny, const int &disparity_index);
    Mat computeDisparityMap(const Mat &source_left, const Mat &source_right);
    
    /////////////
    // Filters //
    /////////////
    void PolarFiltFilt2(const Mat &source, Mat &output, float pole, int edge, int wrap);
    void disparityFilter(std::vector<Mat> & disp);

    //////////////
    // Get maps //
    //////////////
    
    std::vector<Mat> getLogPolarEtaWarpMaps();
    std::vector<Mat> getLogPolarRhoWarpMaps();
    std::vector<Mat> getDeltaLogPolarEtaWarpMaps();
    std::vector<Mat> getDeltaLogPolarRhoWarpMaps();
    
    /**
     *Destructor
     */
    ~LogPolar();
    
protected:

    std::vector<double> disparities;
    
    std::vector<Mat> eta_cortical_warp;
    std::vector<Mat> rho_cortical_warp;

    std::vector<Mat> delta_eta_cortical_disparity_map;
    std::vector<Mat> delta_rho_cortical_disparity_map;
    
    std::vector<Mat> x_foveal_to_cortical_warp;
    std::vector<Mat> y_foveal_to_cortical_warp;
    std::vector<Mat> likelihood_images;
    float occlusion_likelihood;
    float sigma;
    float pole;
    float q_like;
    bool high_pass;
    
    void create_map(int M, int N, int R, int S, double ro0);
};



#endif

#endif

