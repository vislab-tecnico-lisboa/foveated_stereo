
#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "logpolar/logpolar.hpp"
#include "mex.h"
#include "foveal_handle.hpp"
#include "mc_convert/mc_convert.hpp"
// The class that we are interfacing to

#include "logpolar/logpolar.hpp"
//
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // Get the command string
    char cmd[64];
    if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
        mexErrMsgTxt("First input should be a command string less than 64 characters long.");
    
    // New
    if (!strcmp("new", cmd))
    {
        
        // Check parameters
        if(nrhs!=16&&nrhs!=17)
        {
            mexErrMsgTxt("Need exactly 10 or 14 inputs: width, height, xc, yc, NRINGS, NSECTORS, RMIN, RMAX, interp, sp, full, disparities(optional), sigma(optional)  .\n");
        }
        
        double *width = (double *) mxGetPr(prhs[1]);
        double *height = (double *) mxGetPr(prhs[2]);
        double *xc = (double *) mxGetPr(prhs[3]);
        double *yc = (double *) mxGetPr(prhs[4]);
        double *nrings = (double *) mxGetPr(prhs[5]);
        double *nsectors = (double *) mxGetPr(prhs[6]);
        double *rmin = (double *) mxGetPr(prhs[7]);
        double *rmax = (double *) mxGetPr(prhs[8]);
        double *interp = (double *) mxGetPr(prhs[9]);
        double *sp = (double *) mxGetPr(prhs[10]);
        double *full = (double *) mxGetPr(prhs[11]);
        
        mwSize NStructElems = mxGetNumberOfElements(prhs[12]);
        double *disparities_ptr = (double *) mxGetPr(prhs[12]);
        std::vector<double> disparities(disparities_ptr,disparities_ptr+NStructElems);
        float sigma = (float) *mxGetPr(prhs[13]);
        float q = (float) *mxGetPr(prhs[14]);
        float pole = (float) *mxGetPr(prhs[15]);


        cv::Point2i center(*xc,*yc);
        std::cout << "width:" << *width << std::endl;
        std::cout << "height:" << *height << std::endl;
        std::cout << "xc:" << *xc << std::endl;
        std::cout << "yc:" << *yc << std::endl;
        std::cout << "nrings:" << *nrings << std::endl;
        std::cout << "nsectors:" << *nsectors << std::endl;
        std::cout << "rmin:" << *rmin << std::endl;
        std::cout << "rmax:" << *rmax << std::endl;
        std::cout << "interp:" << *interp << std::endl;
        std::cout << "sp:" << *sp << std::endl;
        std::cout << "full:" << *full << std::endl;
        
        std::cout << "sigma:" << sigma << std::endl;
        std::cout << "q:" << q << std::endl;
        std::cout << "pole:" << pole << std::endl;
        
        bool high_pass=false;
        if(nrhs==17)
          high_pass = (bool) *mxGetPr(prhs[16]);
        
        std::cout << "high_pass:" << high_pass << std::endl;

        
        plhs[0] = convertPtr2Mat<LogPolar>(new LogPolar(
                *width, *height, center,
                *nrings, *rmin, *interp,
                *full, *nsectors, *sp, disparities, sigma, q, pole, high_pass));
        
        
        return;
    }
    
// Check there is a second input, which should be the class instance handle
    if (nrhs < 2)
        mexErrMsgTxt("Second input should be a class instance handle.");
    
// Delete
    if (!strcmp("delete", cmd)) {
        // Destroy the C++ object
        destroyObject<LogPolar>(prhs[1]);
        // Warn if other commands were ignored
        if (nlhs != 0 || nrhs != 2)
            mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
        return;
    }
    
// Get the class instance pointer from the second input
    LogPolar *class_instance = convertMat2Ptr<LogPolar>(prhs[1]);
    
// Call the various class methods
// to_cortical
    if (!strcmp("to_cortical", cmd))
    {
        // Check parameters
        if (nrhs !=3)
            mexErrMsgTxt("to_cortical: Unexpected arguments.");
        
        // Convert from matlab to opencv
        const mwSize* size=mxGetDimensions(prhs[2]);
        const cv::Mat opencv_const=cv::Mat(size[1],size[0],CV_8UC1,mxGetData(prhs[2]),0);
        //std::cout << "OPEN CV CONST: " << opencv_const << std::endl;
        // Call the method
        cv::Mat cortical=class_instance->to_cortical(opencv_const);
        //std::cout << "CORTICAL CV CONST: " << cortical << std::endl;

        // Convert from opencv to matlab
        plhs[0]=Converter(cortical);
        
        return;
    }
    
// to_cartesian
    if (!strcmp("to_cartesian", cmd))
    {
        // Check parameters
        if (nrhs !=3)
            mexErrMsgTxt("to_cartesian: Unexpected arguments.");
        
        // Convert from matlab to opencv
        
        const mwSize* size=mxGetDimensions(prhs[2]);
        
        const cv::Mat opencv_const=cv::Mat(size[1],size[0],CV_8UC1,mxGetData(prhs[2]),0);
        
        // Call the method
        cv::Mat cartesian=class_instance->to_cartesian(opencv_const);
        //std::cout << "OPEN CV CONST: " << opencv_const << std::endl;
        //std::cout << "cartesian CV CONST: " << cartesian << std::endl;
        
        plhs[0]=Converter(cartesian);
        
        return;
    }
    
    if (!strcmp("get_logpolar_warp_maps", cmd))
    {
        // Check parameters
        if (nrhs !=2)
            mexErrMsgTxt("get_logpolar_warp_maps: Unexpected arguments.");
        
        // Call the method
        std::vector<Mat> logpolar_eta_warp_maps=class_instance->getLogPolarEtaWarpMaps();
        std::vector<Mat> logpolar_rho_warp_maps=class_instance->getLogPolarRhoWarpMaps();
        
        //Find the matlab data type
        
        //We support max 3 dimensions
        mwSignedIndex dims[3];
        dims[0]=logpolar_eta_warp_maps[0].rows;
        dims[1]=logpolar_eta_warp_maps[0].cols;
        dims[2]=logpolar_eta_warp_maps.size();
        
        //Create the array to be returned
        mxArray *eta_matlab=mxCreateNumericArray(3,dims,mxSINGLE_CLASS,mxREAL);
        mxArray *rho_matlab=mxCreateNumericArray(3,dims,mxSINGLE_CLASS,mxREAL);
        
        //Create opencv header as a "flat" image for the matlab data
        cv::Mat eta_tmp=cv::Mat(dims[1]*dims[2],dims[0],CV_MAKETYPE(logpolar_eta_warp_maps[0].type(),1),mxGetData(eta_matlab),0);
        cv::Mat rho_tmp=cv::Mat(dims[1]*dims[2],dims[0],CV_MAKETYPE(logpolar_rho_warp_maps[0].type(),1),mxGetData(rho_matlab),0);
        
        for(int i=0;i<dims[2];i++)
        {
            //transpose the opencv channels image to row major matlab data
            cv::Mat eta_tmp2=logpolar_eta_warp_maps[i].t();
            //Copy the data to the flat matlab data
            eta_tmp2.copyTo(eta_tmp.rowRange(i*dims[1],(i+1)*dims[1]));
            
            //transpose the opencv channels image to row major matlab data
            cv::Mat rho_tmp2=logpolar_rho_warp_maps[i].t();
            //Copy the data to the flat matlab data
            rho_tmp2.copyTo(rho_tmp.rowRange(i*dims[1],(i+1)*dims[1]));
        }
        plhs[0]=eta_matlab;
        plhs[1]=rho_matlab;
        
        return;
        
    }
    
    if (!strcmp("get_delta_logpolar_warp_maps", cmd))
    {
        // Check parameters
        if (nrhs !=2)
            mexErrMsgTxt("get_delta_logpolar_warp_maps: Unexpected arguments.");
        
        // Call the method
        std::vector<Mat> delta_logpolar_eta_warp_maps=class_instance->getDeltaLogPolarEtaWarpMaps();
        std::vector<Mat> delta_logpolar_rho_warp_maps=class_instance->getDeltaLogPolarRhoWarpMaps();
        
        //Find the matlab data type
        //mxClassID type=itr->second;
        
        //We support max 3 dimensions
        mwSignedIndex dims[3];
        dims[0]=delta_logpolar_eta_warp_maps[0].rows;
        dims[1]=delta_logpolar_eta_warp_maps[0].cols;
        dims[2]=delta_logpolar_eta_warp_maps.size();
        
        //Create the array to be returned
        mxArray *delta_eta_matlab=mxCreateNumericArray(3,dims,mxSINGLE_CLASS,mxREAL);
        mxArray *delta_rho_matlab=mxCreateNumericArray(3,dims,mxSINGLE_CLASS,mxREAL);
        
        //Create opencv header as a "flat" image for the matlab data
        cv::Mat eta_tmp=cv::Mat(dims[1]*dims[2],dims[0],CV_MAKETYPE(delta_logpolar_eta_warp_maps[0].type(),1),mxGetData(delta_eta_matlab),0);
        cv::Mat rho_tmp=cv::Mat(dims[1]*dims[2],dims[0],CV_MAKETYPE(delta_logpolar_rho_warp_maps[0].type(),1),mxGetData(delta_rho_matlab),0);
        
        for(int i=0;i<dims[2];i++)
        {
            //transpose the opencv channels image to row major matlab data
            cv::Mat eta_tmp2=delta_logpolar_eta_warp_maps[i].t();
            //Copy the data to the flat matlab data
            eta_tmp2.copyTo(eta_tmp.rowRange(i*dims[1],(i+1)*dims[1]));
            
            //transpose the opencv channels image to row major matlab data
            cv::Mat rho_tmp2=delta_logpolar_rho_warp_maps[i].t();
            //Copy the data to the flat matlab data
            rho_tmp2.copyTo(rho_tmp.rowRange(i*dims[1],(i+1)*dims[1]));
        }
        plhs[0]=delta_eta_matlab;
        plhs[1]=delta_rho_matlab;
        
        return;
    }
    
    if (!strcmp("warp_logpolar", cmd))
    {
        // Check parameters
        if (nrhs !=4)
            mexErrMsgTxt("warp_logpolar: Unexpected arguments.");
        
        // Convert from matlab to opencv
        const mwSize* size=mxGetDimensions(prhs[2]);
        const cv::Mat origin_cv=cv::Mat(size[1],size[0],CV_8UC1,mxGetData(prhs[2]),0);
        
        // Convert disparity index
        double disparity_index = (double) *mxGetPr(prhs[3]);
        
        // c
        disparity_index=disparity_index-1; //MATLAB TO C++ indices!!!
        cv::Mat destiny_cv;
        
        // Call the method
        class_instance->warpLogPolar(origin_cv,destiny_cv,disparity_index);
        
        // Convert from opencv to matlab
        plhs[0]=Converter(destiny_cv);
        
        return;
    }
    
    if (!strcmp("warp_cartesian_logpolar", cmd))
    {
        // Check parameters
        if (nrhs !=4)
            mexErrMsgTxt("warp_cartesian_logpolar: Unexpected arguments.");
        
        // Convert from matlab to opencv
        const mwSize* size=mxGetDimensions(prhs[2]);
        const cv::Mat origin_cv=cv::Mat(size[1],size[0],CV_8UC1,mxGetData(prhs[2]),0);
        
        // Convert disparity index
        double disparity_index = (double) *mxGetPr(prhs[3]);
        
        disparity_index=disparity_index-1; //MATLAB TO C++ indices!!!
        cv::Mat destiny_cv;
        
        // Call the method
        class_instance->warpCartesianLogPolar(origin_cv, destiny_cv, disparity_index);
        
        // Convert from opencv to matlab
        plhs[0]=Converter(destiny_cv);
        
        return;
    }
    
    if (!strcmp("compute_disparity_map", cmd))
    {
        // Check parameters
        if (nrhs !=4)
            mexErrMsgTxt("compute_disparity_map: Unexpected arguments.");
        
        // Convert from matlab to opencv
        const mwSize* size=mxGetDimensions(prhs[2]);
        const cv::Mat left_image_in_cv=cv::Mat(size[1],size[0],CV_8UC1,mxGetData(prhs[2]),0);
        const cv::Mat right_image_in_cv=cv::Mat(size[1],size[0],CV_8UC1,mxGetData(prhs[3]),0);
        
        // Call the method
        Mat disparity_map=class_instance->computeDisparityMap(left_image_in_cv, right_image_in_cv);
        
        // Convert from opencv to matlab
        plhs[0]=Converter(disparity_map);
        
        return;
    }
    
    if (!strcmp("filter", cmd))
    {
        // Check parameters
        if (nrhs !=6)
            mexErrMsgTxt("filter: Unexpected arguments.");
        
        // Convert from matlab to opencv
        const mwSize* size=mxGetDimensions(prhs[2]);
        const cv::Mat source=cv::Mat(size[1],size[0],CV_8UC1,mxGetData(prhs[2]),0);
        Mat source_double;
        source.convertTo( source_double, CV_32FC1 );
        cv::Mat output;//=cv::Mat(size[1],size[0],CV_8UC1,mxGetData(prhs[2]),0);
        //std::cout << "source_double:" << source_double;
        float pole  = (float)* mxGetPr(prhs[3]);
        int edge    = (int)* mxGetPr(prhs[4]);
        int wrap    = (int)* mxGetPr(prhs[5]);
        
        // Call the method
        class_instance->PolarFiltFilt2(source_double, output, pole, edge, wrap);
        
        Mat output_int;
        output.convertTo( output_int, CV_8UC1 );
        // Convert from opencv to matlab
        plhs[0]=Converter(output_int);
        
        return;
    }
    
    
// Got here, so command not recognized
    mexErrMsgTxt("Command not recognized.");
}
