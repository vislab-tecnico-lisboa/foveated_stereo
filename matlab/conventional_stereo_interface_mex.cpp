#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cpp/ConventionalStereo.h"
#include "mex.h"
#include "foveal_handle.hpp"
#include "mc_convert/mc_convert.hpp"
#include <string>
// The class that we are interfacing to
//

const char *fieldsCartPoint[] = {"x", "y", "z"};
const char *fieldsSphericalPoint[] = {"rho", "phi", "theta"};

const char *fieldsGauss[] = {"mean", "std_dev"};
const char *fieldsStruct[] = {"cartesian_topology", "spherical_topology", "depth", "brightness"};

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
        std::cout << "nrhs: " << nrhs << std::endl;
        if(nrhs!=31)
        {
            mexErrMsgTxt("Need exactly 31 inputs: width, height, xc, yc, NRINGS, NSECTORS, RMIN, RMAX, interp, sp, full, disparities(optional), sigma(optional)  .\n");
        }
        // Convert from matlab to opencv
        const mwSize* size=mxGetDimensions(prhs[1]);
        cv::Mat left_camera_intrinsic=cv::Mat(size[1],size[0],CV_64F,mxGetData(prhs[1]),0);
        cv::Mat right_camera_intrinsic=cv::Mat(size[1],size[0],CV_64F,mxGetData(prhs[2]),0);
        double *width = (double *) mxGetPr(prhs[3]);
        double *height = (double *) mxGetPr(prhs[4]);
        double *xc = (double *) mxGetPr(prhs[5]);
        double *yc = (double *) mxGetPr(prhs[6]);
        cv::Point2i center(*xc,*yc);
        
        double *nrings = (double *) mxGetPr(prhs[7]);
        double *nsectors = (double *) mxGetPr(prhs[8]);
        double *rmin = (double *) mxGetPr(prhs[9]);
        double *rmax = (double *) mxGetPr(prhs[10]);
        double *interp = (double *) mxGetPr(prhs[11]);
        double *sp = (double *) mxGetPr(prhs[12]);
        double *full = (double *) mxGetPr(prhs[13]);
        double fovea_rows=*(double *) mxGetPr(prhs[14]);
        double fovea_columns=*(double *) mxGetPr(prhs[15]);

        std::string ego_frame="ego_frame";
        double L=*(double *) mxGetPr(prhs[16]);
        double alpha=*(double *) mxGetPr(prhs[17]);
        double ki=*(double *) mxGetPr(prhs[18]);
        double beta=*(double *) mxGetPr(prhs[19]);
        double scaling_factor=*(double *) mxGetPr(prhs[20]);
        double number_of_disparities=*(double *) mxGetPr(prhs[21]);
        double pre_filter_cap=*(double *) mxGetPr(prhs[22]);
        double sad_window_size=*(double *) mxGetPr(prhs[23]);
        double P1=*(double *) mxGetPr(prhs[24]);
        double P2=*(double *) mxGetPr(prhs[25]);
        double min_disparity=*(double *) mxGetPr(prhs[26]);
        double uniqueness_ratio=*(double *) mxGetPr(prhs[27]);
        double speckle_window_size=*(double *) mxGetPr(prhs[28]);
        double speckle_range=*(double *) mxGetPr(prhs[29]);
        double disp_12_max_diff=*(double *) mxGetPr(prhs[30]);
        double full_dp=*(double *) mxGetPr(prhs[31]);
        int ignore_border_left=0;
        std::cout << "left_camera_intrinsic:" << left_camera_intrinsic << std::endl;
        std::cout << "right_camera_intrinsic:" << right_camera_intrinsic << std::endl;
        
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
        
        std::cout << "ego_frame:" << ego_frame << std::endl;
        std::cout << "L:" << L << std::endl;
        std::cout << "alpha:" << alpha << std::endl;
        std::cout << "ki:" << ki << std::endl;
        std::cout << "beta:" << beta << std::endl;
        std::cout << "scaling_factor:" << scaling_factor << std::endl;
        std::cout << "number_of_disparities:" << number_of_disparities << std::endl;
        std::cout << "pre_filter_cap:" << pre_filter_cap << std::endl;
        std::cout << "sad_window_size:" << sad_window_size << std::endl;
        std::cout << "P1:" << P1 << std::endl;
        std::cout << "P2:" << P2 << std::endl;
        std::cout << "min_disparity:" << min_disparity << std::endl;
        std::cout << "uniqueness_ratio:" << uniqueness_ratio << std::endl;
        std::cout << "speckle_window_size:" << speckle_window_size << std::endl;
        std::cout << "speckle_range:" << speckle_range << std::endl;
        std::cout << "disp_12_max_diff:" << disp_12_max_diff << std::endl;
        std::cout << "full_dp:" << full_dp << std::endl;
        std::cout << "ignore_border_left:" << ignore_border_left << std::endl;
        
        plhs[0] = convertPtr2Mat<ConventionalStereo>(new ConventionalStereo(
                left_camera_intrinsic,
                right_camera_intrinsic,
                *width,
                *height,
                center,
                *nrings,
                *rmin,
                *interp,
                *full,
                *nsectors,
                *sp,
                fovea_rows,
                fovea_columns,
                ego_frame,
                L,
                alpha,
                ki,
                beta,
                scaling_factor,
                number_of_disparities,
                pre_filter_cap,
                sad_window_size,
                P1,
                P2,
                min_disparity,
                uniqueness_ratio,
                speckle_window_size,
                speckle_range,
                disp_12_max_diff,
                full_dp,
                ignore_border_left
                ));
        
        return;
    }
    
// Check there is a second input, which should be the class instance handle
    if (nrhs < 2)
        mexErrMsgTxt("Second input should be a class instance handle.");
    
// Delete
    if (!strcmp("delete", cmd)) {
        // Destroy the C++ object
        destroyObject<ConventionalStereo>(prhs[1]);
        // Warn if other commands were ignored
        if (nlhs != 0 || nrhs != 2)
            mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
        return;
    }
    
    // Get the class instance pointer from the second input
    ConventionalStereo *class_instance = convertMat2Ptr<ConventionalStereo>(prhs[1]);
        
    // rectify stereo
    if (!strcmp("rectify_stereo", cmd))
    {
        // Check parameters
        if (nrhs !=6)
            mexErrMsgTxt("rectify_stereo: Unexpected arguments.");
        
        // Convert from matlab to opencv
        const mwSize* size=mxGetDimensions(prhs[2]);
        
        cv::Mat image_left=Converter(mxDuplicateArray(prhs[2]));
        cv::Mat image_right=Converter(mxDuplicateArray(prhs[3]));
        
        
        const mwSize* size2=mxGetDimensions(prhs[4]);
        cv::Mat R1=  cv::Mat(size2[1],size2[0],CV_64F,mxGetData(prhs[4]),0);
        
        const mwSize* size3=mxGetDimensions(prhs[5]);
        cv::Mat R2=  cv::Mat(size3[1],size3[0],CV_64F,mxGetData(prhs[5]),0);
        
        const mwSize* size4=mxGetDimensions(prhs[6]);
        cv::Mat P1=  cv::Mat(size4[1],size4[0],CV_64F,mxGetData(prhs[6]),0);
        
        const mwSize* size5=mxGetDimensions(prhs[7]);
        cv::Mat P2=  cv::Mat(size5[1],size5[0],CV_64F,mxGetData(prhs[7]),0);
        
        cv::Mat image_left_rectified=  cv::Mat(size[1],size[0],image_left.type());
        cv::Mat image_right_rectified=  cv::Mat(size[1],size[0],image_right.type());
        
        class_instance->stereoRectify(
                image_left,
                image_right,
                R1,
                R2,
                P1,
                P2,
                image_left_rectified,
                image_right_rectified);

        
        cv::Mat H1;
        class_instance->H1.copyTo(H1);
        
        cv::Mat H2;
        class_instance->H2.copyTo(H2);
        
        double translation=class_instance->trans2.at<double>(0,0);
        
        cv::Mat stereo_rectification_map1_left;
        class_instance->stereo_rectification_map1_left.copyTo(stereo_rectification_map1_left);
        cv::Mat stereo_rectification_map2_left;
        class_instance->stereo_rectification_map2_left.copyTo(stereo_rectification_map2_left);
        cv::Mat stereo_rectification_map1_right;
        class_instance->stereo_rectification_map1_right.copyTo(stereo_rectification_map1_right);
        cv::Mat stereo_rectification_map2_right;
        class_instance->stereo_rectification_map2_right.copyTo(stereo_rectification_map2_right);
        
        cv::Mat rectifiedCameraMatrix1;
        class_instance->rectifiedCameraMatrix1.copyTo(rectifiedCameraMatrix1);
        cv::Mat rectifiedCameraMatrix2;
        class_instance->rectifiedCameraMatrix2.copyTo(rectifiedCameraMatrix2);
        
        plhs[0]=Converter(image_left_rectified);
        plhs[1]=Converter(image_right_rectified);
        plhs[2]=Converter(H1);
        plhs[3]=Converter(H2);
        plhs[4]=Converter(stereo_rectification_map1_left);
        plhs[5]=Converter(stereo_rectification_map2_left);
        plhs[6]=Converter(stereo_rectification_map1_right);
        plhs[7]=Converter(stereo_rectification_map2_right);
        plhs[8]=Converter(rectifiedCameraMatrix1);
        plhs[9]=Converter(rectifiedCameraMatrix2);
        plhs[10]=mxCreateDoubleScalar(translation);
        //std::cout << "STEREO RCT MAP:" << stereo_rectification_map1_left << std::endl;

        return;
    }
    
    // get_uncertainty
    if (!strcmp("get_uncertainty", cmd))
    {
        // Check parameters
        if (nrhs !=11)
            mexErrMsgTxt("get_uncertainty: Unexpected arguments.");
        
        // Convert from matlab to opencv
        const mwSize* size=mxGetDimensions(prhs[2]);
        const cv::Mat transf=cv::Mat(size[1],size[0],CV_64F,mxGetData(prhs[2]),0);
        
        const mwSize* size2=mxGetDimensions(prhs[3]);
        const cv::Mat disparities=cv::Mat(size2[1],size2[0],CV_64F,mxGetData(prhs[3]),0);
        
        const mwSize* size3=mxGetDimensions(prhs[4]);
        const cv::Mat H1=cv::Mat(size3[1],size3[0],CV_64F,mxGetData(prhs[4]),0);
        const cv::Mat H2=cv::Mat(size3[1],size3[0],CV_64F,mxGetData(prhs[5]),0);
        
        const mwSize* size4=mxGetDimensions(prhs[6]);
        const cv::Mat stereo_rectification_map1_left=cv::Mat(size4[1],size4[0],CV_32F,mxGetData(prhs[6]),0);
        const cv::Mat stereo_rectification_map2_left=cv::Mat(size4[1],size4[0],CV_32F,mxGetData(prhs[7]),0);
        const cv::Mat stereo_rectification_map1_right=cv::Mat(size4[1],size4[0],CV_32F,mxGetData(prhs[8]),0);
        const cv::Mat stereo_rectification_map2_right=cv::Mat(size4[1],size4[0],CV_32F,mxGetData(prhs[9]),0);
        double translation=*(double *) mxGetPr(prhs[10]);
        std::cout << "trans:" << translation << std::endl;
        class_instance->cartesian_sensor.computeUncertainty(disparities,
                H1,
                H2,
                stereo_rectification_map1_left,
                stereo_rectification_map2_left,
                stereo_rectification_map1_right,
                stereo_rectification_map2_right,
                translation);
        
        std::vector<std::vector<cv::Mat> > covariances=class_instance->cartesian_sensor.stereo_data.getCovariances();
        std::vector<std::vector<cv::Mat> > informations=class_instance->cartesian_sensor.stereo_data.getInformations();
        
        mwSignedIndex dims[2];
        
        dims[0]=covariances.size()*covariances[0][0].rows;
        dims[1]=covariances[0].size()*covariances[0][0].cols;
        
        //Create the array to be returned
        mxArray *matlab_covs=mxCreateDoubleMatrix(dims[0],dims[1],mxREAL);
        mxArray *matlab_infos=mxCreateDoubleMatrix(dims[0],dims[1],mxREAL);
        
        //Create opencv header as a "flat" image for the matlab data
        cv::Mat covs_tmp=cv::Mat(dims[1],dims[0],CV_MAKETYPE(covariances[0][0].type(),1),mxGetData(matlab_covs),0);
        cv::Mat infos_tmp=cv::Mat(dims[1],dims[0],CV_MAKETYPE(informations[0][0].type(),1),mxGetData(matlab_infos),0);
        // Fill with data
        std::cout << "fill with data" << std::endl;
        for(int i=0;i<covariances.size();++i)
        {
            for(int j=0;j<covariances[0].size();++j)
            {
                //transpose the opencv channels image to row major matlab data
                cv::Mat tmp=covariances[i][j].t();
                tmp.copyTo(covs_tmp.colRange(i*3,(i+1)*3).rowRange(j*3,(j+1)*3)); 
                tmp=informations[i][j].t();
                tmp.copyTo(infos_tmp.colRange(i*3,(i+1)*3).rowRange(j*3,(j+1)*3));
            }
        }

        //std::cout << covs_tmp << std::endl;
        //std::cout << covs_tmp.at<double>(100,200) << std::endl;
        
        cv::Mat mean_3d;
        class_instance->cartesian_sensor.stereo_data.getMean().copyTo(mean_3d);        
        cv::Mat real_3d;
        cv::Mat disparities32F;
        disparities.convertTo(disparities32F, CV_32F);

        class_instance->get3DpointCloud(disparities32F,real_3d,transf);

        cv::Mat informations_determinants;
        class_instance->cartesian_sensor.stereo_data.getInformationsDeterminants().copyTo(informations_determinants);
        
        cv::Mat mean_sigma_points;
        class_instance->cartesian_sensor.sigma_points[0].copyTo(mean_sigma_points);

        cv::Mat mean_sigma_points_rectified_1;
        class_instance->cartesian_sensor.sigma_points_1_rectified[0].copyTo(mean_sigma_points_rectified_1);
        
        cv::Mat mean_sigma_points_rectified_2;
        class_instance->cartesian_sensor.sigma_points_2_rectified[0].copyTo(mean_sigma_points_rectified_2);
        
        std::cout << "done"<< std::endl;
        plhs[0]=matlab_covs;
        plhs[1]=matlab_infos;
        plhs[2]=Converter(informations_determinants);
        plhs[3]=Converter(mean_3d);
        plhs[4]=Converter(real_3d);
        plhs[5]=Converter(mean_sigma_points);
        plhs[6]=Converter(mean_sigma_points_rectified_1);
        plhs[7]=Converter(mean_sigma_points_rectified_2);

        return;
    }
   
    
// Got here, so command not recognized
    mexErrMsgTxt("Command not recognized.");
}
