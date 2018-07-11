#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cpp/PeripheralFovealStereo.h"
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
        if(nrhs!=32)
        {
            mexErrMsgTxt("wrong inputs number");
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

        int ignore_border_left=10;
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
        std::cout << "fovea_rows:" << fovea_rows << std::endl;
        std::cout << "fovea_columns:" << fovea_columns << std::endl;

        plhs[0] = convertPtr2Mat<PeripheralFovealStereo>(new PeripheralFovealStereo(
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
        destroyObject<PeripheralFovealStereo>(prhs[1]);
        // Warn if other commands were ignored
        if (nlhs != 0 || nrhs != 2)
            mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
        return;
    }
    
    // Get the class instance pointer from the second input
    PeripheralFovealStereo *class_instance = convertMat2Ptr<PeripheralFovealStereo>(prhs[1]);
    
    // Call the various class methods
    // to_cortical
    if (!strcmp("to_cortical", cmd))
    {
        // Check parameters
        if (nrhs !=3)
            mexErrMsgTxt("to_cortical: Unexpected arguments.");
        
        // Convert from matlab to opencv
        const mwSize* size=mxGetDimensions(prhs[2]);
        const cv::Mat opencv_const=cv::Mat(size[1],size[0],CV_8UC3,mxGetData(prhs[2]),0);

        // Call the method
        cv::Mat cortical=class_instance->sensor.to_cortical(opencv_const);

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
        const cv::Mat opencv_const=cv::Mat(size[1],size[0],CV_8UC3,mxGetData(prhs[2]),0);
        
        // Call the method
        std::cout << "OLA1  " << std::endl;
        cv::Mat cartesian=class_instance->sensor.to_cartesian(opencv_const);
        std::cout << "ADEUS1" << std::endl;
        
        // Convert from opencv to matlab
        plhs[0]=Converter(cartesian);
        
        return;
    }
    
    // rectify stereo
    if (!strcmp("rectify_stereo", cmd))
    {
        // Check parameters
        if (nrhs !=8)
            mexErrMsgTxt("rectify_stereo: Unexpected arguments.");
        
        // Convert from matlab to opencv
        const mwSize* size=mxGetDimensions(prhs[2]);
        
        cv::Mat image_left=Converter(mxDuplicateArray(prhs[2]));
        cv::Mat image_right=Converter(mxDuplicateArray(prhs[3]));
        
        
        const mwSize* size2=mxGetDimensions(prhs[4]);
        cv::Mat R_left_cam_to_right_cam=  cv::Mat(size2[1],size2[0],CV_64F,mxGetData(prhs[4]),0);
        
        const mwSize* size3=mxGetDimensions(prhs[5]);
        cv::Mat t_left_cam_to_right_cam=  cv::Mat(size3[1],size3[0],CV_64F,mxGetData(prhs[5]),0);
        
        const mwSize* size4=mxGetDimensions(prhs[6]);
        cv::Mat cameraMatrix1=  cv::Mat(size4[1],size4[0],CV_64F,mxGetData(prhs[6]),0);
        cv::Mat cameraMatrix2=  cv::Mat(size4[1],size4[0],CV_64F,mxGetData(prhs[7]),0);

        cv::Mat image_left_rectified=  cv::Mat(size[1],size[0],image_left.type());
        cv::Mat image_right_rectified=  cv::Mat(size[1],size[0],image_right.type());

        cv::Mat Q,R1,R2,P1,P2;
        cv::stereoRectify(cameraMatrix1,
                      cv::Mat::zeros(5,1,CV_64F),
                      cameraMatrix2,
                      cv::Mat::zeros(5,1,CV_64F),
                      cv::Size(image_left.cols,
                               image_left.rows),
                      R_left_cam_to_right_cam,
                      t_left_cam_to_right_cam,
                      R1,
                      R2,
                      P1,
                      P2,
                      Q,
                      CV_CALIB_ZERO_DISPARITY,
                      0,
                      cv::Size(image_left.cols,
                               image_left.rows));
        
        
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
        plhs[11]=Converter(Q);

        return;
    }
    
    // get_uncertainty
    if (!strcmp("get_uncertainty", cmd))
    {
        // Check parameters
        if (nrhs !=12)
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
        
        const mwSize* size5=mxGetDimensions(prhs[11]);
        const cv::Mat Q=cv::Mat(size5[1],size5[0],CV_64F,mxGetData(prhs[11]),0);
        class_instance->sensor.computeUncertainty(disparities,
                H1,
                H2,
                stereo_rectification_map1_left,
                stereo_rectification_map2_left,
                stereo_rectification_map1_right,
                stereo_rectification_map2_right,
                translation);
        
        std::vector<std::vector<cv::Mat> > covariances=class_instance->sensor.stereo_data.getCovariances();
        std::vector<std::vector<cv::Mat> > informations=class_instance->sensor.stereo_data.getInformations();
        
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

        cv::Mat mean_3d;
        class_instance->sensor.stereo_data.getMean().copyTo(mean_3d);
        cv::Mat real_3d;
        cv::Mat disparities32F;
        disparities.convertTo(disparities32F, CV_32F);

        class_instance->get3DpointCloud(disparities32F,real_3d,transf,Q);

        cv::Mat informations_determinants;
        class_instance->sensor.stereo_data.getInformationsDeterminants().copyTo(informations_determinants);

        cv::Mat mean_sigma_points;
        class_instance->sensor.sigma_points[0].copyTo(mean_sigma_points);

        cv::Mat mean_sigma_points_rectified_1;
        class_instance->sensor.sigma_points_1_rectified[0].copyTo(mean_sigma_points_rectified_1);
        
        cv::Mat mean_sigma_points_rectified_2;
        class_instance->sensor.sigma_points_2_rectified[0].copyTo(mean_sigma_points_rectified_2);
        
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
    
    
    
    /*if (!strcmp("get_logpolar_warp_maps", cmd))
     * {
     * // Check parameters
     * if (nrhs !=2)
     * mexErrMsgTxt("get_logpolar_warp_maps: Unexpected arguments.");
     *
     * // Call the method
     * std::vector<Mat> logpolar_eta_warp_maps=class_instance->getLogPolarEtaWarpMaps();
     * std::vector<Mat> logpolar_rho_warp_maps=class_instance->getLogPolarRhoWarpMaps();
     *
     * //Find the matlab data type
     *
     * //We support max 3 dimensions
     * mwSignedIndex dims[3];
     * dims[0]=logpolar_eta_warp_maps[0].rows;
     * dims[1]=logpolar_eta_warp_maps[0].cols;
     * dims[2]=logpolar_eta_warp_maps.size();
     *
     * //Create the array to be returned
     * mxArray *eta_matlab=mxCreateNumericArray(3,dims,mxSINGLE_CLASS,mxREAL);
     * mxArray *rho_matlab=mxCreateNumericArray(3,dims,mxSINGLE_CLASS,mxREAL);
     *
     * //Create opencv header as a "flat" image for the matlab data
     * cv::Mat eta_tmp=cv::Mat(dims[1]*dims[2],dims[0],CV_MAKETYPE(logpolar_eta_warp_maps[0].type(),1),mxGetData(eta_matlab),0);
     * cv::Mat rho_tmp=cv::Mat(dims[1]*dims[2],dims[0],CV_MAKETYPE(logpolar_rho_warp_maps[0].type(),1),mxGetData(rho_matlab),0);
     *
     * for(int i=0;i<dims[2];i++)
     * {
     * //transpose the opencv channels image to row major matlab data
     * cv::Mat eta_tmp2=logpolar_eta_warp_maps[i].t();
     * //Copy the data to the flat matlab data
     * eta_tmp2.copyTo(eta_tmp.rowRange(i*dims[1],(i+1)*dims[1]));
     *
     * //transpose the opencv channels image to row major matlab data
     * cv::Mat rho_tmp2=logpolar_rho_warp_maps[i].t();
     * //Copy the data to the flat matlab data
     * rho_tmp2.copyTo(rho_tmp.rowRange(i*dims[1],(i+1)*dims[1]));
     * }
     * plhs[0]=eta_matlab;
     * plhs[1]=rho_matlab;
     *
     * return;
     *
     * }
     *
     * if (!strcmp("get_delta_logpolar_warp_maps", cmd))
     * {
     * // Check parameters
     * if (nrhs !=2)
     * mexErrMsgTxt("get_delta_logpolar_warp_maps: Unexpected arguments.");
     *
     * // Call the method
     * std::vector<Mat> delta_logpolar_eta_warp_maps=class_instance->getDeltaLogPolarEtaWarpMaps();
     * std::vector<Mat> delta_logpolar_rho_warp_maps=class_instance->getDeltaLogPolarRhoWarpMaps();
     *
     * //Find the matlab data type
     * //mxClassID type=itr->second;
     *
     * //We support max 3 dimensions
     * mwSignedIndex dims[3];
     * dims[0]=delta_logpolar_eta_warp_maps[0].rows;
     * dims[1]=delta_logpolar_eta_warp_maps[0].cols;
     * dims[2]=delta_logpolar_eta_warp_maps.size();
     *
     * //Create the array to be returned
     * mxArray *delta_eta_matlab=mxCreateNumericArray(3,dims,mxSINGLE_CLASS,mxREAL);
     * mxArray *delta_rho_matlab=mxCreateNumericArray(3,dims,mxSINGLE_CLASS,mxREAL);
     *
     * //Create opencv header as a "flat" image for the matlab data
     * cv::Mat eta_tmp=cv::Mat(dims[1]*dims[2],dims[0],CV_MAKETYPE(delta_logpolar_eta_warp_maps[0].type(),1),mxGetData(delta_eta_matlab),0);
     * cv::Mat rho_tmp=cv::Mat(dims[1]*dims[2],dims[0],CV_MAKETYPE(delta_logpolar_rho_warp_maps[0].type(),1),mxGetData(delta_rho_matlab),0);
     *
     * for(int i=0;i<dims[2];i++)
     * {
     * //transpose the opencv channels image to row major matlab data
     * cv::Mat eta_tmp2=delta_logpolar_eta_warp_maps[i].t();
     * //Copy the data to the flat matlab data
     * eta_tmp2.copyTo(eta_tmp.rowRange(i*dims[1],(i+1)*dims[1]));
     *
     * //transpose the opencv channels image to row major matlab data
     * cv::Mat rho_tmp2=delta_logpolar_rho_warp_maps[i].t();
     * //Copy the data to the flat matlab data
     * rho_tmp2.copyTo(rho_tmp.rowRange(i*dims[1],(i+1)*dims[1]));
     * }
     * plhs[0]=delta_eta_matlab;
     * plhs[1]=delta_rho_matlab;
     *
     * return;
     * }
     *
     * if (!strcmp("warp_logpolar", cmd))
     * {
     * // Check parameters
     * if (nrhs !=4)
     * mexErrMsgTxt("warp_logpolar: Unexpected arguments.");
     *
     * // Convert from matlab to opencv
     * const mwSize* size=mxGetDimensions(prhs[2]);
     * const cv::Mat origin_cv=cv::Mat(size[1],size[0],CV_8UC1,mxGetData(prhs[2]),0);
     *
     * // Convert disparity index
     * double disparity_index = (double) *mxGetPr(prhs[3]);
     *
     * // c
     * disparity_index=disparity_index-1; //MATLAB TO C++ indices!!!
     * cv::Mat destiny_cv;
     *
     * // Call the method
     * class_instance->warpLogPolar(origin_cv,destiny_cv,disparity_index);
     *
     * // Convert from opencv to matlab
     * plhs[0]=Converter(destiny_cv);
     *
     * return;
     * }
     *
     * if (!strcmp("warp_cartesian_logpolar", cmd))
     * {
     * // Check parameters
     * if (nrhs !=4)
     * mexErrMsgTxt("warp_cartesian_logpolar: Unexpected arguments.");
     *
     * // Convert from matlab to opencv
     * const mwSize* size=mxGetDimensions(prhs[2]);
     * const cv::Mat origin_cv=cv::Mat(size[1],size[0],CV_8UC1,mxGetData(prhs[2]),0);
     *
     * // Convert disparity index
     * double disparity_index = (double) *mxGetPr(prhs[3]);
     *
     * disparity_index=disparity_index-1; //MATLAB TO C++ indices!!!
     * cv::Mat destiny_cv;
     *
     * // Call the method
     * class_instance->warpCartesianLogPolar(origin_cv, destiny_cv, disparity_index);
     *
     * // Convert from opencv to matlab
     * plhs[0]=Converter(destiny_cv);
     *
     * return;
     * }
     *
     * if (!strcmp("get_egosphere", cmd))
     * {
     * // Check parameters
     * if (nrhs !=2)
     * mexErrMsgTxt("get_egosphere: Unexpected arguments.");
     *
     * // Convert from opencv to matlab
     *
     * std::vector<MemoryPatch> patches=class_instance->full_geodesic_dome.patches;
     * // convert it to MATLAB struct array and return it as output
     * mxArray *p;
     * mxArray *d;
     *
     * plhs[0] = mxCreateStructMatrix(1, patches.size(), 4, fieldsStruct);
     * for (int i=0; i<patches.size(); ++i)
     * {
     * // cartesian topology
     * p = mxCreateStructMatrix(1, 1, 3, fieldsCartPoint);
     * mxSetField(p, 0, "x", mxCreateDoubleScalar(patches[i].cartesian_position.data.at<double>(0,0)));
     * mxSetField(p, 0, "y", mxCreateDoubleScalar(patches[i].cartesian_position.data.at<double>(1,0)));
     * mxSetField(p, 0, "z", mxCreateDoubleScalar(patches[i].cartesian_position.data.at<double>(2,0)));
     * mxSetField(plhs[0], i, "cartesian_topology", p);
     *
     * // spherical topology
     * p = mxCreateStructMatrix(1, 1, 3, fieldsSphericalPoint);
     * mxSetField(p, 0, "rho",   mxCreateDoubleScalar(patches[i].spherical_position.data.at<double>(0,0)));
     * mxSetField(p, 0, "phi",   mxCreateDoubleScalar(patches[i].spherical_position.data.at<double>(1,0)));
     * mxSetField(p, 0, "theta", mxCreateDoubleScalar(patches[i].spherical_position.data.at<double>(2,0)));
     * mxSetField(plhs[0], i, "spherical_topology", p);
     *
     * // depth
     * p = mxCreateStructMatrix(1, 1, 2, fieldsGauss);
     * mxSetField(p, 0, "mean",    mxCreateDoubleScalar(patches[i].depth.mean));
     * mxSetField(p, 0, "std_dev", mxCreateDoubleScalar(patches[i].depth.std_dev));
     * mxSetField(plhs[0], i, "depth", p);
     *
     * // brightness
     * p = mxCreateStructMatrix(1, 1, 2, fieldsGauss);
     * mxSetField(p, 0, "mean",    mxCreateDoubleScalar(patches[i].brightness.mean));
     * mxSetField(p, 0, "std_dev", mxCreateDoubleScalar(patches[i].brightness.std_dev));
     * mxSetField(plhs[0], i, "brightness", p);
     * }
     *
     * return;
     * }
     *
     * if (!strcmp("compute_disparity_map", cmd))
     * {
     * // Check parameters
     * if (nrhs !=6)
     * mexErrMsgTxt("compute_disparity_map: Unexpected arguments.");
     *
     * // Convert from matlab to opencv
     * const mwSize* size=mxGetDimensions(prhs[2]);
     * const cv::Mat left_image_in_cv=cv::Mat(size[1],size[0],CV_8UC1,mxGetData(prhs[2]),0);
     * const cv::Mat right_image_in_cv=cv::Mat(size[1],size[0],CV_8UC1,mxGetData(prhs[3]),0);
     * float disparities_number = (float) *mxGetPr(prhs[4]);
     * float min_disparity = (float) *mxGetPr(prhs[5]);
     * // Call the method
     * Mat disparity_map=class_instance->getCartesianDisparityMap(left_image_in_cv, right_image_in_cv,disparities_number,min_disparity);
     *
     * // Convert from opencv to matlab
     * plhs[0]=Converter(disparity_map);
     *
     * return;
     * }
     *
     * if (!strcmp("filter", cmd))
     * {
     * // Check parameters
     * if (nrhs !=6)
     * mexErrMsgTxt("filter: Unexpected arguments.");
     *
     * // Convert from matlab to opencv
     * const mwSize* size=mxGetDimensions(prhs[2]);
     * const cv::Mat source=cv::Mat(size[1],size[0],CV_8UC1,mxGetData(prhs[2]),0);
     * Mat source_double;
     * source.convertTo( source_double, CV_32FC1 );
     * cv::Mat output;//=cv::Mat(size[1],size[0],CV_8UC1,mxGetData(prhs[2]),0);
     * //std::cout << "source_double:" << source_double;
     * float pole  = (float)* mxGetPr(prhs[3]);
     * int edge    = (int)* mxGetPr(prhs[4]);
     * int wrap    = (int)* mxGetPr(prhs[5]);
     *
     * // Call the method
     * class_instance->PolarFiltFilt2(source_double, output, pole, edge, wrap);
     *
     * Mat output_int;
     * output.convertTo( output_int, CV_8UC1 );
     * // Convert from opencv to matlab
     * plhs[0]=Converter(output_int);
     *
     * return;
     * }*/
    
    
// Got here, so command not recognized
    mexErrMsgTxt("Command not recognized.");
}
