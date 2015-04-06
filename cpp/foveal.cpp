/*==========================================================
 * mexcpp.cpp - example in MATLAB External Interfaces
 *
 * Illustrates how to use some C++ language features in a MEX-file.
 * It makes use of member functions, constructors, destructors, and the
 * iostream.
 *
 * The routine simply defines a class, constructs a simple object,
 * and displays the initial values of the internal variables.  It
 * then sets the data members of the object based on the input given
 * to the MEX-file and displays the changed values.
 *
 * This file uses the extension .cpp.  Other common C++ extensions such
 * as .C, .cc, and .cxx are also supported.
 *
 * The calling syntax is:
 *
 *		mexcpp( num1, num2 )
 *
 * Limitations:
 * On Windows, this example uses mexPrintf instead cout.  Iostreams
 * (such as cout) are not supported with MATLAB with all C++ compilers.
 *
 * This is a MEX-file for MATLAB.
 * Copyright 1984-2013 The MathWorks, Inc.
 *
 *========================================================*/

#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "logpolar/logpolar.hpp"
#include "mex.h"
#include "mc_convert/mc_convert.hpp"

using namespace std;

void _main();

/****************************/
class MyData {
    
public:
    void display();
    void set_data(double v1, double v2);
    MyData(double v1 = 0, double v2 = 0);
    ~MyData() { }
private:
    double val1, val2;
};

MyData::MyData(double v1, double v2)
{
    val1 = v1;
    val2 = v2;
}

void MyData::display()
{
#ifdef _WIN32
    mexPrintf("Value1 = %g\n", val1);
    mexPrintf("Value2 = %g\n\n", val2);
#else
    cout << "Value1 = " << val1 << "\n";
    cout << "Value2 = " << val2 << "\n\n";
#endif
}

void MyData::set_data(double v1, double v2) { val1 = v1; val2 = v2; }








/*********************/

/*static
        void mexcpp(
        double num1,
        double num2
        )
{
    std::cout <<"YEHHH"<<std::endl;
    LogPolar log_polar;
#ifdef _WIN32
    mexPrintf("\nThe initialized data in object:\n");
#else
    cout << "\nThe initialized data in object:\n";
#endif
    MyData *d = new MyData; // Create a  MyData object
    d->display();           // It should be initialized to
    // zeros
    d->set_data(num1,num2); // Set data members to incoming
    // values
#ifdef _WIN32
    mexPrintf("After setting the object's data to your input:\n");
#else
    cout << "After setting the object's data to your input:\n";
#endif
    d->display();           // Make sure the set_data() worked
    delete(d);
    flush(cout);
    return;
}//*/












void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) 
{
    
    /* Check for proper number of arguments */
    if(nrhs!=7)
    {
         mexErrMsgTxt("Need exactly 7 inputs: left_image_gray, RMIN, RMAX, xc, yc, NRINGS, NSECTORS.\n");
    }

    //mxArray * new_prh;
    mxArray *new_prh=mxDuplicateArray(prhs[0]);

    cv::Mat cv_image=Converter(new_prh);
    
    std::cout << "YA" << std::endl;
    int height = cv_image.rows;
    int width = cv_image.cols;
    double *rmin = (double *) mxGetPr(prhs[1]);
    double *rmax = (double *) mxGetPr(prhs[2]);
    double *xc = (double *) mxGetPr(prhs[3]);
    double *yc = (double *) mxGetPr(prhs[4]);
    double *nrings = (double *) mxGetPr(prhs[5]);
    double *nsectors = (double *) mxGetPr(prhs[6]);
        //cv::Point2i center(width/2.0,height/2.0);
    cv::Point2i center(*xc,*yc);

    std::cout << *xc << std::endl;
    //cv::Point2i center(prhs[1],prhs[2]);
    LogPolar log_polar(width, height, center, *nrings, *rmin, 1, *nsectors, 0);
    //LogPolar log_polar(); 
        std::cout << "YA" << std::endl;

    cv::Mat cortical=log_polar.to_cortical(cv_image);
             std::cout << "YA" << std::endl;

     
    plhs[0]=Converter(cortical);
    /*cv::imshow("rectified_left_image", Inputs[0]);
    

    cv::waitKey(1);*/
    return;
}

