#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <vector>
/**
*Bilinear interpolation technique.
*
*The value of a desired cortical pixel is obtained through a bilinear interpolation of the values
*of the four nearest neighbouring Cartesian pixels to the center of the RF.
*The same principle is applied to the inverse transformation.
*
*More details can be found in http://dx.doi.org/10.1007/978-3-642-23968-7_5
*/

namespace cv
{
class LogPolar_Interp
{
public:

    LogPolar_Interp() {}

    /**
    *Constructor
    *\param w the width of the input image
    *\param h the height of the input image
    *\param center the transformation center: where the output precision is maximal
    *\param R the number of rings of the cortical image (default value 70 pixel)
    *\param ro0 the radius of the blind spot (default value 3 pixel)
    *\param full \a 1 (default value) means that the retinal image (the inverse transform) is computed within the circumscribing circle.
    *            \a 0 means that the retinal image is computed within the inscribed circle.
    *\param S the number of sectors of the cortical image (default value 70 pixel).
    *         Its value is usually internally computed to obtain a pixel aspect ratio equals to 1.
    *\param sp \a 1 (default value) means that the parameter \a S is internally computed.
    *          \a 0 means that the parameter \a S is provided by the user.
    */
    LogPolar_Interp(int w, int h, Point2i center, int R=70, double ro0=3.0,
                    int interp=INTER_LINEAR, int full=1, int S=117, int sp=1);
    /**
    *Transformation from Cartesian image to cortical (log-polar) image.
    *\param source the Cartesian image
    *\return the transformed image (cortical image)
    */
    const Mat to_cortical(const Mat &source);
    /**
    *Transformation from cortical image to retinal (inverse log-polar) image.
    *\param source the cortical image
    *\return the transformed image (retinal image)
    */
    const Mat to_cartesian(const Mat &source);
    /**
    *Destructor
    */
    ~LogPolar_Interp();

protected:

    Mat Rsri;
    Mat Csri;

    int S, R, M, N;
    int top, bottom,left,right;
    double ro0, romax, a, q;
    int interp;

    Mat ETAyx;
    Mat CSIyx;

    void create_map(int M, int N, int R, int S, double ro0);
};

/**
    *Overlapping circular receptive fields technique
    *
    *The Cartesian plane is divided in two regions: the fovea and the periphery.
    *The fovea (oversampling) is handled by using the bilinear interpolation technique described above, whereas in
    *the periphery we use the overlapping Gaussian circular RFs.
    *
    *More details can be found in http://dx.doi.org/10.1007/978-3-642-23968-7_5
    */
    class LogPolar_Overlapping
    {
    public:
        LogPolar_Overlapping() {}

        /**
        *Constructor
        *\param w the width of the input image
        *\param h the height of the input image
        *\param center the transformation center: where the output precision is maximal
        *\param R the number of rings of the cortical image (default value 70 pixel)
        *\param ro0 the radius of the blind spot (default value 3 pixel)
        *\param full \a 1 (default value) means that the retinal image (the inverse transform) is computed within the circumscribing circle.
        *            \a 0 means that the retinal image is computed within the inscribed circle.
        *\param S the number of sectors of the cortical image (default value 70 pixel).
        *         Its value is usually internally computed to obtain a pixel aspect ratio equals to 1.
        *\param sp \a 1 (default value) means that the parameter \a S is internally computed.
        *          \a 0 means that the parameter \a S is provided by the user.
        */
        LogPolar_Overlapping(int w, int h, Point2i center, int R=70,
                             double ro0=3.0, int full=1, int S=117, int sp=1);
        /**
        *Transformation from Cartesian image to cortical (log-polar) image.
        *\param source the Cartesian image
        *\return the transformed image (cortical image)
        */
        const Mat to_cortical(const Mat &source);
        /**
        *Transformation from cortical image to retinal (inverse log-polar) image.
        *\param source the cortical image
        *\return the transformed image (retinal image)
        */
        const Mat to_cartesian(const Mat &source);
        /**
        *Destructor
        */
        ~LogPolar_Overlapping();

    protected:

        Mat Rsri;
        Mat Csri;
        std::vector<int> Rsr;
        std::vector<int> Csr;
        std::vector<double> Wsr;

        int S, R, M, N, ind1;
        int top, bottom,left,right;
        double ro0, romax, a, q;

        struct kernel
        {
            kernel() { w = 0; }
            std::vector<double> weights;
            int w;
        };

        Mat ETAyx;
        Mat CSIyx;
        std::vector<kernel> w_ker_2D;

        void create_map(int M, int N, int R, int S, double ro0);
    };

    /**
    * Adjacent receptive fields technique
    *
    *All the Cartesian pixels, whose coordinates in the cortical domain share the same integer part, are assigned to the same RF.
    *The precision of the boundaries of the RF can be improved by breaking each pixel into subpixels and assigning each of them to the correct RF.
    *This technique is implemented from: Traver, V., Pla, F.: Log-polar mapping template design: From task-level requirements
    *to geometry parameters. Image Vision Comput. 26(10) (2008) 1354-1370
    *
    *More details can be found in http://dx.doi.org/10.1007/978-3-642-23968-7_5
    */
    class LogPolar_Adjacent
    {
    public:
        LogPolar_Adjacent() {}

        /**
         *Constructor
         *\param w the width of the input image
         *\param h the height of the input image
         *\param center the transformation center: where the output precision is maximal
         *\param R the number of rings of the cortical image (default value 70 pixel)
         *\param ro0 the radius of the blind spot (default value 3 pixel)
         *\param smin the size of the subpixel (default value 0.25 pixel)
         *\param full \a 1 (default value) means that the retinal image (the inverse transform) is computed within the circumscribing circle.
         *            \a 0 means that the retinal image is computed within the inscribed circle.
         *\param S the number of sectors of the cortical image (default value 70 pixel).
         *         Its value is usually internally computed to obtain a pixel aspect ratio equals to 1.
         *\param sp \a 1 (default value) means that the parameter \a S is internally computed.
         *          \a 0 means that the parameter \a S is provided by the user.
         */
        LogPolar_Adjacent(int w, int h, Point2i center, int R=70, double ro0=3.0, double smin=0.25, int full=1, int S=117, int sp=1);
        /**
         *Transformation from Cartesian image to cortical (log-polar) image.
         *\param source the Cartesian image
         *\return the transformed image (cortical image)
         */
        const Mat to_cortical(const Mat &source);
        /**
         *Transformation from cortical image to retinal (inverse log-polar) image.
         *\param source the cortical image
         *\return the transformed image (retinal image)
         */
        const Mat to_cartesian(const Mat &source);
        /**
         *Destructor
         */
        ~LogPolar_Adjacent();

    protected:
        struct pixel
        {
            pixel() { u = v = 0; a = 0.; }
            int u;
            int v;
            double a;
        };
        int S, R, M, N;
        int top, bottom,left,right;
        double ro0, romax, a, q;
        std::vector<std::vector<pixel> > L;
        std::vector<double> A;

        void subdivide_recursively(double x, double y, int i, int j, double length, double smin);
        bool get_uv(double x, double y, int&u, int&v);
        void create_map(int M, int N, int R, int S, double ro0, double smin);
    };


}
