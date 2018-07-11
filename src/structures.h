#ifndef STRUCTURES_H
#define STRUCTURES_H
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>


struct stereo_disparity_data
{
    cv::Mat disparity_image;
    cv::Mat disparity_values;
    cv::Mat point_cloud_xyz;
    cv::Mat point_cloud_rgb;
    //cv::Mat point_cloud_ego;
};


template <unsigned int DIM>
class Gaussian
{
private:
    Eigen::Matrix<double, DIM, 1> init_mean;
    Eigen::Matrix<double, DIM, DIM> init_info;
    bool empty;
public:
    Eigen::Matrix<double, DIM, DIM> symmetricInverseEigen(const Eigen::Matrix<double, DIM, DIM> & input)
    {
        Eigen::Matrix<double, DIM, DIM> out=(input+input.transpose())/2;
        return out.inverse();
    }
    Eigen::Matrix<double, DIM, 1> mean;
    Eigen::Matrix<double, DIM, DIM> information;
    Eigen::Matrix<double, DIM, DIM> cov;

    void reset()
    {
        meanReset();
        informationReset();
        //empty=true;
    }

    void meanReset()
    {
        mean=init_mean;

    }

    void informationReset()
    {
        information=init_info;
    }

    Gaussian() ://mean(Mat(DIM, 1, CV_64F, cv::Scalar(0))),
        //information(Eigen::Matrix<double, DIM, DIM>::Zero()),
        //mean(Eigen::Matrix<double, DIM,1>::Zero()),
        empty(false)
    {
        reset();
        //information=0.0000001*Eigen::Matrix<double, DIM, DIM>::Identity();
        //cov=10000.0*Eigen::Matrix<double, DIM, DIM>::Identity();
        //mean.setConstant(1000000.0);
    }

    bool isEmpty()
    {
        return empty;
    }

    Gaussian(const Eigen::Matrix<double, DIM, 1> & mean_,
             const Eigen::Matrix<double, DIM, DIM> & information_) :
        mean(mean_),
        information(information_),
        init_mean(mean_),
        init_info(information_),
        empty(false)
    {}



    bool checkOk(const double & sensory_filtering_sphere_radius_,
                 const Eigen::Matrix<double, DIM, DIM> & prev_information_,
                 const Eigen::Matrix<double, DIM, DIM> & observation_information_)
    {
        // INSIDE EGO BOLHA!!
        double mean_norm=mean.norm();
        if(std::isnan(mean_norm)||mean[2]<0.2||mean_norm<sensory_filtering_sphere_radius_)
        {
            //std::cout << "mean_norm:"<<mean_norm<< std::endl;
            //std::cout << "mean z:"<<mean[2]<< std::endl;
            meanReset();
        }
        if(std::isnan(getVolume()))
        {
            /*std::cout << "information_volume:"<<getVolume()<< std::endl;
            std::cout << "observation_information_:"<<observation_information_.determinant()<< std::endl;
            std::cout << "prev_information_:"<<prev_information_.determinant()<< std::endl;
            std::cout << "information:" << information.determinant() << std::endl;//*/
            informationReset();
        }

    }

    bool checkOk(const double & sensory_filtering_sphere_radius_)
    {
        // INSIDE EGO BOLHA!!
        double mean_norm=mean.norm();
        if(std::isnan(mean_norm)||mean[2]<0.2||mean_norm<sensory_filtering_sphere_radius_)
        {
            /*std::cout << "mean_norm:"<<mean_norm<< std::endl;
            std::cout << "mean z:"<<mean[2]<< std::endl;//*/
            meanReset();
        }
        if(std::isnan(getVolume()))
        {
            /*std::cout << "information_volume:"<<getVolume()<< std::endl;
            std::cout << "information:" << information.determinant() << std::endl;//*/
            informationReset();
        }
    }


    void update(const Gaussian & observation,
                const double & mahalanobis_distance_threshold_,
                const double & sensory_filtering_sphere_radius_)
    {
        double mean_norm=observation.mean.norm();
        if(mean_norm<sensory_filtering_sphere_radius_)
        {
            return;
        }

        Eigen::Matrix<double, DIM, DIM> prev_information(information);
        Eigen::Matrix<double, DIM, 1> prev_mean(mean);

        /*if(distance(observation.mean,this->mean,this->information)>mahalanobis_distance_threshold_)
        {
            if(distance(this->mean,observation.mean,observation.information)>mahalanobis_distance_threshold_)
            {
                // Here we could initialize a new gaussian (budget of gaussians) but lets choose the new observation
                information=observation.information;
                mean=observation.mean;
            }
        }
        else*/
        {
            // See gaussian multiplication (gaussian identities)
            information=observation.information+prev_information;
            mean=information.inverse()*(observation.information*observation.mean+prev_information*prev_mean);
        }
        /*if(information.determinant()<init_info.determinant())
        {
            informationReset();
        }*/

        checkOk(sensory_filtering_sphere_radius_,
                prev_information,
                observation.information);

        /*else
        {
            empty=false;
        }*/
    }

    Gaussian (const Gaussian &other)
    {
        if(!std::isnan(other.mean.norm())&&!std::isnan(other.information.determinant()))
        {
            mean=other.mean;
            information=other.information;
            cov=other.cov;
        }
    }

    Gaussian & operator= (const Gaussian & other)
    {
        if(!std::isnan(other.mean.norm())&&!std::isnan(other.information.determinant()))
        {
            mean=other.mean;
            information=other.information;
            cov=other.cov;
        }
    }

    ~Gaussian()
    {}

    double distance(const Eigen::Matrix<double, DIM, 1> & point_,const Eigen::Matrix<double, DIM, 1> & mean_,const Eigen::Matrix<double, DIM, DIM> & information_)
    {
        double distance=((point_-mean_).transpose()*information_*(point_-mean_));
        return distance;
    }

    double getVolume()
    {
        return information.determinant();
    }


    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & mean;
        ar & information;
        ar & cov;
    }

};

class Coordinate
{
public:
    cv::Mat data;

    Coordinate() {}

    Coordinate (const cv::Mat & data_) : data(data_)
    {}

    Coordinate (const Coordinate &other)
    {
        // body of constructor
        data=other.data;
    }


    Coordinate & operator= (const Coordinate & other)
    {

        data=other.data;

    }

    virtual ~Coordinate()
    {}
};

class SphericalCoordinate : public Coordinate
{
public:
    SphericalCoordinate(){}

    /*SphericalCoordinate(const double & _rho, const double & _phi, const double & _theta)
    {
        data=Mat(3, 1, CV_64F);
        data.at<double>(0,0)=_rho;
        data.at<double>(1,0)=_phi;
        data.at<double>(2,0)=_theta;
    }*/

    SphericalCoordinate(const cv::Mat & _data) : Coordinate(_data)
    {}

    ~SphericalCoordinate()
    {}
};

class CartesianCoordinate : public Coordinate
{

public:
    CartesianCoordinate(){}

    /*CartesianCoordinate(const double & _x,const double & _y, const double & _z)
    {
        data=Mat(3, 1, CV_64F);
        data.at<double>(0,0)=_x;
        data.at<double>(1,0)=_y;
        data.at<double>(2,0)=_z;
    }*/

    CartesianCoordinate(const cv::Mat & _data) : Coordinate(_data)
    {}

    ~CartesianCoordinate()
    {}
};

class SensoryData
{
public:
    Gaussian<3> position;
    Gaussian<1> depth;
    Gaussian<1> brightness;
    pcl::PointXYZRGB rgb;

    SensoryData()
    {}

    SensoryData(const Eigen::Matrix<double, 3, 1> & init_mean_,
                const Eigen::Matrix<double, 3, 3> & init_information_):
        position(init_mean_,init_information_)
    {}

    SensoryData(Gaussian<1> & _depth, Gaussian<1> & _brightness):
        depth(_depth),
        brightness(_brightness),
        rgb(0,0,0)
    {}

    SensoryData(Gaussian<1> & _depth, Gaussian<1> & _brightness, const pcl::PointXYZRGB & rgb_):
        depth(_depth),
        brightness(_brightness),
        rgb(rgb_)
    {}


    SensoryData (const SensoryData &other)
    {
        position=other.position;
        depth=other.depth;
        brightness=other.brightness;
        rgb=other.rgb;
    }

    void reset()
    {
        position.reset();
        depth.reset();
        brightness.reset();
    }

    SensoryData & operator= (const SensoryData & other)
    {
        position=other.position;
        depth=other.depth;
        brightness=other.brightness;
        rgb=other.rgb;
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & position;
        ar & brightness;
        ar & depth;
        ar & rgb;
    }
};



// Facet
class MemoryPatch
{
public:
    // Coordinates
    Eigen::Matrix<double, 3, 1> cartesian_position;
    Eigen::Matrix<double, 3, 1> spherical_position;
    //CartesianCoordinate cartesian_position;
    //SphericalCoordinate spherical_position;

    // Sensory data
    SensoryData sensory_data;

    MemoryPatch()
    {
        //sensory_data=boost::shared_ptr<SensoryData>(new SensoryData);
    }

    MemoryPatch(const Eigen::Matrix<double, 3, 1> & cartesian_position_,
                const Eigen::Matrix<double, 3, 1> & init_mean_,
                const Eigen::Matrix<double, 3, 3> & init_information_) :
        cartesian_position(cartesian_position_),
        sensory_data(init_mean_,init_information_)
    {
        //sensory_data=boost::shared_ptr<SensoryData>(new SensoryData);
        // Convert to phi theta
        /*double x=cartesian_position.data.at<double>(0,0);
        double y=cartesian_position.data.at<double>(1,0);
        double z=cartesian_position.data.at<double>(2,0);
        double r=1; // NORMALIZED SPHERE
        double theta=atan2(y,x);
        double phi=acos(z/r);

        spherical_position = SphericalCoordinate(r,phi,theta);*/
    }

    // Receives normalized vector directions
    MemoryPatch(const Eigen::Matrix<double, 3, 1> & cartesian_position_,
                const Eigen::Matrix<double, 3, 1> & spherical_position_) : cartesian_position(cartesian_position_),
        spherical_position(spherical_position_)
    {}

    MemoryPatch (const MemoryPatch &other)
    {
        sensory_data=other.sensory_data;
        cartesian_position=other.cartesian_position;
        spherical_position=other.spherical_position;
    }

    MemoryPatch & operator= (const MemoryPatch & other)
    {
        sensory_data=other.sensory_data;
        cartesian_position=other.cartesian_position;
        spherical_position=other.spherical_position;
    }

    ~MemoryPatch()
    {}

    void update()
    {}

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & cartesian_position;
        ar & spherical_position;
        ar & sensory_data;
    }
};


#endif // STRUCTURES_H
