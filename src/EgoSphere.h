#ifndef SPHERICALSHELL_H
#define SPHERICALSHELL_H
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>

#include <moveit_msgs/GetPositionIK.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/Float64.h>
#include "structures.h"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include "UpperConfidenceBound.h"
const double interpolate_angle_threshold_=15.0;


template<typename T, typename A>
inline std::vector<T, Eigen::aligned_allocator<A> > erase_pcl_indices(const std::vector<T,Eigen::aligned_allocator<A> >& data, std::vector<int>& indicesToDelete/* can't assume copy elision, don't pass-by-value */)
{
    if(indicesToDelete.empty())
        return data;

    std::vector<T,Eigen::aligned_allocator<A> > ret;

    std::sort(indicesToDelete.begin(), indicesToDelete.end());
    indicesToDelete.erase( unique( indicesToDelete.begin(), indicesToDelete.end() ), indicesToDelete.end() );
    ret.reserve(data.size() - indicesToDelete.size());

    // new we can assume there is at least 1 element to delete. copy blocks at a time.
    typename std::vector<T,Eigen::aligned_allocator<A> >::const_iterator itBlockBegin = data.begin();
    for(std::vector<int>::const_iterator it = indicesToDelete.begin(); it != indicesToDelete.end(); ++ it)
    {
        typename std::vector<T,Eigen::aligned_allocator<A> >::const_iterator itBlockEnd = data.begin() + *it;
        if(itBlockBegin != itBlockEnd)
        {
            std::copy(itBlockBegin, itBlockEnd, std::back_inserter(ret));
        }
        itBlockBegin = itBlockEnd + 1;
    }

    // copy last block.
    if(itBlockBegin != data.end())
    {
        std::copy(itBlockBegin, data.end(), std::back_inserter(ret));
    }

    return ret;
}

template<typename T>
inline std::vector<T> erase_indices(const std::vector<T>& data, std::vector<int>& indicesToDelete/* can't assume copy elision, don't pass-by-value */)
{
    if(indicesToDelete.empty())
        return data;

    std::vector<T> ret;
    ret.reserve(data.size() - indicesToDelete.size());

    std::sort(indicesToDelete.begin(), indicesToDelete.end());

    // new we can assume there is at least 1 element to delete. copy blocks at a time.
    typename std::vector<T>::const_iterator itBlockBegin = data.begin();
    for(std::vector<int>::const_iterator it = indicesToDelete.begin(); it != indicesToDelete.end(); ++ it)
    {
        typename std::vector<T>::const_iterator itBlockEnd = data.begin() + *it;
        if(itBlockBegin != itBlockEnd)
        {
            std::copy(itBlockBegin, itBlockEnd, std::back_inserter(ret));
        }
        itBlockBegin = itBlockEnd + 1;
    }

    // copy last block.
    if(itBlockBegin != data.end())
    {
        std::copy(itBlockBegin, data.end(), std::back_inserter(ret));
    }

    return ret;
}




template <class T>
class SphericalShell
{
public:
    double pan_abs_limit;
    double tilt_abs_limit;
    int init_fixation_point_index;
    T structure;
    T structure_aux;
    std::vector<SensoryData> sensory_data_aux;
    pcl::PointCloud<pcl::PointXYZ>::Ptr structure_cloud;
    SphericalShell() : north_direction(Eigen::Vector3d::UnitZ()),
        robot_model_loader("robot_description"),
        robot_model(robot_model_loader.getModel()),
        head_joint_model_group(robot_model->getJointModelGroup("head")),
        head_group(new moveit::planning_interface::MoveGroup("head"))
    {
        head_joint_names=head_group->getActiveJoints();
        head_joint_values.resize(head_joint_names.size());
    }

    // NOTE: ANGLE THRESHOLD IS IN DEGREES
    SphericalShell(const unsigned int & _egosphere_nodes,
                   const double & mahalanobis_distance_threshold_,
                   const cv::Mat & mean_,
                   const cv::Mat & standard_deviation_,
                   const double & y_offset_,
                   const double & neighbour_angle_threshold_,
                   const double & init_scale_mean_,
                   const double & init_scale_information_,
                   const double & pan_abs_limit_,
                   const double & tilt_abs_limit_) :
        egosphere_nodes(_egosphere_nodes),
        mahalanobis_distance_threshold(mahalanobis_distance_threshold_),
        // Moveit stuff
        robot_model_loader("robot_description"),
        robot_model(robot_model_loader.getModel()),
        head_joint_model_group(robot_model->getJointModelGroup("head")),
        head_group(new moveit::planning_interface::MoveGroup("head")),
        y_offset(y_offset_),
        neighbour_dot_product_threshold(cos(neighbour_angle_threshold_*M_PI/180.0)),
        interpolate_dot_product_threshold(cos(interpolate_angle_threshold_*M_PI/180.0)),
        north_direction(Eigen::Vector3d(0,0,1.0)),
        pan_abs_limit(pan_abs_limit_),
        tilt_abs_limit(tilt_abs_limit_)
    {
        sensory_data_aux.reserve(egosphere_nodes);
        structure_aux.reserve(egosphere_nodes);
        structure_cloud=pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
        head_joint_names=head_group->getActiveJoints();
        head_joint_values.resize(head_joint_names.size());
        std::cout << head_joint_names[0] << " " << head_joint_names[1] << " " << head_joint_names[2] << std::endl;

        init(mean_,standard_deviation_,init_scale_mean_,init_scale_information_);
    }

    bool checkKinematics(const Eigen::Vector3d & fixation_point)
    {
        Eigen::Vector3d fixation_point_normalized=fixation_point.normalized();
        std_msgs::Float64 neck_pan_angle;
        std_msgs::Float64 neck_tilt_angle;
        if(fixation_point_normalized.x()!=fixation_point_normalized.x())
        {
            neck_pan_angle.data=0.0;
            neck_tilt_angle.data=0.0;
        }
        else
        {
            double x=fixation_point_normalized.x();
            double y=fixation_point_normalized.y();
            double z=fixation_point_normalized.z();

            double neck_pan_angle_=atan2(x,z);
            double tilt_angle;
            double aux=(-y*y_offset + sqrt((x*x + z*z)*(x*x + y*y - y_offset*y_offset + z*z)));
            if(y<-y_offset)
                tilt_angle=acos(aux/(fixation_point_normalized.squaredNorm()));
            else
                tilt_angle=-acos(aux/(fixation_point_normalized.squaredNorm()));

            neck_pan_angle.data=neck_pan_angle_;
            neck_tilt_angle.data=tilt_angle;
        }

        head_joint_values[0] = neck_pan_angle.data;
        head_joint_values[1] = neck_tilt_angle.data;
        head_joint_values[2] = 0;

        if(fabs(neck_pan_angle.data)>pan_abs_limit||fabs(neck_tilt_angle.data)>tilt_abs_limit)
        {
            return false;
        }

        if(!head_group->setJointValueTarget(head_joint_values))
        {
            return false;
        }

        return true;
    }
    void init(const cv::Mat & mean_mat,
              const cv::Mat & std_dev_mat,
              const double & init_scale_mean,
              const double & init_scale_information)
    {
        std::cout << "initializing random spherical shell..." << std::endl;
        std_msgs::Float64 neck_pan_angle;
        std_msgs::Float64 neck_tilt_angle;

        //head_group->setNamedTarget("head_home");
        //head_group->move();
        double fixation_point_aux_dot=-10000.0;
        int p=0;
        while(p<egosphere_nodes)
        {
            Eigen::Vector3d random_point;
            cv::Mat aux(1, 1, CV_64F);

            // Generate random patch on the sphere surface
            cv::randn(aux, mean_mat.at<double>(0,0), std_dev_mat.at<double>(0,0));
            random_point(0,0)=aux.at<double>(0,0);

            cv::randn(aux, mean_mat.at<double>(1,0), std_dev_mat.at<double>(1,0));
            random_point(1,0)=aux.at<double>(0,0);

            cv::randn(aux, mean_mat.at<double>(2,0), std_dev_mat.at<double>(2,0));
            random_point(2,0)=aux.at<double>(0,0);
            // If not kinematically possible, continue

            Eigen::Vector3d fixation_point;

            fixation_point(0)=random_point(0,0);
            fixation_point(1)=random_point(1,0);
            fixation_point(2)=random_point(2,0);
            if(!checkKinematics(fixation_point))
            {
                continue;
            }
            Eigen::Vector3d fixation_point_normalized=fixation_point.normalized();

            // Store
            Eigen::Matrix<double,3,1> init_mean=init_scale_mean*fixation_point_normalized;
            Eigen::Matrix<double,3,3> init_information=init_scale_information*Eigen::Matrix<double, 3, 3>::Identity();
            boost::shared_ptr<MemoryPatch> patch(new MemoryPatch(fixation_point_normalized,
                                                                 init_mean,
                                                                 init_information));

            structure.push_back(patch);

            ///////////////////////////////
            // Insert in pcl point cloud //
            ///////////////////////////////

            pcl::PointXYZ pcl_point;
            pcl_point.getVector3fMap() = fixation_point_normalized.cast<float>();
            structure_cloud->push_back(pcl_point);

            // store init fixation point
            double aux_fixation_point=north_direction.dot(fixation_point_normalized);
            if(aux_fixation_point>fixation_point_aux_dot)
            {
                fixation_point_aux_dot=aux_fixation_point;
                init_fixation_point_index=p;
            }
            ++p;

            std::cout << "p:"<< p << std::endl;
        }

        structure_cloud_kdtree.setInputCloud (structure_cloud);
        std::cout << "done." << std::endl;
    }

    /*void resample(const boost::shared_ptr<AcquisitionFunction> &acquisition_function)
    {
        std::cout << "resample..." << std::endl;
        std::vector<double> means_;
        means_.resize(structure.size());
        std::fill(means_.begin(),means_.end(),-std::numeric_limits<double>::max());

        std::vector<double> sigmas_;
        sigmas_.resize(structure.size());
        std::fill(sigmas_.begin(),sigmas_.end(),0.0);

        //////////////////////////////////////////////
        // 1. convert data to an objective function //
        //////////////////////////////////////////////

        for(int i=0; i< structure.size(); ++i)
        {
            // Linearize 1/sqrt(x*x + y*y + z*z)
            double mean=structure[i]->sensory_data.position.mean.norm();
            if(std::isnan(mean))
            {
                ROS_FATAL("ESTA MAL!!!!");
                continue;
            }

            Eigen::Vector3d jacobian(structure[i]->sensory_data.position.mean.x()/mean,
                                     structure[i]->sensory_data.position.mean.y()/mean,
                                     structure[i]->sensory_data.position.mean.z()/mean);

            double sigma=sqrt( jacobian.transpose()*structure[i]->sensory_data.position.information.inverse()*jacobian );

            if(std::isnan(sigma))
            {
                continue;
            }

            means_[i]=-mean;
            sigmas_[i]=sigma;
        }

        /////////////////////////////////////////////////
        // 2. get values from the acquisition function //
        /////////////////////////////////////////////////

        std::vector<double> weights_=acquisition_function->getValues(means_,sigmas_);
        double sum=0;
        for(int i=0; i<weights_.size();++i)
        {
            sum+=weights_[i];
        }
        std::cout << "SUMMM: "<< sum << std::endl;
        /////////////////////////////
        // 3. Low variance sampler //
        /////////////////////////////
        ///
        ///
        /// // Probablistic robotics table 4.4 (page 110)
        ///
        structure_aux.clear();
        for(int m=0; m<egosphere_nodes;++m)
        {
            structure_aux.push_back(structure[m]);
        }
        structure.clear();
        structure_cloud->clear();
        std::srand( std::time(0) );
        double m_inv_aux=(1.0/(double)egosphere_nodes);
        double r=m_inv_aux*((double)rand() / (double)(RAND_MAX));
        double c=weights_[0];
        int i=0;
        double noise_std_dev=0.05;

        for(int m=0; m<egosphere_nodes;++m)
        {
            double U=r+m*m_inv_aux;
            while(U>c)
            {
                ++i;
                c+=weights_[i];
            }

            Eigen::Matrix<double,3,1> new_point;

            // Generate new sample
            do
            {
                Eigen::Matrix<double,3,1> random_noise;
                cv::Mat aux(1, 1, CV_64F);

                // Generate gaussian addictive noise
                cv::randn(aux, 0.0, noise_std_dev);
                random_noise(0,0)=aux.at<double>(0,0);

                cv::randn(aux, 0.0, noise_std_dev);
                random_noise(1,0)=aux.at<double>(0,0);

                cv::randn(aux, 0.0, noise_std_dev);
                random_noise(2,0)=aux.at<double>(0,0);

                new_point=structure_aux[i]->sensory_data.position.mean+random_noise;
            }
            while(!checkKinematics(new_point)&&ros::ok());

            structure_aux[i]->sensory_data.position.mean=new_point;
            structure_aux[i]->sensory_data.position.information.noalias()=structure_aux[i]->sensory_data.position.information+noise_std_dev*noise_std_dev*Eigen::Matrix<double, 3, 3>::Identity();
            structure_aux[i]->cartesian_position=structure_aux[i]->sensory_data.position.mean.normalized();

            // Add new sample
            structure.push_back(structure_aux[i]);
            pcl::PointXYZ pcl_point;

            pcl_point.getVector3fMap() = structure_aux[i]->cartesian_position.template cast<float>();

            //std::cout << structure_aux[i]->cartesian_position.norm() << std::endl;
            structure_cloud->push_back(pcl_point);
        }
        structure_cloud_kdtree.setInputCloud (structure_cloud);

        std::cout << "done." << std::endl;
    }*/

    void insertKdTree(const pcl::PointCloud<pcl::PointXYZRGB> & observations,
                      const std::vector<Eigen::Matrix3d> & covariances,
                      const Eigen::Vector3d & sensor_direction,
                      const double & field_of_view,
                      const double & sensory_filtering_sphere_radius)
    {
        double cos_field_of_view=cos(0.5*field_of_view*M_PI/180.0);
        // K nearest neighbor search
        if (observations.size()>0)
        {
            int K = 1;
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
            std::vector<int> obs_interp_indices;
            obs_interp_indices.reserve(observations.size());

            // Construct observations tree
            pcl::PointCloud<pcl::PointXYZ> observations_normalized;
            observations_normalized.reserve(observations.size());
            for(int o=0; o<observations.size();++o)
            {
                pcl::PointXYZ point_normalized;
                point_normalized.getVector3fMap() = observations[o].getVector3fMap().normalized();
                observations_normalized.push_back(point_normalized);
            }

            pcl::KdTreeFLANN<pcl::PointXYZ> observations_cloud_kdtree;
            observations_cloud_kdtree.setInputCloud (observations_normalized.makeShared());

            // FIRST INTERPOLATE REGION CLOSE TO SENSOR CENTER TO GUARANTEE THAT THE FIXATION POINT IS OBSERVED
            for(std::vector<boost::shared_ptr<MemoryPatch> >::iterator structure_it = structure.begin(); structure_it != structure.end(); ++structure_it)
            {
                // Check if it is within the field of view of the sensor
                // INTERPOLATION REGION
                double dot_product=(*structure_it)->cartesian_position.dot(sensor_direction);
                if(dot_product>interpolate_dot_product_threshold&&dot_product>cos_field_of_view)
                {
                    // 1. Update structure cell vector
                    pcl::PointXYZ searchPoint;
                    searchPoint.getVector3fMap() = (*structure_it)->cartesian_position.cast<float>();

                    if ( observations_cloud_kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
                    {
                        unsigned int nn_index=pointIdxNKNSearch[0];

                        Gaussian<3> gaussian_cartesian_position(observations[nn_index].getArray3fMap().cast<double>(),covariances[nn_index]);

                        // update position
                        (*structure_it)->sensory_data.position.update(gaussian_cartesian_position,mahalanobis_distance_threshold, sensory_filtering_sphere_radius);

                        // update rgb
                        (*structure_it)->sensory_data.rgb=observations[nn_index];
                        obs_interp_indices.push_back(nn_index);

                        // Check if it is within the dot product threshold ( faster than angle )

                    }
                    // Extract the inliers
                    /*extract.setInputCloud (cloud_filtered);
                    extract.setIndices (inliers);
                    extract.setNegative (false);
                    extract.filter (*cloud_p);*/
                }
            }
            pcl::PointCloud<pcl::PointXYZRGB> observations_(observations);

            observations_normalized.points=erase_pcl_indices<pcl::PointXYZ, pcl::PointXYZ>(observations_normalized.points,obs_interp_indices);

            observations_.points=erase_pcl_indices<pcl::PointXYZRGB, pcl::PointXYZRGB>(observations_.points,obs_interp_indices);

            // THEN PUT THE REST OF OBSERVATIONS ACCORDING TO THE ARGMAX CRITERIUM
            for(int i=0; i<observations_.size();++i)
            {
                double dot_product=sensor_direction.dot(observations_normalized[i].getVector3fMap().cast<double>());

                if(dot_product>cos_field_of_view)
                {

                    Gaussian<3> gaussian_cartesian_position(observations_[i].getArray3fMap().cast<double>(),covariances[i]);

                    if ( structure_cloud_kdtree.nearestKSearch (observations_normalized[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
                    {
                        unsigned int nn_index=pointIdxNKNSearch[0];

                        double dot_product=structure[nn_index]->cartesian_position.dot(observations_normalized[i].getVector3fMap().cast<double>());
                        if(dot_product>neighbour_dot_product_threshold)
                        {
                            // update xyz
                            structure[nn_index]->sensory_data.position.update(gaussian_cartesian_position,mahalanobis_distance_threshold, sensory_filtering_sphere_radius);

                            // update rgb
                            structure[nn_index]->sensory_data.rgb=observations_[i];
                        }
                    }
                }
            }
        }
    }

    // Update when moved
    bool transform(const Eigen::Matrix4d & pose_shift_m,
                   const bool & update_mode,
                   const double & sensory_filtering_sphere_radius)
    {
        Eigen::AngleAxis<double> rot(pose_shift_m.block<3,3>(0,0));

        /*// CONSIDER ONLY DISPLACEMENTS ABOVE A THRESHOLD
        if(rot.angle()<0.02&&pose_shift_m.block<3,1>(0,3).norm()<0.01)
        {
            return false;
        }//*/

        ////////////////////////////////////
        // 1. update ego sphere structure //
        ////////////////////////////////////

        Eigen::Transform<double, 3, Eigen::Affine> pose_shift_transform(pose_shift_m);

        // update north
        Eigen::Vector3d aux=pose_shift_transform.inverse() * north_direction;
        north_direction=aux;

        //ROS_ERROR_STREAM(pose_shift_transform.matrix());
        structure_cloud->clear();
        //sensory_data_aux.clear();

        for(std::vector<boost::shared_ptr<MemoryPatch> >::iterator structure_it = structure.begin(); structure_it != structure.end(); ++structure_it)
        {
            // 1. Update structure cell vector
            Eigen::Vector3d aux=pose_shift_transform * (*structure_it)->cartesian_position;
            (*structure_it)->cartesian_position=aux.normalized();
            pcl::PointXYZ pcl_point;
            pcl_point.getVector3fMap() = (*structure_it)->cartesian_position.cast<float>();
            structure_cloud->push_back(pcl_point);

            // 2. Transform sensory_data and find new associated cell
            // Transform only informative points
            if(!(*structure_it)->sensory_data.position.isEmpty())
            {
                // TRANSFORM MEAN
                (*structure_it)->sensory_data.position.mean.noalias()=pose_shift_transform*(*structure_it)->sensory_data.position.mean;

                // ROTATE INFORMATION MATRICES
                (*structure_it)->sensory_data.position.information.noalias()=pose_shift_m.block(0,0,3,3)*(*structure_it)->sensory_data.position.information*pose_shift_m.block(0,0,3,3).transpose();

                (*structure_it)->sensory_data.position.checkOk(sensory_filtering_sphere_radius);
                //sensory_data_aux.push_back((*structure_it)->sensory_data);

                //(*structure_it)->sensory_data.reset();
            }
        }
        structure_cloud_kdtree.setInputCloud (structure_cloud);

        return true;

        //////////////////////////////////////
        // 2. update sensory data (re-fused)//
        //////////////////////////////////////

        // K nearest neighbor search
        if (sensory_data_aux.size()>0)
        {
            int K = 1;
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);

            // Construct observations tree
            pcl::PointCloud<pcl::PointXYZ> observations_normalized;
            observations_normalized.reserve(sensory_data_aux.size());
            for(std::vector<SensoryData>::iterator sensory_data_it = sensory_data_aux.begin(); sensory_data_it != sensory_data_aux.end(); ++sensory_data_it)
            {
                pcl::PointXYZ point_normalized;
                point_normalized.getVector3fMap() = sensory_data_it->position.mean.normalized().cast<float>();
                observations_normalized.push_back(point_normalized);
            }

            pcl::KdTreeFLANN<pcl::PointXYZ> observations_cloud_kdtree;
            observations_cloud_kdtree.setInputCloud (observations_normalized.makeShared());



            // Option one: from observations to the egosphere (DATA NEVER GETS LOST AND IS NOT RECPLICATED)
            if(update_mode)
            {
                for(int i=0; i<observations_normalized.points.size(); ++i)
                {

                    if ( structure_cloud_kdtree.nearestKSearch (observations_normalized[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
                    {
                        unsigned int nn_index=pointIdxNKNSearch[0];

                        // update xyz
                        structure[nn_index]->sensory_data.position.update(sensory_data_aux[i].position,mahalanobis_distance_threshold,0.0);

                        // update rgb
                        structure[nn_index]->sensory_data.rgb=sensory_data_aux[i].rgb;

                    }
                }
            }
            // Option two: from the egosphere to the observations (DATA GETS RECPLICATED EVERYTIME WE MOVE)
            else
            {
                for(std::vector<boost::shared_ptr<MemoryPatch> >::iterator structure_it = structure.begin(); structure_it != structure.end(); ++structure_it)
                {
                    // 1. Update structure cell vector
                    pcl::PointXYZ searchPoint;
                    searchPoint.getVector3fMap() = (*structure_it)->cartesian_position.cast<float>();

                    if ( observations_cloud_kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
                    {
                        unsigned int obs_nn_index=pointIdxNKNSearch[0];

                        // Check if it is within the dot product threshold ( faster than angle )
                        double dot_product=(*structure_it)->cartesian_position.dot(observations_normalized[obs_nn_index].getVector3fMap().cast<double>());
                        if(dot_product>neighbour_dot_product_threshold)
                        {
                            // update xyz
                            (*structure_it)->sensory_data.position.update(sensory_data_aux[obs_nn_index].position,mahalanobis_distance_threshold,0.0);

                            // update rgb
                            (*structure_it)->sensory_data.rgb=sensory_data_aux[obs_nn_index].rgb;
                        }
                    }
                }
            }
        }
        return true;
    }

    pcl::PointCloud<pcl::PointXYZRGB> getPointCloud()
    {
        pcl::PointCloud<pcl::PointXYZRGB> point_cloud;

        for(std::vector<boost::shared_ptr<MemoryPatch> >::iterator structure_it = structure.begin(); structure_it != structure.end(); ++structure_it)
        {
            double volume=(*structure_it)->sensory_data.position.getVolume();

            if(volume!=volume)
            {
                continue;
            }

            pcl::PointXYZRGB point((*structure_it)->sensory_data.rgb);

            point.getVector3fMap() = (*structure_it)->sensory_data.position.mean.cast<float>();
            point_cloud.push_back(point);
        }

        return point_cloud;
    }


    pcl::PointCloud<pcl::PointXYZI> getPointCloudUncertainty()
    {
        pcl::PointCloud<pcl::PointXYZI> point_cloud;

        for(std::vector<boost::shared_ptr<MemoryPatch> >::iterator structure_it = structure.begin(); structure_it != structure.end(); ++structure_it)
        {
            //double norm_=(information_matrix.norm()); // L2 norm
            double volume=(*structure_it)->sensory_data.position.getVolume();

            if(std::isnan(log(volume)) || volume!=volume || std::isinf(log(volume)))
            {
                continue;
            }

            pcl::PointXYZI point;
            point.intensity=volume;

            point.getVector3fMap() = (*structure_it)->sensory_data.position.mean.cast<float>();
            point_cloud.push_back(point);


        }


        return point_cloud;
    }

    pcl::PointCloud<pcl::PointXYZI> getPointCloudUncertaintyViz()
    {
        pcl::PointCloud<pcl::PointXYZI> point_cloud;

        for(std::vector<boost::shared_ptr<MemoryPatch> >::iterator structure_it = structure.begin(); structure_it != structure.end(); ++structure_it)
        {
            //double norm_=(information_matrix.norm()); // L2 norm
            double volume=(*structure_it)->sensory_data.position.getVolume();

            if(std::isnan(log(volume)) ||volume!=volume || std::isinf(log(volume)))
            {
                continue;
            }

            pcl::PointXYZI point;
            point.intensity=log(volume);

            point.getVector3fMap() = (*structure_it)->sensory_data.position.mean.cast<float>();
            point_cloud.push_back(point);
        }

        return point_cloud;
    }

    std::vector<Eigen::Matrix<double, 3, 3> > getCovariances()
    {
        std::vector<Eigen::Matrix<double, 3, 3> > covs;

        for(std::vector<boost::shared_ptr<MemoryPatch> >::iterator structure_it = structure.begin(); structure_it != structure.end(); ++structure_it)
        {
            //double norm_=(information_matrix.norm()); // L2 norm
            double volume=(*structure_it)->sensory_data.position.getVolume();

            if(std::isnan(log(volume)) ||volume!=volume)
            {
                continue;
            }

            //double norm_=(information_matrix.norm()); // L2 norm
            covs.push_back((*structure_it)->sensory_data.position.information.inverse());
        }

        return covs;
    }

    pcl::PointCloud<pcl::PointXYZ> getMeans()
    {
        pcl::PointCloud<pcl::PointXYZ> point_cloud;

        for(std::vector<boost::shared_ptr<MemoryPatch> >::iterator structure_it = structure.begin(); structure_it != structure.end(); ++structure_it)
        {
            //double norm_=(information_matrix.norm()); // L2 norm
            double volume=(*structure_it)->sensory_data.position.getVolume();

            if(std::isnan(log(volume)) || volume!=volume)
            {
                continue;
            }

            pcl::PointXYZ point;

            point.getVector3fMap() = (*structure_it)->sensory_data.position.mean.cast<float>();
            point_cloud.push_back(point);
        }

        return point_cloud;
    }

private:

    //void initFovGeodesicDome();
    unsigned int egosphere_nodes;
    double neighbour_dot_product_threshold;
    double y_offset;
    double mahalanobis_distance_threshold;
    double field_of_view;
    double interpolate_dot_product_threshold;



    pcl::KdTreeFLANN<pcl::PointXYZ> structure_cloud_kdtree;

    //std::vector<std::vector<std::vector<boost::shared_ptr<MemoryPatch> > > > hash_table; // Phi , Theta

    Eigen::Vector3d north_direction;

    // MoveIt! stuff
    moveit::core::RobotStatePtr kinematic_state;
    robot_model_loader::RobotModelLoader robot_model_loader;//("robot_description");
    moveit::core::RobotModelPtr robot_model;// = robot_model_loader.getModel();

    const moveit::core::JointModelGroup* head_joint_model_group;//kinematic_model->getJointModelGroup("head");
    moveit::planning_interface::MoveGroup* head_group;

    std::vector<double> head_joint_values;
    std::vector<std::string> head_joint_names;

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & egosphere_nodes;
        ar & y_offset;
        ar & mahalanobis_distance_threshold;
        ar & structure;
        ar & structure_cloud;
        ar & sensory_data_aux;
        ar & structure_aux;
        ar & neighbour_dot_product_threshold;
        ar & interpolate_dot_product_threshold;
        ar & north_direction;
        ar & init_fixation_point_index;
        ar & pan_abs_limit;
        ar & tilt_abs_limit;
        structure_cloud_kdtree.setInputCloud (structure_cloud);
    }
};


namespace boost{
namespace serialization{

template<   class Archive,
            class S,
            int Rows_,
            int Cols_,
            int Ops_,
            int MaxRows_,
            int MaxCols_>
inline void save(
        Archive & ar,
        const Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_> & g,
        const unsigned int version)
{
    int rows = g.rows();
    int cols = g.cols();

    ar & rows;
    ar & cols;
    ar & boost::serialization::make_array(g.data(), rows * cols);
}

template<   class Archive,
            class S,
            int Rows_,
            int Cols_,
            int Ops_,
            int MaxRows_,
            int MaxCols_>
inline void load(
        Archive & ar,
        Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_> & g,
        const unsigned int version)
{
    int rows, cols;
    ar & rows;
    ar & cols;
    g.resize(rows, cols);
    ar & boost::serialization::make_array(g.data(), rows * cols);
}

template<   class Archive,
            class S,
            int Rows_,
            int Cols_,
            int Ops_,
            int MaxRows_,
            int MaxCols_>
inline void serialize(
        Archive & ar,
        Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_> & g,
        const unsigned int version)
{
    split_free(ar, g, version);
}


} // namespace serialization
} // namespace



namespace boost
{
namespace serialization
{


template<class Archive>
void serialize(Archive & ar, pcl::PointCloud<pcl::PointXYZ>& g, const unsigned int version)
{
    ar & g.points;
}

template<class Archive>
void serialize(Archive & ar, pcl::PointXYZ& g, const unsigned int version)
{
    ar & g.getVector3fMap().data()[0];
    ar & g.getVector3fMap().data()[1];
    ar & g.getVector3fMap().data()[2];
}

template<class Archive>
void serialize(Archive & ar, pcl::PointXYZRGB& g, const unsigned int version)
{
    ar & g.getVector3fMap().data()[0];
    ar & g.getVector3fMap().data()[1];
    ar & g.getVector3fMap().data()[2];
    ar & g.r;
    ar & g.g;
    ar & g.b;
}

} // namespace serialization
} // namespace boost

#endif // SPHERICALSHELL_H
