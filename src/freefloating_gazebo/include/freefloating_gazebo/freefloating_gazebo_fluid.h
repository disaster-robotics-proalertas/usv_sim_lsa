#ifndef FREEFLOATINGGAZEBOFLUID_H
#define FREEFLOATINGGAZEBOFLUID_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>

namespace gazebo
{

    class model_st
    {
	public:
	model_st() { std::cerr<<"\n Init model_st()";};
        std::string name;
        physics::ModelPtr model_ptr;
        ros::Publisher state_publisher;
	math::Vector3 waterSurface;
	ros::Subscriber water_subscriber;

	void processSurfaceData(const geometry_msgs::Point::ConstPtr& pt)
	{
		waterSurface.x = pt->x;
		waterSurface.y = pt->y;
		waterSurface.z = pt->z;

		std::cerr<<"\n "<<name<<" m estah com z: "<<waterSurface.z;
	}

	void createSubscriber(ros::NodeHandle *nh, std::string topic)
	{
		water_subscriber = nh->subscribe(topic, 1, &model_st::processSurfaceData, this);
	}
    };

    class link_st
    {
	public: link_st() { std::cerr<<"\n Init link_st()"; waterSurface.Set(0,0,0);};
        std::string model_name;
        physics::LinkPtr link;
        math::Vector3 buoyant_force;
        math::Vector3 buoyancy_center;
        math::Vector3 linear_damping;
        math::Vector3 angular_damping;

	math::Vector3 waterSurface;
	ros::Subscriber water_subscriber;

        double limit;

	void processSurfaceData(const geometry_msgs::Point::ConstPtr& pt)
	{

		waterSurface.x = pt->x;
		waterSurface.y = pt->y;
		waterSurface.z = pt->z;
		std::cerr<<"\n "<<model_name<<" L estah com z: "<<waterSurface.z;
	}

	void createSubscriber(ros::NodeHandle *nh, std::string topic)
	{
		water_subscriber = nh->subscribe(topic, 1, &link_st::processSurfaceData, this);
	}
    };

class FreeFloatingFluidPlugin : public  WorldPlugin
{
public:
    FreeFloatingFluidPlugin() {}
    ~FreeFloatingFluidPlugin()
    {
	ROS_INFO("Closing FreeFloatingFluidPlugin");
        rosnode_->shutdown();
        delete rosnode_;
    }

    virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
    virtual void Update();

private:




    // parse a Vector3 string
    void ReadVector3(const std::string &_string, math::Vector3 &_vector);
    // parse a new model
    void ParseNewModel(const physics::ModelPtr &_model);
    // removes a deleted model
    void RemoveDeletedModel(std::vector<model_st*>::iterator &_model_it);
    // parse received fluid velocity message
    void FluidVelocityCallBack(const geometry_msgs::Vector3ConstPtr& _msg);

private:
    // plugin options
    bool has_surface_;
    math::Vector4 surface_plane_;
    std::string description_;

    // general data
    ros::NodeHandle* rosnode_;
    ros::CallbackQueue callback_queue_;
    physics::WorldPtr world_;
    event::ConnectionPtr update_event_;

    // links that are subject to fluid effects
    std::vector<link_st*> buoyant_links_;
    // models that have been parsed
    std::vector<model_st*> parsed_models_;

    // subscriber to fluid velocity (defined in the world frame)
    ros::Subscriber fluid_velocity_subscriber_;
    math::Vector3 fluid_velocity_;

};
GZ_REGISTER_WORLD_PLUGIN(FreeFloatingFluidPlugin)
}
#endif // FREEFLOATINGGAZEBOFLUID_H
