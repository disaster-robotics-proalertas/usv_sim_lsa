#ifndef FREEFLOATINGGAZEBOFLUID_H
#define FREEFLOATINGGAZEBOFLUID_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include "usv_water_current/GetSpeed.h"
#include <thread>

namespace gazebo
{

    class model_st
    {
	public:
	model_st() { std::cerr<<"\n Init model_st()";};
        std::string name;
        physics::ModelPtr model_ptr;
        ros::Publisher state_publisher;
    };

    class link_st
    {
	public: link_st()
		{
			std::cerr<<"\n Init link_st()"; waterSurface.Set(0,0,0);
			usingLocalFluidVelocity = false;
		};
        std::string model_name;
        physics::LinkPtr link;
        math::Vector3 buoyant_force;
        math::Vector3 buoyancy_center;
        math::Vector3 linear_damping;
        math::Vector3 angular_damping;

	math::Vector3 waterSurface;
	ros::Subscriber water_subscriber;

	math::Vector3 fluid_velocity_;
	bool usingLocalFluidVelocity;
	bool usingNoneFluidVelocity;
    ros::Subscriber fluidVelocity_subscriber;
    ros::ServiceClient fluid_velocity_serviceClient_;
    bool running;

        double limit;
        void initServiceClient(ros::NodeHandle* rosnode)
        {
        	std::cerr<<"\n initializing service client";
        	fluid_velocity_serviceClient_ = rosnode->serviceClient<usv_water_current::GetSpeed>("/waterCurrent");
        	std::cerr<<" ... done";
        	running = true;
        }
        void stop() { running = false; }

        ~link_st(){
        	running = false;
			if(the_thread.joinable()) the_thread.join();
		}

        void ThreadLoop() {
        	ros::Rate r(10);
				while(running)
				{

					usv_water_current::GetSpeed srv;
					srv.request.x = waterSurface.x;
					srv.request.y = waterSurface.y;
					if (fluid_velocity_serviceClient_.call(srv))
					{
						fluid_velocity_.x = srv.response.x;
						fluid_velocity_.y = srv.response.y;
						//std::cout << "\n ============== fluidWater "<<model_name<<"="<<link->GetName()<<" ("<<fluid_velocity_.x<<", "<<fluid_velocity_.y<<")";
					}
					else
					{
							ROS_WARN("Failed to call service waterCurrent %s::%s", model_name.c_str(), link->GetName().c_str());

							ros::Rate s(1);
							s.sleep();
					}
					r.sleep();
				}
            }

	void processSurfaceData(const geometry_msgs::Point::ConstPtr& pt)
	{

		waterSurface.x = pt->x;
		waterSurface.y = pt->y;
		waterSurface.z = pt->z;
	}

	void createSubscriberWaterSurface(ros::NodeHandle *nh, std::string topic)
	{
		water_subscriber = nh->subscribe(topic, 1, &link_st::processSurfaceData, this);
	}

	void Start(){
		the_thread = std::thread(&link_st::ThreadLoop,this);
	    }
	private:
	    std::thread the_thread;
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
        for (int i = 0; i < buoyant_links_.size(); i++)
		{
			buoyant_links_[i]->stop();
			delete buoyant_links_[i];
		}


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


    // parse received fluid velocity message
    void FluidVelocityCallBack(const geometry_msgs::Vector3ConstPtr& _msg);

    bool getSpeedCallback(usv_water_current::GetSpeed::Request &req, usv_water_current::GetSpeed::Request &res);
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
    tf::TransformBroadcaster broadcaster;


};
GZ_REGISTER_WORLD_PLUGIN(FreeFloatingFluidPlugin)
}
#endif // FREEFLOATINGGAZEBOFLUID_H
