
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <gazebo/gazebo.hh>

#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/math/Pose.hh>

#include <freefloating_gazebo/freefloating_gazebo_control.h>

using std::cout;
using std::endl;
using std::string;

namespace gazebo
{

void FreeFloatingControlPlugin::ReadVector3(const std::string &_string, math::Vector3 &_vector)
{
    std::stringstream ss(_string);
    double xyz[3];
    for(unsigned int i=0;i<3;++i)
        ss >> xyz[i];
    _vector.Set(xyz[0], xyz[1], xyz[2]);
}

bool FreeFloatingControlPlugin::SwitchService(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    if(controller_is_running_)
        ROS_INFO("Switching freefloating_control OFF");
    else
        ROS_INFO("Switching freefloating_control ON");
    controller_is_running_ = !controller_is_running_;
    return true;
}

void FreeFloatingControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // get model and name
    model_ = _model;
    robot_namespace_ = model_->GetName();
    controller_is_running_ = true;

    // register ROS node & time
    rosnode_ = ros::NodeHandle(robot_namespace_);
    ros::NodeHandle control_node(rosnode_, "controllers");
    t_prev_ = 0;

    // get surface Z
    rosnode_.getParam("/gazebo/surface", z_surface_);

    // check for body or joint param
    control_body_ = false;
    control_joints_ = false;

    while(!(control_body_ || control_joints_))
    {
        control_body_ = control_node.hasParam("config/body");
        control_joints_ = control_node.hasParam("config/joints");
    }

    // *** SET UP BODY CONTROL
    char param[FILENAME_MAX];
    std::string body_command_topic, body_state_topic;
    unsigned int i,j;
    if(control_body_)
    {
        // read topics
        control_node.param("config/body/command", body_command_topic, std::string("body_command"));
        control_node.param("config/body/state", body_state_topic, std::string("body_state"));

        // read control type: thruster vs wrench
        wrench_control_ = true;
        if(control_node.hasParam("config/body/control_type"))
        {
            string control_type;
            control_node.getParam("config/body/control_type", control_type);
            if(control_type == "thruster")
            {
                wrench_control_ = false;
                body_command_topic = "thruster_command";
            }
        }

        if(_sdf->HasElement("link"))
            body_ = model_->GetLink(_sdf->Get<std::string>("link"));
        else
            body_ = model_->GetLinks()[0];

        // parse thruster elements
        thruster_names_.clear();
        thruster_links_.clear();
        thruster_fixed_idx_.clear();
        thruster_steer_idx_.clear();
        thruster_map_.resize(6,0);
        double map_coef;
        sdf::ElementPtr sdf_element = _sdf->GetFirstElement();
        while(sdf_element)
        {
            if(sdf_element->GetName() == "thruster")
            {
                // check if it is a steering or fixed thruster
                if(sdf_element->HasElement("map"))  // fixed
                {
                    // add this index to fixed thrusters
                    thruster_fixed_idx_.push_back(thruster_names_.size());

                    // register map coefs
                    std::stringstream ss(sdf_element->Get<std::string>("map"));
                    const unsigned int nb = thruster_map_.cols();
                    thruster_map_.conservativeResize(6,nb+1);
                    for(i=0;i<6;++i)
                    {
                        ss >> map_coef;
                        thruster_map_(i,nb) = map_coef;
                    }

                    // check for any name
                    if(sdf_element->HasElement("name"))
                        thruster_names_.push_back(sdf_element->Get<std::string>("name"));
                    else
                    {
                        std::ostringstream name;
                        name << "thr" << thruster_names_.size();
                        thruster_names_.push_back(name.str());
                    }

                    ROS_INFO("Adding %s as a fixed thruster", thruster_names_[thruster_names_.size()-1].c_str());
                }
                else if(sdf_element->HasElement("name"))
                {
                    // add this index to steering thrusters
                    thruster_steer_idx_.push_back(thruster_names_.size());

                    // add the link name
                    thruster_names_.push_back(sdf_element->Get<std::string>("name"));

                    // find and register corresponding link
                    thruster_links_.push_back(model_->GetLink(sdf_element->Get<std::string>("name")));

                    ROS_INFO("Adding %s as a steering thruster", thruster_names_[thruster_names_.size()-1].c_str());
                }

                // register maximum effort
                if(sdf_element->HasElement("effort"))
                    thruster_max_command_.push_back(sdf_element->Get<double>("effort"));
                else
                    thruster_max_command_.push_back(100);
            }
            sdf_element = sdf_element->GetNextElement();
        }
        // check consistency
        if(wrench_control_ && thruster_steer_idx_.size())
        {
            ROS_WARN("%s has steering thrusters and cannot be controlled with wrench", model_->GetName().c_str());
            wrench_control_ = false;
        }

        // initialize subscriber to body commands
        thruster_command_.resize(thruster_names_.size());
        // initialize publisher to thruster_use
        thruster_use_.name = thruster_names_;
        thruster_use_.position.resize(thruster_max_command_.size());
        thruster_use_publisher_ = rosnode_.advertise<sensor_msgs::JointState>("thruster_use", 1);

        ros::SubscribeOptions ops;
        if(wrench_control_)
        {
            ops = ros::SubscribeOptions::create<geometry_msgs::Wrench>(
                        body_command_topic, 1,
                        boost::bind(&FreeFloatingControlPlugin::BodyCommandCallBack, this, _1),
                        ros::VoidPtr(), &callback_queue_);
        }
        else
        {
            ops = ros::SubscribeOptions::create<sensor_msgs::JointState>(
                        body_command_topic, 1,
                        boost::bind(&FreeFloatingControlPlugin::ThrusterCommandCallBack, this, _1),
                        ros::VoidPtr(), &callback_queue_);
        }
        body_command_subscriber_ = rosnode_.subscribe(ops);
        body_command_received_ = false;

        // if wrench control, compute map pseudo-inverse and help PID's with maximum forces by axes
        std::string axe[] = {"x", "y", "z", "roll", "pitch", "yaw"};
        std::vector<std::string> controlled_axes;
        if(thruster_fixed_idx_.size() && !thruster_steer_idx_.size())
        {
            ComputePseudoInverse(thruster_map_, thruster_inverse_map_);

            // push control data to parameter server
            control_node.setParam("config/body/link", body_->GetName());

            unsigned int thr_i, dir_i;

            // compute max force in each direction, writes controlled axes
            double thruster_max_effort;

            for(dir_i=0;dir_i<6;++dir_i)
            {
                thruster_max_effort = 0;
                for(thr_i=0;thr_i<thruster_fixed_idx_.size();++thr_i)
                    thruster_max_effort += thruster_max_command_[thruster_fixed_idx_[thr_i]] * std::abs(thruster_map_(dir_i,thr_i));
                if(thruster_max_effort != 0)
                {
                    controlled_axes.push_back(axe[dir_i]);
                    // push to position PID
                    sprintf(param, "%s/position/i_clamp", axe[dir_i].c_str());
                    control_node.setParam(param, thruster_max_effort);
                    // push to velocity PID
                    sprintf(param, "%s/velocity/i_clamp", axe[dir_i].c_str());
                    control_node.setParam(param, thruster_max_effort);
                }
            }
        }
        else
        {
            // thruster-controlled? assume all axes can be controlled
            for(unsigned int i=0;i<6;++i)
                controlled_axes.push_back(axe[i]);
        }
        // push controlled axes
        control_node.setParam("config/body/axes", controlled_axes);
    }

    // *** END BODY CONTROL

    // *** JOINT CONTROL
    joints_.clear();

    if(control_joints_ && model_->GetJointCount() != 0)
    {
        std::string joint_command_topic, joint_state_topic;
        control_node.param("config/joints/command", joint_command_topic, std::string("joint_command"));
        control_node.param("config/joints/state", joint_state_topic, std::string("joint_state"));

        // initialize subscriber to joint commands
        ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::JointState>(
                    joint_command_topic, 1,
                    boost::bind(&FreeFloatingControlPlugin::JointCommandCallBack, this, _1),
                    ros::VoidPtr(), &callback_queue_);
        joint_command_subscriber_ = rosnode_.subscribe(ops);
        joint_command_received_ = false;

        // push joint limits and setup joint states
        std::vector<std::string> joint_names;
        std::vector<double> joint_min, joint_max, vel_max;
        std::string name;
        physics::JointPtr joint;
        bool cascaded_position = true;
        if(control_node.hasParam("config/joints/cascaded_position"))
            control_node.getParam("config/joints/cascaded_position", cascaded_position);

        for(i=0;i<model_->GetJointCount();++i)
        {
            joint = model_->GetJoints()[i];
            name = joint->GetName();

            if(control_node.hasParam(name))
            {
                joints_.push_back(joint);
                // set max velocity or max effort for the position PID
                sprintf(param, "%s/position/i_clamp", name.c_str());
                if(cascaded_position)
                    control_node.setParam(param, joint->GetVelocityLimit(0));
                else
                    control_node.setParam(param, joint->GetEffortLimit(0));

                // set max effort for the velocity PID
                sprintf(param, "%s/velocity/i_clamp", name.c_str());
                control_node.setParam(param, joint->GetEffortLimit(0));

                // set antiwindup to true - why would anyone set it to false?
                sprintf(param, "%s/position/antiwindup", name.c_str());
                control_node.setParam(param, true);
                sprintf(param, "%s/velocity/antiwindup", name.c_str());
                control_node.setParam(param, true);

                // save name and joint limits
                joint_names.push_back(name);
                joint_min.push_back(joint->GetLowerLimit(0).Radian());
                joint_max.push_back(joint->GetUpperLimit(0).Radian());
                vel_max.push_back(joint->GetVelocityLimit(0));
            }
        }

        // push setpoint topic, name, lower and bound
        control_node.setParam("config/joints/name", joint_names);
        control_node.setParam("config/joints/lower", joint_min);
        control_node.setParam("config/joints/upper", joint_max);
        control_node.setParam("config/joints/velocity", vel_max);

        // setup joint_states publisher
        joint_state_publisher_ = rosnode_.advertise<sensor_msgs::JointState>(joint_state_topic, 1);
        joint_states_.name = joint_names;
        joint_states_.position.resize(joints_.size());
        joint_states_.velocity.resize(joints_.size());
    }
    // *** END JOINT CONTROL


    // set up switch service between position and velocity control
    //switch_service_ = rosnode_.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>("switch", &FreeFloatingControlPlugin::SwitchService, this);

    // store update rate
    if(_sdf->HasElement("updateRate"))
        update_T_ = 1./_sdf->Get<double>("updateRate");
    else
        update_T_ = 0;

    // Register plugin update
    update_event_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&FreeFloatingControlPlugin::Update, this));

    ros::spinOnce();
    ROS_INFO("Started FreeFloating Control Plugin for %s.", _model->GetName().c_str());
}

void FreeFloatingControlPlugin::Update()
{
    // activate callbacks
    callback_queue_.callAvailable();

    if(controller_is_running_)
    {
        // deal with joint control
        if(control_joints_ && joint_command_received_)
        {
            physics::JointPtr joint;
            unsigned int idx;
            for(unsigned int i=0;i<joint_command_.name.size();++i)
            {
                // find corresponding model joint
                idx = std::distance(joint_states_.name.begin(), std::find(joint_states_.name.begin(), joint_states_.name.end(), joint_command_.name[i]));
                joint = joints_[idx];
                joint->SetForce(0,joint_command_.effort[i]);
            }
        }

        // deal with body control if underwater
        if(control_body_ && body_command_received_ && (body_->GetWorldCoGPose().pos.z < z_surface_))
        {
            // saturate thruster use
            double norm_ratio = 1;
            unsigned int i;
            for(i=0;i<thruster_fixed_idx_.size();++i)
                norm_ratio = std::max(norm_ratio, std::abs(thruster_command_(i)) / thruster_max_command_[i]);
            thruster_command_ *= 1./norm_ratio;

            // build and apply wrench for fixed thrusters
            if(thruster_fixed_idx_.size())
            {
                Eigen::VectorXd fixed(thruster_fixed_idx_.size());
                for(unsigned int i=0;i<thruster_fixed_idx_.size();++i)
                    fixed(i) = thruster_command_(thruster_fixed_idx_[i]);
                // to wrench
                fixed = thruster_map_ * fixed;
                // apply this wrench to body
                body_->AddForceAtWorldPosition(body_->GetWorldPose().rot.RotateVector(math::Vector3(fixed(0), fixed(1), fixed(2))), body_->GetWorldCoGPose().pos);
                body_->AddRelativeTorque(math::Vector3(fixed(3), fixed(4), fixed(5)));
            }

            // apply command for steering thrusters
            if(thruster_steer_idx_.size())
            {
                for(unsigned int i=0;i<thruster_steer_idx_.size();++i)
                    thruster_links_[i]->AddRelativeForce(math::Vector3(0,0,-thruster_command_(thruster_steer_idx_[i])));
            }

            // compute and publish thruster use in %
            for(i=0;i<thruster_command_.size();++i)
                thruster_use_.position[i] = 100*std::abs(thruster_command_(i) / thruster_max_command_[i]);
            thruster_use_publisher_.publish(thruster_use_);
        }
    }

    // publish joint states anyway
    double t = ros::Time::now().toSec();
    if((t-t_prev_) > update_T_ && joints_.size() != 0)
    {
        t_prev_ = t;
        joint_states_.header.stamp = ros::Time::now();

        for(unsigned int i=0;i<joints_.size();++i)
        {
            joint_states_.position[i] = joints_[i]->GetAngle(0).Radian();
            joint_states_.velocity[i] = joints_[i]->GetVelocity(0);
        }
        joint_state_publisher_.publish(joint_states_);
    }
}


void FreeFloatingControlPlugin::ComputePseudoInverse(const Eigen::MatrixXd &_M, Eigen::MatrixXd &_pinv_M)
{
    _pinv_M.resize(_M.cols(), _M.rows());
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_M(_M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd dummy_in;
    Eigen::VectorXd dummy_out(_M.cols());
    unsigned int i,j;
    for(i=0;i<_M.rows();++i)
    {
        dummy_in = Eigen::VectorXd::Zero(_M.rows());
        dummy_in(i) = 1;
        dummy_out = svd_M.solve(dummy_in);
        for(j = 0; j<_M.cols();++j)
            _pinv_M(j,i) = dummy_out(j);
    }
}

void FreeFloatingControlPlugin::BodyCommandCallBack(const geometry_msgs::WrenchConstPtr &_msg)
{
    if(!control_body_)
        return;
    body_command_received_ = true;

    // compute corresponding thruster command
    Eigen::VectorXd body_command(6);
    body_command << _msg->force.x, _msg->force.y, _msg->force.z, _msg->torque.x, _msg->torque.y, _msg->torque.z;

    thruster_command_ = thruster_inverse_map_ * body_command;
}


void FreeFloatingControlPlugin::ThrusterCommandCallBack(const sensor_msgs::JointStateConstPtr &_msg)
{
    if(!control_body_)
        return;

    const bool read_effort = _msg->effort.size();

    if(read_effort && (_msg->name.size() != _msg->effort.size()))
    {
        ROS_WARN("Received inconsistent thruster command, name and effort dimension do not match");
        return;
    }
    else
        if(!read_effort && (_msg->name.size() != _msg->position.size()))
        {
            ROS_WARN("Received inconsistent thruster command, name and position dimension do not match");
            return;
        }

    body_command_received_ = true;
    // store thruster command
    for(unsigned int i=0;i<thruster_names_.size();++i)
    {
        for(unsigned int j=0;j<_msg->name.size();++j)
        {
            if(thruster_names_[i] == _msg->name[j])
                thruster_command_(i) = read_effort?_msg->effort[j]:_msg->position[j];
        }
    }
}




}   // namespace gazebo
