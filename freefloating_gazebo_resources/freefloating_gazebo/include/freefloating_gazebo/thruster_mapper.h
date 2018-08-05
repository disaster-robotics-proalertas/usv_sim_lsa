#ifndef MAPPER_H
#define MAPPER_H

#include <Eigen/Core>
#include <Eigen/SVD>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/JointState.h>
#include <tinyxml.h>

namespace ffg
{

class ThrusterMapper
{
public:
    ThrusterMapper()  {}

    bool checkName(TiXmlElement* elem, std::string name)
    {
        for(TiXmlAttribute* att = elem->FirstAttribute(); att != NULL; att = att->Next())
        {
            if(att->NameTStr() == "name" && att->ValueStr() == name)
                return true;
        }
        return false;
    }


    // parse raw param
    // only for fixed thrusters (for now)
    void parse(ros::NodeHandle &nh);

    // parse for Gazebo
    void parse(std::string sdf_str);

    void initWrenchControl(ros::NodeHandle &_control_node,
                           std::string _body_name,
                           std::vector<std::string> &_axe,
                           std::vector<std::string> &_controlled_axes);

    void saturate(Eigen::VectorXd &_command);

    sensor_msgs::JointState wrench2Thrusters(const geometry_msgs::Wrench  & cmd);

    std::vector<unsigned int> steer_idx, fixed_idx;
    std::vector<std::string> names;
    std::vector<double> max_command, max_vel;

    Eigen::MatrixXd map;
    Eigen::MatrixXd inverse_map;
};

}



#endif // MAPPER_H
