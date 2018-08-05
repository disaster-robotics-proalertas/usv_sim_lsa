#include <freefloating_gazebo/thruster_mapper.h>

namespace ffg
{

// parse from robot_description param
void ThrusterMapper::parse(ros::NodeHandle &nh)
{
    std::string s;
    nh.getParam("robot_description", s);

    TiXmlDocument doc;
    doc.Parse(s.c_str());
    auto root = doc.RootElement();
    std::string base_link = "base_link";

    // find ffg plugin part
    for(auto elem = root->FirstChildElement("gazebo"); elem != NULL; elem = elem->NextSiblingElement("gazebo"))
    {
        // check for plugin
        TiXmlElement* plugin = elem->FirstChildElement("plugin");
        if(plugin != NULL)
        {
            if(checkName(plugin, "freefloating_gazebo_control"))
            {
                TiXmlPrinter printer;
                plugin->Accept(&printer);
                // parse sdf
                parse(printer.CStr());

                // also get base link name
                auto link = plugin->FirstChildElement("link");
                if(link)
                    base_link = link->GetText();
                break;
            }
        }
    }

    // find max velocity on each axis
    max_vel.resize(6, 0);
    for(auto elem = root->FirstChildElement("link"); elem != NULL; elem = elem->NextSiblingElement("link"))
    {
        if(checkName(elem, base_link))
        {

            auto buoy = elem->FirstChildElement("buoyancy");
            if(buoy)
            {
                auto damp = buoy->FirstChildElement("damping");
                // get damping
                std::vector<double> damping(6, 0);
                for(TiXmlAttribute* att = damp->FirstAttribute(); att != NULL; att = att->Next())
                {
                    if(att->NameTStr() == "xyz")
                    {
                        std::stringstream ss(att->Value());
                        ss >> damping[0];
                        ss >> damping[1];
                        ss >> damping[2];
                    }
                    else if(att->NameTStr() == "rpy")
                    {
                        std::stringstream ss(att->Value());
                        ss >> damping[3];
                        ss >> damping[4];
                        ss >> damping[5];
                    }
                }
                for(int i = 0; i < 6; ++i)
                {
                    if(damping[i])
                    {
                        // get max effort in this direction
                        double max_thrust = 0;
                        for(int j = 0; j < map.cols(); j++)
                            max_thrust += std::abs(map(i, j)) * max_command[j];
                        // to max velocity
                        max_vel[i] = max_thrust / damping[i];
                    }
                }
            }
            break;
        }
    }
}


// parse from Gazebo-passed SDF
void ThrusterMapper::parse(std::string sdf_str)
{
    names.clear();
    fixed_idx.clear();
    steer_idx.clear();
    max_command.clear();

    TiXmlDocument doc;
    doc.Parse(sdf_str.c_str());
    auto sdf = doc.RootElement();

    double map_coef;
    auto sdf_element = sdf->FirstChildElement();
    while(sdf_element)
    {
        if(sdf_element->ValueStr() == "thruster")
        {
            // check if it is a steering or fixed thruster
            auto map_info = sdf_element->FirstChildElement("map");
            if(map_info)  // fixed
            {
                // add this index to fixed thrusters
                fixed_idx.push_back(names.size());

                // register map coefs
                std::stringstream ss(map_info->GetText());
                const unsigned int nb = map.cols();
                map.conservativeResize(6,nb+1);
                for(int i=0;i<6;++i)
                {
                    ss >> map_coef;
                    map(i,nb) = map_coef;
                }

                // check for any name
                auto name_info = sdf_element->FirstChildElement("name");
                if(name_info)
                    names.push_back(std::string(name_info->GetText()));
                else
                {
                    std::ostringstream name;
                    name << "thr" << names.size();
                    names.push_back(name.str());
                }

                ROS_INFO("Adding %s as a fixed thruster", names[names.size()-1].c_str());
            }
            else
            {
                auto name_info = sdf_element->FirstChildElement("name");
                if(name_info)
                {
                    // add this index to steering thrusters
                    steer_idx.push_back(names.size());

                    // add the link name
                    names.push_back(name_info->ValueStr());

                    ROS_INFO("Adding %s as a steering thruster", names[names.size()-1].c_str());
                }
            }

            // register maximum effort
            auto effort_info = sdf_element->FirstChildElement("effort");
            if(effort_info)
            {
                std::stringstream ss(effort_info->GetText());
                double val;
                ss >> val;
                max_command.push_back(val);
            }
            else
                max_command.push_back(100);
        }
        sdf_element = sdf_element->NextSiblingElement();
    }
}

void ThrusterMapper::initWrenchControl(ros::NodeHandle &_control_node,
                                       std::string _body_name,
                                       std::vector<std::string> &_axe,
                                       std::vector<std::string> &_controlled_axes)
{
    // compute map pseudo-inverse
    inverse_map.resize(map.cols(), map.rows());
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_M(map, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd dummy_in;
    Eigen::VectorXd dummy_out(map.cols());
    unsigned int i,j;
    for(i=0;i<map.rows();++i)
    {
        dummy_in = Eigen::VectorXd::Zero(map.rows());
        dummy_in(i) = 1;
        dummy_out = svd_M.solve(dummy_in);
        for(j = 0; j<map.cols();++j)
            inverse_map(j,i) = dummy_out(j);
    }

    // push control data to parameter server
    _control_node.setParam("config/body/link", _body_name);

    unsigned int thr_i, dir_i;

    // compute max force in each direction, writes controlled axes
    double thruster_max_effort;

    for(dir_i=0;dir_i<6;++dir_i)
    {
        thruster_max_effort = 0;
        for(thr_i=0;thr_i<fixed_idx.size();++thr_i)
            thruster_max_effort += max_command[fixed_idx[thr_i]] * std::abs(map(dir_i,thr_i));
        if(thruster_max_effort != 0)
        {
            _controlled_axes.push_back(_axe[dir_i]);
            char param[FILENAME_MAX];
            // push to position PID
            sprintf(param, "%s/position/i_clamp", _axe[dir_i].c_str());
            _control_node.setParam(param, thruster_max_effort);
            // push to velocity PID
            sprintf(param, "%s/velocity/i_clamp", _axe[dir_i].c_str());
            _control_node.setParam(param, thruster_max_effort);
        }
    }
}

void ThrusterMapper::saturate(Eigen::VectorXd &_command)
{
    double norm_ratio = 1;
    unsigned int i;
    for(i=0;i<fixed_idx.size();++i)
        norm_ratio = std::max(norm_ratio, std::abs(_command(i)) / max_command[i]);
    _command *= 1./norm_ratio;
}

sensor_msgs::JointState ThrusterMapper::wrench2Thrusters(const geometry_msgs::Wrench  & cmd)
{
    Eigen::VectorXd wrench(6);
    wrench << cmd.force.x, cmd.force.y, cmd.force.z,
            cmd.torque.x, cmd.torque.y, cmd.torque.z;
    sensor_msgs::JointState msg;
    msg.name = names;
    msg.effort.reserve(names.size());

    Eigen::VectorXd thrust = inverse_map * wrench;
    saturate(thrust);

    for(int i = 0; i < thrust.rows(); i++)
    {
        msg.effort.push_back(thrust[i]);
    }
    return msg;
}

}
