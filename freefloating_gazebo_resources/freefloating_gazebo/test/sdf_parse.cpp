#include <ros/ros.h>
#include <freefloating_gazebo/thruster_mapper.h>

using namespace std;

int main(int argc, char ** argv)
{
ros::init(argc, argv, "sdf_parse");
ros::NodeHandle nh("pirov");

ffg::ThrusterMapper mapper;
mapper.parse(nh);

std::cout << mapper.map << std::endl;
for(auto v: mapper.max_vel)
    std::cout << v << " ";


}
