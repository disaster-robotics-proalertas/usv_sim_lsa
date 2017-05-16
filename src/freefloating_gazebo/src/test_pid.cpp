
#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <tinyxml.h>
#include <time.h>

using std::cout;
using std::endl;
using std::string;

void ParseParams(TiXmlNode* _param_node, std::vector<std::string>&_pids)
{

    for(; _param_node!=0; _param_node = _param_node->NextSibling())
    {

        if(_param_node->ValueStr() == "i_clamp")
        {
            std::string param = "";
            TiXmlNode* parent = _param_node->Parent()->Parent()->Parent();
            while(parent)
            {
                std::string name = parent->ValueStr();
                if(name == "member")
                    param = string(parent->FirstChildElement()->GetText()) + "/" + param;
                parent = parent->Parent();
            }
            _pids.push_back(param);
            return;
        }
        ParseParams(_param_node->FirstChild(),  _pids);
    }
}

int main(int argc, char ** argv)
{
    // init ROS node
    ros::init(argc, argv, "dymamic_pid");
    ros::NodeHandle node;

    XmlRpc::XmlRpcValue param_st;
    if(node.getParam("controllers", param_st))
        cout << "Controllers found" << endl;

    // find PID controllers
    TiXmlDocument param_xml;
    param_xml.Parse(param_st.toXml().c_str(), 0);

    cout << param_st.toXml() << endl;
    TiXmlNode* param_root = param_xml.FirstChild();

    // explore xml recursively
    std::vector<std::string> pid_params;
    pid_params.clear();
    ParseParams(param_root, pid_params);

    // create PID's
    std::vector<control_toolbox::Pid> pids(pid_params.size());
    for(unsigned int i=0;i<pid_params.size();++i)
        pids[i].init(ros::NodeHandle(node, "controllers/"+pid_params[i]));

    unsigned int iter = 0;
    while(ros::ok())
    {
        ros::spinOnce();
        sleep(1);
    }




















}
