#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Group>
#include <osgGA/TrackballManipulator>
#include <osgGA/GUIEventHandler>

#include <osg_interactive_markers/interactive_marker_display.h>
#include <osg_utils/osg_utils.h>
#include <osg_utils/frame_manager.h>

using namespace osg_utils;
using namespace osg_interactive_markers;

int main(int argc, char *argv[])
{

   osg::Group *root=new osg::Group();

   osgViewer::Viewer viewer;

   viewer.setSceneData( root );
   viewer.setUpViewInWindow (0, 0, 800, 600);

   osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
   tb->setHomePosition( osg::Vec3f(20,20,20), osg::Vec3f(0,-15,0), osg::Vec3f(0,0,1) );
   viewer.setCameraManipulator( tb );

   viewer.addEventHandler( new osgViewer::StatsHandler );
   viewer.addEventHandler(new osgViewer::WindowSizeHandler);

   ros::init(argc, argv, "osg_interactive_markers");

   boost::shared_ptr<FrameManager> frame_manager = FrameManager::instance();
   frame_manager->setFixedFrame("/base_link");
   InteractiveMarkerDisplay marker_cli("osg_im","/basic_controls/update", root, *(frame_manager->getTFClient()));

   viewer.realize();
   viewer.frame();
    
    ros::WallTime last_wall_time = ros::WallTime::now();
    ros::Time last_ros_time = ros::Time::now();
    while( !viewer.done() && ros::ok())
    {
	ros::spinOnce();
        
        viewer.frame();
       
        ros::WallTime current_wall_time=ros::WallTime::now();
        ros::Time current_ros_time=ros::Time::now();
	marker_cli.update((current_wall_time-last_wall_time).toSec(), (current_ros_time-last_ros_time).toSec());
	last_wall_time=current_wall_time;
	last_ros_time=current_ros_time;
    }


    return 0;
}
