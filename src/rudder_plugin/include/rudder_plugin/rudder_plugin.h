

/*

	Developed by Marcelo Paravisi.
	Based in code LiftDragPlu

*/

#include <string>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <ros/ros.h>
#include <gazebo/plugins/LiftDragPlugin.hh>

namespace gazebo
{
  /// \brief A plugin that simulates lift and drag.
  class Rudder_plugin : public LiftDragPlugin
  {


    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();
  };

}
