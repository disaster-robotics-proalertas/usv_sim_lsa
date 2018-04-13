#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <gazebo/gazebo.hh>

#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <tinyxml.h>
#include <urdf_parser/urdf_parser.h>
#include <gazebo/math/Pose.hh>

#include <freefloating_gazebo/freefloating_gazebo_fluid.h>

using std::cout;
using std::endl;
using std::string;

namespace gazebo
{

	void
	FreeFloatingFluidPlugin::ReadVector3 (const std::string &_string, math::Vector3 &_vector)
	{
		std::stringstream ss (_string);
		double xyz[3];
		for (unsigned int i = 0; i < 3; ++i)
			ss >> xyz[i];
		_vector.Set (xyz[0], xyz[1], xyz[2]);
	}

	void
	FreeFloatingFluidPlugin::Load (physics::WorldPtr _world, sdf::ElementPtr _sdf)
	{
		std::cerr << "\n ===== FreeFloatingFluidPlugin loading";
		ROS_INFO("############### Loading freefloating_fluid plugin");
		this->world_ = _world;

		// register ROS node
		rosnode_ = new ros::NodeHandle ("gazebo");

		// parse plugin options
		description_ = "robot_description";
		has_surface_ = false;
		surface_plane_.Set (0, 0, 1, 0); // default ocean surface plane is Z=0
		std::string fluid_topic = "current";

		if (_sdf->HasElement ("descriptionParam"))
			description_ = _sdf->Get < std::string > ("descriptionParam");
		if (_sdf->HasElement ("surface"))
		{
			has_surface_ = true;
			// get one surface point
			math::Vector3 surface_point;
			ReadVector3 (_sdf->Get < std::string > ("surface"), surface_point);
			// get gravity
			const math::Vector3 WORLD_GRAVITY = world_->GetPhysicsEngine ()->GetGravity ().Normalize ();
			// water surface is orthogonal to gravity
			surface_plane_.Set (WORLD_GRAVITY.x, WORLD_GRAVITY.y, WORLD_GRAVITY.z, WORLD_GRAVITY.Dot (surface_point));
			// push on parameter server
			rosnode_->setParam ("surface", surface_point.z);
		}

		if (_sdf->HasElement ("fluidTopic"))
			fluid_topic = _sdf->Get < std::string > ("fluidTopic");


		// initialize subscriber to water current
		ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Vector3> (
		    fluid_topic, 1, boost::bind (&FreeFloatingFluidPlugin::FluidVelocityCallBack, this, _1), ros::VoidPtr (),
		    &callback_queue_);
		fluid_velocity_.Set (0, 0, 0);
		fluid_velocity_subscriber_ = rosnode_->subscribe (ops);

		// Register plugin update
		update_event_ = event::Events::ConnectWorldUpdateBegin (boost::bind (&FreeFloatingFluidPlugin::Update, this));

		// Clear existing links
		buoyant_links_.clear ();
		parsed_models_.clear ();

		ROS_INFO("############### Loaded freefloating_fluid plugin.");
	}

	void
	FreeFloatingFluidPlugin::Update ()
	{
		//std::cerr<<"\n FreeFloatingFluidPlugin::Update ()";
		// activate callbacks
		callback_queue_.callAvailable ();

		// look for new world models
		unsigned int i;
		std::vector<model_st*>::iterator model_it;
		bool found;
		for (i = 0; i < world_->GetModelCount (); ++i)
		{
			found = false;
			for (model_it = parsed_models_.begin (); model_it != parsed_models_.end (); ++model_it)
			{
				if (world_->GetModel (i)->GetName () == (*model_it)->name)
					found = true;
			}
			if (!found && !(world_->GetModel (i)->IsStatic ())) // model not in listand not static, parse it for potential buoyancy flags
				ParseNewModel (world_->GetModel (i));
		}

		/*
		 for(unsigned int model_index = 0; model_index < parsed_models_.size(); model_index++)
		 {
		 physics::ModelPtr _model = parsed_models_[model_index]->model_ptr;
		 for(unsigned int joint_index = 0; joint_index < _model->GetJoints().size(); ++joint_index)
		 {
		 physics::LinkPtr parentLink = _model->GetJoints()[joint_index]->GetParent();
		 physics::LinkPtr childLink;
		 unsigned int childCount = _model->GetJoints()[joint_index]->GetChildCount();
		 childLink = _model->GetJoints()[joint_index]->GetChild();

		 math::Pose parentPose = parentLink->GetWorldPose();
		 math::Pose linkPose = childLink->GetWorldPose();
		 tf::Vector3 trans(linkPose.pos.x-parentPose.pos.x, linkPose.pos.y-parentPose.pos.y, linkPose.pos.z-parentPose.pos.z);
		 //math::Quaternion quat = linkPose.rot+parentPose.rot;
		 math::Quaternion quat = linkPose.rot;
		 quat.Normalize();
		 //ROS_WARN(" %s -> %s : quat %f %f %f %f", parentLink->GetName().c_str(), childLink->GetName().c_str(), quat.x, quat.y, quat.z, quat.w);

		 broadcaster.sendTransform(
		 tf::StampedTransform(
		 tf::Transform(tf::Quaternion(quat.x, quat.y, quat.z, quat.w), trans),
		 //tf::Transform(tf::Quaternion(0, 0, 0, 1), trans),
		 ros::Time::now(), parentLink->GetName(), _model->GetName()+"/"+childLink->GetName()));

		 }
		 }
		 */

		// look for deleted world models
		model_it = parsed_models_.begin ();
		while (model_it != parsed_models_.end ())
		{
			found = false;
			for (i = 0; i < world_->GetModelCount (); ++i)
			{
				if (world_->GetModel (i)->GetName () == (*model_it)->name)
					found = true;
			}
			if (!found) // model name not in world anymore, remove the corresponding links
				RemoveDeletedModel (model_it);
			else
				++model_it;
		}

		// here buoy_links is up-to-date with the links that are subject to buoyancy, let's apply it
		math::Vector3 actual_force, cob_position, velocity_difference, torque;
		double signed_distance_to_surface;

		const math::Vector3 WORLD_GRAVITY = world_->GetPhysicsEngine ()->GetGravity ().Normalize ();
		for (std::vector<link_st*>::iterator link_it = buoyant_links_.begin (); link_it != buoyant_links_.end (); ++link_it)
		{
			// get world position of the center of buoyancy
			cob_position = (*link_it)->link->GetWorldPose ().pos
			    + (*link_it)->link->GetWorldPose ().rot.RotateVector ((*link_it)->buoyancy_center);
			// start from the theoretical buoyancy force
			actual_force = (*link_it)->buoyant_force;

			if (has_surface_)
			{
				//surface_plane_.Set(0, 0, 1, (*link_it)->waterSurface.z);
				//surface_plane_.Set(0, 0, 1, 0);

				// water surface is orthogonal to gravity
				surface_plane_.Set (WORLD_GRAVITY.x, WORLD_GRAVITY.y, WORLD_GRAVITY.z,
				                    WORLD_GRAVITY.Dot ((*link_it)->waterSurface));

				// adjust force depending on distance to surface (very simple model)
				signed_distance_to_surface = surface_plane_.w - surface_plane_.x * cob_position.x
				    - surface_plane_.y * cob_position.y - surface_plane_.z * cob_position.z;
				//std::cerr<<"\n signed_distance_to_surface: "<<signed_distance_to_surface<<" z: "<<(*link_it)->waterSurface.z;
				//if ((*link_it)->model_name.compare("barcoDiferencial")==0)
				//std::cerr<<"\n "<<(*link_it)->model_name<<"  : "<<signed_distance_to_surface<<" b.z: "<<cob_position.z<<" w.z: "<<surface_plane_.w;
				//std::cerr<<"\n "<<(*link_it)->model_name<<" estah com z: "<<(*link_it)->waterSurface.z;
				if (signed_distance_to_surface > -(*link_it)->limit)
				{
					if (signed_distance_to_surface > (*link_it)->limit)
					{
						actual_force *= 0;
					}
					else
					{
						actual_force *= cos (
						M_PI / 4. * (signed_distance_to_surface / (*link_it)->limit + 1));
					}
				}
			}

			// calc wind forces and moments (See Chapter 8 of Handbook of Marine Craft Hydrodynamics and Motion Control - Thor I. Fossen)
			// Vw is the wind Speed; Bw is the angle of wind
			// Yrw angle of attack

			if ((*link_it)->usingNoneWindVelocity != true)
			{
				double mult = (cob_position.z + (*link_it)->limit - surface_plane_.w)/(2.0 * (*link_it)->limit);
				if (mult > 1.0)
					mult = 1.0;
				if (mult < 0.0)
					mult = 0.0;

				math::Vector3 Vw  = (*link_it)->wind;
				double Afw = (*link_it)->frontal_area*mult; // frontal area
				double Alw = (*link_it)->lateral_area*mult; // lateral area
				double Hlw = signed_distance_to_surface; // height of centroid lateral area
				double Hfw = signed_distance_to_surface; // height of centroid frontal area
				double Loa = (*link_it)->lateral_length; // length overall
				//double urw = u - Vw * cos(Bw - W);
				double urw = (*link_it)->link->GetWorldLinearVel ().x - Vw.x;
				//double vrw = v - Vw * sin(Bw - W);
				double vrw = (*link_it)->link->GetWorldLinearVel ().y - Vw.y;
				double Pa = 1.184;
				double Sl = 0.1;

				// speed boat values
				double CDt = 0.90;
				double CDlaf0 = 0.55;  // coefficient to front wind
				double CDlafPI = 0.60; // coefficient to tail wind
				double tetha = 0.60;
				double k = 1.1;
				// fishing vessel values
				/*double CDt = 0.95;
				double CDlaf0 = 0.70;
				double CDlafPI = 0.70;
				double tetha = 0.40;
				double k = 1.1;*/



				double Yrw = - std::atan2(vrw, urw);
				double Vrw = std::sqrt(urw*urw + vrw *vrw);
				double sin2 = std::sin(2*Yrw);


				double CDlaf = CDlaf0;
				if (Yrw >= 1.57)
					CDlaf = CDlafPI;
				double CDl = CDlaf * Afw / Alw;
				if (Alw < 0.01)
					CDl = 0.0;
				double denominator = 1-0.5*tetha*(1-CDl/CDt)*sin2;
				double Cx = - CDl * (Alw/Afw) * std::cos(Yrw)/ denominator;
				if (Afw < 0.01)
					Cx = 0.0;
				double Cy = CDt * std::sin(Yrw) / denominator;
				//double Cz = 0; // as Fossen said: "only Cx, Cy and Cn are needed"
				double Ck = k*Cy;
				//double Cm;
				double Cn = (Sl/Loa - 0.18*(Yrw-3.1415/2.0))*Cy;
				double Xw = 0.5 * Pa * Vrw * Vrw * Cx * Afw;
				double Yw = 0.5 * Pa * Vrw * Vrw * Cy * Alw;
				//double Zw = 0.5 * Pa * Vrw * Vrw * Cz * Afw;
				double Zw = 0.0;
				double Kw = 0.5 * Pa * Vrw * Vrw * Ck * Alw * Hlw;
				//double Kw = 0.0;
				//double Mw = 0.5 * Pa * Vrw * Vrw * Cm * Afw * Hfw;
				double Mw = 0.0;
				double Nw = 0.5 * Pa * Vrw * Vrw * Cn * Alw * Loa;


				//std::cerr<<"\n "<<(*link_it)->model_name<<" mult: "<<mult<<" signed_distance_to_surface: "<<signed_distance_to_surface<< " w: "<< surface_plane_.z<<" z: "<< cob_position.z;
				//std::cerr<<"\n "<<(*link_it)->model_name<<" "<<Xw<<", "<<Yw<<", "<<Zw<<") Vw ("<<Vw.x<<", "<<Vw.y<<", "<<Vw.z<<") M ("<<Kw<<", "<<Mw<<", "<<Nw<<") CN: "<<Cn<<" CY: "<<Cy<<" Yrw: "<<Yrw<<" speed ("<<urw<<", "<<vrw<<") linSpeed: "<<(*link_it)->link->GetWorldLinearVel ().x<<", "<<(*link_it)->link->GetWorldLinearVel ().y<<")";
				(*link_it)->link->AddRelativeTorque (math::Vector3(Kw, Mw, Nw) );
				(*link_it)->link->AddForceAtWorldPosition (math::Vector3(Xw, Yw, Zw), cob_position);
			}

			// get velocity damping
			// linear velocity difference in the link frame
			if ((*link_it)->usingLocalFluidVelocity)
			{
				velocity_difference = (*link_it)->link->GetWorldPose ().rot.RotateVectorReverse (
				    (*link_it)->link->GetWorldLinearVel () - (*link_it)->fluid_velocity_);
			}
			else if ((*link_it)->usingNoneFluidVelocity)
			{
				velocity_difference = (*link_it)->link->GetWorldPose ().rot.RotateVectorReverse (
								    (*link_it)->link->GetWorldLinearVel () );
			}
			else
				velocity_difference = (*link_it)->link->GetWorldPose ().rot.RotateVectorReverse (
				    (*link_it)->link->GetWorldLinearVel () - fluid_velocity_);

			// to square
			velocity_difference.x *= fabs (velocity_difference.x);
			velocity_difference.y *= fabs (velocity_difference.y);
			velocity_difference.z *= fabs (velocity_difference.z);
			// apply damping coefficients
			actual_force -= (*link_it)->link->GetWorldPose ().rot.RotateVector (
			    (*link_it)->linear_damping * velocity_difference);

			//link_it->link->AddForceAtRelativePosition(link_it->link->GetWorldPose().rot.RotateVectorReverse(link_it->buoyant_force),
			//                                          link_it->buoyancy_center);
			(*link_it)->link->AddForceAtWorldPosition (actual_force, cob_position);

			// same for angular damping
			velocity_difference = (*link_it)->link->GetRelativeAngularVel ();
			velocity_difference.x *= fabs (velocity_difference.x);
			velocity_difference.y *= fabs (velocity_difference.y);
			velocity_difference.z *= fabs (velocity_difference.z);
			(*link_it)->link->AddRelativeTorque (-(*link_it)->angular_damping * velocity_difference);

			// publish states as odometry message
			nav_msgs::Odometry state;
			state.header.frame_id = "world";
			state.header.stamp = ros::Time::now ();
			math::Vector3 vec;
			math::Pose pose;
			for (model_it = parsed_models_.begin (); model_it != parsed_models_.end (); ++model_it)
			{
				// which link
				state.child_frame_id = "base_link";
				// write absolute pose
				pose = (*model_it)->model_ptr->GetWorldPose ();
				state.pose.pose.position.x = pose.pos.x;
				state.pose.pose.position.y = pose.pos.y;
				state.pose.pose.position.z = pose.pos.z;
				state.pose.pose.orientation.x = pose.rot.x;
				state.pose.pose.orientation.y = pose.rot.y;
				state.pose.pose.orientation.z = pose.rot.z;
				state.pose.pose.orientation.w = pose.rot.w;

				// write relative linear velocity
				vec = (*model_it)->model_ptr->GetRelativeLinearVel ();
				state.twist.twist.linear.x = vec.x;
				state.twist.twist.linear.y = vec.y;
				state.twist.twist.linear.z = vec.z;
				// write relative angular velocity
				vec = (*model_it)->model_ptr->GetRelativeAngularVel ();
				state.twist.twist.angular.x = vec.x;
				state.twist.twist.angular.y = vec.y;
				state.twist.twist.angular.z = vec.z;

				// publish
				(*model_it)->state_publisher.publish (state);
			}

			//  ROS_INFO("Link %s: Applying buoyancy force (%.01f, %.01f, %.01f)", link.name.c_str(), link.buoyant_force.x, link.buoyant_force.y, link.buoyant_force.z);
		}
	}

	void
	FreeFloatingFluidPlugin::ParseNewModel (const physics::ModelPtr &_model)
	{

		std::cerr << "\n ############################################### START FreeFloatingFluidPlugin::ParseNewModel";
		std::cerr << "\n ############################################### START FreeFloatingFluidPlugin::ParseNewModel";
		std::cerr << "\n ############################################### START FreeFloatingFluidPlugin::ParseNewModel";
		std::cerr << "\n ############################################### START FreeFloatingFluidPlugin::ParseNewModel";
		// define new model structure: name / pointer / publisher to odometry
		model_st* new_model = new model_st ();
		new_model->name = _model->GetName ();
		new_model->model_ptr = _model;
		new_model->state_publisher = rosnode_->advertise<nav_msgs::Odometry> ("/" + _model->GetName () + "/state", 1);
		//std::string topic = "/" + _model->GetName() + "/Surface";

		/*ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Vector3>(
		 topic, 1,
		 boost::bind(&model_st::processSurfaceData, new_model, _1),
		 ros::VoidPtr(), &callback_queue_);*/
		//new_model->createSubscriber(rosnode_, topic);
		// tells this model has been parsed
		parsed_models_.push_back (new_model);

		// get robot description from model name
		// we cannot do anything without the robot_description, as a custom parsing is required to get buoyancy tags
		if (!rosnode_->hasParam ("/" + _model->GetName () + "/" + description_))
			return;

		const unsigned int previous_link_number = buoyant_links_.size ();
		std::string urdf_content;
		rosnode_->getParam ("/" + _model->GetName () + "/" + description_, urdf_content);
		// parse actual URDF as XML (that's ugly) to get custom buoyancy tags

		// links from urdf
		TiXmlDocument urdf_doc;
		urdf_doc.Parse (urdf_content.c_str (), 0);

		const math::Vector3 WORLD_GRAVITY = world_->GetPhysicsEngine ()->GetGravity ();

		TiXmlElement* urdf_root = urdf_doc.FirstChildElement ();
		TiXmlNode* urdf_node, *link_node, *buoy_node;
		double compensation;
		unsigned int link_index, joint_index;
		physics::LinkPtr sdf_link;
		bool found;

		for (urdf_node = urdf_root->FirstChild (); urdf_node != 0; urdf_node = urdf_node->NextSibling ())
		{
			if (urdf_node->ValueStr () == "link")
			{
				//ROS_INFO("++++++++++++++++++++++++++++++++++++NODE : %s", urdf_node->ToElement()->Attribute("name"));

				// find corresponding sdf model link if any
				found = false;
				for (link_index = 0; link_index < _model->GetLinks ().size (); ++link_index)
				{
					//ROS_INFO("match %s", _model->GetLinks()[link_index]->GetName().c_str());
					ROS_WARN(" LINK: %s", _model->GetLinks ()[link_index]->GetName ().c_str ());
					if (urdf_node->ToElement ()->Attribute ("name") == _model->GetLinks ()[link_index]->GetName ())
					{
						found = true;
						sdf_link = _model->GetLinks ()[link_index];
						break;
					}
				}

				if (found)
				{
					for (link_node = urdf_node->FirstChild (); link_node != 0; link_node = link_node->NextSibling ())
					{
						if (link_node->ValueStr () == "buoyancy")
						{
							// this link is subject to buoyancy, create an instance
							link_st* new_buoy_link = new link_st ();
							new_buoy_link->model_name = _model->GetName (); // in case this model is deleted
							new_buoy_link->link = sdf_link;    // to apply forces
							new_buoy_link->limit = .1;
							std::string topic = "/" + _model->GetName () + "/Surface/" + urdf_node->ToElement ()->Attribute ("name");
							new_buoy_link->frontal_area = 0.04;
							new_buoy_link->lateral_area = 0.06;
							new_buoy_link->lateral_length = 1.2;

							new_buoy_link->createSubscriberWaterSurface (rosnode_, topic);
							// get data from urdf
							// default values
							new_buoy_link->buoyancy_center = sdf_link->GetInertial ()->GetCoG ();
							new_buoy_link->linear_damping = new_buoy_link->angular_damping = 5 * math::Vector3::One
							    * sdf_link->GetInertial ()->GetMass ();

							compensation = 0;
							for (buoy_node = link_node->FirstChild (); buoy_node != 0; buoy_node = buoy_node->NextSibling ())
							{
								if (buoy_node->ValueStr () == "origin")
									ReadVector3 ((buoy_node->ToElement ()->Attribute ("xyz")), new_buoy_link->buoyancy_center);
								else if (buoy_node->ValueStr () == "compensation")
									compensation = atof (buoy_node->ToElement ()->GetText ());
								else if (buoy_node->ValueStr () == "limit")
								{
									std::stringstream ss (buoy_node->ToElement ()->Attribute ("radius"));
									ss >> new_buoy_link->limit;
								}
								else if (buoy_node->ValueStr () == "frontal_area")
								{
									std::stringstream ss (buoy_node->ToElement ()->GetText ());
									ss >> new_buoy_link->frontal_area;
								}
								else if (buoy_node->ValueStr () == "lateral_area")
								{
									std::stringstream ss (buoy_node->ToElement ()->GetText ());
									ss >> new_buoy_link->lateral_area;
								}
								else if (buoy_node->ValueStr () == "lateral_length")
								{
									std::stringstream ss (buoy_node->ToElement ()->GetText ());
									ss >> new_buoy_link->lateral_length;
								}
								else if (buoy_node->ValueStr () == "fluidVelocity")
								{
									std::string nameLocal = "local";
									std::string nameGlobal = "global";
									std::string nameNone = "none";
									if (nameLocal.compare (buoy_node->ToElement ()->GetText ()) == 0)
									{
										new_buoy_link->usingLocalFluidVelocity = true;
										new_buoy_link->initServiceClient (rosnode_);

										//new_buoy_link->Start ();
									}
									else if (nameNone.compare (buoy_node->ToElement ()->GetText ()) == 0)
									{
										new_buoy_link->usingLocalFluidVelocity = false;
										new_buoy_link->usingNoneFluidVelocity = true;
										fluid_velocity_.Set (0, 0, 0);
									}
									else if (nameGlobal.compare (buoy_node->ToElement ()->GetText ()) == 0)
									{
										new_buoy_link->usingLocalFluidVelocity = false;
										new_buoy_link->usingNoneFluidVelocity = false;
										new_buoy_link->createSubscriberWaterCurrent(rosnode_, "\gazebo\current");
									}
								}
								else if (buoy_node->ValueStr () == "windVelocity")
								{
									std::string nameLocal = "local";
									std::string nameGlobal = "global";
									std::string nameNone = "none";
									if (nameLocal.compare (buoy_node->ToElement ()->GetText ()) == 0)
									{
										new_buoy_link->usingLocalWindVelocity = true;
										new_buoy_link->initWindServiceClient (rosnode_);

										//new_buoy_link->Start ();
									}
									else if (nameNone.compare (buoy_node->ToElement ()->GetText ()) == 0)
									{
										new_buoy_link->usingLocalWindVelocity = false;
										new_buoy_link->usingNoneWindVelocity = true;
										new_buoy_link->wind.Set (0, 0, 0);
									}
									else if (nameGlobal.compare (buoy_node->ToElement ()->GetText ()) == 0)
									{
										new_buoy_link->usingLocalWindVelocity = false;
										new_buoy_link->usingNoneWindVelocity = false;
										double x, y;
										rosnode_->getParam ("/uwsim/wind/x", x);
										rosnode_->getParam ("/uwsim/wind/y", y);
										new_buoy_link->wind.Set (x, y, 0);
									}
								}
								else if (buoy_node->ValueStr () == "damping")
								{
									if (buoy_node->ToElement ()->Attribute ("xyz") != NULL)
									{
										ReadVector3 ((buoy_node->ToElement ()->Attribute ("xyz")), new_buoy_link->linear_damping);
									}
									if (buoy_node->ToElement ()->Attribute ("rpy") != NULL)
									{
										ReadVector3 ((buoy_node->ToElement ()->Attribute ("rpy")), new_buoy_link->angular_damping);
									}
								}
								else
									ROS_WARN("Unknown tag <%s/> in buoyancy node for model %s", buoy_node->ValueStr ().c_str (),
									         _model->GetName ().c_str ());
							}
							if (new_buoy_link->usingLocalFluidVelocity || new_buoy_link->usingLocalWindVelocity)
								new_buoy_link->Start ();
							new_buoy_link->buoyant_force = -compensation * sdf_link->GetInertial ()->GetMass () * WORLD_GRAVITY;

							// store this link
							buoyant_links_.push_back (new_buoy_link);
						}
					}   // out of loop: buoyancy-related nodes
				}       // out of condition: in sdf
			}           // out of loop: links
		}               // out of loop: all urdf nodes
		if (previous_link_number == buoyant_links_.size ())
			ROS_INFO_NAMED("Buoyancy plugin", "No links subject to buoyancy inside %s", _model->GetName ().c_str ());
		else
			ROS_INFO_NAMED("Buoyancy plugin", "Added %i buoy links from %s",
			               (int ) buoyant_links_.size () - previous_link_number, _model->GetName ().c_str ());
		std::cerr << "\n ### FINISHED FreeFloatingFluidPlugin::ParseNewModel";
	}

	void
	FreeFloatingFluidPlugin::RemoveDeletedModel (std::vector<model_st*>::iterator &_model_it)
	{
		ROS_INFO("Removing deleted model: %s", (*_model_it)->name.c_str ());

		// remove model stored links
		std::vector<link_st*>::iterator link_it = buoyant_links_.begin ();
		while (link_it != buoyant_links_.end ())
		{
			if ((*link_it)->model_name == (*_model_it)->name)
			{
				delete *link_it;
				link_it = buoyant_links_.erase (link_it);
			}
			else
				++link_it;
		}
		delete *_model_it;
		// remove it from the list
		_model_it = parsed_models_.erase (_model_it);
	}

	void
	FreeFloatingFluidPlugin::FluidVelocityCallBack (const geometry_msgs::Vector3ConstPtr &_msg)
	{
		// store fluid velocity
		fluid_velocity_.Set (_msg->x, _msg->y, _msg->z);
	}

}

