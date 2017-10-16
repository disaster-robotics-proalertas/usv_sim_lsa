
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/GetMap.h>
#include <iostream>
#include <fstream>

#include <SDL/SDL_image.h>
#include <yaml-cpp/yaml.h>

#include "usv_water_current/GetSpeed.h"

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))


class WaterCurrentServer
{
public:
	WaterCurrentServer(const std::string& filename)
	{


		std::ifstream fileInput(filename.c_str());

		if (fileInput.fail())
		{
          	ROS_ERROR("ERROR: water_current could not open %s.", filename.c_str());
        	exit(-1);
		}

		
		YAML::Node document = YAML::LoadFile(filename.c_str());


		try 
		{
			resolution = document["resolution"].as<double>();
		} 
	    	catch (YAML::InvalidScalar) 
	    	{
		    ROS_ERROR("ERROR: The %s does not contain a valid resolution value.", filename.c_str());
		    exit(-1);
		}
		try 
		{
			origin[0] = document["origin"][0].as<double>();
			origin[1] = document["origin"][1].as<double>();
			origin[2] = document["origin"][2].as<double>();
		} 
	    	catch (YAML::InvalidScalar) 
	    	{
		    ROS_ERROR("ERROR: The %s does not contain a valid origin value.", filename.c_str());
		    exit(-1);
		}
		try 
		{
			imageFilename = document["image"].as<std::string>();
		} 
	    	catch (YAML::InvalidScalar) 
	    	{
		    ROS_ERROR("ERROR: The %s does not contain a valid image filename.", filename.c_str());
		    exit(-1);
		}
		// opens imageFilename
		LoadImage();
		//publish services for ROSViz 
		serviceFull = nh.advertiseService("static_map", &WaterCurrentServer::fullMapCallback, this);
		serviceValue = nh.advertiseService("value_map", &WaterCurrentServer::valueMapCallback, this);
		/*
		map_resp_.map.info.map_load_time = ros::Time::now();
		map_resp_.map.header.frame_id = frame_id;
		map_resp_.map.header.stamp = ros::Time::now();
		 meta_data_message_ = map_resp_.map.info;
		// Latched publisher for metadata
		metadata_pub= n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
		metadata_pub.publish( meta_data_message_ );

		// Latched publisher for data
		map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
		map_pub.publish( map_resp_.map );
		*/

		// Latched publisher for data
		map_pub = nh.advertise<nav_msgs::OccupancyGrid>("flowmap", 1, true);
		map_pub.publish( map_resp_.map );
	};

	void LoadImage()
	{
		ROS_INFO("Loading image");
		ptrImage = IMG_Load(imageFilename.c_str());
		if(!ptrImage)
  		{
		    ROS_ERROR("ERROR: water_current couldn't open image file: %s", imageFilename.c_str());
    		exit(-1);
		}
		else
		{
			ROS_INFO("Image OPENED.... ok");
		}

		int width = ptrImage->w;
		int height = ptrImage->h;
		map_resp_.map.info.resolution = 0.05;
		map_resp_.map.info.width = width;
		map_resp_.map.info.height = height;
		map_resp_.map.info.origin.orientation.x = 0;
		map_resp_.map.info.origin.orientation.y = 0;
		map_resp_.map.info.origin.orientation.z = 0;
		map_resp_.map.info.origin.orientation.w = 1;
		//map_resp_.map.data = ptrImage->data;
		map_resp_.map.data.resize(width * height);
		unsigned int n_channels = ptrImage->format->BytesPerPixel;
		int rowstride = ptrImage->pitch;
		ROS_INFO(" CHANNELS: %d", n_channels);
		ROS_INFO(" rowstride: %d", rowstride);
		unsigned char* p;
		unsigned char* pixels = (unsigned char*)(ptrImage->pixels);
		unsigned char value;
		int color_sum;
		unsigned int i;
		unsigned int j;
		int color_avg;
		for(j = 0; j < height; j++)
		{
		    	for (i = 0; i < width; i++)
			{

				p = pixels + j*rowstride + i*n_channels;
				//ROS_INFO(" p: %d", p);
 			        color_sum = 0;
				for(int k=0;k<n_channels;k++)
				{
					color_sum += *(p + (k));
					//ROS_INFO("K: %d color: %d sum: %d",k, *(p + (k)), color_sum);
				}
				//ROS_INFO(" sum: %d", color_sum);
				value = color_sum / (double)n_channels;
				map_resp_.map.data[MAP_IDX(width,i,height - j - 1)] = value;
			}
		}
		ROS_INFO("IMAGE LOADED!");
	};


	bool fullMapCallback(nav_msgs::GetMap::Request  &req,
	                     nav_msgs::GetMap::Response &res )
	    {
	      // request is empty; we ignore it

	      // = operator is overloaded to make deep copy (tricky!)
	      res = map_resp_;
	      ROS_INFO("Sending map");

	      return true;
	};

	bool valueMapCallback(usv_water_current::GetSpeed::Request  &req,
			      usv_water_current::GetSpeed::Response &res)
	{
		res.x = map_resp_.map.data[0];
//		res.y = MAP_IDX(map_resp_.map.info.width,req.x,map_resp_.map.info.height - req.y - 1);
		res.y = map_resp_.map.data[MAP_IDX(map_resp_.map.info.width,req.x,map_resp_.map.info.height - req.y - 1)];
		return true;
	}

private:
	double resolution;
	double origin[3];
	std::string imageFilename;

	SDL_Surface* ptrImage;

	nav_msgs::GetMap::Response map_resp_;
	ros::NodeHandle nh;
	ros::ServiceServer serviceFull;
	ros::ServiceServer serviceValue;
        ros::Publisher map_pub;
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "usv_water_current", ros::init_options::AnonymousName);

	if(argc != 2)
	{
		ROS_ERROR("%s", "ERROR: Few arguments!");
		exit(-1);
	}
	std::string filename(argv[1]);

	try
	{
		WaterCurrentServer waterCurrent(filename);
		ros::spin();
	}
	catch(std::runtime_error& e)
	{
		ROS_ERROR("water_current exception: %s", e.what());
		return -1;
	}



	return 0;
}
