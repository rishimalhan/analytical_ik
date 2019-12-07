#ifndef urdf_to_collada
#define urdf_to_collada

#include "collada_urdf/collada_urdf.h"

namespace urdf_to_collada
{
	void convert_urdf(std::string urdf_name, std::string robot_name)
	{
		// std::cout<< "URDF File name:  " << urdf_name << std::endl;
		// std::cout<< "Collada File name:  " << robot_name+".dae" << std::endl;
		
		urdf::Model robot_model;
		robot_model.initFile(urdf_name);
		collada_urdf::WriteUrdfModelToColladaFile(robot_model, robot_name+".dae");
	};
}


#endif