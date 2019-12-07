#include <iostream>
#include <urdf_to_collada.hpp>
#include <boost/filesystem.hpp>
// #include <filesystem>

int main(int argc, char** argv)
{
	std::cout<< "########################################################################" << std::endl;
	std::cout<< "Generating analytical IK for the robot" << std::endl;
	std::cout<< "########################################################################" << std::endl;

	std::cout<< "Validating input arguments." << std::endl;
	if (argc!=3)
	{
		std::cout<< "Not enough arguments. Check if the following arguments are present:" << std::endl;
		std::cout<< "1. Full Path to urdf file/filename" << std::endl;
		std::cout<< "2. Robot Name" << std::endl;
		std::cout<< std::endl;
	}

	std::string urdf_filename = argv[1];
	std::string rob_name = argv[2];
	std::string collada_path = "../data/collada/";
	std::cout<< "Looking for file:  " << urdf_filename << std::endl;
	if (boost::filesystem::exists(urdf_filename))
	{
		std::cout<< "########################################################################" << std::endl;
		std::cout<< "Converting URDF to Collada" << std::endl;
		urdf_to_collada::convert_urdf(urdf_filename,collada_path + rob_name);

		std::cout<< "Conversion complete" << std::endl;	
		std::cout<< "########################################################################" << std::endl;
	}
	else
	{
		std::cout<< std::endl;
		std::cout<< "ERROR!" << std::endl;
		std::cout<< "File not found. Check name and path and try again" << std::endl;
	}

	std::string collda_name = collada_path + rob_name + ".dae";
	std::string prec_conv_command = "rosrun moveit_kinematics round_collada_numbers.py " + collda_name + " " + collda_name + " \"5\"";
	int ret_val = system(prec_conv_command.c_str());
	std::string link_info_cmd = "openrave-robot.py " + collda_name + " --info links";
	ret_val = system(link_info_cmd.c_str());

	int base_link;
	int tool_link;
	int free_link;

	std::cout<< std::endl;
	std::cout<< std::endl;

	std::cout<< "########################################################################" << std::endl;
	std::cout<< "USER INPUT REQUIRED" << std::endl;

	std::cout<< std::endl;
	std::cout<< "Refer to the link information above." << std::endl;
	std::cout<< "A typical 6-DOF manipulator should have 6 arm links + a dummy base_link as required by ROS specifications."  << std::endl;
	std::cout<< "If no extra links are present in the model, this gives: baselink=0 and eelink=6." << std::endl;
	std::cout<< "Often, an additional tool_link will be provided to position the grasp/tool frame, giving eelink=7." << std::endl;
	std::cout<< std::endl;

	std::cout<< "Enter base link (base_link) index" << std::endl;
	std::cin >> base_link;
	std::cout<< "Base link index Entered is: " << base_link << std::endl;
	std::cout<< std::endl;

	std::cout<< "Enter end-effector (tool0) link index" << std::endl;
	std::cin >> tool_link;
	std::cout<< "End-effector link index Entered is: " << tool_link << std::endl;
	std::cout<< std::endl;

	std::cout<< "Enter free link index if it is a >6 DOF arm or enter 0" << std::endl;
	std::cin >> free_link;
	std::cout<< "Free link index Entered is: " << free_link << std::endl;
	std::cout<< "########################################################################" << std::endl;
	std::cout<< std::endl;

	std::cout<< "########################################################################" << std::endl;
	std::cout<< "GENERATING IK" << std::endl;
	std::cout<< "########################################################################" << std::endl;
	std::string ik_file = "../src/" + rob_name + ".cpp";

	// For 6DOF arm
	if (free_link==0)
	{
		std::string ik_cmd = "python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=" + collda_name + " --iktype=transform6d --baselink=\"" + std::to_string(base_link) + "\" --eelink=\"" + std::to_string(tool_link) + "\" --savefile=" + ik_file;
		// std::cout<< ik_cmd << std::endl;
		ret_val = system(ik_cmd.c_str());
	}	
	else
	{
		std::string ik_cmd = "python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=" + collda_name + " --iktype=transform6d --baselink=" + std::to_string(base_link) + " --eelink=" + std::to_string(tool_link) + " --freeindex="+ std::to_string(free_link) +" --savefile=" + ik_file;
		ret_val = system(ik_cmd.c_str());
	}

	return 0;
}

// /home/rmalhan/Work/USC/Repositories/Inverse_Kinematics/analytical_IK/data/urdf/irb2600/abb_irb_2600.urdf