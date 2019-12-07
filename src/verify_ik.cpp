/* AUTHORED BY: Rishi Malhan
Center for Advanced Manufacturing
University of Southern California, LA, USA.
EMAIL: rmalhan@usc.edu */


#include <iostream>
#include <ikfast_gateway.hpp>
#include <Eigen/Eigen>

int main(int argc, char** argv)
{
    Eigen::MatrixXd joint_config(7,1);
    joint_config << 0,0,0,0,0,0,0;

    std::cout<< "Checking FK" << std::endl;
    Eigen::Matrix4d tf = ik_analytical::compute_FK(joint_config);    
    std::cout<< tf << std::endl;

    std::cout<< "Checking IK" << std::endl;
    bool isValid; // Success flag. True is solution found
    std::vector<double> free_links(1);
    free_links[0] = 1;
    Eigen::MatrixXd sol_mat; // N solutions. Nxdof matrix
    ik_analytical::compute_IK(tf, free_links, isValid, sol_mat);
    if (isValid)
    {
        std::cout<< "Solution Found" << std::endl;
        std::cout<< sol_mat << std::endl;
    }
    else
    {
        std::cout<< "Solution Not Found" << std::endl;
    }
    return 0;
}