/* AUTHORED BY: Rishi Malhan
Center for Advanced Manufacturing
University of Southern California, LA, USA.
EMAIL: rmalhan@usc.edu */


#include <iostream>
#include <ikfast_gateway.hpp>
#include <Eigen/Eigen>
#include <random>


int main(int argc, char** argv)
{

    std::random_device rd; // obtain a random number from hardware
    std::mt19937 eng(rd()); // seed the generator
    std::uniform_real_distribution<> distr(-M_PI, M_PI); // define the range

    
    Eigen::MatrixXd joint_config(7,1);
    Eigen::Matrix4d tf;
    Eigen::Matrix4d ik_tf;
    bool isValid; // Success flag. True is solution found
    std::vector<double> free_links(1);
    free_links[0] = 1;
    
    bool status = true;
    joint_config << 0,0,0,0,0,0,0;
    std::cout<< "FK at zero location of all joints:  " << std::endl;
    std::cout<< ik_analytical::compute_FK(joint_config)  << std::endl;

    for (int i=0; i<100000; ++i)
    {
        joint_config << distr(eng),distr(eng),distr(eng),distr(eng),distr(eng),distr(eng),distr(eng); // 6DOF
        tf = ik_analytical::compute_FK(joint_config);    
        Eigen::MatrixXd sol_mat; // N solutions. Nxdof matrix
        ik_analytical::compute_IK(tf, free_links, isValid, sol_mat);
        if (isValid)
        {
            for (int j=0; j<sol_mat.rows();++j)
            {
                joint_config = sol_mat.row(j);
                ik_tf = ik_analytical::compute_FK(joint_config);
                // True if equal
                if (!ik_tf.isApprox(tf,1e-4))
                {
                    std::cout<< "Error in files. Invalid FK for IK solution" << std::endl;
                    status = false;
                    std::cout<< "Reference FK: " << std::endl;
                    std::cout<< tf << std::endl;
                    std::cout<< std::endl;
                    std::cout<< "FK from Ik solution: " << std::endl;
                    std::cout<< ik_tf << std::endl;
                    break;
                }           
            }
        }
        else
        {
            std::cout<< "Error in files. IK doesnot exist for FK" << std::endl;
            status = false;
            break;
        }

        if (!status)
            break;
    }

    if (status)
        std::cout<< "Verification Successful" << std::endl;
    return 0;
}