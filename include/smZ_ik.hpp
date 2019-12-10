/* AUTHORED BY: Rishi Malhan
Center for Advanced Manufacturing
University of Southern California, LA, USA.
EMAIL: rmalhan@usc.edu */

#ifndef smZ_ik
#define smZ_ik

#include <iostream>
#include <ikfast_gateway.hpp>
#include <Eigen/Eigen>
#include <Eigen/Geometry> 

namespace smZ_ik
{
    inline void compute_smZ_ik(Eigen::Matrix4d &target, Eigen::Matrix4d &ff_T_tool, std::vector<Eigen::MatrixXd> &smZ_sol)
    {
        bool isValid;
        std::vector<double> free_links(1);
        Eigen::MatrixXd sol_mat; // N solutions. Nxdof matrix
        Eigen::Matrix4d ffTarget;

        std::vector<double> angles = {0,0.174533,-0.174533,0.349066,-0.349066,0.523599,-0.523599,0.698132,-0.698132,0.872665,-0.872665,1.047198,-1.047198,1.221730,-1.221730,1.396263,-1.396263,1.570796,-1.570796,1.745329,-1.745329,1.919862,-1.919862,2.094395,-2.094395,2.268928,-2.268928,2.443461,-2.443461,2.617994,-2.617994,2.792527,-2.792527,2.967060,-2.967060,3.141593,-3.141593};
        // Sample orientations about Z for the target
        for (int ctr=0; ctr<angles.size(); ctr++)
        {
            ffTarget = target;
            ffTarget.block(0,0,3,3) = Eigen::AngleAxisd(angles[ctr], Eigen::Vector3d(target(0,2), target(1,2), target(2,2))) * ffTarget.block(0,0,3,3);
            ffTarget = ff_T_tool * ffTarget; // Transform new target to the flange

            ik_analytical::compute_IK(ffTarget, free_links, isValid, sol_mat);
            if (isValid)
            {
                for (int i=0; i<sol_mat.rows(); i++)
                {
                    if (false) // Add collision and joint limit checkers here
                    {
                        continue;
                    }
                    smZ_sol.push_back(sol_mat.row(i));
                }
            }
        }
        
    };
}

#endif