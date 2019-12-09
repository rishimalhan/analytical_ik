/* AUTHORED BY: Rishi Malhan
Center for Advanced Manufacturing
University of Southern California, LA, USA.
EMAIL: rmalhan@usc.edu */

#ifndef smZ_ik
#define smZ_ik
#define M_PI = 3.14159265358979323846;

#include <iostream>
#include <ikfast_gateway.hpp>
#include <Eigen/Eigen>
#include <Eigen/Geometry> 

namespace smZ_ik
{
    inline void compute_smZ_ik(Eigen::Matrix4d &target, std::vector<Eigen::MatrixXd> &smZ_sol)
    {
        bool isValid;
        std::vector<double> free_links(1);
        Eigen::MatrixXd sol_mat; // N solutions. Nxdof matrix
        double resolution = 10 * M_PI / 180; // Change the resolution as required.

        // Sample orientations about Z for the target
        for (double alpha=0; alpha<2*M_PI; alpha+=resolution)
        {
            target.block(0,0,3,3) = Eigen::AngleAxisd(resolution, Eigen::Vector3d(target(0,2), target(1,2), target(2,2))) * target.block(0,0,3,3);
            ik_analytical::compute_IK(target, free_links, isValid, sol_mat);
            if (isValid)
            {
                for (int i=0; i<sol_mat.rows(); i++)
                {
                    if (false) // Add collision and joint limit checkers here
                    {
                        continue;
                    }

                    smZ_sol.push_back(sol_mat);
                }
            }
        }
        
    };
}

#endif