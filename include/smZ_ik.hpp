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
    bool within_joint_limits(Eigen::MatrixXd joints){
        double delta = 0;
        if(joints(0,0)<-3.14159+delta || joints(0,0)> 3.14159-delta)
            return false;
        if(joints(0,1)<-2.705+delta || joints(0,1)> 1.658-delta)
            return false;
        if(joints(0,2)<-2.705+delta || joints(0,2)> 1.309-delta)
            return false;
        if(joints(0,3)<-6.981+delta || joints(0,3)> 6.981-delta)
            return false;
        if(joints(0,4)<-2.094+delta || joints(0,4)> 2.094-delta)
            return false;
        if(joints(0,5)<-6.981+delta || joints(0,5)> 6.981-delta)
            return false;   
        return true;     
    }

    inline void compute_smZ_ik(Eigen::Matrix4d target, Eigen::Matrix4d &ff_T_tool, std::vector<Eigen::MatrixXd> &smZ_sol)
    {
        bool isValid;
        std::vector<double> free_links(1);
        Eigen::MatrixXd sol_mat; // N solutions. Nxdof matrix
        Eigen::Matrix4d ffTarget;
        Eigen::Matrix4d ffTarget_copy;
        Eigen::Matrix4d tool_T_ff = Eigen::Matrix4d::Identity();

        // std::vector<double> angles = {0,0.174533,-0.174533,0.349066,-0.349066,0.523599,-0.523599,0.698132,-0.698132,0.872665,-0.872665,1.047198,-1.047198,1.221730,-1.221730,1.396263,-1.396263,1.570796,-1.570796,1.745329,-1.745329,1.919862,-1.919862,2.094395,-2.094395,2.268928,-2.268928,2.443461,-2.443461,2.617994,-2.617994,2.792527,-2.792527,2.967060,-2.967060,3.141593,-3.141593};

        tool_T_ff.block(0,0,3,3) = ff_T_tool.block(0,0,3,3).transpose();
        tool_T_ff.block(0,3,3,1) = -ff_T_tool.block(0,0,3,3).transpose()*ff_T_tool.block(0,3,3,1);

        tool_T_ff.block(0,0,3,1) = tool_T_ff.block(0,0,3,1) / tool_T_ff.block(0,0,3,1).norm();
        tool_T_ff.block(0,1,3,1) = tool_T_ff.block(0,1,3,1) / tool_T_ff.block(0,1,3,1).norm();
        tool_T_ff.block(0,2,3,1) = tool_T_ff.block(0,2,3,1) / tool_T_ff.block(0,2,3,1).norm();

        ffTarget = target;
        ffTarget.block(0,0,3,1) = target.block(0,0,3,1) / target.block(0,0,3,1).norm();
        ffTarget.block(0,1,3,1) = target.block(0,1,3,1) / target.block(0,1,3,1).norm();
        ffTarget.block(0,2,3,1) = target.block(0,2,3,1) / target.block(0,2,3,1).norm();
        ffTarget_copy = ffTarget;
        // std::cout<< "Incoming Target:  " << target << std::endl;

        // Sample orientations about Z for the target
        // for (int ctr=0; ctr<angles.size(); ctr++)
        for (int ctr=0; ctr<180; ++ctr)
        {
            for (double sign = -1; sign<2; sign+=2 ){
                double rm = sign * ctr/180*3.1416;
                ffTarget = ffTarget_copy;
                ffTarget.block(0,0,3,3) = Eigen::AngleAxisd(rm, Eigen::Vector3d(ffTarget_copy(0,2), ffTarget_copy(1,2), ffTarget_copy(2,2))) * ffTarget_copy.block(0,0,3,3);
                ffTarget = ffTarget * tool_T_ff; // Transform new target to the flange
                // std::cout<< "Target:  " << ffTarget << std::endl;
                ik_analytical::compute_IK(ffTarget, free_links, isValid, sol_mat);

                if (isValid)
                {
                    for (int i=0; i<sol_mat.rows(); i++)
                    {
                        if (within_joint_limits(sol_mat.block(i,0,1,6))) // Add collision and joint limit checkers here
                            smZ_sol.push_back(sol_mat.row(i));
                    }
                }
            }
        }   
    };

}

#endif