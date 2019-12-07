/* AUTHORED BY: Rishi Malhan
Center for Advanced Manufacturing
University of Southern California, LA, USA.
EMAIL: rmalhan@usc.edu */


#ifndef IKFAST_GATEWAY
#define IKFAST_GATEWAY
#define IKFAST_NO_MAIN

#include <Eigen/Eigen>

// include <robot_name.cpp>
#include <irb2600.cpp>
// #include <gp8.cpp>
// #include <iiwa7.cpp>

using namespace ikfast;

namespace ik_analytical
{
	inline Eigen::Matrix4d compute_FK(Eigen::MatrixXd &_joint_config)
	{
		IkReal joint_config[GetNumJoints()];
        for (int i=0; i<GetNumJoints(); i++)
            joint_config[i] = _joint_config(i);
        
        // Convert to IKfast datatypes
        // FK
        IkReal rotation[9],translation[3];
        ComputeFk(joint_config, translation, rotation);
        Eigen::Matrix4d tf;
        tf << rotation[0], rotation[1], rotation[2], translation[0],
            rotation[3], rotation[4], rotation[5], translation[1],
            rotation[6], rotation[7], rotation[8], translation[2],
            0, 0, 0, 1;
        
        return tf;
	};

	inline void compute_IK(Eigen::Matrix4d &target, std::vector<double> &free_links, bool &bSuccess, Eigen::MatrixXd &sol_mat)
	{
		// IK
        IkSolutionList<IkReal> solutions;

        // Free link indices. Not needed for this robot. 6DOF
        std::vector<IkReal> vfree(GetNumFreeParameters());
        for(std::size_t i = 0; i < vfree.size(); ++i)
            vfree[i] = free_links[i];
            
        // Convert to IKfast datatypes
        IkReal eerot[9],eetrans[3];
        eerot[0] = target(0,0); eerot[1] = target(0,1); eerot[2] = target(0,2);
        eerot[3] = target(1,0); eerot[4] = target(1,1); eerot[5] = target(1,2);
        eerot[6] = target(2,0); eerot[7] = target(2,1); eerot[8] = target(2,2);

        eetrans[0] = target(0,3); eetrans[1] = target(1,3); eetrans[2] = target(2,3);
        // IK
        bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
        
        int no_sols = solutions.GetNumSolutions();
        sol_mat.resize(no_sols,GetNumJoints());

        std::vector < IkReal > solvalues(GetNumJoints());
        std::vector < double > temp_sols(GetNumJoints());         
        for (int i=0; i<no_sols; i++)
        {
            const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
            std::vector<IkReal> vsolfree(sol.GetFree().size());
            sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
            temp_sols = static_cast<std::vector<double>> (solvalues);
            // std::cout<< temp_sols[0] << " , " << temp_sols[1] << " , " << std::endl;
            sol_mat.row(i) = Eigen::VectorXd::Map(&temp_sols[0], temp_sols.size());
        }
	};

}


#endif