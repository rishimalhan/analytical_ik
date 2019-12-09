/* AUTHORED BY: Rishi Malhan
Center for Advanced Manufacturing
University of Southern California, LA, USA.
EMAIL: rmalhan@usc.edu */

#include <smZ_ik.hpp>

int main(int argc, char** argv)
{
	Eigen::Matrix4d target;
	target<< 1,0,0,1.03,
			0,1,0,0,
			0,0,1,1.26,
			0,0,0,1;
			
	std::vector<Eigen::MatrixXd> smZ_sol;
	smZ_ik::compute_smZ_ik(target,smZ_sol);

	for (int i=0; i<smZ_sol.size(); i++)
	{
		std::cout<< smZ_sol[i] << std::endl;
	}
	return 0;
}