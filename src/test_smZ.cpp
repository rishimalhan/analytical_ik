/* AUTHORED BY: Rishi Malhan
Center for Advanced Manufacturing
University of Southern California, LA, USA.
EMAIL: rmalhan@usc.edu */

#include <smZ_ik.hpp>
#include <boost/date_time.hpp>

int main(int argc, char** argv)
{
	Eigen::Matrix4d target;
	target<< 1,0,0,1.03,
			0,1,0,0,
			0,0,1,1.26,
			0,0,0,1;

	std::vector<Eigen::MatrixXd> smZ_sol;

	double elapsed;
	boost::posix_time::ptime start_time;
	boost::posix_time::time_duration time_diff;
	start_time = boost::posix_time::microsec_clock::local_time();

	for (int i=0; i<100; i++)
		smZ_ik::compute_smZ_ik(target,smZ_sol);

	time_diff = boost::posix_time::microsec_clock::local_time() - start_time;
	elapsed = time_diff.total_nanoseconds() / 1e9;	
	std::cout<< "compute time: " << elapsed << std::endl;	
	
	// for (int i=0; i<smZ_sol.size(); i++)
	// {
	// 	std::cout<< smZ_sol[i] << std::endl;
	// }
	return 0;
}