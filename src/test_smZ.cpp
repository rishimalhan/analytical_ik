/* AUTHORED BY: Rishi Malhan
Center for Advanced Manufacturing
University of Southern California, LA, USA.
EMAIL: rmalhan@usc.edu */

#include <smZ_ik.hpp>
#include <boost/date_time.hpp>

int main(int argc, char** argv)
{
	Eigen::Matrix4d ff_T_tool = Eigen::Matrix4d::Identity(); // tool to flange transform
	ff_T_tool << 0,0.7029,0.7113,0.0255843,
                   0,-0.7113,0.7029,0.0258896,
                   1,0,0,0.2511166,
                   0,0,0,1 ;

	Eigen::Matrix4d target;
	target<< 0.0546203,   -0.010611,   -0.998451,     1.09245,
		         -0,    0.999887,  -0.0106274,    0.333887,
				   0.998451, 0.000580473,   0.0546203,     0.44867,
		          0,           0,           0,           1;

	std::vector<Eigen::MatrixXd> smZ_sol;

	double elapsed;
	boost::posix_time::ptime start_time;
	boost::posix_time::time_duration time_diff;
	start_time = boost::posix_time::microsec_clock::local_time();

	// for (int i=0; i<100; ++i)
		smZ_ik::compute_smZ_ik(target,ff_T_tool,smZ_sol);

	time_diff = boost::posix_time::microsec_clock::local_time() - start_time;
	elapsed = time_diff.total_nanoseconds() / 1e9;	
	std::cout<< "compute time: " << elapsed << std::endl;	

	std::cout<< "Number of Solutions Found: " << smZ_sol.size() << std::endl;
	// for (int i=0; i<smZ_sol.size(); i++)
	// {
	// 	std::cout<< smZ_sol[i] << std::endl;
	// }
	return 0;
}