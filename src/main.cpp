#include <cstdlib>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/random.hpp>

#include <iostream>
#include <fstream>
#include <cmath>

#include <vector>

#include "kalman.hpp"
#include "randn.hpp"

namespace ublas = boost::numeric::ublas;



inline double
z_sin (double tick, Randn &rng)
{
	return 1 + sin(tick*.1) + rng.variate() * .10;
}


inline double
z_cos (double tick, Randn &rng)
{
	return 1 + cos(tick*.04) + rng.variate() * .10;
}



void
kalman_test ()
{
	// --- SETUP --------------------------------------------------------------

	// dimension of measurements, system output, control
	const size_t dim_output  = 4;
	const size_t dim_input   = 2;
	const size_t dim_control = 0;

	//
	// control matrices
	//
	KalmanParams<dim_control, dim_input, dim_output> params;

	// A =
	// 1 0 1 0
	// 0 1 0 1
	// 0 0 1 0
	// 0 0 0 1
	params.A += ublas::identity_matrix<double>(dim_output);
	params.A(0,2) = 1.0;
	params.A(1,3) = 1.0;
	params.update_A_trans();	// DON't forget this after A was altered!

	// B = 0 -> nothing to do, is initialized to 0

	// H =
	// 1 0 0 0
	// 0 1 0 0
	params.H(0,0) = 1.0;
	params.H(1,1) = 1.0;
	params.update_H_trans();	// DON'T forget this after H was altered!

	// Q : system noise
	params.Q += .05 * ublas::identity_matrix<double>(dim_output);

	// R : measurement noise
	params.R += 64 * ublas::identity_matrix<double>(dim_input);

	//
	// initial 'guessed' output of the system. not required
	//
	ublas::c_vector<double, dim_output> x;
	x[0] = 1;
	x[1] = 1;
	x[2] = 0;
	x[3] = 0;



	// Finally, set up the kalman filter
	typedef Kalman<dim_control, dim_input, dim_output>::Type kalman_type;
	kalman_type kalman(params, x);
	kalman_type kalman2(params, x);

	std::vector<void*> vec;
	vec.push_back(reinterpret_cast<void*>(&kalman));
	vec.push_back(reinterpret_cast<void*>(&kalman2));


	std::vector<kalman_type*> vec2;
	vec2.push_back(&kalman);
	vec2.push_back(&kalman2);





	// --- RUN ----------------------------------------------------------------
	//
	// will create a matlab file for plotting purposes...
	//

	int ki = 0;
	for (std::vector<kalman_type*>::iterator it = vec2.begin();
		it < vec2.end();
		it++, ki++) {

		// kalman_type k = *(reinterpret_cast<kalman_type*>(*it));
		kalman_type k = **it;

		// setting up boost random number generator with normal distribution
		// between -1 and 1
		Randn rng(0.0, 1.0);

		// timesteps to iterate
		int t = 250;

		// measurement. just allocate some space here
		ublas::c_vector<double, dim_input> z;

		// make it easy to generate the matlab file afterwards
		std::vector<double> x_0;
		std::vector<double> x_1;
		std::vector<double> z_0;
		std::vector<double> z_2;

		typedef ublas::c_vector<double, dim_input> vecIn_t;
		typedef ublas::c_vector<double, dim_output> vecOut_t;
		std::vector<vecOut_t> X;
		std::vector<vecIn_t> Z;

		for (int i = 0; i < t; i++) {
			if (i > 40 && i < 120) {
				z[0] = 1;
				z[1] = 1;
			}
			else {
				z[0] = z_sin(i+1, rng);
				z[1] = z_cos(i+1, rng);
			}

			Z.push_back(vecIn_t(z));

			// predict
			X.push_back(vecOut_t( k.predict() ));
			// correct
			k.correct(z);
		}

	// --- MATLAB -------------------------------------------------------------

		// create a nice matlab file for plotting
		std::ofstream out;
		char fname[128];
		sprintf(fname, "kalman_demo_%d.m", ki);
		out.open(fname);
		if (!out) {
			std::cerr << "could not acquire file handle for kalman_demo.m to write results to" << std::endl;
			return;
		}
		out << "clc; clear all; close all;" << std::endl;
		std::vector<vecIn_t>::iterator z_it;
		std::vector<vecOut_t>::iterator x_it;

		// z data
		out << "z = [";
		int i = 0;
		for (z_it = Z.begin(); z_it != Z.end(); z_it++, i++) {
			out << (*z_it)[0];
			if (i < t) {
				out << " ";
			}
		}
		out << "; ..." << std::endl;

		out << "     ";
		i = 0;
		for (z_it = Z.begin(); z_it != Z.end(); z_it++, i++) {
			out << (*z_it)[1];
			if (i < t) {
				out << " ";
			}
		}
		out << "];" << std::endl;

		// x data
		out << "x = [";
		i = 0;
		for (x_it = X.begin(); x_it != X.end(); x_it++, i++) {
			out << (*x_it)[0];
			if (i < t) {
				out << " ";
			}
		}
		out << "; ..." << std::endl;

		out << "     ";
		i = 0;
		for (x_it = X.begin(); x_it != X.end(); x_it++, i++) {
			out << (*x_it)[1];
			if (i < t) {
				out << " ";
			}
		}
		out << "];" << std::endl;

		out << "duration = " << t << ";" << std::endl
			<< "timeline = 1:1:duration;" << std::endl
			<< "subplot(1,1,1)" << std::endl
			<< "plot3(timeline, z(2,:), z(1,:), '-k', ..." << std::endl
			<< "      timeline, x(2,:), x(1,:), '-r');" << std::endl
			<< "xlabel('t');" << std::endl
			<< "ylabel('x');" << std::endl
			<< "zlabel('y');" << std::endl
			<< "title('kalman');" << std::endl
			<< "legend('orig', 'est')" << std::endl
		;

		out.close();
	}
}



int
main (int, char**)
{
	// testing kalman
	kalman_test();

	return EXIT_SUCCESS;
}
