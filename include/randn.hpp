#ifndef __RANDN_HPP__
#define __RANDN_HPP

#include <boost/random.hpp>

struct Randn
{
	typedef boost::rand48 ngn_t;
	typedef boost::normal_distribution<> dst_t;
	typedef boost::variate_generator<ngn_t, dst_t> vgen_t;

	Randn (double mean, double std) : variate(ngn_t(), dst_t(mean, std)) {}

	vgen_t variate;
};

#endif /* __RANDN_HPP__ */
