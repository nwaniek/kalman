#ifndef __KALMAN_HPP__
#define __KALMAN_HPP__

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/lu.hpp>

#ifdef DEBUG_TXTOUT
#include <iostream>
#endif

namespace ublas = boost::numeric::ublas;


/**
 * Controlling matrices for the Kalman filter. As they might be constant over
 * time, they are packed into a class to make usage of the Kalman filter more
 * easy and less tedious.
 *
 * One might change these matrices in a running filtering session by using the
 * appropriate methods provided by the Kalman class.
 *
 * Note: Whenever you change the value of A or H, call either update_A_trans or
 * update_H_trans, respectively. this will ensure that transpose computation is
 * not required in each predict/correct step of the filter. This will fasten
 * computation of those steps a little bit when A and H don't change that much
 */
template <size_t L, size_t M, size_t N>
struct KalmanParams
{
	KalmanParams ()
		: A(ublas::zero_matrix<double>(N, N))
		, B(ublas::zero_matrix<double>(N, L))
		, H(ublas::zero_matrix<double>(M, N))
		, Q(ublas::zero_matrix<double>(N, N))
		, R(ublas::zero_matrix<double>(M, M))
		, A_trans(ublas::trans(A))
		, H_trans(ublas::trans(H))
	{};

	// controlling matrices:
	ublas::c_matrix<double, N, N> A; //! state transition matrix
	ublas::c_matrix<double, N, L> B; //! input control matrix
	ublas::c_matrix<double, M, N> H; //! state-to-measurement matrix

	// measurement noise
	ublas::c_matrix<double, N, N> Q; //! system noise. assumed to be normal distributed
	ublas::c_matrix<double, M, M> R; //! process noise. assumed to be normal distributed

	// transposes
	ublas::c_matrix<double, N, N> A_trans;
	ublas::c_matrix<double, N, M> H_trans;

	// auto-inline
	void update_A_trans ()
	{
		A_trans = ublas::trans(A);
	}

	void update_H_trans ()
	{
		H_trans = ublas::trans(H);
	}
};


/**
 * Implementation of the Kalman Filter as described in "An Introduction to the
 * Kalman Filter" by Greg Welch and Gary Bishop, Department of Computer
 * Science, University of North Carolina, July 24, 2006, further refered to as
 * "The Paper".
 *
 * The discrete Kalman filter consists of two steps:
 *   1) predict / time update
 *      In this step the Kalman filter will try to predict the next value of x
 *   2) correct / measurement update
 *      The filter will apply a new measurement to its 'knowledge base', where
 *      the knowledge base are the matrices that are used in computation of the
 *      prediction.
 *
 *
 *
 * N : output dimension
 * M : measurement dimension
 * L : control input dimension
 *
 */
template <size_t L, size_t M, size_t N>
class Kalman
{
public:
	typedef Kalman<L, M, N> Type;
public:
	/**
	 * Create a new instance of the Kalman filter. Internal states will be
	 * initialized as
	 *
	 * x     = 0 (zero vector)
	 * x_est = 0 (zero vector)
	 * u     = 0 (zero vector)
	 * P     = identity matrix
	 * P_est = identity matrix
	 * K     = zero matrix
	 * I     = identity matrix
	 *
	 */
	Kalman (KalmanParams<L, M, N> & params)
		: x(ublas::zero_vector<double>(N))
		, x_est(ublas::zero_vector<double>(N))
		, u(ublas::zero_vector<double>(L))
		, P(ublas::identity_matrix<double>(N, N))
		, P_est(ublas::identity_matrix<double>(N,N))
		, K(ublas::zero_matrix<double>(N, M))
		, I(ublas::identity_matrix<double>(N))
		, m_params(params)
	{}

	/**
	 * Create a new instance of the Kalman filter. Internal states will be
	 * initialized as
	 *
	 * x     = x_init
	 * x_est = x_init
	 * u     = 0 (zero vector)
	 * P     = identity matrix
	 * P_est = identity matrix
	 * K     = zero matrix
	 * I     = identity matrix
	 *
	 */
	Kalman (KalmanParams<L, M, N> & params,
			ublas::c_vector<double, N> const x_init)
		: x(x_init)
		, x_est(x_init)
		, u(ublas::zero_vector<double>(L))
		, P(ublas::identity_matrix<double>(N, N))
		, P_est(ublas::identity_matrix<double>(N,N))
		, K(ublas::zero_matrix<double>(N, M))
		, I(ublas::identity_matrix<double>(N))
		, m_params(params)
	{}


// Kalman Filter routines
public:
	/**
	 *
	 * As one might not like using the name 'predict' for this step, the
	 * function is aliased to the name 'time_update' to reflect the naming in
	 * some papers occupied with the Kalman filter.
	 */
	ublas::c_vector<double, N>& predict ();
	ublas::c_vector<double, N>& time_update () __attribute__((alias("predict")));

	/**
	 *
	 * As one might not like using the name 'correct' for this step, the
	 * function is aliased to the name 'measurement_update' to reflect the
	 * naming in some papers occupied with the Kalman filter.
	 */
	ublas::c_vector<double, N>& correct (ublas::c_vector<double, M> const z);
	ublas::c_vector<double, N>& measurement_update (ublas::c_vector<double, M> const z); __attribute__((alias("correct")));


// modifiers of the Kalman Filter state
public:
	/**
	 * Update the control Input (references as 'u' in The Paper)
	 */
	void update_control_input (ublas::c_vector<double, L> cntrl_input)
	{
		this->u = cntrl_input;
	}

	/**
	 * Return the used parameters. As they are returned by reference,
	 * they might be modified directly. Changes to the parameters will be reflected
	 * in the Kalman Filter routines.
	 */
	KalmanParams<L, M, N>& get_params ()
	{
		return this->m_params;
	}


// const GETTERs
public:
	/**
	 * return the currently predicted value for x_est
	 */
	ublas::c_vector<double, N> const& get_x_est () const
	{
		return this->x_est;
	}

	/**
	 * return the currently stored value for x
	 */
	ublas::c_vector<double, N> const& get_x () const
	{
		return this->x;
	}

	/**
	 * return the currently stored error covariance matrix P
	 */
	ublas::c_matrix<double, N, N> const& get_P () const
	{
		return this->P;
	}

	/**
	 * return the currently predicted error covariance matrix P
	 */
	ublas::c_matrix<double, N, N> const& get_P_est () const
	{
		return this->P_est;
	}

	/**
	 * return the currently used kalman gain
	 */
	ublas::c_matrix<double, M, N> const& get_K () const
	{
		return this->K;
	}


private:
	// current and predicted value for x
	ublas::c_vector<double, N> x;
	ublas::c_vector<double, N> x_est;

	// current control input
	ublas::c_vector<double, L> u;

	// Error covariance matrix P
	ublas::c_matrix<double, N, N> P;
	ublas::c_matrix<double, N, N> P_est;

	// Kalman gain
	ublas::c_matrix<double, N, M> K;

	// Identity Matrix for computational purposes
	ublas::identity_matrix<double> I;

	// Parameters to be used
	KalmanParams<L, M, N> &m_params;
};


/**
 * invert a matrix. T has to be ublas::c_matrix<T, E, F>. If LAPACK
 * is installed, use it to gain speed
 *
 * TODO: tidy up, implement LAPACK solution
 */
template <class T>
inline void invert_matrix (T const& input, T &inverse)
{
#ifdef HAVE_LAPACK
	// TODO: implement LAPACK solution
#else
	T A(input);
	ublas::permutation_matrix<size_t> pm(A.size1());
	ublas::lu_factorize(A, pm);
	inverse.assign(ublas::identity_matrix<double>(A.size1()));
	ublas::lu_substitute(A, pm, inverse);
#endif
}


template <size_t L, size_t M, size_t N>
ublas::c_vector<double, N>& Kalman<L, M, N> ::
predict ()
{
	/*
	 * x_est = A * x + B * u;
	 * P_est = A * P * A' + Q;
	 */

	// x_est =          A         * x  +             B         * u
	x_est = ublas::prod(m_params.A, x) + ublas::prod(m_params.B, u);
#ifdef DEBUG_TXTOUT
	std::cout << "predict: x_est = " << x_est << std::endl;
#endif


	P_est = ublas::prod(m_params.A, P);
	P_est = ublas::prod(P_est, m_params.A_trans) + m_params.Q;
#ifdef DEBUG_TXTOUT
	std::cout << "predict: P_est = " << P_est << std::endl;
#endif

	return x_est;
}


template <size_t L, size_t M, size_t N>
ublas::c_vector<double, N>& Kalman<L, M, N> ::
correct (ublas::c_vector<double, M> const z)
{
	// make the code more readable
	using namespace ublas;
	typedef c_matrix<double,M,M> MxM_t;
	typedef c_matrix<double,N,M> NxM_t;

	/*
	 * K: Kalman Gain
	 *    K = P_est * H' * inv(H*P_est*H' + R)
	 *
	 * x = x_est + K*(z - H*x_est)
	 *
	 * P = (I - K*H)*P_est
	 */

	// precompute P_est*H' as it is used twice
	//    PestHtrans =      P_est * H'
	NxM_t PestHtrans = prod(P_est, m_params.H_trans);
#ifdef DEBUG_TXTOUT
	std::cout << "correct: PestHtrans = " << PestHtrans << std::endl;
#endif

	// calculate the inverse
	MxM_t inverse;
	//                          H *         P_est * H'  + R
	invert_matrix< MxM_t >(prod(m_params.H, PestHtrans) + m_params.R, inverse);
#ifdef DEBUG_TXTOUT
	std::cout << "correct: inverse = " << inverse << std::endl;
#endif

	// K =   P_est * H' * inverse
	K = prod(PestHtrans, inverse);
#ifdef DEBUG_TXTOUT
	std::cout << "correct: K = " << K << std::endl;
#endif

	// x = x_est +      K * (z -     (H *         x_est))
	x =    x_est + prod(K,  (z - prod(m_params.H, x_est)));
#ifdef DEBUG_TXTOUT
	std::cout << "correct: x = " << x << std::endl;
#endif

	// P =  (I -      K* H)         * P_est
	P = prod(I - prod(K, m_params.H), P_est);
#ifdef DEBUG_TXTOUT
	std::cout << "correct: P = " << P << std::endl;
#endif

	return x;
}



#endif /* __KALMAN_HPP__ */
