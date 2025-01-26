#ifndef CIRCLE_FIT_HPP_
#define CIRCLE_FIT_HPP_

#include <vector>
#include <ceres/ceres.h>

struct Circle
{
	double x, y, r;
};

class CircleFitting
{
public:
	CircleFitting(double aX, double aY, double aRadious)
			: mX(aX), mY(aY), mRadious(aRadious)
	{
	}

	template <typename T>
	bool operator()(const T *const aX, const T *const aY, T *aResidual) const
	{
		T xp = mX - *aX;
		T yp = mY - *aY;
		// sqrt() adds strong nonlinearities to the cost function, units distance^2
		// produces more robust fits when there are outliers.
		aResidual[0] = mRadious * mRadious - xp * xp - yp * yp;
		return true;
	}

private:
	double mX, mY, mRadious;
};

Circle CircleFitByCeres(const std::vector<float> &aVecX,
												const std::vector<float> &aVecY,
												float aRadious);

#endif