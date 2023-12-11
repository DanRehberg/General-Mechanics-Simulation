#ifndef __DISTANCE_CONSTRAINT_HPP__
#define __DISTANCE_CONSTRAINT_HPP__

#include "constraint.hpp"

class DistanceConstraint final : public Constraint
{
public:
	DistanceConstraint();
	DistanceConstraint(Particle& incident, Particle& reference, float distance);
	DistanceConstraint(const DistanceConstraint& cp);
	DistanceConstraint& operator=(const DistanceConstraint& cp);

	void solve();
private:
	float distance;
};

#endif