#ifndef __LINE_POINT_CONSTRAINT_HPP__
#define __LINE_POINT_CONSTRAINT_HPP__

#include <glm/glm.hpp>
#include "constraint.hpp"

class LinePointConstraint final : public Constraint
{
public:
	LinePointConstraint();
	LinePointConstraint(Particle& incident, Particle& ref0, Particle& ref1, Particle& refCenter);
	LinePointConstraint(const LinePointConstraint& cp);

	LinePointConstraint& operator=(const LinePointConstraint& cp);

	void solve();
private:
	Particle* refB1;
	Particle* refBCenter;
};

#endif