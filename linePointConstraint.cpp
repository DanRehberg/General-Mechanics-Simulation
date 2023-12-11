#include "linePointConstraint.hpp"

LinePointConstraint::LinePointConstraint() : Constraint(), refB1(nullptr), refBCenter(nullptr)
{

}

LinePointConstraint::LinePointConstraint(Particle& a, Particle& b, Particle& b1, Particle& bc) :
	Constraint(a, b), refB1(&b1), refBCenter(&bc)
{

}

LinePointConstraint::LinePointConstraint(const LinePointConstraint& cp) : Constraint(cp)
{
	refB1 = cp.refB1;
	refBCenter = cp.refBCenter;
}

LinePointConstraint& LinePointConstraint::operator=(const LinePointConstraint& cp)
{
	refB1 = cp.refB1;
	refBCenter = cp.refBCenter;
	return *this;
}

void LinePointConstraint::solve()
{
	Particle& a(*refA);
	Particle& b(*refB);
	Particle& b1(*refB1);
	Particle& bCenter(*refBCenter);
	glm::vec3 forward(0.0f, 0.0f, 1.0f);
	glm::vec3 bLine = b.getPosition() - b1.getPosition();
	glm::vec3 norm = glm::normalize(glm::cross(bLine, forward));
	
	if (glm::dot(norm, bCenter.getPosition()) > 0.0f) norm = -norm;

	//Inequality allowed here.. just projecting to a plane if on the wrong side of it
	float distance = glm::dot(norm, (a.getPosition() - b.getPosition()));
	if (distance >= 0.03f) return;

	//FUTURE testing: the reference object may not be an infinite mass in the future
	//	For now, assumed so for Shock Propagation testing of Input Space Partition
	a.addPosition(-distance * norm);
	
	if (!Simulation::verletResolution) lagrangian(norm, a, b);
	else a.verlet(Simulation::deltaTime);
}