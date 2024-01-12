#include "distanceConstraint.hpp"

DistanceConstraint::DistanceConstraint() : Constraint(), distance(0.0f)
{

}

DistanceConstraint::DistanceConstraint(Particle& A, Particle& B, float d) : Constraint(A, B)
{
	distance = d;
}

DistanceConstraint::DistanceConstraint(const DistanceConstraint& cp) : Constraint(cp)
{
	distance = cp.distance;
}

DistanceConstraint& DistanceConstraint::operator=(const DistanceConstraint& cp)
{
	distance = cp.distance;
	return *this;
}

void DistanceConstraint::solve()
{
	Particle& a(*refA);
	Particle& b(*refB);
	glm::vec3 incToRef = b.getPosition() - a.getPosition();
	float dist = glm::length(incToRef);
	float violation = dist - distance;

	if (dist == 0.0f)
		incToRef = glm::vec3(0.0f, 1.0f, 0.0f);
	else
		incToRef *= (1.0f / dist);

	float massSum = b.getMass() + a.getMass();
	float invSum = (massSum == 0.0f) ? 0.0f : 1.0f / massSum;
	float ratioA = (massSum == 0.0f) ? 0.0f : 1.0f - (a.getMass() * invSum);
	float ratioB = (massSum == 0.0f) ? 0.0f : 1.0f - (b.getMass() * invSum);

	glm::vec3 correction = violation * incToRef;
	a.addPosition(ratioA * correction);
	b.addPosition(-ratioB * correction);

	if (!Simulation::verletResolution)
	{
		//glm::vec3 relativeNormalVelocity = incToRef * (glm::dot(b.getVelocity(), incToRef) - glm::dot(a.getVelocity(), incToRef));
		//lagrangian(relativeNormalVelocity, a, b);
		lagrangian(b.getVelocity() - a.getVelocity(), a, b);
	}
	else
	{
		a.verlet(Simulation::deltaTime);
		b.verlet(Simulation::deltaTime);
	}
}
