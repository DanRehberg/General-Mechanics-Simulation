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

	if (glm::dot(norm, bCenter.getPosition() - b1.getPosition()) > 0.0f) norm = -norm;

	//Inequality allowed here.. just projecting to a plane if on the wrong side of it
	float distance = glm::dot(norm, (a.getPosition() - b.getPosition()));
	if (distance >= 0.03f) return;

	//Testing new
	float massSum = b.getMass() + a.getMass();
	float invSum = (massSum == 0.0f) ? 0.0f : 1.0f / massSum;
	float ratioA = (massSum == 0.0f) ? 0.0f : (a.getMass() * invSum);
	float ratioB = (massSum == 0.0f) ? 0.0f : (bCenter.getMass() * invSum);

	//glm::vec3 a0 = a.getPosition();
	//glm::vec3 a1 = a0 - distance * norm;
	//norm = (glm::length(a1 - a0) < 0.0001f) ? norm : a1 - a0;
	//norm = glm::normalize(norm);

	glm::vec3 correction = -distance * norm;
	a.addPosition(ratioA * correction);
	bCenter.addPosition(-ratioB * correction);

	

	if (!Simulation::verletResolution)
	{
		//BEGIN TEST
		// Jacobi solution IFF moving towards violating direction
		//glm::vec3 relativeNormalVelocity = norm * (glm::dot(b.getVelocity(), norm) - glm::dot(a.getVelocity(), norm));
		//lagrangian(relativeNormalVelocity, a, b);
		//if (norm.x * (a.getVelocity().x - bCenter.getVelocity().x) > 0.0f) norm.x = 0;
		//if (norm.y * (a.getVelocity().y - bCenter.getVelocity().y) > 0.0f) norm.y = 0;
		//if (norm.x == 0.0f && norm.y == 0.0f) return;
		//END TEST
		
		float l = lagrangian(norm, a, bCenter);
	}
	else
	{
		a.verlet(Simulation::deltaTime);
		bCenter.verlet(Simulation::deltaTime);
	}
}
