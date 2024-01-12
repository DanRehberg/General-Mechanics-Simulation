#include "constraint.hpp"
#include <iostream>

Constraint::Constraint() : refA(nullptr), refB(nullptr), infiniteMass(false)
{

}

Constraint::Constraint(Particle& incident, Particle& reference) :
	refA(&incident), refB(&reference)
{
	if (refA->getMass() == 0.0f || refB->getMass() == 0.0f)infiniteMass = true;
}

Constraint::Constraint(const Constraint& cp)
{
	refA = cp.refA;
	refB = cp.refB;
	infiniteMass = cp.infiniteMass;
}

Constraint& Constraint::operator=(const Constraint& cp)
{
	refA = cp.refA;
	refB = cp.refB;
	infiniteMass = cp.infiniteMass;
	return *this;
}

float Constraint::lagrangian(const glm::vec3& J, Particle& A, Particle& B)//Expects n w/r to reference object (which is object B)
{
	//J dot V / JM^-1J^t
	float jv = glm::dot(J, A.getVelocity()) + glm::dot(-J, B.getVelocity());//Note, can be simplified
	float jmj = glm::dot(J, A.getShockInverseMass() * J) + glm::dot(-J, B.getShockInverseMass() * (-J));
	float langrange = (jmj == 0.0f) ? 0.0f : jv / jmj;
	glm::vec3 weightedImpulse = langrange * J;
	A.impulse(-A.getShockInverseMass() * weightedImpulse);
	B.impulse(B.getShockInverseMass() * weightedImpulse);
	return langrange;
}

float Constraint::maximumDirection(const glm::vec3& dir) const
{
	float a = glm::dot(dir, refA->getPrevPosition());
	float b = glm::dot(dir, refB->getPrevPosition());
	return (a > b) ? a : b;
}

void Constraint::solveShock(const glm::vec3& dir)
{
	if (!infiniteMass)
	{
		if (glm::dot(refA->getPosition(), dir) > glm::dot(refB->getPosition(), dir))
		{
			refA->setShockPropagationMass();
			refB->setShockPropagationMass(true);
		}
		else
		{
			refB->setShockPropagationMass();
			refA->setShockPropagationMass(true);
		}
	}
	else
	{
		refA->setShockPropagationMass(true);
		refB->setShockPropagationMass(true);
	}
	solve();
}
