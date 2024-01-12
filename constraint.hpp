#ifndef __CONSTRAINT_HPP__
#define __CONSTRAINT_HPP__

#include <glm/glm.hpp>
#include "collision.hpp"
#include "particle.hpp"

class Constraint
{
public:
	Constraint();
	Constraint(Particle& incident, Particle& reference);
	Constraint(const Constraint& cp);
	Constraint& operator=(const Constraint& cp);
	float lagrangian(const glm::vec3& jacobi, Particle& bodyA, Particle& bodyB);
	float maximumDirection(const glm::vec3& searchDirection) const;
	virtual void solve() = 0;
	void solveShock(const glm::vec3& dir);

protected:
	friend class Collision;//tag return
	Particle* refA;
	Particle* refB;
	bool infiniteMass;
};

#endif
