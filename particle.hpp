#ifndef __PARTICLE_HPP__
#define __PARTICLE_HPP__

#include "integration.hpp"
#include <glm/glm.hpp>

class Particle final : public Integration
{
public:
	Particle();
	Particle(const float mass, const glm::vec3& position);
	Particle(const Particle& cp);

	Particle& operator=(const Particle& cp);

	void addPosition(const glm::vec3& offset);
	float getInverseMass() const;
	const glm::vec3& getPosition() const;
	const glm::vec3& getPrevPosition() const;
	float getMass() const;
	float getShockInverseMass() const;
	float getShockMass() const;
	const glm::vec3 getVelocity() const;
	void setPosition(const glm::vec3& location);
	void setShockPropagationMass(bool reset = false);

	void impulse(const glm::vec3& linear, const glm::vec3& angular = glm::vec3(0.f));
	void integrateAcceleration(float dt, const glm::vec3& acceleration);
	void integrateVelocity(float dt);
	void verlet(float dt);

	size_t id;

private:
	friend class Collision;
	float curInvMass;
	float curMass;
	float invMass;
	float mass;
	glm::vec3 position;
	glm::vec3 prevPosition;
	glm::vec3 velocity;
};

#endif
