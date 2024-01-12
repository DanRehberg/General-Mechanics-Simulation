#include "particle.hpp"

Particle::Particle() : invMass(0.0f), mass(0.0f), position(0.0f, 0.0f, 0.0f), prevPosition(0.0f, 0.0f, 0.0f), velocity(0.0f, 0.0f, 0.0f)
{
	curInvMass = 0.0f;
	curMass = 0.0f;
}

Particle::Particle(const float mass, const glm::vec3& position) : mass(mass), position(position), prevPosition(position), velocity(0.0f, 0.0f, 0.0f)
{
	invMass = (mass == 0.0f) ? 0.0f : 1.0f / mass;
	curMass = mass;
	curInvMass = invMass;
}

Particle::Particle(const Particle& cp)
{
	curInvMass = cp.curInvMass;
	curMass = cp.curMass;
	invMass = cp.invMass;
	mass = cp.mass;
	position = cp.position;
	prevPosition = cp.prevPosition;
	velocity = cp.velocity;
}

Particle& Particle::operator=(const Particle& cp)
{
	curInvMass = cp.curInvMass;
	curMass = cp.curMass;
	invMass = cp.invMass;
	mass = cp.mass;
	position = cp.position;
	prevPosition = cp.prevPosition;
	velocity = cp.velocity;

	return *this;
}

void Particle::addPosition(const glm::vec3& offset)
{
	position += offset;
}

float Particle::getInverseMass() const
{
	if (Simulation::shockInfiniteMasses)
		return curInvMass;
	return invMass;
}

const glm::vec3& Particle::getPosition() const
{
	return position;
}

const glm::vec3& Particle::getPrevPosition() const
{
	return prevPosition;
}

float Particle::getMass() const
{
	if (Simulation::shockInfiniteMasses)
		return curMass;
	return mass;
}

float Particle::getShockInverseMass() const
{
	return curInvMass;
}

float Particle::getShockMass() const
{
	return curMass;
}

const glm::vec3 Particle::getVelocity() const
{
	return velocity;
}

void Particle::impulse(const glm::vec3& linear, const glm::vec3& angular)
{
	velocity += linear;
}

void Particle::integrateAcceleration(float dt, const glm::vec3& acceleration)
{
	velocity += dt * acceleration;
}

void Particle::integrateVelocity(float dt)
{
	prevPosition = position;
	position += dt * velocity;
	//Reset mass states if shock propagation is on
	if (Simulation::shockpropagation)
	{
		curInvMass = invMass;
		curMass = mass;
	}
}

void Particle::setPosition(const glm::vec3& val)
{
	position = val;
}

void Particle::setShockPropagationMass(bool reset)
{
	if (!reset)
	{
		curInvMass = 0.0f;
		curMass = 0.0f;
	}
	else
	{
		curInvMass = invMass;
		curMass = mass;
	}
}

void Particle::verlet(float dt)
{
	velocity = (position - prevPosition) / dt;
}
