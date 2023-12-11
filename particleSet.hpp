#ifndef __PARTICLE_SET_HPP__
#define __PARTICLE_SET_HPP__

#include <exception>
#include <glm/glm.hpp>
#include <vector>
#include "collision.hpp"
#include "integration.hpp"

class ParticleSet final : private Collision, private Integration
{
public:
	ParticleSet(float mass);

	
	virtual void update(float dT);
protected:
	float mass;
	glm::vec3 prevPosition;
	std::vector<glm::vec3> prevVerts;
	std::vector<glm::vec3> velocities;

};

#endif