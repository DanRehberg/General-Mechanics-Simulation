/*
Author: Dan Rehberg
Purpose: Rebuilding Java mechanics project in C++ for further data testing
Date(s): 11/20/2023 - Input Space Partioning for Shock Propagation order
*/

#ifndef __INTEGRATION_HPP__
#define __INTEGRATION_HPP__

#include <glm/glm.hpp>

namespace Simulation
{
	extern float deltaTime;
	extern float gravity;
	extern glm::vec3 shockDirection;
	extern bool shockDirectionAlternate;
	extern bool shockpropagation;
	extern bool shockInfiniteMasses;
	extern size_t solverIterations;
	extern long long updateInterval;
	extern bool verletResolution;
}

class Integration
{
public:
	
private:
	virtual void impulse(const glm::vec3& linear, const glm::vec3& angular) = 0;
	virtual void integrateAcceleration(float dt, const glm::vec3& acceleration) = 0;
	virtual void integrateVelocity(float dt) = 0;
	virtual void verlet(float dt) = 0;
};

#endif