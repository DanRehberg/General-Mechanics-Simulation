#include "integration.hpp"

namespace Simulation
{
	float deltaTime = 0.05f;
	float gravity = -1.f;
	glm::vec3 shockDirection(0.0f, -1.0f, 0.0f); //(0.903252f, 0.429111f, 0.f);// (0.0f, -1.0f, 0.0f);
	bool shockDirectionAlternate = false;
	bool shockpropagation = false;
	bool shockInfiniteMasses = true;
	size_t solverIterations = 10;
	long long updateInterval = 1000;
	bool verletResolution = true;
}
