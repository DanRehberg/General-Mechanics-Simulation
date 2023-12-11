#include "integration.hpp"

namespace Simulation
{
	float deltaTime = 0.5f;
	float gravity = -1.f;
	glm::vec3 shockDirection(0.0f, -1.0f, 0.0f);
	bool shockDirectionAlternate = false;
	bool shockpropagation = false;
	bool shockInfiniteMasses = false;
	size_t solverIterations = 1;
	long long updateInterval = 10000;
	bool verletResolution = true;
}