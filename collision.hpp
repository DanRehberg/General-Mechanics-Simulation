#ifndef __COLLISION_HPP__
#define __COLLISION_HPP__

#include <vector>
#include <glm/glm.hpp>
#include "particle.hpp"

class NGon;

class Collision
{
public:
	Collision();
	Collision(const glm::vec3& position);
	Collision(const glm::vec3& min, const glm::vec3& max);
	Collision(const Collision& cp);
	virtual ~Collision();
	Collision& operator=(const Collision& cp);
	const glm::vec3& getMax() const;
	const glm::vec3& getMin() const;
	const glm::vec3& getPosition() const;

	static bool aabb(const Collision& a, const Collision& b);
	static bool aabb(const glm::vec3& minA, const glm::vec3& maxA, const glm::vec3& minB, const glm::vec3& maxB);
	static glm::vec3 barycentricWeights(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& test);
	static bool pointTriangle2D(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& test);
	static void simulatePair(NGon& incident, NGon& reference);

	virtual void update(float dt) = 0;

protected:
	Particle center; // intended as center of mass
	glm::vec3 min, max;
	std::vector<Particle> vertices; // shape or mesh structure

};

#endif