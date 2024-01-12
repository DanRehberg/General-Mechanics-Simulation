#ifndef __NGON_HPP__
#define __NGON_HPP__

#include <ostream>
#include <vector>
#include <utility>
#include <cstdint>
#include <glm/glm.hpp>
#include "collision.hpp"
#include "particle.hpp"

class NGon final : public Collision
{
public:
	NGon();
	NGon(const glm::vec3& position, const float length, const size_t sides, const float mass);
	NGon(const NGon& cp);
	NGon& operator=(const NGon& cp);

	float getArea(size_t index) const;
	void getColor(float vals[3]) const;
	size_t getID() const;
	void getLine(float vals[6], size_t index) const;
	size_t getN() const;
	float getNeighbor() const;
	glm::vec3 getParticle(size_t index) const;
	float getParticleMass(size_t index) const;
	float getRadius() const;
	float getShortEdge() const;
	const glm::vec3& getVelocity() const;
	void setColor(const glm::vec3& val);
	void update(float dt);

private:
	glm::vec3 color;
	size_t id;
	float invSize;
	std::vector<size_t> lines;
	float neighbor; //distance to a neighboring vertex
	float radius; //distance from a vertex to the center
	float shortEdge; //distance from CoM to hull edge
};

std::ostream& operator<<(std::ostream& output, const NGon& input);

#endif
