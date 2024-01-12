#include <stdexcept>
#include <cmath>
#include "ngon.hpp"

NGon::NGon() : Collision(glm::vec3(0.0f))
{

}

NGon::NGon(const glm::vec3& position, const float length, const size_t sides, const float mass) : Collision(position), color(glm::vec3(1.0f)), invSize(1.0f / static_cast<float>(sides))
{
	if (sides <= 2 || sides > 100) throw std::domain_error::domain_error("NGon must be less then 100 sided and greater than 2 sided.");
	lines.resize(sides);
	vertices.resize(sides);

	id = objectID++;

	float r = length;
	glm::vec3 radius(-r, -r, 0.0f);
	float rads = 0;
	float radIncrements = (2.0f * 3.14159265f) / static_cast<float>(sides);
	min = glm::vec3(999999.9f);
	max = glm::vec3(-999999.9f);
	glm::vec3 centroid(0.0f);
	for (size_t i = 0; i < sides; ++i)
	{
		vertices[i].id = id;
		float cosRad = std::cosf(rads), sinRad = std::sinf(rads);
		float x = -r, y = -r;
		radius.x = x * cosRad - y * sinRad;
		radius.y = x * sinRad + y * cosRad;
		vertices[i] = Particle(mass, radius + position);
		centroid += vertices[i].getPosition();
		rads += radIncrements;
		lines[i] = (i + 1) % sides;
		min = glm::min(min, vertices[i].getPosition());
		max = glm::max(max, vertices[i].getPosition());
	}
	min.z = 0.0f;
	max.z = 0.0f;
	//center = Particle(mass, centroid / static_cast<float>(vertices.size()));
	center = Particle(mass, 0.5f * (min + max));
	center.id = id;
	this->radius = std::sqrtf(length * length * 2);
	glm::vec3 hullEdgeNorm = glm::normalize(glm::cross(vertices[0].getPosition() - vertices[1].getPosition(), glm::vec3(0.0f, 0.0f, 1.0f)));
	shortEdge = glm::dot(hullEdgeNorm, center.getPosition() - vertices[0].getPosition());
	if (shortEdge < 0.0f) shortEdge = -shortEdge;
	neighbor = glm::length(vertices[0].getPosition() - vertices[1].getPosition());
}

NGon::NGon(const NGon& cp)
{
	color = cp.color;
	id = cp.id;
	invSize = cp.invSize;
	lines = cp.lines;
	neighbor = cp.neighbor;
	radius = cp.radius;
	shortEdge = cp.shortEdge;
	center = cp.center;
	vertices = cp.vertices;
	min = cp.min;
	max = cp.max;
}

NGon& NGon::operator=(const NGon& cp)
{
	color = cp.color;
	id = cp.id;
	invSize = cp.invSize;
	lines = cp.lines;
	neighbor = cp.neighbor;
	radius = cp.radius;
	shortEdge = cp.shortEdge;
	center = cp.center;
	vertices = cp.vertices;
	min = cp.min;
	max = cp.max;
	return *this;
}

float NGon::getArea(size_t index) const
{
	if (index >= getN()) throw std::range_error::range_error("getArea called with an out of range index.");
	glm::vec3 edge0 = (index == getN() - 1) ? getParticle(0) - getParticle(index) : getParticle(index + 1) - getParticle(index);
	glm::vec3 edge1 = getPosition() - getParticle(index);

	return glm::length(glm::cross(edge0, edge1)) * 0.5f;
}

void NGon::getColor(float vals[3]) const
{
	vals[0] = color.x;
	vals[1] = color.y;
	vals[2] = color.z;
}

size_t NGon::getID() const
{
	return id;
}

void NGon::getLine(float vals[6], size_t index) const
{
	glm::vec3 vert = getParticle(index);
	vals[0] = vert.x;
	vals[1] = vert.y;
	vals[2] = vert.z;
	vert = getParticle(lines[index]);
	vals[3] = vert.x;
	vals[4] = vert.y;
	vals[5] = vert.z;
}

size_t NGon::getN() const
{
	return vertices.size();
}

float NGon::getNeighbor() const
{
	return neighbor;
}

glm::vec3 NGon::getParticle(size_t index) const
{
	if (index >= getN()) throw std::range_error::range_error("getParticle called with an out of range index.");
	return vertices[index].getPosition();
}

float NGon::getParticleMass(size_t index) const
{
	if (index >= getN()) throw std::range_error::range_error("getParticleMass called with an out of range index.");
	return vertices[index].getMass();
}

float NGon::getRadius() const
{
	return radius;
}

float NGon::getShortEdge() const
{
	return shortEdge;
}

const glm::vec3& NGon::getVelocity() const
{
	return center.getVelocity();
}

void NGon::setColor(const glm::vec3& val)
{
	color = val;
}

void NGon::update(float dt)
{
	min = vertices[0].getPosition();
	max = min;
	float minX = 999.f, minY = 999.f, maxX = -9999.f, maxY = -9999.f;
	glm::vec3 accel(0.0f, Simulation::gravity, 0.0f);
	for (size_t i = 0; i < vertices.size(); ++i)
	{
		vertices[i].integrateAcceleration(dt, accel);
		vertices[i].integrateVelocity(dt);
		min = glm::min(min, vertices[i].getPosition());
		max = glm::max(max, vertices[i].getPosition());
	}
	center.integrateAcceleration(dt, accel);
	center.integrateVelocity(dt);
}

std::ostream& operator<<(std::ostream& output, const NGon& input)
{
	for (size_t i = 0; i < input.getN(); ++i)
	{
		glm::vec3 s = input.getParticle(i);
		output << s.x << ", " << s.y << ", " << s.z << ", ";
	}
	return output;
}
