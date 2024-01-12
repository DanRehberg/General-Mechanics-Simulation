#include "collision.hpp"
#include "ngon.hpp"
#include "distanceConstraint.hpp"
#include "linePointConstraint.hpp"
#include <algorithm>

size_t objectID = 0;

Collision::Collision() : min(0.f), max(0.f)
{

}

Collision::Collision(const glm::vec3& position) : min(position), max(position)
{
	center.position = position;
}

Collision::Collision(const glm::vec3& m0, const glm::vec3& m1) : min(min), max(max)
{
	glm::vec3 testMax = glm::max(min, max);
	glm::vec3 testMin = glm::min(min, max);

	this->max = testMax;
	this->min = testMin;

	center.position = 0.5f * (max + min);
}

Collision::Collision(const Collision& cp)
{
	min = cp.min;
	max = cp.max;
}

Collision::~Collision()
{

}

Collision& Collision::operator=(const Collision& cp)
{
	min = cp.min;
	max = cp.max;
	return *this;
}

const glm::vec3& Collision::getMax() const
{
	return max;
}

const glm::vec3& Collision::getMin() const
{
	return min;
}

const glm::vec3& Collision::getPosition() const
{
	return center.position;
}

bool Collision::aabb(const Collision& a, const Collision& b)
{
	return aabb(a.getMin(), a.getMax(), b.getMin(), b.getMax());
}

bool Collision::aabb(const glm::vec3& minA, const glm::vec3& maxA, const glm::vec3& minB, const glm::vec3& maxB)
{
	if (minA.x <= maxB.x && minB.x <= maxA.x)
	{
		if (minA.y <= maxB.y && minB.y <= maxA.y)
		{
			if (minA.z <= maxB.z && minB.z <= maxA.z)
			{
				return true;
			}
		}
	}
	return false;
}

//Christer Ericson
glm::vec3 Collision::barycentricWeights(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& s)
{
	glm::vec3 AB = b - a;
	glm::vec3 AC = c - a;
	glm::vec3 AS = s - a;

	float abab = glm::dot(AB, AB);
	float abac = glm::dot(AB, AC);
	float acac = glm::dot(AC, AC);
	float asab = glm::dot(AS, AB);
	float asac = glm::dot(AS, AC);

	float divisor = 1.0f / (abab * acac - abac * abac);

	float v = divisor * (acac * asab - abac * asac);
	float w = divisor * (abab * asac - abac * asab);
	float u = 1.0f - v - w;

	return glm::vec3(u, v, w);
}

bool Collision::pointTriangle2D(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& test)
{
	glm::vec3 bary = barycentricWeights(a, b, c, test);
	if (bary.x >= 0.0f && bary.y >= 0.0f && bary.z >= 0.0f)
	{
		return true;
	}
	//Below causes a cohesion effect with some added constraints for adjacent corners of objects
	//	Goal was to correct corner case of vertex hull edge overlap
	/*glm::vec3 BA = b - a, TA = test - a, TB = test - b;
	float mBA = glm::length(BA), mTA = glm::length(TA), mTB = glm::length(TB);
	if (mTA < 2.75f || mTB < 2.75f)return true;*/
	return false;
}

std::vector<LinePointConstraint> lineError;
std::vector<DistanceConstraint> selfError;

void Collision::selfConstraints(NGon& obj)
{
	Collision* o = &obj;
	std::vector<Particle>& vO(o->vertices);
	if (obj.getParticleMass(0) != 0.0f)
	{
		for (size_t i = 0; i < obj.getN(); ++i)
		{
			size_t nextIndex = (i == obj.getN() - 1) ? 0 : i + 1;
			selfError.push_back(DistanceConstraint(vO[i], o->center, obj.getRadius()));
			selfError.push_back(DistanceConstraint(vO[i], vO[nextIndex], obj.getNeighbor()));
		}
	}
}

void Collision::simulatePair(NGon& A, NGon& B)
{
	Collision* a = &A;
	Collision* b = &B;

	glm::vec3& oA(a->center.position);
	glm::vec3& oB(b->center.position);

	std::vector<Particle>& vertA(a->vertices);
	std::vector<Particle>& vertB(b->vertices);

	if (aabb(A.getMin(), A.getMax(), B.getMin(), B.getMax()))
	{
		//CoM case A
		//	provide distance constraint between CoMs IFF they are within there shortest collision distance (distance from CoM to hull edge)
		glm::vec3 rv = a->center.getVelocity() - b->center.getVelocity();
		float rvn = glm::dot(rv, glm::normalize(oA - oB));
		if (rvn < 0.0f)
		{
			if (glm::length(oA - oB) <= A.getShortEdge() + B.getShortEdge())
			{
				selfError.push_back(DistanceConstraint(a->center, b->center, A.getShortEdge() + B.getShortEdge()));
			}
		}

		//Hull vertice collision tests (effectively, is hull inside NGon "pie" triangle)
		for (size_t i = 0; i < B.getN(); ++i)
		{
			glm::vec3 b0 = vertB[i].getPosition(), b1;
			size_t index1 = i + 1;
			if (i == B.getN() - 1)
			{
				b1 = vertB[0].getPosition();
				index1 = 0;
			}
			else b1 = vertB[i + 1].getPosition();

			//CoM case B
			//	provide a line distance constraint for a CoM to an edge segment (fails to work with 3 and 4 gons)
			/*if (pointTriangle2D(b0, b1, oB, oA))
			{
				lineError.push_back(LinePointConstraint(a->center, vertB[i], vertB[index1], b->center));
			}*/

			//Hull vertices
			for (size_t j = 0; j < A.getN(); ++j)
			{
				//TEST BEGIN
				//	Expanding object B's hull
				/*glm::vec3 offsetB0 = b0 + 0.5f * glm::normalize(b0 - oB);
				glm::vec3 offsetB1 = b1 + 0.5f * glm::normalize(b1 - oB);
				if (pointTriangle2D(offsetB0, offsetB1, oB, vertA[j].getPosition()))*/
				//TEST END
				if (pointTriangle2D(b0, b1, oB, vertA[j].getPosition()))
				{
					lineError.push_back(LinePointConstraint(vertA[j], vertB[i], vertB[index1], b->center));
				}
			}
		}
		for (size_t i = 0; i < A.getN(); ++i)
		{
			glm::vec3 a0 = vertA[i].getPosition(), a1;
			size_t index1 = i + 1;
			if (i == A.getN() - 1)
			{
				a1 = vertA[0].getPosition();
				index1 = 0;
			}
			else a1 = vertA[i + 1].getPosition();

			//CoM case B
			//	provide a line distance constraint for a CoM to an edge segment (fails to work with 3 and 4 gons)
			/*if (pointTriangle2D(a0, a1, oA, oB))
			{
				lineError.push_back(LinePointConstraint(b->center, vertA[i], vertA[index1], a->center));
			}*/

			//Hull vertices
			for (size_t j = 0; j < B.getN(); ++j)
			{
				//TEST BEGIN
				//	Expanding object A's hull
				/*glm::vec3 offsetA0 = a0 + 0.5f * glm::normalize(a1 - a0);//2.5f * glm::normalize(a0 - oA);
				glm::vec3 offsetA1 = a1 + 0.5f * glm::normalize(a0 - a1);//2.5f * glm::normalize(a1 - oA);
				if (pointTriangle2D(offsetA0, offsetA1, oA, vertB[j].getPosition()))*/
				//TEST END
				if (pointTriangle2D(a0, a1, oA, vertB[j].getPosition()))
				{
					lineError.push_back(LinePointConstraint(vertB[j], vertA[i], vertA[index1], a->center));
				}
			}
		}
	}

}

void Collision::solveConstraints()
{
	if (Simulation::shockpropagation)
	{
		std::vector<Constraint*> c;
		c.resize(lineError.size() + selfError.size());
		std::vector<size_t> indices;
		indices.resize(c.size());
		for (size_t i = 0; i < lineError.size(); ++i)
		{
			c[i] = &lineError[i];
			indices[i] = i;
		}
		for (size_t i = lineError.size(); i < (lineError.size() + selfError.size()); ++i)
		{
			c[i] = &selfError[i - lineError.size()];
			indices[i] = i;
		}
		//initial iterations
		for (size_t i = 0; i < Simulation::solverIterations; ++i)
		{
			for (size_t j = 0; j < c.size(); ++j)
				c[j]->solve();
		}
		//ordered for last iteration
		std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
			return c[a]->maximumDirection(Simulation::shockDirection) > c[b]->maximumDirection(Simulation::shockDirection);
			});
		for (size_t i = 0; i < indices.size(); ++i)
		{
			if (!Simulation::shockInfiniteMasses) c[indices[i]]->solve();
			else
			{
				if (Simulation::shockDirectionAlternate)
					c[indices[i]]->solveShock(-Simulation::shockDirection);
				else
					c[indices[i]]->solveShock(Simulation::shockDirection);
			}
		}
	}
	else
	{
		for (size_t i = 0; i < Simulation::solverIterations; ++i)
		{
			for (size_t j = 0; j < lineError.size(); ++j)
			{
				lineError[j].solve();
			}
			for (size_t j = 0; j < selfError.size(); ++j)
			{
				selfError[j].solve();
			}
		}
	}

	lineError.clear();
	selfError.clear();
}
