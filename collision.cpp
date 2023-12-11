#include "collision.hpp"
#include "ngon.hpp"
#include "distanceConstraint.hpp"
#include "linePointConstraint.hpp"
#include <algorithm>

/*
11/26/2023 Quick Notes:

The manner in which this system is used could be meaningful to generating an alternative approach to
	position-based dynamics over sparse shape structures.
	Notably, the stack of positions filling a volume is meaningful in the sense that it does correctly
		build out interactions with large coupled bodies by unifying everything under distance constraints.
		However, it can be too many particles for some systems with soft/hard time constraints.
			E.g., small compute budget for offline, or real-time constraints.
	So, what about generating new contact points dynamically along features that are greater than 0D surfaces?
		Keeping a record of these higher-dimensional surfaces could be meaningful to creating the correct
			projection constraints of surfaces as well.
			I.e., in 2D, a point under a line (violating distance to the line) could generate a temporary
				new edge segment between the endpoints of the line with its own distance constraint to the CoM
				of the object it is attached.

*/

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
	return false;
}

void Collision::simulatePair(NGon& A, NGon& B)
{
	Collision* a = &A;
	Collision* b = &B;

	glm::vec3& oA(a->center.position);
	glm::vec3& oB(b->center.position);

	std::vector<Particle>& vertA(a->vertices);
	std::vector<Particle>& vertB(b->vertices);

	//Validating software questions through the input space partition
	//	- General overview is can a system converge to a stable solution faster given the order of constraints
	//		Shock propagation says yes IFF the final pass of the constraints is ordered from the closest point of gravity outwards
	//		-- This is an assumption that in an impulse-based solve (Brian Mirtich) the persistant acceleration influence is gravity
	//	- Position-based dynamics is an appealing approach to solve rigid body problems because it unifies all simulation into distance cosntraints
	//	- PBD has shown that Guendelmann et al. appraoch of shock propagation holds
	//	? However, building a physics simulation for a given system requires calibration ?
	//		Assuming someone chose to implement a PBD simulation given its nice simplifications


	std::vector<LinePointConstraint> lineError;
	std::vector<DistanceConstraint> selfError;

	if (aabb(A.getMin(), A.getMax(), B.getMin(), B.getMax()))
	{
		for (size_t i = 0; i < B.getN(); ++i)
		{
			glm::vec3 b0 = vertB[i].getPosition(), b1;
			size_t index1 = i + 1;
			bool intersection = false;
			if (i == B.getN() - 1)
			{
				b1 = vertB[0].getPosition();
				index1 = 0;
			}
			else b1 = vertB[i + 1].getPosition();

			for (size_t j = 0; j < A.getN(); ++j)
			{
				if (pointTriangle2D(b0, b1, oB, vertA[j].getPosition()))
				{
					lineError.push_back(LinePointConstraint(vertA[j], vertB[i], vertB[index1], b->center));
				}
			}
		}
	}

	for (size_t i = 0; i < A.getN(); ++i)
	{
		size_t nextIndex = (i == A.getN() - 1) ? 0 : i + 1;
		selfError.push_back(DistanceConstraint(vertA[i], a->center, A.getRadius()));
		selfError.push_back(DistanceConstraint(vertA[i], vertA[nextIndex], A.getNeighbor()));
	}
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
	
}