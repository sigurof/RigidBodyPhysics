#ifndef COLLISION_H
#define COLLISION_H

enum CollisionType
{
	WALLCOLLISION, PARTICLECOLLISION, UNDEFINED
};

class Collision
{
public:

	Collision(){}

	Collision(const std::vector<Particle*>& ptcls) : t(0) {
		for (unsigned int i = 0; i < ptcls.size(); i++)
		{
			particles.push_back(ptcls[i]);
		}
	}
	Collision(double t) : t(t) { }
	Collision(Particle* p1, Particle* p2, double t) : t(t), type(PARTICLECOLLISION)
	{
		particles.push_back(p1); particles.push_back(p2);
	}

	Collision(Particle* p, Wall* w, double t) : t(t), wall(w), type(WALLCOLLISION)
	{
		particles.push_back(p);
	}

	bool operator<(Collision& rhs) {
		return this->t < rhs.t;
	}

	const Wall* getWall() const {
		return wall;
	}

	const CollisionType getType() const { return type; }

	std::vector<Particle*> particles;
	Wall* wall;
	double t; // global time for this collision

	std::vector<glm::dvec3> newVelocities;
	std::vector<glm::dvec3> newPositions;

	static glm::dvec3 gravity;

	void calculateNewVelocities(double lastCollTime) 
	{
		double Dt = t - lastCollTime;
		glm::dvec3 v0 = glm::dvec3(0, 0, 0);
		glm::dvec3 v1 = glm::dvec3(0, 0, 0);
		double frac, M;
		glm::dvec3 r1_m_r0, v1_m_v0;
		glm::dvec3 r0 = glm::dvec3(0, 0, 0);
		glm::dvec3 r1 = glm::dvec3(0, 0, 0);
		switch (type)
		{
		case WALLCOLLISION:
			r0 = particles[0]->r0 + particles[0]->v0*Dt + particles[0]->gravity*Dt*Dt / 2.0;
			v0 = particles[0]->v0 + gravity*Dt;
			newVelocities.push_back(glm::reflect(v0, wall->unitNormal));
			
			break;
		case PARTICLECOLLISION:
			r1 = particles[1]->r0 + particles[1]->v0*Dt + particles[1]->gravity*Dt*Dt / 2.0;

			v0 = particles[0]->v0 + gravity*Dt;
			v1 = particles[1]->v0 + gravity*Dt;
			
			M = particles[0]->mass + particles[1]->mass;
			r1_m_r0 = r1 - r0;
			v1_m_v0 = v1 - v0;
			frac = glm::dot(v1_m_v0, r1_m_r0) / glm::dot(r1_m_r0, r1_m_r0);
			v0 = (v0 + 2 * particles[1]->mass / M * frac*r1_m_r0);
			v1 = (v1 - 2 * particles[0]->mass / M * frac*r1_m_r0);
			newVelocities.push_back(v0);
			newVelocities.push_back(v1);
			break;
		default:
			std::cout << "In Collision::calculateNewVelocities():\n";
			throw(std::exception("ERROR! Collision type not set.\n"));
			break;
		}
	}

	const std::vector<glm::dvec3> getVelocitiesRightBeforeCollision() const
	{
		double Dt = t - particles[0]->t0;
		glm::dvec3 v0, v1;
		v0 = particles[0]->v0 + gravity*Dt;
		std::vector<glm::dvec3> velocities = { v0 };
		if (type == PARTICLECOLLISION)
		{
			v1 = particles[1]->v0 + gravity*Dt;
			velocities.push_back(v1);
		}
		return velocities;
	}

	CollisionType type;

};

#endif // !COLLISION_H
