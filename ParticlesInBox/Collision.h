#ifndef COLLISION_H
#define COLLISION_H

enum CollisionType
{
	WALLCOLLISION, PARTICLECOLLISION, UNDEFINED
};

class Collision
{
public:
	CollisionType type;
	Collision(){}
	//Collision(std::vector<Particle>& ptcls) : time(std::numeric_limits<double>::infinity()) {
	//	for (unsigned int i = 0; i < ptcls.size(); i++)
	//	{
	//		particles.push_back(&ptcls[i]);
	//	}
	//}
	Collision(std::vector<Particle*>& ptcls) : time(std::numeric_limits<double>::infinity()) {
		for (unsigned int i = 0; i < ptcls.size(); i++)
		{
			particles.push_back(ptcls[i]);
		}
	}
	Collision(double time) : time(time) { }
	Collision(Particle* p1, Particle* p2, double time) : time(time), type(PARTICLECOLLISION)
	{
		particles.push_back(p1); particles.push_back(p2);
	}

	Collision(Particle* p, Wall* w, double time) : time(time), wall(w), type(WALLCOLLISION)
	{
		particles.push_back(p);
	}
	//Collision(const Collision& other) {
	//
	//}

	bool operator<(Collision& rhs) {
		return this->time < rhs.time;
	}

	std::vector<Particle*> particles;
	Wall* wall;
	double time;

	std::vector<glm::vec3> newVelocities;
	std::vector<glm::vec3> newPositions;

	void calculateNewVelocities() 
	{
		float frac, M;
		glm::vec3 r2_min_r1, v2_min_v1;
		switch (type)
		{
		case WALLCOLLISION:
			newVelocities.push_back(glm::reflect(particles[0]->velocity, wall->unitNormal));
			break;
		case PARTICLECOLLISION:
			M = particles[0]->mass + particles[1]->mass;
			r2_min_r1 = particles[1]->position - particles[0]->position;
			v2_min_v1 = particles[1]->velocity - particles[0]->velocity;
			frac = glm::dot(v2_min_v1, r2_min_r1) / glm::dot(r2_min_r1, r2_min_r1);
			newVelocities.push_back(particles[0]->velocity + 2 * particles[1]->mass / M*frac*r2_min_r1);
			newVelocities.push_back(particles[1]->velocity - 2 * particles[0]->mass / M*frac*r2_min_r1);
			break;
		default:
			std::cout << "In Collision::calculateNewVelocities():\n";
			throw(std::exception("ERROR! Collision type not set.\n"));
			break;
		}
	}
	void calculateNewPositions() 
	{
		switch (type)
		{
		case WALLCOLLISION:
			/* This should be done correctly at a later stage */
			break;
		case PARTICLECOLLISION:
			/* This should be done correctly at a later stage */
			break;
		default:
			std::cout << "In Collision::calculateNewPositions():\n";
			throw(std::exception("ERROR! Collision type not set.\n"));
			break;
		}
	}

};

#endif // !COLLISION_H
