#ifndef PARTICLE_H
#define PARTICLE_H

//enum EntityType
//{
//	PARTICLE, WALL
//};

class Particle
{
public:
	//EntityType type;
	Particle(){}
	Particle(const Particle& other) : Particle(other.position, other.velocity, other.radius, other.mass, other.index) {}
	Particle(glm::vec3 pos, glm::vec3 vel, float rad, float m, unsigned int index) : position(pos), velocity(vel), radius(rad), mass(m), index(index) {}

	glm::vec3 position;
	glm::vec3 velocity;
	float radius;
	float mass;

	unsigned int index;

};

#endif // !PARTICLE_H
