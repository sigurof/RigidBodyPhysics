#ifndef PARTICLE_H
#define PARTICLE_H


class Particle
{
public:
	Particle(){}

	Particle(const Particle& other) : 
		Particle(other.r0, other.v0, other.t0, other.radius, other.mass, other.index) 
	{
		
	}
	
	Particle(glm::dvec3 pos, glm::dvec3 vel, double t, double rad, double m, unsigned int index) : 
		r0(pos), v0(vel), t0(t), radius(rad), mass(m), index(index)
	{
		position = pos;
	}

	static glm::dvec3 gravity;

	void update_r0(double t)
	{
		double Dt = t - t0; //equals time interval between this particle's last collision and now
		r0 += (v0 * Dt + gravity * Dt*Dt / 2.0);
	}

	void update_v0(double t) 
	{
		double Dt = t - t0; //equals time interval between this particle's last collision and now
		v0 += gravity * Dt;
	}

	void update_v0(const glm::dvec3& vel)
	{
		/* Calculating the new v0 requires info on wall collision vs ptcl collision, so the caluclation is currently performed in the Collision class*/
		this->v0 = vel;
	}

	void update_r0(const glm::dvec3& pos) {
		this->r0 = pos;
	}

	void update_t0(double t) {
		t0 = t;
	}

	glm::dvec3 r0; // position at last collision
	glm::dvec3 v0; // velocity directly after last collision
	double t0; // global time at last collision

	glm::dvec3 position; // instantaneous position
	//glm::dvec3 velocity; // instantaneous velocity
	
	double radius;
	double mass;

	unsigned int index; 

};

#endif // !PARTICLE_H
