// Source.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"



const float pi = (float)3.14159265359;

float randf(float low, float high) {
	float random = ((float)rand()) / (float)RAND_MAX;
	return (high - low)*random + low;

}

glm::vec3 makeRandomVelocity() {
	glm::vec3 vel(randf(-2, 2), randf(-2, 2), randf(-2, 2));
	return vel;
}

glm::vec3 makeRandomPosition() {
	glm::vec3 pos(randf(-1, 1), randf(-1, 1), randf(-1, 1));
	return pos;
}


bool posIsInsideBox(glm::vec3 pos, Box& box) {
	/* Assumes convex polyhedron box */
	for (unsigned int i = 0; i < box.walls.size(); i++)
	{
		if (box.walls[i].d - glm::dot(pos, box.walls[i].unitNormal) >= 0)
		{
			return false;
		}
	}
	return true;
}

double calculateVolumeOfBoxMonteCarlo(Box box, unsigned int n) {
	unsigned int nPointsInside = 0;
	double volume = 0;
	glm::vec3 pos(0, 0, 0);
	for (unsigned int i = 0; i < n; i++)
	{
		pos = makeRandomPosition();
		if (posIsInsideBox(pos,box))
		{
			nPointsInside++;
		}
	}
	double fraction = (double)nPointsInside / (double)n;
	volume = fraction * 8.0;
	return volume;
}


Box makeCubicBox(float sidelength)
{
	float a = sidelength / 2;
	std::vector<Wall> walls = {
		/* Negative side of axis    Positive side of axis */
		Wall(glm::vec3(1,0,0),-a), Wall(glm::vec3(-1,0,0),-a),
		Wall(glm::vec3(0,1,0),-a), Wall(glm::vec3(0,-1,0),-a),
		Wall(glm::vec3(0,0,1),-a), Wall(glm::vec3(0,0,-1),-a)
	};
	Box box(walls, glm::vec3(sidelength,sidelength,sidelength));
	return box;
}

//Box makeTetrahedronBox(float sidelength) {
//	float a = sidelength / sqrt(3);
//	glm::vec3 v1 = a * glm::vec3( sqrt(3) / 2, -1 / 2,		-sqrt(3) / 3);
//	glm::vec3 v2 = a * glm::vec3(-sqrt(3) / 2, -1 / 2,		-sqrt(3) / 3);
//	glm::vec3 v3 = a * glm::vec3(			0,		1,		-sqrt(3) / 3);
//	glm::vec3 v4 = a * glm::vec3(			0,		0,	 2 * sqrt(3) / 3);
//
//}

bool particleOverlapsWithExistingParticle(Particle particle, std::vector<Particle> particles) {
	for (unsigned int i = 0; i < particles.size(); i++)
	{
		glm::vec3 dist = particle.position - particles[i].position;
		if (glm::length(dist) <= (particle.radius + particles[i].radius))
		{
			return true;
		}
	}
	return false;
}

glm::vec3 makeRandomPositionWithinBox(Box box) {
	glm::vec3 pos(0, 0, 0);
	bool ok = false;
	while (!ok)
	{
		pos = makeRandomPosition();
		if (posIsInsideBox(pos, box))
		{
			ok = true;
		}
	}
	return pos;
}

void addParticleToEnsemble(std::vector<Particle>* particles, Box box, unsigned int index) {
	Particle newParticle;
	glm::vec3 vel = makeRandomVelocity();
	float radmax = 1.0f / 10.0f;
	float radmin = 1.0f / 40.0f;
	float radius = randf(radmin, radmax);
	float mass = radius*radius*radius;
	bool ok = false;
	while (!ok)
	{
		glm::vec3 pos = makeRandomPositionWithinBox(box);
		newParticle = Particle(pos, vel, radius, mass, index);
		if (!particleOverlapsWithExistingParticle(newParticle,*particles))
		{
			ok = true;
		}
	}
	particles->push_back(newParticle);
}

std::vector<Particle> makeRandomParticleEnsemble(unsigned int N, Box box) {
	std::vector<Particle> particles;
	for (unsigned int i = 0; i < N; i++)
	{
		addParticleToEnsemble(&particles, box, i);
	}
	return particles;
}

std::vector<Particle> makeRandomParticleEnsemble2(unsigned int N, Box box) {
	bool ok = false;
	float dh = -box.walls[0].d;
	float radius = dh/20;
	float spherevol = 4.0f * 3.1415926536f / 3.0f * radius*radius*radius;
	float mass = spherevol;
	float boxvol = 8 * dh*dh*dh;
	float packfrac = 3.1415926536f / (3.0f * sqrt(2.0f));
	if (spherevol*N > 0.5*packfrac*boxvol)
	{
		throw(std::exception("Problem! You are trying to create too many particles. They might not fit into this volume!"));
	}
	std::vector<Particle> particles;
	int j = 0;
	for (unsigned int i = 0; i < N; i++)
	{
		Particle newParticle;
		do
		{
			j++;
			ok = false;
			newParticle = Particle(glm::vec3(randf(-dh, dh), randf(-dh, dh), randf(-dh, dh)) , glm::vec3(randf(-dh, dh), randf(-dh, dh), randf(-dh, dh)), radius, mass, i);
			if (posIsInsideBox(newParticle.position, box) && (!particleOverlapsWithExistingParticle(newParticle, particles)))
			{
				ok = true;
			}
			if (j % 1000 == 0)
			{
				std::cout << "i = " << i << "\n";
			}
		} while (!ok);
		particles.push_back(newParticle);
	}
	return particles;
}

std::vector<Particle> make1234or5Particles(unsigned int N) {
	std::vector<Particle> particles, part;
	Particle p1(glm::vec3(0, 0, 0), glm::vec3(0, 0, 0.1), 0.05f, 0.05f, 0);
	Particle p2(glm::vec3(0.25, 0.25, 0.25), glm::vec3(0, 0.51, 0), 0.05f, 0.05f, 0);
	Particle p3(glm::vec3(0.25, -0.25, 0.25), glm::vec3(0, 0.51, -0.21), 0.05f, 0.05f, 0);
	Particle p4(glm::vec3(0.25, 0.25, -0.25), glm::vec3(0.3, 0.51, 0), 0.05f, 0.05f, 0);
	Particle p5(glm::vec3(-0.25, -0.25, 0.25), glm::vec3(0, -0.51, 0), 0.05f, 0.05f, 0);
	part = std::vector<Particle>{ p1, p2, p3, p4, p5 };
	for (unsigned int i = 0; i < N; i++)
	{
		particles.push_back(part[i]);
	}
	return particles;
}

std::vector<Particle> makeCollidingParticles() {
	std::vector<Particle> particles;
	Particle p1(glm::vec3(-0.2, 0.001, 0), glm::vec3( 1, 0, 0), 0.05f, 0.05f, 0);
	Particle p2(glm::vec3( 0.2, 0, 0), glm::vec3(-1, 0, 0), 0.05f, 0.05f, 1);
	particles.push_back(p1); particles.push_back(p2);
	return particles;
}

void addParticleOfGivenRadiusToEnsemble(std::vector<Particle>* particles, Box box, unsigned int index, float radius) {
	Particle newParticle;
	glm::vec3 vel = makeRandomVelocity();
	float mass = radius * radius*radius;
	bool ok = false;
	while (!ok)
	{
		glm::vec3 pos = makeRandomPositionWithinBox(box);
		newParticle = Particle(pos, vel, radius, mass, index);
		if (!particleOverlapsWithExistingParticle(newParticle, *particles))
		{
			ok = true;
		}
	}
	particles->push_back(newParticle);
}

std::vector<Particle> makeManySmallOneLarge(unsigned int nSmall, Box box) {
	std::vector<Particle> particles;
	float smallRadius = 1.0f / 20.0f;
	float largeRadius = 1.0f / 3.0f;
	addParticleOfGivenRadiusToEnsemble(&particles, box, 0, largeRadius);
	//particles[0].velocity = glm::vec3(1, 0, 0);
	for (unsigned int i = 1; i < nSmall+1; i++)
	{
		addParticleOfGivenRadiusToEnsemble(&particles, box, i, smallRadius);
	}
	return particles;
}

std::vector<Particle> allParticlesInSameDirection(unsigned int N, Box box) {
	std::vector<Particle> particles = makeRandomParticleEnsemble(N, box);
	for (unsigned int i = 0; i < N; i++)
	{
		particles[i].velocity = glm::vec3(0, 5, 0);
	}
	return particles;
}

//glm::vec3& operator*(float lhs, glm::vec3& rhs) {
//	rhs.x *= lhs;
//	rhs.y *= lhs;
//	rhs.z *= lhs;
//	return rhs;
//}

std::vector<Particle> pool3D(unsigned int N, Box box) {
	std::vector<Particle> particles;
	for (unsigned int i = 0; i < N; i++)
	{
		addParticleOfGivenRadiusToEnsemble(&particles, makeCubicBox(0.5), i, 1.0f/20.0f);
		particles[i].velocity = glm::vec3(0, 0, 0);
	}
	addParticleOfGivenRadiusToEnsemble(&particles, box, N, 1.0f/20.0f);
	particles[particles.size() - 1].velocity = -particles[particles.size() - 1].position;
	particles[particles.size() - 1].velocity.x *= 20;
	particles[particles.size() - 1].velocity.y *= 20;
	particles[particles.size() - 1].velocity.z *= 20;
	return particles;
}


int main()
{
	//std::string a = std::string("a");
	try
	{
		srand((unsigned int)time(NULL));
		Box box = makeCubicBox(2);
		std::cout << "Making particle ensemble:\n";
		std::vector<Particle> particles = pool3D(10, box);
		std::cout << "Successfully made particle ensemble\n";
		double time = 15;
		Simulation particlesInBox(particles, box, time);
		//particlesInBox.run();
		particlesInBox.runAndSave(time, 3000, std::string("newtwo.pos"));
	}
	catch (const std::exception& err)
	{
		std::cout << "In main():\n";
		std::cout << err.what() << std::endl;
	}

}

