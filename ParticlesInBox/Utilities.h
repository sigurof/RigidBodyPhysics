#ifndef UTILITIES_H
#define UTILITIES_H


double calculateSquaredNorm(glm::dvec3 vec) {
	return vec.x*vec.x + vec.y*vec.y + vec.z*vec.z;
}

template <class T>
T smallestPositiveArgument(T arg0, T arg1) {
	T smallestArgument = (arg0 < arg1)*arg0 + (arg1 < arg0)*arg1;
	if (smallestArgument > 0)
	{
		return smallestArgument;
	}
	return std::numeric_limits<T>::infinity();
}


const double pi = (double)3.14159265359;

double randf(double low, double high) {
	double random = ((double)rand()) / (double)RAND_MAX;
	return (high - low)*random + low;

}

const glm::dvec3 makeRandomVelocity(double velmax) {
	glm::dvec3 vel(randf(-velmax, velmax), randf(-velmax, velmax), randf(-velmax, velmax));
	return vel;
}

const glm::dvec3 makeRandomPosition() {
	glm::dvec3 pos(randf(-1, 1), randf(-1, 1), randf(-1, 1));
	return pos;
}

const Particle makeRandomParticle(unsigned int index, double velmax) {
	glm::dvec3 pos = makeRandomPosition();
	glm::dvec3 vel = makeRandomVelocity(velmax);
	double radmax = 1.0 / 10.0;
	double radmin = 1.0 / 40.0;
	double radius = randf(radmin, radmax);
	double mass = radius * radius*radius;
	Particle newParticle(pos, vel, 0.0, radius, mass, index);
	return newParticle;
}


bool posIsInsideBox(const glm::dvec3& pos, const Box& box) {
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
	glm::dvec3 pos(0, 0, 0);
	for (unsigned int i = 0; i < n; i++)
	{
		pos = makeRandomPosition();
		if (posIsInsideBox(pos, box))
		{
			nPointsInside++;
		}
	}
	double fraction = (double)nPointsInside / (double)n;
	volume = fraction * 8.0;
	return volume;
}

Box makeCuboidBox(double a, double b, double c) {
	double d1 = a / 2.0;
	double d2 = b / 2.0;
	double d3 = c / 2.0;
	std::vector<Wall> walls = {
		/* Negative side of axis    Positive side of axis */
		Wall(glm::dvec3(1,0,0), d1), Wall(glm::dvec3(-1,0,0), d1),
		Wall(glm::dvec3(0,1,0), d2), Wall(glm::dvec3(0,-1,0), d2),
		Wall(glm::dvec3(0,0,1), d3), Wall(glm::dvec3(0,0,-1), d3)
	};
	Box box(walls, glm::dvec3(a, b, c));
	return box;
}

Box makeCubicBox(double sidelength)
{
	double a = sidelength / 2;
	std::vector<Wall> walls = {
		/* Negative side of axis    Positive side of axis */
		Wall(glm::dvec3(1,0,0), a), Wall(glm::dvec3(-1,0,0), a),
		Wall(glm::dvec3(0,1,0), a), Wall(glm::dvec3(0,-1,0), a),
		Wall(glm::dvec3(0,0,1), a), Wall(glm::dvec3(0,0,-1), a)
	};
	Box box(walls, glm::dvec3(sidelength, sidelength, sidelength));
	return box;
}


bool particleOverlapsWithExistingParticle(const Particle& particle, const std::vector<Particle>& particles) {
	for (unsigned int i = 0; i < particles.size(); i++)
	{
		glm::dvec3 dist = particle.position - particles[i].position;
		if (glm::length(dist) <= (particle.radius + particles[i].radius))
		{
			return true;
		}
	}
	return false;
}



const glm::dvec3 makeRandomPositionWithinBox(const Box& box) {
	glm::dvec3 pos(0, 0, 0);
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


void addRandomParticleOfGivenRadiusWithinBoxToEnsemble(std::vector<Particle>& particles, const Box& box, unsigned int index, double radius, double velmax)
{
	Particle newParticle;
	bool ok = false;
	while (!ok)
	{
		newParticle = makeRandomParticle(index, velmax); newParticle.radius = radius;
		if (!particleOverlapsWithExistingParticle(newParticle, particles) && particleIsInsideBox(newParticle, box))
		{
			ok = true;
		}
	}
	particles.push_back(newParticle);
}


void addRandomParticleWithinBoxToEnsemble(std::vector<Particle>& particles, const Box& box, unsigned int index, double velmax)
{
	Particle newParticle;
	bool ok = false;
	while (!ok)
	{
		newParticle = makeRandomParticle(index, velmax);
		if (!particleOverlapsWithExistingParticle(newParticle, particles) && particleIsInsideBox(newParticle, box))
		{
			ok = true;
		}
	}
	particles.push_back(newParticle);
}

const std::vector<Particle> makeRandomParticleEnsemble(unsigned int N, const Box& box, double velmax) {
	std::vector<Particle> particles;
	for (unsigned int i = 0; i < N; i++)
	{
		addRandomParticleWithinBoxToEnsemble(particles, box, i, velmax);
	}
	return particles;
}

void addParticleOfGivenRadiusToEnsemble(std::vector<Particle>* particles, Box box, unsigned int index, double radius, double velmax) {
	Particle newParticle;
	glm::dvec3 vel = makeRandomVelocity(velmax);
	double mass = radius * radius*radius;
	bool ok = false;
	while (!ok)
	{
		glm::dvec3 pos = makeRandomPositionWithinBox(box);
		newParticle = Particle(pos, vel, 0.0, radius, mass, index);
		if (!particleOverlapsWithExistingParticle(newParticle, *particles) && particleIsInsideBox(newParticle, box))
		{
			ok = true;
		}
	}
	particles->push_back(newParticle);
}

const std::vector<Particle> makeManySmallOneLarge(unsigned int nSmall, const Box& box, double velmax)
{
	std::vector<Particle> particles;
	addParticleOfGivenRadiusToEnsemble(&particles, box, 0, 1.0 / 3.0, velmax);
	for (unsigned int i = 1; i < nSmall + 1; i++)
	{
		addParticleOfGivenRadiusToEnsemble(&particles, makeCubicBox(1.0), i, 1.0 / 20.0, velmax);
		particles[i].v0 = glm::dvec3(0, 0, 0);
	}
	particles[0].v0 = -particles[0].position;
	particles[0].v0.x *= 20;
	particles[0].v0.y *= 20;
	particles[0].v0.z *= 20;
	return particles;
}

const std::vector<Particle> allParticlesInSameDirection(unsigned int N, const Box& box, double velmax)
{
	std::vector<Particle> particles = makeRandomParticleEnsemble(N, box, velmax);
	for (unsigned int i = 0; i < N; i++)
	{
		particles[i].v0 = glm::dvec3(0, 5, 0);
	}
	return particles;
}

const std::vector<Particle> pool3D(unsigned int N, Box box, double velmax) {
	std::vector<Particle> particles;
	for (unsigned int i = 0; i < N; i++)
	{
		addParticleOfGivenRadiusToEnsemble(&particles, makeCubicBox(1.0), i, 1.0 / 20.0, velmax);
		particles[i].v0 = glm::dvec3(0, 0, 0);
	}
	addParticleOfGivenRadiusToEnsemble(&particles, box, N, 1.0 / 20.0, velmax);
	particles[particles.size() - 1].v0 = -particles[particles.size() - 1].position;
	particles[particles.size() - 1].v0.x *= 20;
	particles[particles.size() - 1].v0.y *= 20;
	particles[particles.size() - 1].v0.z *= 20;
	return particles;
}

void readInput(int argc, char* argv[]) {

	glm::dvec3 vel(0, 0, 0);
	std::cout << "argc = " << argc << "\n";

	if (argc > 1)
	{
		double x;
		for (int i = 0; i < argc - 1; i++)
		{
			std::istringstream ss(argv[i + 1]);
			if (ss >> x)
			{
				vel[i] = x;
				std::cout << "arg" << i + 1 << " = " << x << "\n";
			}
			else
			{
				throw(std::exception("ERROR! Could not parse command line arguments\n"));
			}
		}
	}

}

#endif // !UTILITIES_H
