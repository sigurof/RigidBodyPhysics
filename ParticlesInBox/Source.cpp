// Source.cpp : Defines the entry point for the console application.
//

//#define DEBUG
#include "stdafx.h"
#include "Wall.h"
#include "Geometry.h"
#include "Box.h"
#include "Particle.h"
#include "Collision.h"
#include "Simulation.h"
#include "Utilities.h"

std::string outputPath = "C:\\Users\\Sigurd\\Documents\\GitHub\\ParticlesInBox\\RigidBody\\ParticlesInBox\\newtwo.pos";

const glm::dvec3 Gravity = glm::dvec3(0, -9.0, 0);
glm::dvec3 Simulation::gravity = Gravity;
glm::dvec3 Collision::gravity = Gravity;
glm::dvec3 Particle::gravity = Gravity;


int main(int argc, char* argv[])
{
	srand((unsigned int)time(NULL));
	try
	{
		double time = 3;
		unsigned int nFrames = 1100;
		unsigned int nParticles = 100;
		double maxvelocity = 5.0;
		bool collisions = true;
#ifdef DEBUG
		std::cout << "Debug mode active\n";
#endif // DEBUG
		Box box = makeCuboidBox(1.0, 2.0, 3.0);
		std::cout << "Making particle ensemble with " << nParticles << " particles\n";
		std::vector<Particle> particles = makeRandomParticleEnsemble(nParticles, box, maxvelocity);
		std::cout << "Successfully made particle ensemble\n";
		std::cout << "Running simulation. time = " << time << ", number of frames = " << nFrames << "\n";

		Simulation particlesInBox(particles, box, time, nFrames, outputPath);
		particlesInBox.setUseMutualInteraction(collisions);
		particlesInBox.runAndSave();
		
		return 0;
	}
	catch (const std::exception& err)
	{
		std::cout << "In main():\n";
		std::cout << err.what() << std::endl;
		return 1;
	}
}

