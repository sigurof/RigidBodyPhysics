#ifndef SIMULATION_H
#define SIMULATION_H
#include <algorithm>
//
//std::ostream& operator<<(std::ostream& os, const glm::vec3 vec) {
//	os << vec.x << "\t" << vec.y << "\t" << vec.z << "\t";
//	return os;
//}

//std::ostream& operator<<(std::ostream& os, const glm::vec3& vec) {
//	os << vec.x << "\t" << vec.y << "\t" << vec.z << "\t";
//	return os;
//}

class Simulation
{
public:
	Simulation(std::vector<Particle> ptcls, Box box, double time) : T(time), t(0), dt(0), box(box) {
		candidateCollisions = std::vector<std::vector<Collision>>
																(ptcls.size(), std::vector<Collision>
																										(ptcls.size() + box.walls.size(), Collision(
																																						std::numeric_limits<double>::infinity())
																																																	)
																																																		);
		for (unsigned int i = 0; i < ptcls.size(); i++)
		{
			particles.push_back(new Particle(ptcls[i]));
		}
		lastcollision = Collision(particles);//A collision object with all particles as members.
		nextcollision = findNextCollisionForSystem(lastcollision);
	}

	~Simulation() {
		for (unsigned int i = 0; i < particles.size(); i++)
		{
			delete particles[i];
		}
	}

	void run() {
		try
		{
			while (t < T)
			{
				goToNextCollision(nextcollision);
				lastcollision = nextcollision;
				nextcollision = findNextCollisionForSystem(lastcollision);
			}
		}
		catch (const std::exception& err)
		{
			throw err;
			std::cout << "In run():\n";
		}
	}

	void runAndSave(double time, unsigned int frames, std::string &path) {
		try
		{
			unsigned int nextOutputFrame = 0;
			//unsigned int frames = fps * (unsigned int)time;
			double timePerFrame = time / (double)frames;
			std::vector<std::vector<glm::vec3>> positions(std::vector<std::vector<glm::vec3>>(frames, std::vector<glm::vec3>(particles.size(), glm::vec3(0, 0, 0))));
			std::vector<double> times(frames, 0);
			assertDataSizeIsOk(frames);
			for (unsigned int iFrame = 0; iFrame < frames; iFrame++)
			{
				progressByTime(timePerFrame);
				for (int iPtcl = 0; iPtcl < particles.size(); iPtcl++)
				{
					positions[iFrame][iPtcl] = particles[iPtcl]->position;
					times[iFrame] = t;
				}
				if (iFrame == nextOutputFrame)
				{
					std::cout <<"\r"<< (double)(iFrame)*(double)100 / (double)frames << "%";
					nextOutputFrame = (unsigned int)((double)frames / 10 + iFrame);
				}
			}
			std::cout<<"\r"<< 100 << "%\n";
			writeHeaderToFile(path, frames);
			writeDataToFile(path, times, positions);
		}
		catch (const std::exception& err)
		{
			std::cout << "In runAndSave\n";
			throw(err);
		}
		

	}

private:

	Box box;
	std::vector<Particle*> particles;
	std::vector<std::vector<Collision>> candidateCollisions;

	double t;
	double T;
	double dt;


	Collision lastcollision;
	Collision nextcollision;

	void updatePos(double deltaTime) {
		for (unsigned int i_ptcl = 0; i_ptcl < particles.size(); i_ptcl++)
		{
			particles[i_ptcl]->position.x += (float)((double)particles[i_ptcl]->velocity.x * deltaTime); /* I think this casting will make positions well behaved */
			particles[i_ptcl]->position.y += (float)((double)particles[i_ptcl]->velocity.y * deltaTime); /* I think this casting will make positions well behaved */
			particles[i_ptcl]->position.z += (float)((double)particles[i_ptcl]->velocity.z * deltaTime); /* I think this casting will make positions well behaved */
		}
	}

	void updateVel() 
	{
		for (unsigned int i = 0; i < nextcollision.particles.size(); i++)
		{
			nextcollision.particles[i]->velocity = nextcollision.newVelocities[i];
		}
	}

	void goToNextCollision(Collision collision) {
		dt = collision.time - t;
		updatePos(dt);
		updateVel();
		t = collision.time;
	}

	Collision findNextCollisionForSystem(Collision lastCollision) {
		/*
		For each particle in the last collision:
			Find the collision between that particle and every other particle and wall and store it in candidateCollisions
		Find which collision in candidateCollisions occurs in least time from now
		 */
		for each (Particle* particle in lastCollision.particles) {
			/*******************  For each other particle  *******************/
			for (unsigned int i_row = 0; i_row < particle->index; i_row++)
			{
				try
				{
					candidateCollisions[i_row][particle->index] = Collision(particle, particles[i_row], calculateCollisionTimeForParticleOnParticle(particle, particles[i_row]) + t);
				}
				catch (const std::out_of_range& oor)
				{
					std::cout << "In row loop " << i_row << ":\n";
					throw oor;
				}
			}
			for (unsigned int i_col = particle->index + 1; i_col < particles.size(); i_col++)
			{
				try
				{
					candidateCollisions[particle->index][i_col] = Collision(particle, particles[i_col], calculateCollisionTimeForParticleOnParticle(particle, particles[i_col]) + t);
				}
				catch (const std::out_of_range& oor)
				{
					std::cout << "In col loop " << i_col << ":\n";
					throw oor;
				}
			}
			/************************  For each wall  ************************/
			for (unsigned int i_col = (unsigned int)particles.size(); i_col < (unsigned int)(particles.size()+box.walls.size()); i_col++)
			{
				unsigned int i_wall = i_col - (unsigned int)particles.size(); 
				try
				{
					candidateCollisions[particle->index][i_col] = Collision(particle, &box.walls[i_wall], calculateCollisionTimeForParticleOnWall(particle, &box.walls[i_wall]) + t);
				}
				catch (const std::out_of_range& oor)
				{
					std::cout << "In wall loop: " << i_wall << "\n";
					throw oor;
				}
			}
		}
		//std::cout << std::endl;
		/* For each row, find the collision of least time from now, and then take the minimum of all these to find the collision of least time from now */
		Collision collisionOfLeastTime = 0;
		std::vector<Collision> collisionsOfLeastTimeOnRow;
		for (unsigned int i_row = 0; i_row < particles.size(); i_row++)
		{
			collisionsOfLeastTimeOnRow.push_back(*std::min_element(candidateCollisions[i_row].begin(), candidateCollisions[i_row].end()));
		}
		collisionOfLeastTime = *std::min_element(collisionsOfLeastTimeOnRow.begin(), collisionsOfLeastTimeOnRow.end()); /* This could be a wall collision, identified by "particl0s" whos indices exede the number of particles */

		collisionOfLeastTime.calculateNewVelocities(); /* Calculates new velocity for member particles of the collision and stores it in a member variable*/
		collisionOfLeastTime.calculateNewPositions();  /* Calculates new position for member particles of the collision and stores it in a member variable*/
		
		//std::cout << "least time: " << collisionOfLeastTime.time << "\nleasttime minus time = " << collisionOfLeastTime.time - t << std::endl;

		return collisionOfLeastTime;
		
		}

	double calculateCollisionTimeForParticleOnParticle(Particle* jon, Particle* may) 
	{
		double time = 0;
		double A = glm::dot((jon->velocity - may->velocity), (jon->velocity - may->velocity));
		double B = 2 * glm::dot((jon->velocity - may->velocity), (jon->position - may->position));
		double C = glm::dot((jon->position - may->position), (jon->position - may->position)) - (jon->radius + may->radius)*(jon->radius + may->radius);
		double BBm4AC = B*B - 4 * A*C;
		if ((B < 0) && (BBm4AC >= 0))
		{
			time = -(B + sqrt(BBm4AC)) / 2 / A;
		}
		else
		{
			time = std::numeric_limits<double>::infinity();
		}
		assert(isfinite(time) || time == std::numeric_limits<double>::infinity());
		return time;
	}

	double calculateCollisionTimeForParticleOnWall(Particle* jon, Wall* wall) 
	{
		double time = 0;
		double normal_dot_pos = glm::dot(wall->unitNormal, jon->position);
		double normal_dot_vel = glm::dot(wall->unitNormal, jon->velocity);
		time = ((wall->d+jon->radius) - normal_dot_pos) / normal_dot_vel;
		if (time >= 0 && (normal_dot_vel < 0))
		{
			return time;
		}
		return std::numeric_limits<double>::infinity();
	}

	void writePositionsToScreen() {
		std::cout << "Time is: " << t << std::endl;
		for (unsigned int i = 0; i < particles.size(); i++)
		{
			std::cout << std::fixed << std::setprecision(3) << "\t\t" << particles[i]->position[0] << "\t\t" << particles[i]->position[1] << "\t\t" << particles[i]->position[2] << "\n";

		}
	}

	void writeDataToFile(std::string& path, std::vector<double>& times, std::vector<std::vector<glm::vec3>>& positions) {
		try
		{
			std::ofstream ofile;
			ofile.exceptions(std::ofstream::failbit | std::ofstream::badbit);
			/* Open files */
			ofile.open(path, std::ios_base::app);
			if (ofile.good())
			{
				ofile << "\n";
				for (int irow = 0; irow < positions.size(); irow++)
				{
					ofile << times[irow] << "\t";
					for (int icol = 0; icol < positions[0].size(); icol++)
					{
						ofile << positions[irow][icol].x << "\t" << positions[irow][icol].y << "\t" << positions[irow][icol].z << "\t";
					}
					ofile << "\n";
				}
			}
			else
			{
				ofile.close();
				throw (std::exception("ERROR:COULD NOT OPEN FILE"));
			}
			ofile.close();
		}
		catch (std::exception& err)
		{
			std::cout << "In writeDataToFile:\n";
			throw err;
		}
	}
	
	//void writeDataToFile(std::string& path, std::vector<double>& times, std::vector<std::vector<glm::vec3>>& positions) {
	//	try
	//	{
	//		std::ofstream ofile;
	//		ofile.exceptions(std::ofstream::failbit | std::ofstream::badbit);
	//		/* Open files */
	//		ofile.open(path, std::ios_base::app);
	//		if (ofile.good())
	//		{
	//			for (int irow = 0; irow < positions.size(); irow++)
	//			{
	//				ofile << times[irow] << "\t";
	//				for (int icol = 0; icol < positions[0].size(); icol++)
	//				{
	//					ofile << positions[irow][icol].x << "\t" << positions[irow][icol].y << "\t" << positions[irow][icol].z << "\t";
	//				}
	//				ofile << "\n";
	//			}
	//		}
	//		else
	//		{
	//			ofile.close();
	//			throw (std::exception("ERROR:COULD NOT OPEN FILE"));
	//		}
	//		ofile.close();
	//	}
	//	catch (std::exception& err)
	//	{
	//		std::cout << "In writeDataToFile:\n";
	//		throw err;
	//	}
	//}

	void writeHeaderToFile(const std::string& path, unsigned int frames) {
		/*
		Format:
		
	0	nDynamicObjects		nStaticObjects		nFrames
	1	staticObject pos rot scale 
	2	staticObject pos rot scale
	3	staticObj.................
	4	..........................
	5	dynamicObject rot scale 	dynamicObject rot scale 	dynamicObject rot scale 
	6	position					position					position
	7	position					position					position
	8	position						........................................
	9	.........................................................................


		*/
		try
		{
			std::ofstream ofile;
			ofile.exceptions(std::ofstream::failbit | std::ofstream::badbit);
			/* Open files */
			ofile.open(path);
			if (ofile.good())
			{
				ofile << particles.size() << "\t" << 1 << "\t" << frames;
				/* Write static objects: */
				ofile << "\n";
				ofile << "Cube\t" << 
					box.getGeometry().getCenter().x << "\t" << box.getGeometry().getCenter().y << "\t" << box.getGeometry().getCenter().z << "\t" <<
					box.getGeometry().getAngles().x << "\t" << box.getGeometry().getAngles().y << "\t" << box.getGeometry().getAngles().z << "\t" <<
					box.getGeometry().getScales().x << "\t" << box.getGeometry().getScales().y << "\t" << box.getGeometry().getScales().z << "\t";
				/*Write dynamic objects header*/
				ofile << "\n";
				for (unsigned int i = 0; i < particles.size(); i++)
				{
					ofile << "Ball\t"
						<< 0 << "\t" << 0 << "\t" << 0 << "\t"
						<< particles[i]->radius << "\t" << particles[i]->radius << "\t" << particles[i]->radius << "\t";
				}
			}
			else
			{
				ofile.close();
				throw (std::exception("ERROR:COULD NOT OPEN FILE"));
			}
			ofile.close();
		}
		catch (std::exception& err)
		{
			std::cout << "In writeHeaderToFile:\n";
			throw err;
		}

	}


	//void writeHeaderToFile(const std::string& path, unsigned int frames) {
	//	try
	//	{
	//		std::ofstream ofile;
	//		ofile.exceptions(std::ofstream::failbit | std::ofstream::badbit);
	//		/* Open files */
	//		ofile.open(path);
	//		if (ofile.good())
	//		{
	//			ofile << frames << "\t" << particles.size() * 3 << "\n";
	//			for (unsigned int i = 0; i < particles.size(); i++)
	//			{
	//				ofile << "Ball\t";
	//			}
	//			ofile << "\n";
	//			for (unsigned int i = 0; i < particles.size(); i++)
	//			{
	//				ofile << particles[i]->radius << "\t";
	//			}
	//			ofile << "\n";
	//			for (unsigned int i = 0; i < particles.size(); i++)
	//			{
	//				ofile << 0.5 << " " << 0.5 << " " << 0.5 << "\t";
	//			}
	//			ofile << "\n";
	//		}
	//		else
	//		{
	//			ofile.close();
	//			throw (std::exception("ERROR:COULD NOT OPEN FILE"));
	//		}
	//		ofile.close();
	//	}
	//	catch (std::exception& err)
	//	{
	//		std::cout << "In writeHeaderToFile:\n";
	//		throw err;
	//	}


	void assertDataSizeIsOk(unsigned int frames) {
		unsigned int dataSizeInBytes = frames * (unsigned int)particles.size() * 3 * 8;
		if (dataSizeInBytes > 10000000)
		{
			throw(std::exception("ERROR! Output file will exceed the set limit\n"));
		}
	}

	void progressByTime(double deltaTime) 
	{
		double deltaTimeAfterNextCollision = t + deltaTime - nextcollision.time;
		if (deltaTimeAfterNextCollision < 0)
		{
			updatePos(deltaTime);
			t += deltaTime;
		}
		else
		{
			updatePos(nextcollision.time - t);
			updateVel();
			t = nextcollision.time;
			nextcollision = findNextCollisionForSystem(lastcollision);
			progressByTime(deltaTimeAfterNextCollision);
		}
	}

};


#endif // !SIMULATION_H
