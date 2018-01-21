#ifndef SIMULATION_H
#define SIMULATION_H


bool particleIsInsideBox(const Particle& particle, const Box& box)
{
	for each (Wall wall in box.walls) {
		double normal_dot_pos = glm::dot(wall.unitNormal, particle.r0);
		double D = glm::abs(normal_dot_pos + wall.d);

		if (D - particle.radius < 0)
		{
			return false;
		}
	}
	return true;
}

class Simulation
{
public:

	Simulation(const std::vector<Particle>& ptcls, const Box& box, double time, unsigned int frames, const std::string& outputPath) : 
		T(time), t(0), box(box), frames(frames), path(outputPath)
	{
		candidateCollisions = std::vector<std::vector<Collision>>(ptcls.size(), 
																				std::vector<Collision>(ptcls.size() + box.walls.size(), 
																																		Collision(std::numeric_limits<double>::infinity())));
		for (unsigned int i = 0; i < ptcls.size(); i++)
		{
			particles.push_back(new Particle(ptcls[i]));
		}
		lastCollision = Collision(particles);//A collision object with all particles as members.
		nextCollision = findNextCollisionForSystem(lastCollision);
		useMutualInteractions = true;
		
	}

	~Simulation() {
		for (unsigned int i = 0; i < particles.size(); i++)
		{
			delete particles[i];
		}
	}

	void setUseMutualInteraction(bool use)
	{
		useMutualInteractions = use;
	}




	void runAndSave() {
		try
		{
			/* Preparation */
			unsigned int nextOutputFrame = 0;
			double timePerFrame = T / (double)frames;
			std::vector<std::vector<glm::dvec3>> positions(std::vector<std::vector<glm::dvec3>>(frames, std::vector<glm::dvec3>(particles.size(), glm::dvec3(0, 0, 0))));
			std::vector<double> times(frames, 0);
			assertDataSizeIsOk(frames);

			/* Main loop */
			for (unsigned int iFrame = 0; iFrame < frames; iFrame++)
			{
				progressByTime(timePerFrame, 0); /* t = t+dt */

				
				for (int iPtcl = 0; iPtcl < particles.size(); iPtcl++)
				{
					positions[iFrame][iPtcl] = particles[iPtcl]->position;
					times[iFrame] = t;
				}
				if (iFrame == nextOutputFrame)
				{
					std::cout <<"\r"<< (double)(iFrame)*(double)100 / (double)frames << "%";
					nextOutputFrame = (unsigned int)((double)frames /20.0 + iFrame);
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

	static glm::dvec3 gravity;

private:

	Box box;
	std::vector<Particle*> particles;
	std::vector<std::vector<Collision>> candidateCollisions;
	double t;
	double T;
	double dt;
	unsigned int frames;
	bool useMutualInteractions;
	Collision lastCollision;
	Collision nextCollision;
	std::string path;

	void updateInstantaneousPosition() 
	{
		double Dt;
		for (unsigned int i_ptcl = 0; i_ptcl < particles.size(); i_ptcl++)
		{
			Dt = t - particles[i_ptcl]->t0; 
			particles[i_ptcl]->position = particles[i_ptcl]->r0 + particles[i_ptcl]->v0 * Dt + gravity * Dt*Dt / 2.0; 
		}
	}

	void update_r0_v0_t0()
	{
		for (unsigned int i = 0; i < particles.size(); i++)
		{
			particles[i]->update_r0(nextCollision.t);
			particles[i]->update_v0(nextCollision.t);
			particles[i]->update_t0(nextCollision.t);
		}
		for (unsigned int i = 0; i < nextCollision.particles.size(); i++)
		{
			nextCollision.particles[i]->update_v0(nextCollision.newVelocities[i]);
		}
	}

	Collision findNextCollisionForSystem(const Collision& lastCollision) {
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
					if (!useMutualInteractions)
					{
						candidateCollisions[i_row][particle->index].t = std::numeric_limits<double>::infinity();
					}
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
					if (!useMutualInteractions)
					{
						candidateCollisions[particle->index][i_col].t = std::numeric_limits<double>::infinity();
					}
				}
				catch (const std::out_of_range& oor)
				{
					std::cout << "In col loop " << i_col << ":\n";
					throw oor;
				}
			}
			/****************************  For each wall  *****************************/
			for (unsigned int i_col = (unsigned int)particles.size(); i_col < (unsigned int)(particles.size()+box.walls.size()); i_col++)
			{
				unsigned int i_wall = i_col - (unsigned int)particles.size();
				/************************  In the case of lastCollision.type == WALLCOLLISION, we don't want to consider the same wall because of doubleing point trouble  ************************/
				try
				{
					candidateCollisions[particle->index][i_col] = Collision(particle, &box.walls[i_wall], calculateCollisionTimeForParticleOnWall(particle, &box.walls[i_wall]) + t);
					//invalidateCollisionIfIsImpossible(candidateCollisions[particle->index][i_col]);
				}
				catch (const std::out_of_range& oor)
				{
					std::cout << "In wall loop: " << i_wall << "\n";
					throw oor;
				}
			}
		}
		/* For each row, find the collision of least time from now, and then take the minimum of all these to find the collision of least time from now */
		Collision collisionOfLeastTime = 0;
		std::vector<Collision> collisionsOfLeastTimeOnRow;
		for (unsigned int i_row = 0; i_row < particles.size(); i_row++)
		{
			collisionsOfLeastTimeOnRow.push_back(*std::min_element(candidateCollisions[i_row].begin(), candidateCollisions[i_row].end()));
		}
		collisionOfLeastTime = *std::min_element(collisionsOfLeastTimeOnRow.begin(), collisionsOfLeastTimeOnRow.end()); 

		collisionOfLeastTime.calculateNewVelocities(lastCollision.t); /* Calculates new velocity for member particles of the collision and stores it in a member variable*/

		return collisionOfLeastTime;
	}

	double calculateCollisionTimeForParticleOnParticle(Particle* p1, Particle* p2) 
	{
		double time = 0;// double time2 = 0;
		double A = glm::dot((p1->v0 - p2->v0), (p1->v0 - p2->v0));
		double B = 2.0f * glm::dot((p1->v0 - p2->v0), (p1->r0 - p2->r0));
		double C = glm::dot((p1->r0 - p2->r0), (p1->r0 - p2->r0)) - (p1->radius + p2->radius)*(p1->radius + p2->radius);
		double BBm4AC = B*B - 4 * A*C;
		if ((B < 0) && (BBm4AC >= 0))
		{
			//time2 = -(B - sqrt(BBm4AC)) / 2.0f / A;
			//if (time2 )
			//{
			//
			//}
			time = -(B + sqrt(BBm4AC)) / 2.0f / A;
		}
		else
		{
			time = std::numeric_limits<double>::infinity();
		}
		if (!(isfinite(time) || time == std::numeric_limits<double>::infinity()))
		{
			throw(std::exception("ERROR! Time is neither finite nor infinite"));
		}
		return time;
	}

	bool isCollisionAgainstInsideFaceOfWall(const Collision& col)
	{
		if (glm::dot(col.getVelocitiesRightBeforeCollision()[0], col.wall->getUnitNormal()) > 0)
		{
			return false;
		}
		return true;
	}

	double calculateCollisionTimeForParticleOnWall(Particle* p, Wall* w) 
	{
		/* solve At^2 + Bt + C = 0*/
		double time1 = std::numeric_limits<double>::infinity();
		double time2 = std::numeric_limits<double>::infinity();
		double time = std::numeric_limits<double>::infinity();
		double rad = p->radius;
		double d = w->d;
		double n_dot_a = glm::dot(w->unitNormal, p->gravity);
		double n_dot_v = glm::dot(w->unitNormal, p->v0);
		double n_dot_r = glm::dot(w->unitNormal, p->r0);
		double A = n_dot_a / 2.0f;
		double B = n_dot_v;
		double C = n_dot_r + d - rad;
		
		if (A == 0)
		{
			/* ABC formula invalid, only one solution for t */
			if (B >= 0)
			{ /* Fail case. B < 0 means the particle motion is parallel to this wall */
				return std::numeric_limits<double>::infinity();
			}
			time = (-d + rad - n_dot_r) / n_dot_v;
		}
		else
		{
			double BBm4AC = B * B - 4 * A*C;
			if (BBm4AC < 0)
			{ /* Fail case. BBm4AC < 0 gives imaginary time until collision. Motion parallel with wall */
				return std::numeric_limits<double>::infinity();
			}
			time1 = (-B + sqrt(BBm4AC)) / (2.0f * A);
			time2 = (-B - sqrt(BBm4AC)) / (2.0f * A);
			time = pickCorrectTime(p, w, time1, time2);
		}
		if (time < 0)
		{ /* Fail case. Time until collision must be positive */
			return std::numeric_limits<double>::infinity();
		}
		if (!(isfinite(time)/* || time == std::numeric_limits<double>::infinity()*/))
		{
			throw(std::exception("ERROR! Time is not finite"));
		}
		return time;
	}

	double pickCorrectTime(Particle* p, Wall* w, double t1, double t2)
	{
		double time = std::numeric_limits<double>::infinity();
		// Velocity of particle p at time t1 right before collision with wall w
		double Dt1 = t1 + t - p->t0;
		glm::dvec3 v1 = p->v0 + p->gravity*Dt1;
		// Velocity of particle p at time t2 right before collision with wall w
		double Dt2 = t2 + t - p->t0;
		glm::dvec3 v2 = p->v0 + p->gravity*Dt2;
		if (glm::dot(v1, w->getUnitNormal()) < 0)
		{
			return t1;
		}
		else if (glm::dot(v2, w->getUnitNormal()) < 0)
		{
			return t2;

		}
		return -1.0;
	}

	void writePositionsToScreen() {
		std::cout << "Time is: " << t << std::endl;
		for (unsigned int i = 0; i < particles.size(); i++)
		{
			std::cout << std::fixed << std::setprecision(3) << "\t\t" << particles[i]->position[0] << "\t\t" << particles[i]->position[1] << "\t\t" << particles[i]->position[2] << "\n";

		}
	}

	void writeDataToFile(std::string& path, std::vector<double>& times, std::vector<std::vector<glm::dvec3>>& positions) {
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

	void assertDataSizeIsOk(unsigned int frames) {
		unsigned int dataSizeInBytes = frames * (unsigned int)particles.size() * 3 * 8;
		if (dataSizeInBytes > 10000000)
		{
			throw(std::exception("ERROR! Output file will exceed the set limit\n"));
		}
	}

	void progressByTime(double deltaTime, int recursiveIndex) 
	{
		double newTime = deltaTime + t;
		if (newTime > nextCollision.t)
		{	/* collision happens before deltaTime */
			update_r0_v0_t0();
			t = nextCollision.t;
			lastCollision = nextCollision;
			nextCollision = findNextCollisionForSystem(lastCollision);
			progressByTime(newTime - t, ++recursiveIndex);

		}
		else /* collision happens after deltaTime */
		{
			t = newTime;// increment time;
			updateInstantaneousPosition();
		}
		
	}


};


#endif // !SIMULATION_H
