#ifndef WALL_H
#define WALL_H

class Wall
{
public:
	Wall(glm::dvec3 normal, double d) : unitNormal(glm::normalize(normal)), d(d) {}
	glm::dvec3 unitNormal;
	double d;
	const glm::dvec3& getUnitNormal() const { return unitNormal; }
	double getD() const { return d; }
};

#endif // !WALL_H
