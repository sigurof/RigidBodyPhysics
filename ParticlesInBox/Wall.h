#ifndef WALL_H
#define WALL_H

class Wall
{
public:
	Wall(glm::vec3 normal, float d) : unitNormal(glm::normalize(normal)), d(d) {}
	glm::vec3 unitNormal;
	float d;
};

#endif // !WALL_H
