#ifndef GEOMETRY_H
#define GEOMETRY_H
#pragma once

class Geometry
{
public:
	Geometry(const glm::vec3& s) : Geometry(glm::vec3(0,0,0), glm::vec3(0,0,0), s) {}

	Geometry(const glm::vec3& c, const glm::vec3& a, const glm::vec3& s) : center(c), angles(a), scales(s) {}

	const glm::vec3& getCenter() const { return center; }
	const glm::vec3& getAngles() const { return angles; }
	const glm::vec3& getScales() const { return scales; }
	
private:
	glm::vec3 center;
	glm::vec3 angles;
	glm::vec3 scales;
};



#endif //GEOMETRY