#ifndef BOX_H
#define BOX_H

class Box
{
public:
	Box(const std::vector<Wall>& walls, const glm::vec3& scales) : walls(walls), geometry(scales) {}
	std::vector<Wall> walls;

	const Geometry& getGeometry() const { return geometry; }

private:
	Geometry geometry;

};

#endif // !BOX_H
