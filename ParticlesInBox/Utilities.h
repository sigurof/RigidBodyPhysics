#ifndef UTILITIES_H
#define UTILITIES_H
//
//
//std::ostream& operator<<(std::ostream& os, const glm::vec3& vec) {
//	os << vec.x << "\t" << vec.y << "\t" << vec.z << "\t";
//	return os;
//}

float calculateSquaredNorm(glm::vec3 vec) {
	return vec.x*vec.x + vec.y*vec.y + vec.z*vec.z;
}


#endif // !UTILITIES_H
