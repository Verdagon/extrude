
#include "svg.h"
#include "fontpoints.h"
#include "monotonify.h"
#include "utilities.h"
#include "triangulate.h"

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <glm.hpp>
#include <array>

using std::list;
using std::endl;
using std::string;
using std::vector;
using std::tuple;
using std::make_tuple;
using glm::vec2;
using glm::vec3;
using monotonify::GraphRef;
using monotonify::ContourRef;
using monotonify::NodeRef;

struct Vertex {
	vec3 location;
	vec3 normal;
};

struct Triangle {
	std::array<Vertex, 3> vertices;
};

struct Model {
	std::list<Triangle> triangles;
};

void writeObj(const Model & model, const std::string & filename) {
	std::list<std::string> vertexLines;
	std::list<std::string> normalLines;
	std::list<std::string> faceLines;

	for (const Triangle & triangle : model.triangles) {
		int v1Num = vertexLines.size() + 1;
		int n1Num = normalLines.size() + 1;
		for (const Vertex & v : triangle.vertices) {
			vertexLines.push_back(concat("v ", v.location.x, " ", v.location.y, " ", v.location.z));
			normalLines.push_back(concat("vn ", v.normal.x, " ", v.normal.y, " ", v.normal.z));
		}
		faceLines.push_back(concat("f ", v1Num, "//", n1Num, " ", (v1Num + 1), "//", (n1Num + 1), " ", (v1Num + 2), "//", (n1Num + 2)));
	}

	std::ofstream out(filename);
	for (std::string line : vertexLines) {
		out << line << "\n";
	}
	for (std::string line : normalLines) {
		out << line << "\n";
	}
	for (std::string line : faceLines) {
		out << line << "\n";
	}
	out.close();
}

void triangulateFace(GraphRef graph, std::list<Triangle> * triangles, bool raised) {
	vec3 offset = raised ? vec3(0, 0, 1) : vec3(0, 0, 0);
	vec3 desiredNormal = raised ? vec3(0, 0, 1) : vec3(0, 0, -1);

	for (ContourRef contour : *graph) {
		vassert(contour->size() == 3);

		NodeRef nodeA = *contour->begin();
		vec3 pointA = vec3(nodeA->point, 0) + offset;
		Vertex vertexA = { pointA, desiredNormal };

		NodeRef nodeB = *++contour->begin();
		vec3 pointB = vec3(nodeB->point, 0) + offset;
		Vertex vertexB = { pointB, desiredNormal };

		NodeRef nodeC = *++++contour->begin();
		vec3 pointC = vec3(nodeC->point, 0) + offset;
		Vertex vertexC = { pointC, desiredNormal };

		vec3 faceNormal = glm::normalize(glm::cross(
				glm::normalize(pointB - pointA),
				glm::normalize(pointC - pointA)));
		if (glm::dot(faceNormal, desiredNormal) > 0) {
			triangles->push_back({ .vertices = { vertexA, vertexB, vertexC }});
		} else {
			triangles->push_back({ .vertices = { vertexA, vertexC, vertexB }});
		}
	}
}

void extrudeContour(ContourRef contour, std::list<Triangle> * triangles) {
	contour->forInOrder([&](NodeRef, NodeRef currentNode, NodeRef nextNode) {
		vec3 current = vec3(currentNode->point, 0);
		vec3 extrudedCurrent = current + vec3(0, 0, 1);
		vec3 next = vec3(nextNode->point, 0);
		vec3 extrudedNext = next + vec3(0, 0, 1);

		// The normal as if we added the points in the order
		// [current, next, extrudedCurrent]
		vec3 normal = glm::normalize(glm::cross(
				glm::normalize(next - current),
				glm::normalize(extrudedCurrent - current)));
		bool normalIsPointingOutside = glm::dot(normal, vec3(currentNode->insideBisector, 0)) < 0;
		vec3 normalPointingOutside = normalIsPointingOutside ? normal : -normal;

		vec3 currentNormal = currentNode->insideCurve ? vec3(-currentNode->originalInsideBisector, 0) : normalPointingOutside;
		Vertex currentVertex = { current, currentNormal };
		Vertex extrudedCurrentVertex = { extrudedCurrent, currentNormal };
		vec3 nextNormal = nextNode->insideCurve ? vec3(-nextNode->originalInsideBisector, 0) : normalPointingOutside;
		Vertex nextVertex = { next, vec3(-nextNode->originalInsideBisector, 0) };
		Vertex extrudedNextVertex = { extrudedNext, vec3(-nextNode->originalInsideBisector, 0) };

		if (normalIsPointingOutside) {
			// Normal is pointing to outside, goooood.
			triangles->push_back(Triangle { { currentVertex, nextVertex, extrudedCurrentVertex }});
			// Add the other triangle
			triangles->push_back(Triangle { { nextVertex, extrudedNextVertex, extrudedCurrentVertex}});
		} else {
			// The normal is pointing to the inside.
			// Add in the opposite order: [current, extrudedCurrent, next]
			triangles->push_back(Triangle { { currentVertex, extrudedCurrentVertex, nextVertex }});
			// Add the other triangle
			triangles->push_back(Triangle { { nextVertex, extrudedCurrentVertex, extrudedNextVertex }});
		}
	});
}

Model doCharCode(bool svgLogging, std::string fontFilename, int charCode, int detail, float minDist) {
	Model model;

	owning_ptr<SVGLogger> svgLogger =
		svgLogging ?
			owning_ptr<SVGLogger>(new SVGLoggerImpl(concat(charCode))) :
			owning_ptr<SVGLogger>(new FakeSVGLogger());

	vector<vector<PointFromFont>> sampledContours = pointsFromFont(svgLogger, fontFilename, charCode, detail, minDist);

	// std::list<std::tuple<vec3, vec3, vec3>> triangles;
	// for (vector<vec2> & contour : sampledContours) {
	// 	extrudeContour(contour, &triangles);
	// }

	owning_ptr<monotonify::Graph> graph = monotonify::assembleNodes(sampledContours);
	if (!graph->empty()) {
		monotonify::prepareContours(svgLogger, graph);

		std::list<Triangle> triangles;
		for (ContourRef contour : *graph) {
			extrudeContour(contour, &triangles);
		}

		monotonify::monotonifyContours(svgLogger, graph);

		triangulate::triangulate(svgLogger, graph);

		triangulateFace(graph, &triangles, true);
		triangulateFace(graph, &triangles, false);

		graph->clear();

		model.triangles = triangles;
	}

	return model;
}

int main(int argc, char * argv[]) {
	vassert(argc > 1);
	std::string fontFilename = argv[1];

	vassert(argc > 2);
	int charCode = atoi(argv[2]);

	std::string prefix = fontFilename;
	if (argc > 3)
		prefix = argv[3];

	int detail = 5;
	if (argc > 4)
		detail = atoi(argv[4]);

	float minDist = 1.0;
	if (argc > 5)
		minDist = atof(argv[5]);

	bool svgLogging = false;
	if (argc > 6)
		svgLogging = (std::string(argv[6]) == "true");

	try {
		std::cout << "Doing char code " << charCode << std::endl;
		Model model = doCharCode(svgLogging, fontFilename, charCode, detail, minDist);
		writeObj(model, concat(prefix, charCode, ".obj"));
		std::cout << "Done!" << std::endl;
	} catch (const GlyphNotFoundException & e) {
		std::cout << e.what() << std::endl;
	} catch (const GlyphHasSubglyphsException & e) {
		std::cout << e.what() << std::endl;
	} catch (const VAssertFailedException & e) {
		std::cout << e.what() << std::endl;
	}
}
