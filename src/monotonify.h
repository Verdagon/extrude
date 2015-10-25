#pragma once

#include "fontpoints.h"
#include "utilities.h"
#include "svg.h"
#include <glm.hpp>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <list>

namespace monotonify {

enum class PointType { REGULAR, SPLIT, START, END, MERGE };

class ImprovedNode;
using NodeRef = std::shared_ptr<ImprovedNode>;
class Contour;
using ContourRef = std::shared_ptr<Contour>;
class Graph;
using GraphRef = std::shared_ptr<Graph>;

inline bool pointLessY(glm::vec2 a, glm::vec2 b) {
  if (a.y != b.y) {
    return a.y < b.y;
  } else {
    return a.x < b.x;
  }
}

struct ImprovedNode {
	glm::vec2 point;
	bool insideCurve;
	std::set<NodeRef> neighbors;
	PointType pointType;
	glm::vec2 insideBisector;
	glm::vec2 originalInsideBisector;
	ContourRef contour;

	NodeRef getOtherNeighbor(NodeRef from) {
		for (NodeRef neighbor : neighbors) {
			if (neighbor != from) {
				return neighbor;
			}
		}
		vfail();
	}

	NodeRef getHigherNeighbor() {
		NodeRef neighborA = *neighbors.begin();
		NodeRef neighborB = *++neighbors.begin();
		return pointLessY(neighborA->point, neighborB->point) ?
				neighborB : neighborA;
	}

	float getInteriorAngle() {
		glm::vec2 neighborPoint = (*neighbors.begin())->point;
		glm::vec2 pointToNeighbor = glm::normalize(neighborPoint - point);
		return 2 * acos(glm::dot(pointToNeighbor, insideBisector));
	}
};

class Contour {
	using NodeSet = std::unordered_map<shared_ptr<ImprovedNode>, owning_ptr<ImprovedNode>>;
	NodeSet nodes;

public:
	Contour() { }
	Contour(const Contour & that) { vassert(false); }
	Contour(Contour && that) { vassert(false); }
	void operator=(const Contour & that) { vassert(false); }
	void operator=(Contour && that) { vassert(false); }
	~Contour() {
	}
	void clear() {
		std::list<owning_ptr<ImprovedNode>> owns;
		while (!nodes.empty()) {
			auto beg = nodes.begin();
			owning_ptr<ImprovedNode> & own = beg->second;
			owns.emplace_back(std::move(own));
			nodes.erase(beg);
		}
		for (NodeRef node : owns) {
			node->neighbors.clear();
			ContourRef contour = node->contour;
			node->contour = nullptr;
		}
		owns.clear();
	}

	class SecondIterator {
		NodeSet::iterator iter;
	public:
		SecondIterator(NodeSet::iterator iter) : iter(iter) { }
		NodeRef operator*() { return iter->second; }
		NodeRef operator->() { return iter->second; }
		SecondIterator & operator++() { iter++; return *this; }
		bool operator==(const SecondIterator & that) { return iter == that.iter; }
		bool operator!=(const SecondIterator & that) { return iter != that.iter; }
	};

	void insert(owning_ptr<ImprovedNode> node) {
		nodes.insert(std::pair<NodeRef, owning_ptr<ImprovedNode>>(node, std::move(node)));
	}

	owning_ptr<ImprovedNode> remove(NodeRef node) {
		owning_ptr<ImprovedNode> & previousOwn = nodes[node];
		owning_ptr<ImprovedNode> newOwn = std::move(previousOwn);
		int removed = nodes.erase(node);
		vassert(removed);
		return std::move(newOwn);
	}

	SecondIterator begin() {
		return SecondIterator(nodes.begin());
	}

	SecondIterator end() {
		return SecondIterator(nodes.end());
	}

	int size() const {
		return nodes.size();
	}

	void forInOrder(std::function<void(NodeRef, NodeRef, NodeRef)> func) {
		forInOrder(nodes.begin()->second, func);
	}

	void forInOrder(NodeRef startAt, std::function<void(NodeRef, NodeRef, NodeRef)> func) {
		vassert(startAt);
		vassert(startAt->contour.get() == this);
		NodeRef current = startAt;
		NodeRef next = *current->neighbors.begin();
		NodeRef previous = current->getOtherNeighbor(next);
		do {
			vassert(previous);
			vassert(current);
			vassert(next);
			func(previous, current, next);
			previous = current;
			current = next;
			next = current->getOtherNeighbor(previous);
		} while(current != startAt);
	}
};

class Graph {
	using ContourSet = std::unordered_map<ContourRef, owning_ptr<Contour>>;
	ContourSet contours;

public:
	Graph() { }
	Graph(const Graph & that) { vassert(false); }
	Graph(Graph && that) : contours(std::move(that.contours)) { }
	~Graph() { }
	void operator=(const Graph & that) { vassert(false); }
	void operator=(Graph && that) { vassert(false); }

	void clear() {
		std::list<owning_ptr<Contour>> owns;
		while (!contours.empty()) {
			auto beg = contours.begin();
			owning_ptr<Contour> & own = beg->second;
			owns.emplace_back(std::move(own));
			contours.erase(beg);
		}
		for (owning_ptr<Contour> & own : owns) {
			own->clear();
			own = nullptr;
		}
		owns.clear();
	}

	void insert(owning_ptr<Contour> contour) {
		vassert(contour->begin() != contour->end());
		contours.insert(std::pair<ContourRef, owning_ptr<Contour>>(contour, std::move(contour)));
	}

	owning_ptr<Contour> remove(ContourRef contour) {
		ContourSet::iterator iter = contours.find(contour);
		vassert(iter != contours.end());
		owning_ptr<Contour> & previousOwn = iter->second;
		owning_ptr<Contour> newOwn = std::move(previousOwn);
		int prevSize = contours.size();
		contours.erase(iter);
		vassert(contours.size() == prevSize - 1);
		return std::move(newOwn);
	}

	class SecondIterator {
		ContourSet::iterator iter;
	public:
		SecondIterator(ContourSet::iterator iter) : iter(iter) { }
		ContourRef operator*() { return iter->second; }
		ContourRef operator->() { return iter->second; }
		void operator++() { iter++; }
		bool operator==(const SecondIterator & that) { return iter == that.iter; }
		bool operator!=(const SecondIterator & that) { return iter != that.iter; }
	};

	bool empty() { return contours.empty(); }

	SecondIterator begin() {
		return SecondIterator(contours.begin());
	}

	SecondIterator end() {
		return SecondIterator(contours.end());
	}
};

void addLines(shared_ptr<SVG> svg, GraphRef graph);
void addInsideBisectors(shared_ptr<SVG> svg, GraphRef graph);
void addOriginalInsideBisectors(shared_ptr<SVG> svg, GraphRef graph);
void addPointTypes(shared_ptr<SVG> svg, GraphRef graph);
void addContourLabels(shared_ptr<SVG> svg, GraphRef graph);

owning_ptr<Graph> assembleNodes(const std::vector<std::vector<PointFromFont>> & contours);
void fillInsideBisectors(GraphRef graph);
void prepareContours(shared_ptr<SVGLogger> svgLogger, GraphRef graph);
void monotonifyContours(shared_ptr<SVGLogger> svgLogger, GraphRef graph);

owning_ptr<Contour> separate(NodeRef node);

}
