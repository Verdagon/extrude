#include "monotonify.h"
#include <unordered_map>
#include <set>

using std::unordered_set;
using std::unordered_map;
using std::vector;
using std::set;
using std::pair;
using std::make_pair;
using glm::vec2;

namespace monotonify {

owning_ptr<Graph> assembleNodes(const vector<vector<PointFromFont>> & contours) {
	owning_ptr<Graph> graph(new Graph());
	for (int contourI = 0; contourI < contours.size(); contourI++) {
		owning_ptr<Contour> contour(new Contour());

		const vector<PointFromFont> & points = contours[contourI];

		vector<owning_ptr<ImprovedNode>> nodes;
		for (int i = 0; i < points.size(); i++) {
			PointFromFont point = points[i];
			nodes.emplace_back(owning_ptr<ImprovedNode>(new ImprovedNode {
				point.location,
				point.insideCurve,
				std::set<NodeRef>(),
				PointType::REGULAR,
				vec2(0, 0),
				vec2(0, 0),
				contour
			}));
		}

		for (int i = 0; i < nodes.size(); i++) {
			int previousIndex = (i + nodes.size() - 1) % nodes.size();
			int nextIndex = (i + 1) % nodes.size();
			nodes[i]->neighbors.insert(nodes[previousIndex]);
			nodes[i]->neighbors.insert(nodes[nextIndex]);
		}

		for (int i = 0; i < nodes.size(); i++) {
			contour->insert(std::move(nodes[i]));
		}
		if (nodes.size() == 0) {
			std::cerr << "Warning: contour with zero points, skipping." << std::endl;
		} else if (nodes.size() < 3) {
			std::cerr << "Warning: contour with less than three points, skipping." << std::endl;
		} else {
			graph->insert(std::move(contour));
		}
	}

	for (auto contour : *graph) {
		vassert(contour->size() >= 3);
		contour->forInOrder([](NodeRef previous, NodeRef current, NodeRef next) {
			vassert(!veeq(current->point, previous->point));
			vassert(!veeq(current->point, next->point));
		});
	}

	return std::move(graph);
}

vec2 getInsideBisector(vec2 previousPointInsideBisector, vec2 previous, vec2 current, vec2 next, bool allowColinears = false) {
	vassert(previous != current);
	vassert(current != next);
	vassert(previous != next); // added after rust
	vassert(validVec(current));
	vassert(validVec(previous));
	vassert(validVec(next));
	vec2 fromPrevious = glm::normalize(current - previous);
	vec2 fromNext = glm::normalize(current - next);
	vec2 perpendicularFromPrevious = vec2(-fromPrevious.y, fromPrevious.x);
	float dot = glm::dot(perpendicularFromPrevious, previousPointInsideBisector);
	if (dot == 0) {
	    // Then there was a weird point that spiked outwards, like
	    // 1
	    // 2 4   3
	    // 5
	    vassert(false); // Do we even want to handle that?
	}
	vec2 insidePerpendicular = dot > 0 ? perpendicularFromPrevious : -perpendicularFromPrevious;
	vassert(validVec(insidePerpendicular));
	vassert(glm::dot(insidePerpendicular, previousPointInsideBisector) >= 0);
	vassert(glm::dot(insidePerpendicular, fromPrevious) == 0);
	// we now have a vector that's perpendicular to the vector coming from
	// the previous point.
	float posDot = std::abs(glm::dot(insidePerpendicular, fromNext));
	if (posDot < 0.0001) {
	    // then currentPoint is somewhere on a line between previousPoint and nextPoint
		if (!allowColinears) {
		    vassert(false);
		}
	    return insidePerpendicular;
	} else {
		vec2 bisector = glm::normalize(glm::normalize(current - previous) + glm::normalize(current - next));
		vassert(validVec(bisector));
		// The bisector on the inside has to have a positive dot product with the inside perpendicular.
		if (glm::dot(bisector, insidePerpendicular) > 0) {
			return bisector;
		} else {
			return -bisector;
		}
	}
}

void fillInsideBisectors(GraphRef graph) {
	NodeRef nodeLeastY;
	for (ContourRef contour : *graph) {
		NodeRef nodeLeastY;
		for (NodeRef node : *contour) {
			if (!nodeLeastY || node->point.y < nodeLeastY->point.y) {
				nodeLeastY = node;
			}
		}
		contour->forInOrder(nodeLeastY, [nodeLeastY](NodeRef previous, NodeRef current, NodeRef next) {
			vassert(!veeq(current->point, previous->point));
			vassert(!veeq(current->point, next->point));
			vec2 bisector = glm::normalize(glm::normalize(current->point - previous->point) + glm::normalize(current->point - next->point));
			if (current == nodeLeastY) {
				current->insideBisector = bisector.y > 0 ? bisector : -bisector;
			} else {
				vec2 previousPointInsideBisector = previous->insideBisector;
				current->insideBisector = getInsideBisector(previousPointInsideBisector, previous->point, current->point, next->point);
			}

			previous = current;
			current = next;
		});
	}
}

PointType getPointType(vec2 point, float interiorAngle, vec2 neighborA, vec2 neighborB) {
	vassert(validFloat(interiorAngle));
	if (pointLessY(neighborA, point) && pointLessY(neighborB, point)) {
		if (interiorAngle < M_PI) {
			return PointType::END;
		} else {
			return PointType::MERGE;
		}
	} else if (pointLessY(point, neighborA) && pointLessY(point, neighborB)) {
		if (interiorAngle < M_PI) {
			return PointType::START;
		} else {
			return PointType::SPLIT;
		}
	} else {
		return PointType::REGULAR;
	}
}

void fillPointTypes(GraphRef graph) {
	for (ContourRef contour : *graph) {
		contour->forInOrder([](NodeRef previous, NodeRef current, NodeRef next) {
			vec2 pointToNext = glm::normalize(next->point - current->point);
			float dot = glm::dot(pointToNext, current->insideBisector);
			vassert(validFloat(dot));
			float interiorAngle = acos(dot) * 2.0;
			vassert(validFloat(interiorAngle));
			current->pointType = getPointType(current->point, interiorAngle, previous->point, next->point);
		});
	}
}

float getXOnSegment(float y, vec2 start, vec2 end) {
	vassert(end.y != start.y);
	if (end.y < start.y) {
		return getXOnSegment(y, end, start);
	} else {
		vassert(start.y < end.y);
		vassert(start.y <= y);
		vassert(y <= end.y);
		float ratio = (y - start.y) / (end.y - start.y);
		return start.x + (end.x - start.x) * ratio;
	}
}

bool pointInContour(vec2 point, ContourRef contour) {
	int intersections = 0;
	contour->forInOrder([point, &intersections](NodeRef previous, NodeRef current, NodeRef next) {
		if (current->point.y == next->point.y) {
			// Horizontal line, do nothing
		} else {
			if (current->point.y <= point.y && point.y < next->point.y) {
				vassert(current->point.y != next->point.y);
				if (point.x < getXOnSegment(point.y, current->point, next->point)) {
					intersections++;
				}
			}
			if (next->point.y <= point.y && point.y < current->point.y) {
				vassert(current->point.y != next->point.y);
				if (point.x < getXOnSegment(point.y, current->point, next->point)) {
					intersections++;
				}
			}
		}
	});
	return intersections % 2 == 1;
}

void flipInnerPolygonInsideBisectors(GraphRef graph) {
	vector<ContourRef> contours;
	for (ContourRef contour : *graph) {
		contours.push_back(contour);
	}
	for (int i = 0; i < contours.size(); i++) {
		ContourRef thisContour = contours[i];
		vec2 thisContourPoint = (*thisContour->begin())->point;
		int numContainingContours = 0;
		for (int j = 0; j < contours.size(); j++) {
			ContourRef thatContour = contours[j];
			if (i != j) {
				if (pointInContour(thisContourPoint, thatContour)) {
					numContainingContours++;
				}
			}
		}
		if (numContainingContours % 2 == 1) {
			for (NodeRef node : *thisContour) {
				node->insideBisector = -node->insideBisector;
				switch (node->pointType) {
				case PointType::REGULAR: break;
				case PointType::START: node->pointType = PointType::SPLIT; break;
				case PointType::SPLIT: node->pointType = PointType::START; break;
				case PointType::END: node->pointType = PointType::MERGE; break;
				case PointType::MERGE: node->pointType = PointType::END; break;
				}
			}
		}
	}
}

void fillOriginalInsideBisectors(GraphRef graph) {
	for (ContourRef contour : *graph) {
		contour->forInOrder([&](NodeRef previous, NodeRef current, NodeRef next) {
			current->originalInsideBisector = current->insideBisector;
		});
	}
}

void addLines(shared_ptr<SVG> svg, GraphRef graph) {
	for (ContourRef contour : *graph) {
		contour->forInOrder([&](NodeRef previous, NodeRef current, NodeRef next) {
			svg->addLine(current->point, next->point);
		});
	}
}

void addInsideBisectors(shared_ptr<SVG> svg, GraphRef graph) {
	for (ContourRef contour : *graph) {
		for (NodeRef node : *contour) {
			svg->addLine(node->point, node->point + node->insideBisector);
		}
	}
}

void addOriginalInsideBisectors(shared_ptr<SVG> svg, GraphRef graph) {
	for (ContourRef contour : *graph) {
		for (NodeRef node : *contour) {
			svg->addLine(node->point, node->point + node->originalInsideBisector, "gray");
		}
	}
}

void addPointTypes(shared_ptr<SVG> svg, GraphRef graph) {
	for (ContourRef contour : *graph) {
		for (NodeRef node : *contour) {
			svg->addPoint(node->point, 0.3, "black");
			std::string color;
			switch (node->pointType) {
			case PointType::REGULAR: color = "black"; break;
			case PointType::MERGE: color = "rgb(255, 0, 0)"; break;
			case PointType::SPLIT: color = "rgb(255, 0, 255)"; break;
			case PointType::START: color = "rgb(0, 255, 255"; break;
			case PointType::END: color = "rgb(255, 255, 0)"; break;
			}
			svg->addPoint(node->point + node->insideBisector, 0.3, color);
		}
	}
}

void addContourLabels(shared_ptr<SVG> svg, GraphRef graph) {
	int contourIndex = 0;
	for (ContourRef contour : *graph) {
		for (NodeRef node : *contour) {
			svg->addText(node->point + node->insideBisector, concat(contourIndex, ":", node->point.x, ",", node->point.y), -20);
		}
		contourIndex++;
	}
}

bool nodeLessY(NodeRef a, NodeRef b) {
	// to fix the yen, change this to nodesLessY, see rust
	if (pointLessY(a->point, b->point))
		return true;
	if (pointLessY(b->point, a->point))
		return false;
	if (pointLessY(a->insideBisector, b->insideBisector))
		return true;
	if (pointLessY(b->insideBisector, a->insideBisector))
		return false;
	// exactly equal?!
	vfail();
}

struct NodeLessY {
	bool operator()(NodeRef a, NodeRef b) const {
		return nodeLessY(a, b);
	}
};

struct PairHasher {
    typedef pair<NodeRef, NodeRef> argument_type;
    typedef std::size_t result_type;

    result_type operator()(const argument_type & p) const {
    	return (std::size_t)p.first.get() * 13 + (std::size_t)p.second.get() * 7;
    }
};

struct PairEqual {
	bool operator()(const pair<NodeRef, NodeRef> & a, const pair<NodeRef, NodeRef> & b) {
		return a.first.get() == b.first.get() && a.second.get() == b.second.get();
	}
};

float pointToLeft(vec2 from, unordered_set<pair<NodeRef, NodeRef>, PairHasher, PairEqual> liveEdges) {
	float nearestLeftX = from.x;
	for (pair<NodeRef, NodeRef> liveEdge : liveEdges) {
		vec2 begin = liveEdge.first->point;
		vec2 end = liveEdge.second->point;
		if (begin.y == end.y) {
			// skip. anything to the left is below me, according to lessY ordering

			// let x = begin.x;
			// if x < from.x && (nearestLeftX == from.x || x > nearestLeftX) {
			//   nearestLeftX = x;
			// }
			// let x = end.x;
			// if x < from.x && (nearestLeftX == from.x || x > nearestLeftX) {
			//   nearestLeftX = x;
			// }
		} else {
			float x = getXOnSegment(from.y, begin, end);
			if (x < from.x && (nearestLeftX == from.x || x > nearestLeftX)) {
				nearestLeftX = x;
			}
		}
	}
	return nearestLeftX;
}

float pointToRight(vec2 from, unordered_set<pair<NodeRef, NodeRef>, PairHasher, PairEqual> liveEdges) {
	float nearestRightX = from.x;
	for (pair<NodeRef, NodeRef> liveEdge : liveEdges) {
		vec2 begin = liveEdge.first->point;
		vec2 end = liveEdge.second->point;
		if (begin.y == end.y) {
			// skip. anything to the right is above me, according to lessY ordering

			// let x = begin.x;
			// if x > from.x && (nearestRightX == from.x || x < nearestRightX) {
			//   nearestRightX = x;
			// }
			// let x = end.x;
			// if x > from.x && (nearestRightX == from.x || x < nearestRightX) {
			//   nearestRightX = x;
			// }
		} else {
			float x = getXOnSegment(from.y, begin, end);
			if (x > from.x && (nearestRightX == from.x || x < nearestRightX)) {
				nearestRightX = x;
			}
		}
	}
	return nearestRightX;
}

NodeRef findHighestBelow(vec2 point, float leftX, float rightX, set<NodeRef, NodeLessY>::iterator nodeYOrderedIter, set<NodeRef, NodeLessY>::iterator limit) {
	vassert(nodeYOrderedIter != limit);
	nodeYOrderedIter--;
	while (true) {
		NodeRef currentNode = *nodeYOrderedIter;
		if (currentNode->point != point) {
			if (currentNode->point.x == leftX || currentNode->point.x == rightX) {
				vec2 currentTowardPoint = glm::normalize(currentNode->point - point);
				// If it's facing towards me
				if (glm::dot(currentNode->insideBisector, currentTowardPoint) < 0) {
					return currentNode;
				}
				// TODO: what happens if multiple are facing towards me? dont we want the one
				// that's most pointed towards me?
			}
			if (currentNode->point.x > leftX && currentNode->point.x < rightX) {
				return currentNode;
			}
		}
		vassert(nodeYOrderedIter != limit);
		nodeYOrderedIter--;
	}
}

NodeRef findLowestAbove(vec2 point, float leftX, float rightX, set<NodeRef, NodeLessY>::iterator nodeYOrderedIter, set<NodeRef, NodeLessY>::iterator limit) {
	vassert(nodeYOrderedIter != limit);
	nodeYOrderedIter++;
	vassert(nodeYOrderedIter != limit);
	while (true) {
		NodeRef currentNode = *nodeYOrderedIter;
		if (currentNode->point != point) {
			if (currentNode->point.x == leftX || currentNode->point.x == rightX) {
				vec2 currentTowardPoint = glm::normalize(currentNode->point - point);
				// If it's facing towards me
				if (glm::dot(currentNode->insideBisector, currentTowardPoint) < 0) {
					return currentNode;
				}
				// TODO: what happens if multiple are facing towards me? dont we want the one
				// that's most pointed towards me?
			}
			if (currentNode->point.x > leftX && currentNode->point.x < rightX) {
				return currentNode;
			}
		}
		nodeYOrderedIter++;
		vassert(nodeYOrderedIter != limit, point, " ", leftX, " ", rightX);
	}
}

pair<NodeRef, NodeRef> getNewEdge(GraphRef graph) {
	set<NodeRef, NodeLessY> nodesYRanked;
	for (ContourRef contour : *graph) {
		for (NodeRef node : *contour) {
			nodesYRanked.insert(node);
		}
	}

	// Lower of the two is first
	unordered_set<pair<NodeRef, NodeRef>, PairHasher, PairEqual> liveEdges;

	set<NodeRef, NodeLessY>::iterator nodeYOrderedIter = nodesYRanked.begin();
	while (nodeYOrderedIter != nodesYRanked.end()) {
		NodeRef node = *nodeYOrderedIter;
		for (NodeRef neighbor : node->neighbors) {
			if (pointLessY(neighbor->point, node->point)) {
				bool removed = liveEdges.erase(make_pair(neighbor, node));
				vassert(removed);
			}
		}
		for (NodeRef neighbor : node->neighbors) {
			if (pointLessY(node->point, neighbor->point)) {
				bool added = liveEdges.insert(make_pair(node, neighbor)).second;
				vassert(added);
			}
		}
		if (node->pointType == PointType::SPLIT) {
			float leftX = pointToLeft(node->point, liveEdges);
			float rightX = pointToRight(node->point, liveEdges);
			NodeRef newEdgemate = findHighestBelow(node->point, leftX, rightX, nodeYOrderedIter, nodesYRanked.begin());
			return make_pair(node, newEdgemate);
		}
		if (node->pointType == PointType::MERGE) {
			float leftX = pointToLeft(node->point, liveEdges);
			float rightX = pointToRight(node->point, liveEdges);
			NodeRef newEdgemate = findLowestAbove(node->point, leftX, rightX, nodeYOrderedIter, nodesYRanked.end());
			return make_pair(node, newEdgemate);
		}
		nodeYOrderedIter++;
	}
	return pair<NodeRef, NodeRef>(NodeRef(), NodeRef());
}

NodeRef disconnect(NodeRef original) {
	NodeRef neighbor0 = *original->neighbors.begin();
	NodeRef neighbor1 = *++original->neighbors.begin();

	int erased0 = neighbor0->neighbors.erase(original);
	vassert(erased0);
	int erased1 = neighbor1->neighbors.erase(original);
	vassert(erased1);

	original->neighbors.clear();

	original->pointType = PointType::REGULAR;
	// original will get the 0th neighbor, new will get the 1th neighbor
	owning_ptr<ImprovedNode> ownedClone(new ImprovedNode {
		original->point,
		original->insideCurve,
		std::set<NodeRef>(),
		PointType::REGULAR,
		original->insideBisector,
		original->originalInsideBisector,
		original->contour
	});
	NodeRef clone = ownedClone;
	original->contour->insert(std::move(ownedClone));

	// Connect original to neighbor 0
	bool added1 = original->neighbors.insert(neighbor0).second;
	vassert(added1);
	bool added2 = neighbor0->neighbors.insert(original).second;
	vassert(added2);
	// Don't connect original to clone
	// Connect clone to neighbor 1
	bool added3 = clone->neighbors.insert(neighbor1).second;
	vassert(added3);
	bool added4 = neighbor1->neighbors.insert(clone).second;
	vassert(added4);

	vassert(neighbor0->neighbors.size() == 2);
	vassert(original->neighbors.size() == 1);
	vassert(clone->neighbors.size() == 1);
	vassert(neighbor1->neighbors.size() == 2);

	return clone;
}

bool colinear(NodeRef node) {
	NodeRef neighbor0 = *node->neighbors.begin();
	NodeRef neighbor1 = *++node->neighbors.begin();
	vec2 towardNeighbor0 = glm::normalize(neighbor0->point - node->point);
	vec2 towardNeighbor1 = glm::normalize(neighbor1->point - node->point);
	vec2 perpendicular(-towardNeighbor0.y, towardNeighbor0.x);
	return std::abs(glm::dot(towardNeighbor1, perpendicular)) < 0.0001;
}

owning_ptr<ImprovedNode> removeNode(NodeRef node) {
	NodeRef neighbor0 = *node->neighbors.begin();
	NodeRef neighbor1 = *++node->neighbors.begin();

	int erased0 = neighbor0->neighbors.erase(node);
	vassert(erased0);
	int erased1 = neighbor1->neighbors.erase(node);
	vassert(erased1);
	node->neighbors.clear();

	neighbor0->neighbors.insert(neighbor1);
	neighbor1->neighbors.insert(neighbor0);

	owning_ptr<ImprovedNode> own = node->contour->remove(node);
	node = nullptr;
	return std::move(own);
}

void classifyPoint(NodeRef node) {
	NodeRef neighbor0 = *node->neighbors.begin();
	NodeRef neighbor1 = *++node->neighbors.begin();

	vec2 pointToNext = glm::normalize(neighbor0->point - node->point);
	float dot = glm::dot(pointToNext, node->insideBisector);
	vassert(validFloat(dot));
	float interiorAngle = acos(dot) * 2.0;
	vassert(validFloat(interiorAngle));

	node->pointType = getPointType(node->point, interiorAngle, neighbor0->point, neighbor1->point);
}

std::vector<owning_ptr<ImprovedNode>> slice(GraphRef graph, NodeRef originalA, NodeRef originalB) {
	bool joiningContours = (originalA->contour != originalB->contour);

	if (joiningContours) {
		// Then we're slicing two contours together.
		// Take originalB's contour and combine it into originalA's.
		std::list<NodeRef> nodesToMove;
		originalB->contour->forInOrder([&nodesToMove](NodeRef previous, NodeRef current, NodeRef next) {
			nodesToMove.push_back(current);
		});
		ContourRef oldContour = originalB->contour;
		for (NodeRef node : nodesToMove) {
			vassert(node->contour == oldContour);
			owning_ptr<ImprovedNode> ownedNode = node->contour->remove(node);
			node->contour = originalA->contour;
			node->contour->insert(std::move(ownedNode));
		}
		owning_ptr<Contour> own = graph->remove(oldContour);
		oldContour = nullptr;
		own->clear();
	}

	vec2 aPoint = originalA->point;
	NodeRef aNeighbor0 = *originalA->neighbors.begin();
	NodeRef aNeighbor1 = *++originalA->neighbors.begin();
	vec2 bPoint = originalB->point;
	NodeRef bNeighbor0 = *originalB->neighbors.begin();
	NodeRef bNeighbor1 = *++originalB->neighbors.begin();

	NodeRef cloneA = disconnect(originalA);
	NodeRef cloneB = disconnect(originalB);

	originalA->insideBisector = getInsideBisector(
			aNeighbor0->insideBisector, aNeighbor0->point, aPoint, bPoint, true);
	cloneA->insideBisector = getInsideBisector(
			aNeighbor1->insideBisector, aNeighbor1->point, aPoint, bPoint, true);
	originalB->insideBisector = getInsideBisector(
			bNeighbor0->insideBisector, bNeighbor0->point, bPoint, aPoint, true);
	cloneB->insideBisector = getInsideBisector(
			bNeighbor1->insideBisector, bNeighbor1->point, bPoint, aPoint, true);

	vec2 aToB = bPoint - aPoint;
	vec2 newEdgePerpendicular = glm::normalize(vec2(-aToB.y, aToB.x));

	bool aOriginalOnPositiveSideOfNewEdge = glm::dot(originalA->insideBisector, newEdgePerpendicular) > 0.0;
	bool aCloneOnPositiveSideOfNewEdge = glm::dot(cloneA->insideBisector, newEdgePerpendicular) > 0.0;
	vassert(aOriginalOnPositiveSideOfNewEdge != aCloneOnPositiveSideOfNewEdge);
	bool bOriginalOnPositiveSideOfNewEdge = glm::dot(originalB->insideBisector, newEdgePerpendicular) > 0.0;
	bool bCloneOnPositiveSideOfNewEdge = glm::dot(cloneB->insideBisector, newEdgePerpendicular) > 0.0;
	vassert(bOriginalOnPositiveSideOfNewEdge != bCloneOnPositiveSideOfNewEdge);
	
	if (aOriginalOnPositiveSideOfNewEdge == bOriginalOnPositiveSideOfNewEdge) {
		// aOriginal and bOriginal are on the same side of the new edge, connect them.
		bool added1 = originalA->neighbors.insert(originalB).second;
		vassert(added1);
		bool added2 = originalB->neighbors.insert(originalA).second;
		vassert(added2);
		bool added3 = cloneA->neighbors.insert(cloneB).second;
		vassert(added3);
		bool added4 = cloneB->neighbors.insert(cloneA).second;
		vassert(added4);
	} else {
		vassert(aOriginalOnPositiveSideOfNewEdge == bCloneOnPositiveSideOfNewEdge);
		vassert(bOriginalOnPositiveSideOfNewEdge == aCloneOnPositiveSideOfNewEdge);
		bool added1 = originalA->neighbors.insert(cloneB).second;
		vassert(added1);
		bool added2 = cloneB->neighbors.insert(originalA).second;
		vassert(added2);
		bool added3 = originalB->neighbors.insert(cloneA).second;
		vassert(added3);
		bool added4 = cloneA->neighbors.insert(originalB).second;
		vassert(added4);
	}

	if (!joiningContours) {
		// Then we're slicing one contour into two

		// Pick a random node, and move it and all those connected to a new graph
		// Well, instead of random, we just pick originalA.
		std::list<NodeRef> nodesToSplitOff;
		originalA->contour->forInOrder([&nodesToSplitOff](NodeRef previous, NodeRef current, NodeRef next) {
			nodesToSplitOff.push_back(current);
		});
		owning_ptr<Contour> newContour(new Contour());
		for (NodeRef node : nodesToSplitOff) {
			owning_ptr<ImprovedNode> ownedNode = node->contour->remove(node);
			node->contour = newContour;
			node->contour->insert(std::move(ownedNode));
		}
		graph->insert(std::move(newContour));
	}

	set<ContourRef> involvedContours;
	involvedContours.insert(originalA->contour);
	involvedContours.insert(originalB->contour);
	involvedContours.insert(cloneA->contour);
	involvedContours.insert(cloneB->contour);
	if (joiningContours) {
		vassert(involvedContours.size() == 1);
	} else {
		vassert(involvedContours.size() == 2);
	}

	vassert(aNeighbor0->neighbors.size() == 2);
	vassert(aNeighbor1->neighbors.size() == 2);
	vassert(bNeighbor0->neighbors.size() == 2);
	vassert(bNeighbor1->neighbors.size() == 2);
	vassert(originalA->neighbors.size() == 2);
	vassert(originalB->neighbors.size() == 2);
	vassert(cloneA->neighbors.size() == 2);
	vassert(cloneB->neighbors.size() == 2);

	classifyPoint(originalA);
	classifyPoint(originalB);
	classifyPoint(cloneA);
	classifyPoint(cloneB);

	std::vector<owning_ptr<ImprovedNode>> doomedNodes;
	if (colinear(originalA)) {
		doomedNodes.emplace_back(removeNode(originalA));
	}
	if (colinear(originalB)) {
		doomedNodes.emplace_back(removeNode(originalB));
	}
	if (colinear(cloneA)) {
		doomedNodes.emplace_back(removeNode(cloneA));
	}
	if (colinear(cloneB)) {
		doomedNodes.emplace_back(removeNode(cloneB));
	}
	return std::move(doomedNodes);
}

void prepareContours(shared_ptr<SVGLogger> svgLogger, GraphRef graph) {
	if (graph->empty()) {
		return;
	}

	for (auto contour : *graph) {
		contour->forInOrder([](NodeRef previous, NodeRef current, NodeRef next) {
			vassert(!veeq(current->point, previous->point));
			vassert(!veeq(current->point, next->point));
		});
	}

	fillInsideBisectors(graph);

	{
		auto svg = svgLogger->makeSVG();
		addLines(svg, graph);
		addInsideBisectors(svg, graph);
		svgLogger->log(svg, "bisectors");
	}

	fillPointTypes(graph);

	{
		auto svg = svgLogger->makeSVG();
		addLines(svg, graph);
		addInsideBisectors(svg, graph);
		addPointTypes(svg, graph);
		svgLogger->log(svg, "pointtypesunadjusted");
	}

	flipInnerPolygonInsideBisectors(graph);

	fillOriginalInsideBisectors(graph);

	{
		auto svg = svgLogger->makeSVG();
		addLines(svg, graph);
		addInsideBisectors(svg, graph);
		addPointTypes(svg, graph);
		svgLogger->log(svg, "pointtypes");
	}
}

void monotonifyContours(shared_ptr<SVGLogger> svgLogger, GraphRef graph) {
	if (graph->empty()) {
		return;
	}

	while (true) {
		pair<NodeRef, NodeRef> newEdge = getNewEdge(graph);
		if (newEdge.first) {
			std::vector<owning_ptr<ImprovedNode>> doomedNodes = slice(graph, newEdge.first, newEdge.second);
			newEdge.first = nullptr;
			newEdge.second = nullptr;
			doomedNodes.clear();

			for (ContourRef contour : *graph) {
				contour->forInOrder([](NodeRef prev, NodeRef current, NodeRef next) {
				});
			}

			{
				auto svg = svgLogger->makeSVG();
				addLines(svg, graph);
				addInsideBisectors(svg, graph);
				addPointTypes(svg, graph);
				addContourLabels(svg, graph);
				svgLogger->log(svg, "afterslice");
			}
		} else {
			break;
		}
	}

	{
		auto svg = svgLogger->makeSVG();
		addLines(svg, graph);
		addOriginalInsideBisectors(svg, graph);
		svgLogger->log(svg, "originsidebisectors");
	}
}

void updateInsideBisector(NodeRef node, NodeRef neighborWithGoodInsideBisector) {
	NodeRef otherNeighbor = node->getOtherNeighbor(neighborWithGoodInsideBisector);
	node->insideBisector = getInsideBisector(
			neighborWithGoodInsideBisector->insideBisector,
			neighborWithGoodInsideBisector->point,
			node->point,
			otherNeighbor->point,
			true);
}

owning_ptr<Contour> separate(NodeRef node) {

	NodeRef neighborA = *node->neighbors.begin();
	NodeRef neighborB = *++node->neighbors.begin();
	NodeRef neighborAOtherNeighbor = neighborA->getOtherNeighbor(node);
	NodeRef neighborBOtherNeighbor = neighborB->getOtherNeighbor(node);

	node->neighbors.clear();
	bool removed1 = neighborA->neighbors.erase(node);
	vassert(removed1);
	bool removed2 = neighborB->neighbors.erase(node);
	vassert(removed2);

	bool added1 = neighborA->neighbors.insert(neighborB).second;
	vassert(added1);
	bool added2 = neighborB->neighbors.insert(neighborA).second;
	vassert(added2);

	updateInsideBisector(neighborA, neighborAOtherNeighbor);
	updateInsideBisector(neighborB, neighborBOtherNeighbor);

	owning_ptr<Contour> newContour(new Contour());

	owning_ptr<ImprovedNode> ownedNode = node->contour->remove(node);
	ownedNode->contour = newContour;
	newContour->insert(std::move(ownedNode));

	owning_ptr<ImprovedNode> neighborAClone(new ImprovedNode {
		neighborA->point,
		neighborA->insideCurve,
		std::set<NodeRef>(),
		PointType::REGULAR,
		vec2(0, 0),
		neighborA->originalInsideBisector,
		newContour
	});

	owning_ptr<ImprovedNode> neighborBClone(new ImprovedNode {
		neighborB->point,
		neighborB->insideCurve,
		std::set<NodeRef>(),
		PointType::REGULAR,
		vec2(0, 0),
		neighborB->originalInsideBisector,
		newContour
	});

	node->neighbors.insert(neighborAClone);
	node->neighbors.insert(neighborBClone);
	neighborAClone->neighbors.insert(node);
	neighborAClone->neighbors.insert(neighborBClone);
	neighborBClone->neighbors.insert(node);
	neighborBClone->neighbors.insert(neighborAClone);
	updateInsideBisector(neighborAClone, node);
	updateInsideBisector(neighborBClone, node);

	newContour->insert(std::move(neighborAClone));
	newContour->insert(std::move(neighborBClone));

	return std::move(newContour);
}

}
