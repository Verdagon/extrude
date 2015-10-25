#include "triangulate.h"

using std::tuple;
using std::make_tuple;
using monotonify::GraphRef;
using monotonify::ContourRef;
using monotonify::NodeRef;
using glm::vec2;
using std::list;
using std::tie;

namespace triangulate {

bool lessY(vec2 a, vec2 b) {
	if (a.y != b.y)
		return a.y < b.y;
	else
		return a.x < b.x;
}

void makeInitialState(NodeRef startNode, list<tuple<NodeRef, bool>> * remainingNodesPtr, list<NodeRef> * reflexChainPtr, bool * reflexChainOnAPtr) {
	list<tuple<NodeRef, bool>> & remainingNodes = *remainingNodesPtr;
	list<NodeRef> & reflexChain = *reflexChainPtr;
	bool & reflexChainOnA = *reflexChainOnAPtr;

	int numNodesInContour = startNode->contour->size();
	startNode->contour->forInOrder(startNode, [&](NodeRef, NodeRef current, NodeRef) {
		if (lessY(current->point, startNode->point))
			startNode = current;
	});
	NodeRef chainANext = startNode;

	NodeRef startNodeNeighborA = *startNode->neighbors.begin();
	NodeRef startNodeNeighborB = *++startNode->neighbors.begin();
	NodeRef chainBNext =
			lessY(startNodeNeighborA->point, startNodeNeighborB->point) ?
			startNodeNeighborA : startNodeNeighborB;
	reflexChain.push_back(chainANext);
	chainANext = chainANext->getOtherNeighbor(chainBNext);

	reflexChain.push_back(chainBNext);
	chainBNext = chainBNext->getOtherNeighbor(startNode);

	for (int i = 0; i < numNodesInContour - 2; i++) {
		if (lessY(chainANext->point, chainBNext->point)) {
			remainingNodes.push_back(make_tuple(chainANext, true));
			chainANext = chainANext->getHigherNeighbor();
		} else {
			remainingNodes.push_back(make_tuple(chainBNext, false));
			chainBNext = chainBNext->getHigherNeighbor();
		}
	}
}

bool isTriangle(NodeRef node) {
	return node->contour->size() == 3;
}

void triangulate(shared_ptr<SVGLogger> svgLogger, GraphRef graph, ContourRef contour) {
	list<tuple<NodeRef, bool>> remainingNodes;
	list<NodeRef> reflexChain;
	bool reflexChainOnA = false;
	makeInitialState(
			*contour->begin(),
			&remainingNodes,
			&reflexChain,
			&reflexChainOnA);

	int numSlices = 0;
	while (true) {
		if (remainingNodes.size() + reflexChain.size() == 3)
			break;
		if (remainingNodes.empty()) {
			vassert(false); // curiousity
			break;
		}
		NodeRef currentNode;
		bool currentNodeOnA;
		tie(currentNode, currentNodeOnA) = remainingNodes.front();
		remainingNodes.pop_front();

		if (currentNodeOnA != reflexChainOnA) {
			while (reflexChain.size() >= 2 && 1 + remainingNodes.size() + reflexChain.size() > 3) {
				owning_ptr<monotonify::Contour> newContour = monotonify::separate(reflexChain.front());
				graph->insert(std::move(newContour));
				numSlices++;
				vassert(isTriangle(reflexChain.front()));
				reflexChain.pop_front();
			}
		} else {
			while (reflexChain.size() >= 2 && reflexChain.back()->getInteriorAngle() < M_PI && 1 + remainingNodes.size() + reflexChain.size() > 3) {
				NodeRef nextToBack = *----reflexChain.end();
				owning_ptr<monotonify::Contour> newContour = monotonify::separate(reflexChain.back());
				graph->insert(std::move(newContour));
				numSlices++;
				vassert(isTriangle(reflexChain.back()));
				reflexChain.pop_back();
			}
		}
		reflexChain.push_back(currentNode);
		reflexChainOnA = currentNodeOnA;

		{
			auto svg = svgLogger->makeSVG();
			monotonify::addLines(svg, graph);
			monotonify::addInsideBisectors(svg, graph);
			monotonify::addPointTypes(svg, graph);
			monotonify::addContourLabels(svg, graph);
			svgLogger->log(svg, "afterseparate");
		}
	}

	{
		auto svg = svgLogger->makeSVG();
		monotonify::addLines(svg, graph);
		monotonify::addOriginalInsideBisectors(svg, graph);
		svgLogger->log(svg, "originsidebisectors");
	}
}

void triangulate(shared_ptr<SVGLogger> svgLogger, GraphRef graph) {
	list<ContourRef> untriangulatedContours;
	for (ContourRef contour : *graph) {
		untriangulatedContours.push_back(contour);
	}
	for (ContourRef contour : untriangulatedContours) {
		triangulate(svgLogger, graph, contour);
	}
}

}
