#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "PathSmoothing.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
	NavGraph* const pNavGraph, std::vector<FVector2D>& debugNodePositions, std::vector<NavLine>& debugPortals)
{
	std::vector<FVector2D> finalPath{};

	TriPolygon const* pNavPoly = pNavGraph->GetNavPolygon();

	// Get start and end triangles
	FVector2D StartSnapped{};
	FVector2D EndSnapped{};
	TriPolygon::Triangle const* pStartTriangle = pNavPoly->GetClosestTriangleToPosition(startPos, StartSnapped);
	TriPolygon::Triangle const* pEndTriangle = pNavPoly->GetClosestTriangleToPosition(endPos, EndSnapped);

	// Check if valid triangles exist
	if (!pStartTriangle || !pEndTriangle)
		return finalPath;

	// If same triangle, just return direct path
	if (*pStartTriangle == *pEndTriangle)
	{
		finalPath.push_back(StartSnapped);
		finalPath.push_back(EndSnapped);
		return finalPath;
	}

	// Clone the graph so we can add temporary start/end nodes
	std::unique_ptr<NavGraph> pClonedGraph = pNavGraph->Clone();

	// Create start node (LineIdx = -1, not on an edge)
	auto pStartNode = std::make_unique<NavGraphNode>(StartSnapped, -1);
	int StartNodeId = pClonedGraph->AddNode(std::move(pStartNode));

	// Connect start node to all nodes on the edges of the start triangle
	for (auto const& Edge : pStartTriangle->GetEdges())
	{
		int EdgeIdx = pNavPoly->FindEdgeIndex(Edge).value_or(-1);
		int NodeId = pClonedGraph->GetNodeIdFromEdgeIndex(EdgeIdx);
		if (NodeId != Graphs::InvalidNodeId)
		{
			FVector2D StartPos2D = StartSnapped;
			FVector2D NodePos = pClonedGraph->GetNode(NodeId)->GetPosition();
			float Cost = FVector2D::Distance(StartPos2D, NodePos);

			auto Conn = std::make_unique<Connection>(StartNodeId, NodeId);
			Conn->SetWeight(Cost);
			pClonedGraph->AddConnection(std::move(Conn));
		}
	}

	// Create end node (LineIdx = -1, not on an edge)
	auto pEndNode = std::make_unique<NavGraphNode>(EndSnapped, -1);
	int EndNodeId = pClonedGraph->AddNode(std::move(pEndNode));

	// Connect end node to all nodes on the edges of the end triangle
	for (auto const& Edge : pEndTriangle->GetEdges())
	{
		int EdgeIdx = pNavPoly->FindEdgeIndex(Edge).value_or(-1);
		int NodeId = pClonedGraph->GetNodeIdFromEdgeIndex(EdgeIdx);
		if (NodeId != Graphs::InvalidNodeId)
		{
			FVector2D NodePos = pClonedGraph->GetNode(NodeId)->GetPosition();
			float Cost = FVector2D::Distance(EndSnapped, NodePos);

			auto Conn = std::make_unique<Connection>(EndNodeId, NodeId);
			Conn->SetWeight(Cost);
			pClonedGraph->AddConnection(std::move(Conn));
		}
	}

	// Run A* on the cloned graph
	AStar Pathfinder{ pClonedGraph.get(), HeuristicFunctions::Euclidean };
	Node* pStart = pClonedGraph->GetNode(StartNodeId).get();
	Node* pEnd = pClonedGraph->GetNode(EndNodeId).get();

	std::vector<Node*> AStarPath = Pathfinder.FindPath(pStart, pEnd);

	
	for (Node* pNode : AStarPath)
	{
		finalPath.push_back(pNode->GetPosition());
		debugNodePositions.push_back(pNode->GetPosition());
	}

	// Optional: Run SSFA path smoothing (uncomment when ready)
	// debugPortals = SSFA::FindPortals(AStarPath, *pNavPoly);
	// finalPath = SSFA::OptimizePortals(debugPortals, *pNavPoly);

	return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}