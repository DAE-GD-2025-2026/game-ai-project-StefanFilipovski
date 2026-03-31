#include "AStar.h"

using namespace GameAI;

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph)
	, HeuristicFunction(hFunction)
{
}

std::vector<Node*> AStar::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
	std::vector<Node*> path{};

	if (!pStartNode || !pGoalNode)
		return path;

	std::vector<NodeRecord> openList{};
	std::vector<NodeRecord> closedList{};

	// 1. Create start record and add to open list
	NodeRecord startRecord{};
	startRecord.pNode = pStartNode;
	startRecord.pConnection = nullptr;
	startRecord.costSoFar = 0.f;
	startRecord.estimatedTotalCost = GetHeuristicCost(pStartNode, pGoalNode);
	openList.push_back(startRecord);

	NodeRecord currentRecord{};

	// 2. While loop
	while (!openList.empty())
	{
		// A. Get record with lowest F-score
		currentRecord = *std::min_element(openList.begin(), openList.end());

		// B. Check if this is the goal node
		if (currentRecord.pNode == pGoalNode)
			break;

		// C. Get all connections from current node
		auto Connections = pGraph->FindConnectionsFrom(currentRecord.pNode->GetId());

		for (Connection* pConnection : Connections)
		{
			Node* pNextNode = pGraph->GetNode(pConnection->GetToId()).get();
			if (!pNextNode || pNextNode->GetId() == Graphs::InvalidNodeId)
				continue;

			float NewGCost = currentRecord.costSoFar + pConnection->GetWeight();

			// D. Check if node is on closed list
			auto ClosedIt = std::find_if(closedList.begin(), closedList.end(),
				[pNextNode](const NodeRecord& r) { return r.pNode == pNextNode; });

			if (ClosedIt != closedList.end())
			{
				// If existing record is cheaper or equal, skip
				if (ClosedIt->costSoFar <= NewGCost)
					continue;
			
				closedList.erase(ClosedIt);
			}

			// E. Check if node is on open list
			auto OpenIt = std::find_if(openList.begin(), openList.end(),
				[pNextNode](const NodeRecord& r) { return r.pNode == pNextNode; });

			if (OpenIt != openList.end())
			{
				// If existing record is cheaper or equal, skip
				if (OpenIt->costSoFar <= NewGCost)
					continue;
			
				openList.erase(OpenIt);
			}

			// F. Create new record and add to open list
			NodeRecord newRecord{};
			newRecord.pNode = pNextNode;
			newRecord.pConnection = pConnection;
			newRecord.costSoFar = NewGCost;
			newRecord.estimatedTotalCost = NewGCost + GetHeuristicCost(pNextNode, pGoalNode);
			openList.push_back(newRecord);
		}

		// G. Move current record from open to closed list
		openList.erase(std::remove(openList.begin(), openList.end(), currentRecord), openList.end());
		closedList.push_back(currentRecord);
	}

	// Check if we actually found the goal
	if (currentRecord.pNode != pGoalNode)
		return path;

	// 3. Backtrack to reconstruct path
	while (currentRecord.pNode != pStartNode)
	{
		path.push_back(currentRecord.pNode);

		// Find the record in closed list whose node matches the connection's from node
		int FromId = currentRecord.pConnection->GetFromId();
		auto It = std::find_if(closedList.begin(), closedList.end(),
			[FromId](const NodeRecord& r) { return r.pNode->GetId() == FromId; });

		if (It == closedList.end())
			break;

		currentRecord = *It;
	}

	// Add start node
	path.push_back(pStartNode);

	// Reverse to get path from start to goal
	std::reverse(path.begin(), path.end());
	return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
	FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition() - pGraph->GetNode(pStartNode->GetId())->GetPosition();
	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}