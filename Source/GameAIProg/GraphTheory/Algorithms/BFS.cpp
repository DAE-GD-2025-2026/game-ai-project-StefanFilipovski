#include "BFS.h"

#include <map>
#include <queue>

#include "Shared/Graph/Graph.h"

using namespace GameAI;

BFS::BFS(Graph* const pGraph)
	: pGraph(pGraph)
{
}

std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
{
	std::vector<Node*> path;

	if (!pStartNode || !pDestinationNode)
		return path;

	// Queue for BFS 
	std::queue<Node*> openList;

	// Map to track visited nodes and their parent (for backtracking)
	std::map<Node*, Node*> parentMap;

	// Start from the start node
	openList.push(pStartNode);
	parentMap[pStartNode] = nullptr;

	bool bFound = false;

	while (!openList.empty())
	{
		Node* pCurrent = openList.front();
		openList.pop();

		// Check if we reached the destination
		if (pCurrent == pDestinationNode)
		{
			bFound = true;
			break;
		}

		// Get all connections from current node
		auto Connections = pGraph->FindConnectionsFrom(pCurrent->GetId());

		for (Connection* pConnection : Connections)
		{
			Node* pNeighbor = pGraph->GetNode(pConnection->GetToId()).get();
			if (!pNeighbor || pNeighbor->GetId() == Graphs::InvalidNodeId)
				continue;

			// Only visit unvisited nodes
			if (parentMap.find(pNeighbor) == parentMap.end())
			{
				parentMap[pNeighbor] = pCurrent;
				openList.push(pNeighbor);
			}
		}
	}

	if (!bFound)
		return path;

	// Backtrack from destination to start
	Node* pCurrent = pDestinationNode;
	while (pCurrent != nullptr)
	{
		path.push_back(pCurrent);
		pCurrent = parentMap[pCurrent];
	}

	// Reverse to get path from start to destination
	std::reverse(path.begin(), path.end());
	return path;
}