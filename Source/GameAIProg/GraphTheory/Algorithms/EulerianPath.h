#pragma once
#include <stack>
#include "Shared/Graph/Graph.h"

namespace GameAI
{
	enum class Eulerianity
	{
		notEulerian,
		semiEulerian,
		eulerian,
	};

	class EulerianPath final
	{
	public:
		EulerianPath(Graph* const pGraph);

		Eulerianity IsEulerian() const;
		std::vector<Node*> FindPath(Eulerianity& eulerianity) const;

	private:
		void VisitAllNodesDFS(const std::vector<Node*>& pNodes, std::vector<bool>& visited, int startIndex) const;
		bool IsConnected() const;

		Graph* m_pGraph;
	};

	inline EulerianPath::EulerianPath(Graph* const pGraph)
		: m_pGraph(pGraph)
	{
	}

	inline Eulerianity EulerianPath::IsEulerian() const
	{
		// If the graph is not connected there can be no Eulerian Trail
		if (!IsConnected())
			return Eulerianity::notEulerian;

		// Count nodes with odd degree
		int OddDegreeCount = 0;
		for (Node* pNode : m_pGraph->GetActiveNodes())
		{
			int Degree = static_cast<int>(m_pGraph->FindConnectionsFrom(pNode->GetId()).size());
			if (Degree % 2 != 0)
				++OddDegreeCount;
		}

		if (OddDegreeCount > 2)
			return Eulerianity::notEulerian;

		
		if (OddDegreeCount == 2)
			return Eulerianity::semiEulerian;

		// No odd degree nodes  is eulerian
		return Eulerianity::eulerian;
	}

	inline std::vector<Node*> EulerianPath::FindPath(Eulerianity& eulerianity) const
	{
		// Get a copy of the graph because this algorithm involves removing edges
		Graph graphCopy = m_pGraph->Clone();
		std::vector<Node*> Path = {};
		std::vector<Node*> Nodes = graphCopy.GetActiveNodes();
		int currentNodeId{ Graphs::InvalidNodeId };

		// Check if there can be an euler path
		eulerianity = IsEulerian();
		if (eulerianity == Eulerianity::notEulerian)
			return Path;

		
		if (eulerianity == Eulerianity::eulerian)
		{
			currentNodeId = Nodes[0]->GetId();
		}
		else // semi eulerian
		{
			for (Node* pNode : Nodes)
			{
				int Degree = static_cast<int>(graphCopy.FindConnectionsFrom(pNode->GetId()).size());
				if (Degree % 2 != 0)
				{
					currentNodeId = pNode->GetId();
					break;
				}
			}
		}

		// Hierholzers algorithm
		std::stack<int> nodeStack;
		nodeStack.push(currentNodeId);

		while (!nodeStack.empty())
		{
			
			auto Connections = graphCopy.FindConnectionsFrom(currentNodeId);

			if (!Connections.empty())
			{
				// Push current node onto stack
				nodeStack.push(currentNodeId);

				// Take any neighbor
				int NextNodeId = Connections[0]->GetToId();

				// Remove the edge between current and next (from the copy)
				graphCopy.RemoveConnection(currentNodeId, NextNodeId);

				// Move to next node
				currentNodeId = NextNodeId;
			}
			else
			{
				
				Path.push_back(m_pGraph->GetNode(currentNodeId).get());

				currentNodeId = nodeStack.top();
				nodeStack.pop();
			}
		}

	
		std::reverse(Path.begin(), Path.end());
		return Path;
	}

	inline void EulerianPath::VisitAllNodesDFS(const std::vector<Node*>& Nodes, std::vector<bool>& visited, int startIndex) const
	{
		// Mark the current node as visited
		visited[startIndex] = true;

		// Get all connections from this node
		int NodeId = Nodes[startIndex]->GetId();
		auto Connections = m_pGraph->FindConnectionsFrom(NodeId);

		for (auto* Connection : Connections)
		{
			int NeighborId = Connection->GetToId();

			// Find the index of the neighbor in the Nodes vector
			for (int i = 0; i < static_cast<int>(Nodes.size()); ++i)
			{
				if (Nodes[i]->GetId() == NeighborId && !visited[i])
				{
					VisitAllNodesDFS(Nodes, visited, i);
					break;
				}
			}
		}
	}

	inline bool EulerianPath::IsConnected() const
	{
		std::vector<Node*> Nodes = m_pGraph->GetActiveNodes();
		if (Nodes.size() == 0)
			return false;

		
		int StartIndex = 0;

		// Track visited nodes
		std::vector<bool> Visited(Nodes.size(), false);

		// Start DFS from the first node
		VisitAllNodesDFS(Nodes, Visited, StartIndex);

		// If any node was never visited, the graph is not connected
		for (bool bVisited : Visited)
		{
			if (!bVisited)
				return false;
		}

		return true;
	}
}