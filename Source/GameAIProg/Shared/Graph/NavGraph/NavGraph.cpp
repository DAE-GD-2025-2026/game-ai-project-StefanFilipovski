#include "NavGraph.h"

#include "NavGraphNode.h"

GameAI::NavGraph::NavGraph(std::unique_ptr<TriPolygon>&& NavPoly)
	: Graph{ false }
	, pNavPoly{ std::move(NavPoly) }
{
	CreateNavigationGraph();
}

GameAI::NavGraph::NavGraph(const NavGraph& Other)
	: Graph(false)
{
	Nodes.reserve(Other.Nodes.size());
	for (std::unique_ptr<Node> const& OtherNode : Other.Nodes)
	{
		Nodes.push_back(std::make_unique<NavGraphNode>(*dynamic_cast<NavGraphNode*>(OtherNode.get())));
	}

	Connections.reserve(Other.Connections.size());
	for (std::unique_ptr<Connection> const& OtherConnection : Other.Connections)
	{
		Connections.push_back(std::make_unique<Connection>(*OtherConnection.get()));
	}
}

std::unique_ptr<GameAI::NavGraph> GameAI::NavGraph::Clone() const
{
	return std::make_unique<NavGraph>(*this);
}

int GameAI::NavGraph::GetNodeIdFromEdgeIndex(int EdgeIdx) const
{
	if (EdgeIdx >= 0)
	{
		for (auto const& pNode : Nodes)
		{
			if (reinterpret_cast<NavGraphNode*>(pNode.get())->GetEdgeIdx() == EdgeIdx)
			{
				return pNode->GetId();
			}
		}
	}

	return Graphs::InvalidNodeId;
}

void GameAI::NavGraph::CreateNavigationGraph()
{
	auto const& Edges = pNavPoly->GetEdges();
	auto const& Triangles = pNavPoly->GetTriangles();

	// 1. Loop over all edges — create a node at the midpoint of each edge
	//    that is shared by more than one triangle (i.e. it connects two triangles)
	for (int EdgeIdx = 0; EdgeIdx < static_cast<int>(Edges.size()); ++EdgeIdx)
	{
		auto const& Edge = Edges[EdgeIdx];

		// Check how many triangles share this edge
		int TriangleCount = 0;
		for (auto const& Tri : Triangles)
		{
			if (Tri.HasEdge(Edge))
				++TriangleCount;
		}

		// Only create a node for edges shared between two triangles
		if (TriangleCount >= 2)
		{
			FVector P1 = Edge.GetP1(*pNavPoly);
			FVector P2 = Edge.GetP2(*pNavPoly);
			FVector2D MidPoint = FVector2D{ (P1.X + P2.X) * 0.5f, (P1.Y + P2.Y) * 0.5f };

			auto NewNode = std::make_unique<NavGraphNode>(MidPoint, EdgeIdx);
			AddNode(std::move(NewNode));
		}
	}

	// 2. Create connections — for each triangle, find valid nodes and connect them
	for (auto const& Triangle : Triangles)
	{
		auto TriEdges = Triangle.GetEdges();

		// Collect valid node IDs for this triangle's edges
		std::vector<int> ValidNodeIds{};
		for (auto const& TriEdge : TriEdges)
		{
			int EdgeIdx = pNavPoly->FindEdgeIndex(TriEdge).value_or(-1);
			int NodeId = GetNodeIdFromEdgeIndex(EdgeIdx);
			if (NodeId != Graphs::InvalidNodeId)
				ValidNodeIds.push_back(NodeId);
		}

		// 2 valid nodes -> 1 connection
		// 3 valid nodes -> 3 connections
		for (int i = 0; i < static_cast<int>(ValidNodeIds.size()); ++i)
		{
			for (int j = i + 1; j < static_cast<int>(ValidNodeIds.size()); ++j)
			{
				// Avoid duplicate connections (Graph handles undirected so just check one direction)
				if (!FindConnection(ValidNodeIds[i], ValidNodeIds[j]))
					AddConnection(ValidNodeIds[i], ValidNodeIds[j]);
			}
		}
	}

	// 3. Set connection costs to actual distances
	SetConnectionCostsToDistances();
}