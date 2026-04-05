#pragma once
#include <vector>

#include "NavGraphPathfinding.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

namespace GameAI
{
	class SSFA final
	{
	public:
		static std::vector<NavLine> FindPortals(std::vector<Node*> const& Path, TriPolygon const& NavPoly)
		{
			std::vector<NavLine> Portals = {};
			if (Path.size() < 2) return Portals;

			// First degenerate portal = start position
			FVector2D StartPos = Path.front()->GetPosition();
			Portals.push_back({ StartPos, StartPos });

			// Middle nodes = portal edges
			for (int i = 1; i < static_cast<int>(Path.size()) - 1; ++i)
			{
				NavGraphNode* pNavNode = dynamic_cast<NavGraphNode*>(Path[i]);
				if (!pNavNode) continue;

				int EdgeIdx = pNavNode->GetEdgeIdx();
				if (EdgeIdx < 0) continue;

				auto const& Edges = NavPoly.GetEdges();
				if (EdgeIdx >= static_cast<int>(Edges.size())) continue;

				auto const& Edge = Edges[EdgeIdx];
				FVector2D P1 = FVector2D{ Edge.GetP1(NavPoly) };
				FVector2D P2 = FVector2D{ Edge.GetP2(NavPoly) };

				// Use direction from previous node toward next node for stability
				FVector2D PrevPos = Path[i - 1]->GetPosition();
				FVector2D NextPos = (i + 1 < static_cast<int>(Path.size()))
					? Path[i + 1]->GetPosition()
					: Path[i]->GetPosition();
				FVector2D PathDir = NextPos - PrevPos;

				FVector2D ToP1 = P1 - PrevPos;
				float CrossP1 = PathDir.X * ToP1.Y - PathDir.Y * ToP1.X;

				if (CrossP1 < 0)
					Portals.push_back({ P1, P2 }); // P1 is right, P2 is left
				else
					Portals.push_back({ P2, P1 }); // P2 is right, P1 is left
			}

			// Last degenerate portal = end position
			FVector2D EndPos = Path.back()->GetPosition();
			Portals.push_back({ EndPos, EndPos });

			return Portals;
		}

		static std::vector<FVector2D> OptimizePortals(std::vector<NavLine> const& Portals, TriPolygon const& NavPoly)
		{
			std::vector<FVector2D> Path{};
			if (Portals.size() < 2)
			{
				if (!Portals.empty()) Path.push_back(Portals[0].P1);
				return Path;
			}

			int AmtPortals = static_cast<int>(Portals.size());
			FVector2D Apex = Portals[0].P1;
			FVector2D RightLeg = Portals[1].P1 - Apex;
			FVector2D LeftLeg = Portals[1].P2 - Apex;
			int RightLegIdx = 1;
			int LeftLegIdx = 1;

			Path.push_back(Apex);

			int PortalIdx = 2;
			while (PortalIdx < AmtPortals)
			{
				FVector2D NewRight = Portals[PortalIdx].P1 - Apex;
				FVector2D NewLeft = Portals[PortalIdx].P2 - Apex;

				
				if (Cross2D(RightLeg, NewRight) >= 0.f)
				{
					if (RightLeg.IsNearlyZero() || Cross2D(LeftLeg, NewRight) > 0.f)
					{
						// Cross over left - left becomes new apex
						Apex = Apex + LeftLeg;
						Path.push_back(Apex);

						// Restart scanning from portal after the left leg
						PortalIdx = LeftLegIdx + 1;
						RightLegIdx = PortalIdx;
						LeftLegIdx = PortalIdx;

						if (PortalIdx < AmtPortals)
						{
							RightLeg = Portals[PortalIdx].P1 - Apex;
							LeftLeg = Portals[PortalIdx].P2 - Apex;
						}
						continue;
					}
					RightLeg = NewRight;
					RightLegIdx = PortalIdx;
				}

				
				if (Cross2D(LeftLeg, NewLeft) <= 0.f)
				{
					if (LeftLeg.IsNearlyZero() || Cross2D(RightLeg, NewLeft) < 0.f)
					{
						// Cross over right - right becomes new apex
						Apex = Apex + RightLeg;
						Path.push_back(Apex);

						// Restart scanning from portal after the right leg
						PortalIdx = RightLegIdx + 1;
						LeftLegIdx = PortalIdx;
						RightLegIdx = PortalIdx;

						if (PortalIdx < AmtPortals)
						{
							RightLeg = Portals[PortalIdx].P1 - Apex;
							LeftLeg = Portals[PortalIdx].P2 - Apex;
						}
						continue;
					}
					LeftLeg = NewLeft;
					LeftLegIdx = PortalIdx;
				}

				++PortalIdx;
			}

			// Add end point
			Path.push_back(Portals.back().P1);
			return Path;
		}

	private:
		static float Cross2D(FVector2D const& A, FVector2D const& B)
		{
			return A.X * B.Y - A.Y * B.X;
		}
		SSFA() {};
		~SSFA() {};
	};
}