#include "SpacePartitioning.h"

// Cell 
Cell::Cell(float Left, float Bottom, float Width, float Height)
{
	BoundingBox.Min = { Left, Bottom };
	BoundingBox.Max = { BoundingBox.Min.X + Width, BoundingBox.Min.Y + Height };
}

std::vector<FVector2D> Cell::GetRectPoints() const
{
	const float left = BoundingBox.Min.X;
	const float bottom = BoundingBox.Min.Y;
	const float width = BoundingBox.Max.X - BoundingBox.Min.X;
	const float height = BoundingBox.Max.Y - BoundingBox.Min.Y;

	std::vector<FVector2D> rectPoints =
	{
		{ left,         bottom         },
		{ left,         bottom + height },
		{ left + width, bottom + height },
		{ left + width, bottom         },
	};

	return rectPoints;
}

//Partitioned Space 
CellSpace::CellSpace(UWorld* pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities)
	: pWorld{ pWorld }
	, SpaceWidth{ Width }
	, SpaceHeight{ Height }
	, NrOfRows{ Rows }
	, NrOfCols{ Cols }
	, NrOfNeighbors{ 0 }
{
	Neighbors.SetNum(MaxEntities);

	CellWidth = Width / Cols;
	CellHeight = Height / Rows;

	// Origin
	CellOrigin = { -Width * 0.5f, -Height * 0.5f };

	// Create all cells
	Cells.reserve(Rows * Cols);
	for (int row = 0; row < Rows; ++row)
	{
		for (int col = 0; col < Cols; ++col)
		{
			float left = CellOrigin.X + col * CellWidth;
			float bottom = CellOrigin.Y + row * CellHeight;
			Cells.emplace_back(left, bottom, CellWidth, CellHeight);
		}
	}
}

void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	int Index = PositionToIndex(Agent.GetPosition());
	if (Index >= 0 && Index < static_cast<int>(Cells.size()))
		Cells[Index].Agents.push_back(&Agent);
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	int OldIndex = PositionToIndex(OldPos);
	int NewIndex = PositionToIndex(Agent.GetPosition());

	if (OldIndex != NewIndex)
	{
		// Remove from old cell
		if (OldIndex >= 0 && OldIndex < static_cast<int>(Cells.size()))
			Cells[OldIndex].Agents.remove(&Agent);

		// Add to new cell
		if (NewIndex >= 0 && NewIndex < static_cast<int>(Cells.size()))
			Cells[NewIndex].Agents.push_back(&Agent);
	}
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	NrOfNeighbors = 0;

	// query rect around the agent
	FVector2D AgentPos = Agent.GetPosition();
	FRect QueryRect;
	QueryRect.Min = { AgentPos.X - QueryRadius, AgentPos.Y - QueryRadius };
	QueryRect.Max = { AgentPos.X + QueryRadius, AgentPos.Y + QueryRadius };

	// Check all cells that overlap with the query rect
	for (Cell& cell : Cells)
	{
		if (!DoRectsOverlap(QueryRect, cell.BoundingBox))
			continue;

		// Check each agent in this cell
		for (ASteeringAgent* pOther : cell.Agents)
		{
			if (!pOther || pOther == &Agent) continue;

			float Distance = FVector2D::Distance(AgentPos, pOther->GetPosition());
			if (Distance <= QueryRadius)
			{
				Neighbors[NrOfNeighbors] = pOther;
				++NrOfNeighbors;
			}
		}
	}
}

void CellSpace::EmptyCells()
{
	for (Cell& c : Cells)
		c.Agents.clear();
}

void CellSpace::RenderCells() const
{
	for (const Cell& cell : Cells)
	{
		// Draw cell border
		auto pts = cell.GetRectPoints();
		for (int i = 0; i < 4; ++i)
		{
			FVector2D A = pts[i];
			FVector2D B = pts[(i + 1) % 4];
			DrawDebugLine(
				pWorld,
				FVector{ A, 90.f },
				FVector{ B, 90.f },
				FColor::White, false, 0.f, 0, 1.f
			);
		}

		// Draw agent count in cell center
		int AgentCount = static_cast<int>(cell.Agents.size());
		if (AgentCount > 0)
		{
			FVector2D Center = (cell.BoundingBox.Min + cell.BoundingBox.Max) * 0.5f;
			DrawDebugString(
				pWorld,
				FVector{ Center, 100.f },
				FString::FromInt(AgentCount),
				nullptr,
				FColor::Yellow,
				0.f  
			);
		}
	}
}

int CellSpace::PositionToIndex(FVector2D const& Pos) const
{
	// Convert world position to grid coordinates
	int Col = static_cast<int>((Pos.X - CellOrigin.X) / CellWidth);
	int Row = static_cast<int>((Pos.Y - CellOrigin.Y) / CellHeight);

	// Clamp
	Col = FMath::Clamp(Col, 0, NrOfCols - 1);
	Row = FMath::Clamp(Row, 0, NrOfRows - 1);

	return Row * NrOfCols + Col;
}

bool CellSpace::DoRectsOverlap(FRect const& RectA, FRect const& RectB)
{
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;
	return true;
}