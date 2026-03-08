#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Movement/SteeringBehaviors/SpacePartitioning/SpacePartitioning.h"
#include "Shared/ImGuiHelpers.h"
#include "imgui.h"

Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{ pWorld }
	, FlockSize{ FlockSize }
	, WorldSize{ WorldSize }
	, pAgentToEvade{ pAgentToEvade }
{
	Agents.SetNum(FlockSize);
	Neighbors.SetNum(FlockSize);
	OldPositions.SetNum(FlockSize);
	NrOfNeighbors = 0;

	// Create spatial partitioning grid
	pPartitionedSpace = std::make_unique<CellSpace>(
		pWorld, WorldSize, WorldSize, NrOfCellsX, NrOfCellsX, FlockSize
	);

	// Spawn all agents
	for (int i = 0; i < FlockSize; ++i)
	{
		FVector SpawnPos{
			FMath::RandRange(-WorldSize * 0.5f, WorldSize * 0.5f),
			FMath::RandRange(-WorldSize * 0.5f, WorldSize * 0.5f),
			90.f
		};

		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

		Agents[i] = pWorld->SpawnActor<ASteeringAgent>(AgentClass, SpawnPos, FRotator::ZeroRotator, SpawnParams);

		if (Agents[i])
		{
			Agents[i]->SetDebugRenderingEnabled(false);
			// Disable auto tick
			Agents[i]->PrimaryActorTick.bCanEverTick = false;

			OldPositions[i] = Agents[i]->GetPosition();
			pPartitionedSpace->AddAgent(*Agents[i]);
		}
	}

	// Create flocking behaviors
	pCohesionBehavior = std::make_unique<Cohesion>(this);
	pSeparationBehavior = std::make_unique<Separation>(this);
	pVelMatchBehavior = std::make_unique<VelocityMatch>(this);
	pSeekBehavior = std::make_unique<Seek>();
	pWanderBehavior = std::make_unique<Wander>();

	if (pAgentToEvade)
		pEvadeBehavior = std::make_unique<Evade>();

	pBlendedSteering = std::make_unique<BlendedSteering>(
		std::vector<BlendedSteering::WeightedBehavior>{
			{ pCohesionBehavior.get(), 0.3f },
			{ pSeparationBehavior.get(), 0.4f },
			{ pVelMatchBehavior.get(),   0.2f },
			{ pSeekBehavior.get(),       0.5f },
			{ pWanderBehavior.get(),     0.2f }
	}
	);

	if (pEvadeBehavior)
	{
		pPrioritySteering = std::make_unique<PrioritySteering>(
			std::vector<ISteeringBehavior*>{
			pEvadeBehavior.get(),
				pBlendedSteering.get()
		}
		);
	}
}

Flock::~Flock()
{
	for (ASteeringAgent* Agent : Agents)
	{
		if (Agent)
			Agent->Destroy();
	}
}

void Flock::Tick(float DeltaTime)
{
	// Update evade target
	if (pAgentToEvade && pEvadeBehavior)
	{
		FTargetData EvadeTarget;
		EvadeTarget.Position = pAgentToEvade->GetPosition();
		EvadeTarget.LinearVelocity = pAgentToEvade->GetLinearVelocity();
		EvadeTarget.Orientation = pAgentToEvade->GetRotation();
		pEvadeBehavior->SetTarget(EvadeTarget);
	}

	for (int i = 0; i < Agents.Num(); ++i)
	{
		ASteeringAgent* Agent = Agents[i];
		if (!Agent) continue;

		// Store old position
		FVector2D OldPos = OldPositions[i];

		
		RegisterNeighbors(Agent);

		// Manually tick
		ISteeringBehavior* Behavior = pPrioritySteering
			? static_cast<ISteeringBehavior*>(pPrioritySteering.get())
			: static_cast<ISteeringBehavior*>(pBlendedSteering.get());

		SteeringOutput Output = Behavior->CalculateSteering(DeltaTime, *Agent);
		Agent->AddMovementInput(FVector{ Output.LinearVelocity, 0.f });

		// Update old position and cell
		FVector2D NewPos = Agent->GetPosition();
		if (bUseSpacePartitioning)
			pPartitionedSpace->UpdateAgentCell(*Agent, OldPos);

		OldPositions[i] = NewPos;
	}
}

void Flock::RenderDebug()
{
	if (DebugRenderNeighborhood)
		RenderNeighborhood();

	if (DebugRenderPartitions && bUseSpacePartitioning && pPartitionedSpace)
		pPartitionedSpace->RenderCells();
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
	{
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Game AI", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place seek target");
		ImGui::Text("WASD: move cam");
		ImGui::Text("Scrollwheel: zoom cam");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();

		ImGui::Checkbox("Debug Steering", &DebugRenderSteering);
		ImGui::Checkbox("Debug Neighborhood", &DebugRenderNeighborhood);
		ImGui::Checkbox("Debug Partitions", &DebugRenderPartitions);

		// Spatial partitioning toggle
		bool bPartitioningCopy = bUseSpacePartitioning;
		if (ImGui::Checkbox("Spatial Partitioning", &bPartitioningCopy))
			bUseSpacePartitioning = bPartitioningCopy;

		ImGui::Spacing();
		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		if (pBlendedSteering)
		{
			auto& Behaviors = pBlendedSteering->GetWeightedBehaviorsRef();

			auto SliderForBehavior = [&](const char* Label, int Index)
				{
					float Val = Behaviors[Index].Weight;
					if (ImGui::SliderFloat(Label, &Val, 0.f, 1.f, "%.2f"))
						Behaviors[Index].Weight = Val;
				};

			SliderForBehavior("Cohesion", 0);
			SliderForBehavior("Separation", 1);
			SliderForBehavior("Alignment", 2);
			SliderForBehavior("Seek", 3);
			SliderForBehavior("Wander", 4);
		}

		ImGui::End();
	}
#endif
}

void Flock::RenderNeighborhood()
{
	if (Agents.Num() == 0 || !Agents[0]) return;

	ASteeringAgent* FirstAgent = Agents[0];
	RegisterNeighbors(FirstAgent);

	// Draw neighborhood radius
	DrawDebugCircle(
		pWorld, FirstAgent->GetActorLocation(),
		NeighborhoodRadius, 32, FColor::Yellow, false, -1.f, 0, 2.f,
		FVector(0, 1, 0), FVector(1, 0, 0)
	);

	// Draw query bounding box when using partitioning
	if (bUseSpacePartitioning)
	{
		FVector2D Pos = FirstAgent->GetPosition();
		FVector BoxMin{ Pos.X - NeighborhoodRadius, Pos.Y - NeighborhoodRadius, 85.f };
		FVector BoxMax{ Pos.X + NeighborhoodRadius, Pos.Y + NeighborhoodRadius, 95.f };
		DrawDebugBox(pWorld, (BoxMin + BoxMax) * 0.5f, (BoxMax - BoxMin) * 0.5f, FColor::Orange, false, -1.f, 0, 2.f);
	}

	// Highlight neighbors
	for (int i = 0; i < GetNrOfNeighbors(); ++i)
	{
		if (GetNeighbors()[i])
		{
			DrawDebugSphere(
				pWorld, GetNeighbors()[i]->GetActorLocation(),
				20.f, 8, FColor::Yellow, false, -1.f, 0, 2.f
			);
		}
	}
}

void Flock::RegisterNeighbors(ASteeringAgent* const Agent)
{
	if (bUseSpacePartitioning && pPartitionedSpace)
	{
		pPartitionedSpace->RegisterNeighbors(*Agent, NeighborhoodRadius);
	}
	else
	{
		// force memory pool approach
		NrOfNeighbors = 0;
		for (ASteeringAgent* Other : Agents)
		{
			if (!Other || Other == Agent) continue;

			float Distance = FVector2D::Distance(Agent->GetPosition(), Other->GetPosition());
			if (Distance <= NeighborhoodRadius)
			{
				Neighbors[NrOfNeighbors] = Other;
				++NrOfNeighbors;
			}
		}
	}
}

int Flock::GetNrOfNeighbors() const
{
	if (bUseSpacePartitioning && pPartitionedSpace)
		return pPartitionedSpace->GetNrOfNeighbors();
	return NrOfNeighbors;
}

const TArray<ASteeringAgent*>& Flock::GetNeighbors() const
{
	if (bUseSpacePartitioning && pPartitionedSpace)
		return pPartitionedSpace->GetNeighbors();
	return Neighbors;
}

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D AvgPosition = FVector2D::ZeroVector;
	int Count = GetNrOfNeighbors();
	if (Count == 0) return AvgPosition;

	const TArray<ASteeringAgent*>& N = GetNeighbors();
	for (int i = 0; i < Count; ++i)
		if (N[i]) AvgPosition += N[i]->GetPosition();

	return AvgPosition / static_cast<float>(Count);
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D AvgVelocity = FVector2D::ZeroVector;
	int Count = GetNrOfNeighbors();
	if (Count == 0) return AvgVelocity;

	const TArray<ASteeringAgent*>& N = GetNeighbors();
	for (int i = 0; i < Count; ++i)
		if (N[i]) AvgVelocity += N[i]->GetLinearVelocity();

	return AvgVelocity / static_cast<float>(Count);
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	if (pSeekBehavior)
		pSeekBehavior->SetTarget(Target);
}