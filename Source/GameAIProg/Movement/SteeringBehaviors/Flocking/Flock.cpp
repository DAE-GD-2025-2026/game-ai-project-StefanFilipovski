#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
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
	, pAgentToEvade{ pAgentToEvade }
{
	Agents.SetNum(FlockSize);

	// pre-allocate neighbors array to max possible size
	Neighbors.SetNum(FlockSize);
	NrOfNeighbors = 0;

	// Spawn all agents
	for (int i = 0; i < FlockSize; ++i)
	{
		FVector SpawnPos{
			FMath::RandRange(-WorldSize * 0.5f, WorldSize * 0.5f),
			FMath::RandRange(-WorldSize * 0.5f, WorldSize * 0.5f),
			90.f
		};
		Agents[i] = pWorld->SpawnActor<ASteeringAgent>(AgentClass, SpawnPos, FRotator::ZeroRotator);
	}

	// Create flocking behaviors
	pCohesionBehavior = std::make_unique<Cohesion>(this);
	pSeparationBehavior = std::make_unique<Separation>(this);
	pVelMatchBehavior = std::make_unique<VelocityMatch>(this);
	pSeekBehavior = std::make_unique<Seek>();
	pWanderBehavior = std::make_unique<Wander>();

	// Create evade behavior if we have a target to evade
	if (pAgentToEvade)
		pEvadeBehavior = std::make_unique<Evade>();

	// Blended steering: Cohesion + Separation + Alignment + Seek + Wander
	pBlendedSteering = std::make_unique<BlendedSteering>(
		std::vector<BlendedSteering::WeightedBehavior>{
			{ pCohesionBehavior.get(), 0.3f },
			{ pSeparationBehavior.get(), 0.4f },
			{ pVelMatchBehavior.get(),   0.2f },
			{ pSeekBehavior.get(),       0.5f },
			{ pWanderBehavior.get(),     0.2f }
	}
	);

	// Evade first, then fall back to BlendedSteering
	if (pEvadeBehavior)
	{
		pPrioritySteering = std::make_unique<PrioritySteering>(
			std::vector<ISteeringBehavior*>{
			pEvadeBehavior.get(),
				pBlendedSteering.get()
		}
		);
	}

	// Assign steering to all agents
	for (ASteeringAgent* Agent : Agents)
	{
		if (Agent)
		{
			if (pPrioritySteering)
				Agent->SetSteeringBehavior(pPrioritySteering.get());
			else
				Agent->SetSteeringBehavior(pBlendedSteering.get());
		}
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
	// Update evade target if we have one
	if (pAgentToEvade && pEvadeBehavior)
	{
		FTargetData EvadeTarget;
		EvadeTarget.Position = pAgentToEvade->GetPosition();
		EvadeTarget.LinearVelocity = pAgentToEvade->GetLinearVelocity();
		EvadeTarget.Orientation = pAgentToEvade->GetRotation();
		pEvadeBehavior->SetTarget(EvadeTarget);
	}

	for (ASteeringAgent* Agent : Agents)
	{
		if (!Agent) continue;

		
		RegisterNeighbors(Agent);
			

		//Trim agent to world bounds
		FVector Pos = Agent->GetActorLocation();
		Pos.X = FMath::Clamp(Pos.X, -WorldSize * 0.5f, WorldSize * 0.5f);
		Pos.Y = FMath::Clamp(Pos.Y, -WorldSize * 0.5f, WorldSize * 0.5f);
		Agent->SetActorLocation(Pos);
	}
}

void Flock::RenderDebug()
{
	if (DebugRenderSteering || DebugRenderNeighborhood)
		RenderNeighborhood();
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
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

		ImGui::Spacing();
		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		// Sliders for blended steering weights
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
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
	// Debug render neighborhood for the first agent only
	if (Agents.Num() == 0 || !Agents[0]) return;

	ASteeringAgent* FirstAgent = Agents[0];

	// Draw neighborhood radius
	DrawDebugCircle(
		pWorld,
		FirstAgent->GetActorLocation(),
		NeighborhoodRadius, 32, FColor::Yellow, false, -1.f, 0, 2.f,
		FVector(0, 1, 0), FVector(1, 0, 0)
	);

	// Highlight neighbors
	RegisterNeighbors(FirstAgent);
	for (int i = 0; i < NrOfNeighbors; ++i)
	{
		if (Neighbors[i])
		{
			DrawDebugSphere(
				pWorld,
				Neighbors[i]->GetActorLocation(),
				20.f, 8, FColor::Yellow, false, -1.f, 0, 2.f
			);
		}
	}
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const Agent)
{
	// reset counter, overwrite slots 
	NrOfNeighbors = 0;

	for (ASteeringAgent* Other : Agents)
	{
		if (!Other || Other == Agent) continue;

		float Distance = FVector2D::Distance(Agent->GetPosition(), Other->GetPosition());
		if (Distance <= NeighborhoodRadius)
		{
			// Write into pre-allocated pool slot
			Neighbors[NrOfNeighbors] = Other;
			++NrOfNeighbors;
		}
	}
}
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D AvgPosition = FVector2D::ZeroVector;

	if (NrOfNeighbors == 0)
		return AvgPosition;

	for (int i = 0; i < NrOfNeighbors; ++i)
	{
		if (Neighbors[i])
			AvgPosition += Neighbors[i]->GetPosition();
	}

	return AvgPosition / static_cast<float>(NrOfNeighbors);
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D AvgVelocity = FVector2D::ZeroVector;

	if (NrOfNeighbors == 0)
		return AvgVelocity;

	for (int i = 0; i < NrOfNeighbors; ++i)
	{
		if (Neighbors[i])
			AvgVelocity += Neighbors[i]->GetLinearVelocity();
	}

	return AvgVelocity / static_cast<float>(NrOfNeighbors);
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	if (pSeekBehavior)
		pSeekBehavior->SetTarget(Target);
}