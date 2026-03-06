#include "Level_CombinedSteering.h"
#include "imgui.h"

ALevel_CombinedSteering::ALevel_CombinedSteering()
{
	PrimaryActorTick.bCanEverTick = true;
}

void ALevel_CombinedSteering::BeginPlay()
{
	Super::BeginPlay();

	// DRUNK AGENT (BlendedSteering: 50% Seek + 50% Wander)
	pDrunkAgent = GetWorld()->SpawnActor<ASteeringAgent>(
		SteeringAgentClass, FVector{ -200.f, 0.f, 90.f }, FRotator::ZeroRotator);

	if (pDrunkAgent)
	{
		pSeek = std::make_unique<Seek>();
		pWander = std::make_unique<Wander>();

		pBlendedSteering = std::make_unique<BlendedSteering>(
			std::vector<BlendedSteering::WeightedBehavior>{
				{ pSeek.get(), 0.5f },
				{ pWander.get(), 0.5f }
		}
		);

		// Set initial seek target to mouse
		pSeek->SetTarget(MouseTarget);
		pDrunkAgent->SetSteeringBehavior(pBlendedSteering.get());
		pDrunkAgent->SetDebugRenderingEnabled(true);
	}

	// EVADING AGENT (PrioritySteering: Evade DrunkAgent to Wander)
	pEvadingAgent = GetWorld()->SpawnActor<ASteeringAgent>(
		SteeringAgentClass, FVector{ 200.f, 0.f, 90.f }, FRotator::ZeroRotator);

	if (pEvadingAgent)
	{
		pEvade = std::make_unique<Evade>();
		pEvadeWander = std::make_unique<Wander>();

		pPrioritySteering = std::make_unique<PrioritySteering>(
			std::vector<ISteeringBehavior*>{
			pEvade.get(),
				pEvadeWander.get()
		}
		);

		pEvadingAgent->SetSteeringBehavior(pPrioritySteering.get());
		pEvadingAgent->SetDebugRenderingEnabled(true);
	}
}

void ALevel_CombinedSteering::BeginDestroy()
{
	Super::BeginDestroy();
}

void ALevel_CombinedSteering::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

#pragma region UI
	{
		bool windowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Game AI", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target (DrunkAgent seek)");
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

		ImGui::Text("Combined Steering");
		ImGui::Spacing();

		if (ImGui::Checkbox("Debug Rendering", &CanDebugRender))
		{
			if (pDrunkAgent)   pDrunkAgent->SetDebugRenderingEnabled(CanDebugRender);
			if (pEvadingAgent) pEvadingAgent->SetDebugRenderingEnabled(CanDebugRender);
		}

		ImGui::Spacing();
		ImGui::Text("DrunkAgent Weights");

		if (pBlendedSteering)
		{
			float seekWeight = *pBlendedSteering->GetWeight(pSeek.get());
			float wanderWeight = *pBlendedSteering->GetWeight(pWander.get());

			if (ImGui::SliderFloat("Seek##drunk", &seekWeight, 0.f, 1.f, "%.2f"))
				*pBlendedSteering->GetWeight(pSeek.get()) = seekWeight;

			if (ImGui::SliderFloat("Wander##drunk", &wanderWeight, 0.f, 1.f, "%.2f"))
				*pBlendedSteering->GetWeight(pWander.get()) = wanderWeight;
		}

		ImGui::Spacing();
		ImGui::Text("EvadingAgent");
		ImGui::Text("Evade Radius: %.0f", EvadeRadius);
		ImGui::SliderFloat("##evaderadius", &EvadeRadius, 50.f, 600.f, "%.0f");

		ImGui::End();
	}
#pragma endregion

	// Update DrunkAgent seek target to mouse
	if (pDrunkAgent && pSeek)
		pSeek->SetTarget(MouseTarget);

	// Update EvadingAgents evade target to DrunkAgent
	if (pEvadingAgent && pEvade && pDrunkAgent)
	{
		FTargetData EvadeTarget;
		EvadeTarget.Position = pDrunkAgent->GetPosition();
		EvadeTarget.LinearVelocity = pDrunkAgent->GetLinearVelocity();
		EvadeTarget.Orientation = pDrunkAgent->GetRotation();
		pEvade->SetTarget(EvadeTarget);

		// Draw evade radius debug circle
		if (CanDebugRender)
		{
			DrawDebugCircle(
				GetWorld(),
				pDrunkAgent->GetActorLocation(),
				EvadeRadius, 32, FColor::Red, false, -1.f, 0, 2.f,
				FVector(0, 1, 0), FVector(1, 0, 0)
			);
		}
	}
}