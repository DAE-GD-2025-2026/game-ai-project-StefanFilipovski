// Fill out your copyright notice in the Description page of Project Settings.

#include "Level_GraphTheory.h"

#include "Algorithms/EulerianPath.h"
#include "Shared/GameAISpectator.h"

using namespace GameAI;

// Sets default values
ALevel_GraphTheory::ALevel_GraphTheory()
{
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_GraphTheory::BeginPlay()
{
	Super::BeginPlay();

	// Add the graph editor to our player
	if (PlayerController = Cast<APlayerController>(GetWorld()->GetFirstLocalPlayerFromController()->PlayerController);
		GraphEditorClass && PlayerController)
	{
		PlayerGraphEditor = NewObject<UGraphEditorComponent>(PlayerController->GetPawn(), GraphEditorClass);
		PlayerGraphEditor->RegisterComponent();
		PlayerGraphEditor->SetEditedGraph(&Graph);
		PlayerGraphEditor->SetNodeFactory(&NodeFactory);
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Unable to get PlayerController from LocalPlayer or GraphEditorClass is null"))
			return;
	}

	// Make the view orthogonal for less perspective issues
	if (AGameAISpectator* Player = Cast<AGameAISpectator>(PlayerController->GetPawnOrSpectator()); Player)
	{
		Player->SetCameraProjection(ECameraProjectionMode::Orthographic);
	}

	// Initialize the renderer with the world
	Renderer = GraphRenderer(GetWorld());

	// Create a default graph with a few connected nodes to start with
	Graph.AddNode(NodeFactory.CreateNode(FVector2D{ -300.f,    0.f }));
	Graph.AddNode(NodeFactory.CreateNode(FVector2D{ 0.f,  300.f }));
	Graph.AddNode(NodeFactory.CreateNode(FVector2D{ 300.f,    0.f }));
	Graph.AddNode(NodeFactory.CreateNode(FVector2D{ 0.f, -300.f }));

	Graph.AddConnection(0, 1);
	Graph.AddConnection(1, 2);
	Graph.AddConnection(2, 3);
	Graph.AddConnection(3, 0);
	Graph.AddConnection(0, 2);

	// Spawn the Agent
	Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass,
		FVector{ 0, 0, 90 }, FRotator::ZeroRotator);
	Agent->SetSteeringBehavior(&PathFollow);

	// Run initial path
	EulerianPath EulerPath(&Graph);
	Eulerianity Eulerianity{};
	auto Trail = EulerPath.FindPath(Eulerianity);
	UpdateAgentPath(Trail);
}

void ALevel_GraphTheory::BeginDestroy()
{
	Super::BeginDestroy();
}

void ALevel_GraphTheory::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

#pragma region UI
	{
		bool windowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
		ImGui::SetWindowFocus();
		ImGui::PushItemWidth(70);

		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: Create node");
		ImGui::Text("Hover node + LMB: Create connection");
		ImGui::Text("Hover node + RMB: Delete node");
		ImGui::Text("Hover node + MMB: Move node");
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

		ImGui::Text("Graph Theory");
		ImGui::Spacing();

		// Show Eulerianity status
		EulerianPath EulerPath(&Graph);
		Eulerianity CurrentEulerianity = EulerPath.IsEulerian();
		switch (CurrentEulerianity)
		{
		case Eulerianity::eulerian:
			ImGui::TextColored(ImVec4(0, 1, 0, 1), "Graph is Eulerian");
			break;
		case Eulerianity::semiEulerian:
			ImGui::TextColored(ImVec4(1, 1, 0, 1), "Graph is Semi-Eulerian");
			break;
		case Eulerianity::notEulerian:
			ImGui::TextColored(ImVec4(1, 0, 0, 1), "Graph is NOT Eulerian");
			break;
		}

		ImGui::End();
	}
#pragma endregion UI


	Renderer.RenderGraph(Graph);

	// Check if the graph has been updated by the editor
	if (PlayerGraphEditor && PlayerGraphEditor->HasGraphUpdated())
	{
		// Run eulerian path algorithm on updated graph
		EulerianPath EulerPath(&Graph);
		Eulerianity CurrentEulerianity{};
		std::vector<Node*> Trail = EulerPath.FindPath(CurrentEulerianity);

		// If a valid path was found, update the agents path
		if (!Trail.empty())
		{
			UpdateAgentPath(Trail);
		}
	}
}

void ALevel_GraphTheory::UpdateAgentPath(std::vector<Node*> const& Trail)
{
	std::vector<FVector2D> path{};

	for (Node* pNode : Trail)
	{
		if (pNode)
			path.push_back(pNode->GetPosition());
	}

	if (path.size() > 0)
	{
		Agent->SetPosition(path[0]);
		PathFollow.SetPath(path);  // SetPath resets currentPathIndex
	}
}