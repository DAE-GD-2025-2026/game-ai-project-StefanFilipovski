// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include <memory>              
#include "CombinedSteeringBehaviors.h"
#include "GameAIProg/Shared/Level_Base.h"
#include "GameAIProg/Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "Level_CombinedSteering.generated.h"

UCLASS()
class GAMEAIPROG_API ALevel_CombinedSteering : public ALevel_Base
{
	GENERATED_BODY()

public:
	ALevel_CombinedSteering();
	virtual void Tick(float DeltaTime) override;

protected:
	virtual void BeginPlay() override;
	virtual void BeginDestroy() override;

private:
	bool CanDebugRender = false;

	// Agents
	ASteeringAgent* pDrunkAgent{ nullptr };
	ASteeringAgent* pEvadingAgent{ nullptr };

	// DrunkAgent behaviors 
	std::unique_ptr<Seek>   pSeek{};
	std::unique_ptr<Wander> pWander{};
	std::unique_ptr<BlendedSteering> pBlendedSteering{};

	// EvadingAgent behaviors 
	std::unique_ptr<Evade>  pEvade{};
	std::unique_ptr<Wander> pEvadeWander{};
	std::unique_ptr<PrioritySteering> pPrioritySteering{};

	// Evade radius for priority steering
	float EvadeRadius{ 300.f };
};