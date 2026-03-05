// Fill out your copyright notice in the Description page of Project Settings.

#include "SteeringAgent.h"


// Sets default values
ASteeringAgent::ASteeringAgent()
{
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ASteeringAgent::BeginPlay()
{
	Super::BeginPlay();
}

void ASteeringAgent::BeginDestroy()
{
	Super::BeginDestroy();
}

// Called every frame
void ASteeringAgent::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (SteeringBehavior)
	{
		SteeringOutput Output = SteeringBehavior->CalculateSteering(DeltaTime, *this);

		// Handle linear movement
		AddMovementInput(FVector{ Output.LinearVelocity, 0.f });

		// Handle angular velocity 
		if (!FMath::IsNearlyZero(Output.AngularVelocity))
		{
			FRotator CurrentRotation = GetActorRotation();
			CurrentRotation.Yaw += Output.AngularVelocity;
			SetActorRotation(CurrentRotation);
		}
	}
}

// Called to bind functionality to input
void ASteeringAgent::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
}

void ASteeringAgent::SetSteeringBehavior(ISteeringBehavior* NewSteeringBehavior)
{
	SteeringBehavior = NewSteeringBehavior;
}