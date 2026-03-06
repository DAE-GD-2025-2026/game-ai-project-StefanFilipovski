#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"

//*******************
//COHESION (FLOCKING)
// Seek toward the average position of neighbors
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& Agent)
{
	// Set target to average neighbor position and use Seek logic
	FTargetData CohesionTarget;
	CohesionTarget.Position = pFlock->GetAverageNeighborPos();
	SetTarget(CohesionTarget);

	SteeringOutput Output = Seek::CalculateSteering(deltaT, Agent);

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugSphere(
			Agent.GetWorld(),
			FVector{ CohesionTarget.Position, Agent.GetActorLocation().Z },
			10.f, 8, FColor::Green, false, -1.f, 0, 2.f
		);
	}

	return Output;
}

//*********************
//SEPARATION (FLOCKING)
// Move away from neighbors, inversely proportional to distance (closer = more impact)
SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& Agent)
{
	SteeringOutput Output{};

	const TArray<ASteeringAgent*>& Neighbors = pFlock->GetNeighbors();
	int NrNeighbors = pFlock->GetNrOfNeighbors();

	if (NrNeighbors == 0)
		return Output;

	FVector2D SeparationVelocity = FVector2D::ZeroVector;

	for (int i = 0; i < NrNeighbors; ++i)
	{
		if (!Neighbors[i]) continue;

		FVector2D ToNeighbor = Agent.GetPosition() - Neighbors[i]->GetPosition();
		float Distance = ToNeighbor.Length();

		if (Distance > 0.f)
		{
			
			ToNeighbor.Normalize();
			SeparationVelocity += ToNeighbor * (1.f / Distance);
		}
	}

	SeparationVelocity.Normalize();
	Output.LinearVelocity = SeparationVelocity;

	// Debug rendering
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugDirectionalArrow(
			Agent.GetWorld(),
			Agent.GetActorLocation(),
			Agent.GetActorLocation() + FVector{ Output.LinearVelocity * 100.f, 0.f },
			15.f, FColor::Red, false, -1.f, 0, 2.f
		);
	}

	return Output;
}

//*************************
//VELOCITY MATCH (FLOCKING)
// Match the average velocity of neighbors
SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent& Agent)
{
	SteeringOutput Output{};

	int NrNeighbors = pFlock->GetNrOfNeighbors();
	if (NrNeighbors == 0)
		return Output;

	FVector2D AvgVelocity = pFlock->GetAverageNeighborVelocity();
	float MaxSpeed = Agent.GetMaxLinearSpeed();

	if (MaxSpeed > 0.f)
		Output.LinearVelocity = AvgVelocity / MaxSpeed; 
	else
		Output.LinearVelocity = AvgVelocity;

	// Debug rendering
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugDirectionalArrow(
			Agent.GetWorld(),
			Agent.GetActorLocation(),
			Agent.GetActorLocation() + FVector{ AvgVelocity, 0.f },
			15.f, FColor::Blue, false, -1.f, 0, 2.f
		);
	}

	return Output;
}