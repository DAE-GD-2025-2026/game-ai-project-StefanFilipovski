#include "CombinedSteeringBehaviors.h"
#include <algorithm>
#include "../SteeringAgent.h"

BlendedSteering::BlendedSteering(const std::vector<WeightedBehavior>& WeightedBehaviors)
	:WeightedBehaviors(WeightedBehaviors)
{
};

//****************
//BLENDED STEERING
SteeringOutput BlendedSteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput BlendedOutput = {};
	float TotalWeight = 0.f;

	for (auto& WeightedBehavior : WeightedBehaviors)
	{
		if (WeightedBehavior.pBehavior && WeightedBehavior.Weight > 0.f)
		{
			SteeringOutput BehaviorOutput = WeightedBehavior.pBehavior->CalculateSteering(DeltaT, Agent);
			BlendedOutput.LinearVelocity += BehaviorOutput.LinearVelocity * WeightedBehavior.Weight;
			BlendedOutput.AngularVelocity += BehaviorOutput.AngularVelocity * WeightedBehavior.Weight;
			TotalWeight += WeightedBehavior.Weight;
		}
	}

	// Normalize by total weight if needed
	if (TotalWeight > 0.f && TotalWeight != 1.f)
	{
		BlendedOutput.LinearVelocity /= TotalWeight;
		BlendedOutput.AngularVelocity /= TotalWeight;
	}

	// Debug rendering
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugDirectionalArrow(
			Agent.GetWorld(),
			Agent.GetActorLocation(),
			Agent.GetActorLocation() + FVector{ BlendedOutput.LinearVelocity * Agent.GetMaxLinearSpeed(), 0.f },
			30.f, FColor::Red, false, -1.f, 0, 3.f
		);
	}

	return BlendedOutput;
}

float* BlendedSteering::GetWeight(ISteeringBehavior* const SteeringBehavior)
{
	auto it = find_if(WeightedBehaviors.begin(),
		WeightedBehaviors.end(),
		[SteeringBehavior](const WeightedBehavior& Elem)
		{
			return Elem.pBehavior == SteeringBehavior;
		}
	);

	if (it != WeightedBehaviors.end())
		return &it->Weight;

	return nullptr;
}

//*****************
//PRIORITY STEERING
SteeringOutput PrioritySteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering = {};

	for (ISteeringBehavior* const pBehavior : m_PriorityBehaviors)
	{
		Steering = pBehavior->CalculateSteering(DeltaT, Agent);

		if (Steering.IsValid)
			break;
	}

	// If none of the behaviors return a valid output, last behavior is returned
	return Steering;
}