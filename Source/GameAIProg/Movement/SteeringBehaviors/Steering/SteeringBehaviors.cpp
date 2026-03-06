#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//***********
// SEEK
//***********
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Output{};

	FVector2D ToTarget = Target.Position - Agent.GetPosition();
	ToTarget.Normalize();
	Output.LinearVelocity = ToTarget;

	// Debug rendering
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugDirectionalArrow(
			Agent.GetWorld(),
			Agent.GetActorLocation(),
			Agent.GetActorLocation() + FVector{ Output.LinearVelocity * Agent.GetMaxLinearSpeed(), 0.f },
			30.f, FColor::Green, false, -1.f, 0, 3.f
		);
	}

	return Output;
}

//***********
// FLEE
//***********
SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Output{};

	FVector2D FromTarget = Agent.GetPosition() - Target.Position;
	FromTarget.Normalize();
	Output.LinearVelocity = FromTarget;

	// Debug rendering
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugDirectionalArrow(
			Agent.GetWorld(),
			Agent.GetActorLocation(),
			Agent.GetActorLocation() + FVector{ Output.LinearVelocity * Agent.GetMaxLinearSpeed(), 0.f },
			30.f, FColor::Red, false, -1.f, 0, 3.f
		);
	}

	return Output;
}

//***********
// ARRIVE
//***********
SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Output{};

	// Store original speed on first call
	if (OriginalMaxSpeed < 0.f)
		OriginalMaxSpeed = Agent.GetMaxLinearSpeed();

	FVector2D ToTarget = Target.Position - Agent.GetPosition();
	float Distance = ToTarget.Length();

	if (Distance < TargetRadius)
	{
		// Inside target radius - stop
		Agent.SetMaxLinearSpeed(0.f);
		Output.LinearVelocity = FVector2D::ZeroVector;
	}
	else if (Distance < SlowRadius)
	{
		// Inside slow radius - scale speed down
		float SpeedFraction = (Distance - TargetRadius) / (SlowRadius - TargetRadius);
		Agent.SetMaxLinearSpeed(OriginalMaxSpeed * SpeedFraction);
		ToTarget.Normalize();
		Output.LinearVelocity = ToTarget;
	}
	else
	{
		// Outside slow radius - full speed
		Agent.SetMaxLinearSpeed(OriginalMaxSpeed);
		ToTarget.Normalize();
		Output.LinearVelocity = ToTarget;
	}

	// Debug rendering
	if (Agent.GetDebugRenderingEnabled())
	{
		FVector AgentLoc = Agent.GetActorLocation();
		
		DrawDebugCircle(Agent.GetWorld(), AgentLoc, SlowRadius, 32, FColor::Blue, false, -1.f, 0, 2.f, FVector(0, 1, 0), FVector(1, 0, 0));
		
		DrawDebugCircle(Agent.GetWorld(), AgentLoc, TargetRadius, 32, FColor::Orange, false, -1.f, 0, 2.f, FVector(0, 1, 0), FVector(1, 0, 0));
	}

	return Output;
}

//***********
// FACE
//***********
SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Output{};
	Output.LinearVelocity = FVector2D::ZeroVector;

	FVector2D ToTarget = Target.Position - Agent.GetPosition();
	if (ToTarget.IsNearlyZero())
		return Output;

	// Desired angle in degrees 
	float DesiredAngle = FMath::RadiansToDegrees(FMath::Atan2(ToTarget.Y, ToTarget.X));
	float CurrentAngle = Agent.GetRotation();

	// Find the shortest angular difference
	float AngleDiff = FMath::FindDeltaAngleDegrees(CurrentAngle, DesiredAngle);

	// Set angular velocity proportional to the difference, clamped to max speed
	float MaxAngSpeed = Agent.GetMaxAngularSpeed();
	Output.AngularVelocity = FMath::Clamp(AngleDiff / DeltaT, -MaxAngSpeed, MaxAngSpeed) * DeltaT;

	// Debug rendering
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugDirectionalArrow(
			Agent.GetWorld(),
			Agent.GetActorLocation(),
			Agent.GetActorLocation() + FVector{ ToTarget.GetSafeNormal() * 100.f, 0.f },
			20.f, FColor::Yellow, false, -1.f, 0, 3.f
		);
	}

	return Output;
}

//***********
// PURSUIT
//***********
SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Output{};

	// Calculate time to reach the target based on distance and agent speed
	FVector2D ToTarget = Target.Position - Agent.GetPosition();
	float Distance = ToTarget.Length();
	float Speed = Agent.GetMaxLinearSpeed();

	float TimeToReach = (Speed > 0.f) ? (Distance / Speed) : 0.f;

	// Predict future position: d = v * t
	FVector2D PredictedPosition = Target.Position + Target.LinearVelocity * TimeToReach;

	// Seek toward predicted position
	FVector2D ToPredict = PredictedPosition - Agent.GetPosition();
	ToPredict.Normalize();
	Output.LinearVelocity = ToPredict;

	
	if (Agent.GetDebugRenderingEnabled())
	{
		
		DrawDebugSphere(
			Agent.GetWorld(),
			FVector{ PredictedPosition, Agent.GetActorLocation().Z },
			15.f, 8, FColor::Magenta, false, -1.f, 0, 2.f
		);
		
		DrawDebugDirectionalArrow(
			Agent.GetWorld(),
			Agent.GetActorLocation(),
			FVector{ PredictedPosition, Agent.GetActorLocation().Z },
			20.f, FColor::Magenta, false, -1.f, 0, 2.f
		);
	}

	return Output;
}

//***********
// EVADE
//***********
SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Output{};

	FVector2D ToTarget = Target.Position - Agent.GetPosition();
	float Distance = ToTarget.Length();

	// If outside evade radius, mark as invalid
	if (Distance > EvadeRadius)
	{
		Output.IsValid = false;
		return Output;
	}

	float Speed = Agent.GetMaxLinearSpeed();
	float TimeToReach = (Speed > 0.f) ? (Distance / Speed) : 0.f;

	FVector2D PredictedPosition = Target.Position + Target.LinearVelocity * TimeToReach;

	// Flee from predicted position
	FVector2D FromPredict = Agent.GetPosition() - PredictedPosition;
	FromPredict.Normalize();
	Output.LinearVelocity = FromPredict;
	Output.IsValid = true;

	// Debug rendering
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugSphere(
			Agent.GetWorld(),
			FVector{ PredictedPosition, Agent.GetActorLocation().Z },
			15.f, 8, FColor::Cyan, false, -1.f, 0, 2.f
		);
		DrawDebugDirectionalArrow(
			Agent.GetWorld(),
			Agent.GetActorLocation(),
			Agent.GetActorLocation() + FVector{ Output.LinearVelocity * Agent.GetMaxLinearSpeed(), 0.f },
			20.f, FColor::Cyan, false, -1.f, 0, 2.f
		);
	}

	return Output;
}

//***********
// WANDER
//***********
SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	// Update wander angle by a random offset clamped to MaxAngleChange
	float AngleOffset = FMath::RandRange(-m_MaxAngleChange, m_MaxAngleChange);
	m_WanderAngle += AngleOffset;

	// Get agent's forward direction 
	float AgentYawRad = FMath::DegreesToRadians(Agent.GetRotation());
	FVector2D AgentForward{ FMath::Cos(AgentYawRad), FMath::Sin(AgentYawRad) };

	// Place circle center in front of the agent
	FVector2D CircleCenter = Agent.GetPosition() + AgentForward * m_OffsetDistance;

	// Calculate the point on the circle using the wander angle
	FVector2D WanderPoint = CircleCenter + FVector2D{ FMath::Cos(m_WanderAngle), FMath::Sin(m_WanderAngle) } *m_Radius;

	// Use Seek logic toward the wander point
	Target.Position = WanderPoint;

	
	if (Agent.GetDebugRenderingEnabled())
	{
		FVector CircleCenterWorld{ CircleCenter, Agent.GetActorLocation().Z };
		
		DrawDebugCircle(Agent.GetWorld(), CircleCenterWorld, m_Radius, 32, FColor::Blue, false, -1.f, 0, 2.f, FVector(0, 1, 0), FVector(1, 0, 0));
		
		DrawDebugLine(Agent.GetWorld(), CircleCenterWorld, FVector{ WanderPoint, Agent.GetActorLocation().Z }, FColor::White, false, -1.f, 0, 2.f);
		
		DrawDebugSphere(Agent.GetWorld(), FVector{ WanderPoint, Agent.GetActorLocation().Z }, 10.f, 8, FColor::White, false, -1.f, 0, 2.f);
	}

	return Seek::CalculateSteering(DeltaT, Agent);
}