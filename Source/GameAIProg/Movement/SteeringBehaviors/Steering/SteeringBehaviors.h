#pragma once

#include <Movement/SteeringBehaviors/SteeringHelpers.h>
#include "Kismet/KismetMathLibrary.h"

class ASteeringAgent;

// SteeringBehavior base, all steering behaviors should derive from this.
class ISteeringBehavior
{
public:
	ISteeringBehavior() = default;
	virtual ~ISteeringBehavior() = default;

	// Override to implement your own behavior
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) = 0;

	void SetTarget(const FTargetData& NewTarget) { Target = NewTarget; }

	template<class T, std::enable_if_t<std::is_base_of_v<ISteeringBehavior, T>>* = nullptr>
	T* As()
	{
		return static_cast<T*>(this);
	}

protected:
	FTargetData Target;
};

//***********
// SEEK
//***********
class Seek : public ISteeringBehavior
{
public:
	Seek() = default;
	virtual ~Seek() override = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

//***********
// FLEE
//***********
class Flee : public ISteeringBehavior
{
public:
	Flee() = default;
	virtual ~Flee() override = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

//***********
// ARRIVE
//***********
class Arrive : public ISteeringBehavior
{
public:
	Arrive() = default;
	virtual ~Arrive() override = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

	void SetSlowRadius(float Radius) { SlowRadius = Radius; }
	void SetTargetRadius(float Radius) { TargetRadius = Radius; }

private:
	float SlowRadius{ 200.f };
	float TargetRadius{ 30.f };
	float OriginalMaxSpeed{ -1.f };
};

//***********
// FACE
//***********
class Face : public ISteeringBehavior
{
public:
	Face() = default;
	virtual ~Face() override = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

//***********
// PURSUIT
//***********
class Pursuit : public ISteeringBehavior
{
public:
	Pursuit() = default;
	virtual ~Pursuit() override = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

//***********
// EVADE
// Sets IsValid=false when target is outside EvadeRadius (used with PrioritySteering)
//***********
class Evade : public ISteeringBehavior
{
public:
	Evade() = default;
	virtual ~Evade() override = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

	void SetEvadeRadius(float Radius) { EvadeRadius = Radius; }

private:
	float EvadeRadius{ 300.f };
};

//***********
// WANDER
//***********
class Wander : public Seek
{
public:
	Wander() = default;
	virtual ~Wander() override = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

	void SetWanderOffset(float Offset) { m_OffsetDistance = Offset; }
	void SetWanderRadius(float Radius) { m_Radius = Radius; }
	void SetMaxAngleChange(float Rad) { m_MaxAngleChange = Rad; }

protected:
	float m_OffsetDistance{ 150.f };  
	float m_Radius{ 100.f };          
	float m_MaxAngleChange{ FMath::DegreesToRadians(45.f) }; 
	float m_WanderAngle{ 0.f };      
};