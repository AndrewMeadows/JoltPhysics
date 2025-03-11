// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Rocket/RocketController.h>

/// Missile is a Rocket with different control mechanisms
class MissileController : public RocketController
{
public:
	MissileController(Body* inBody);

	~MissileController();

	/// OnStep() is called right before each simulation substep and is where all thrust and torque influences are applied.
	void OnStep(const PhysicsStepListenerContext &inContext) override;

private:
	/// To make mMaxThrustRatio and mMaxCruisingSpeed agree with each other we compute a second-order damping model
	/// to use during the integration step.
	void UpdateDampingModel() override;

private:
    // Missile uses direct velocity adjustment for control: it less physically accurate but easier to deal with
    // (because it is mass agnostic, so no need to compute force or impulse)
    float mLinearMotorTimescale { 3.0f };
    float mAngularMotorTimescale { 2.0f };
    float mMaxAngularSpeed { 0.5f * JPH_PI };
};
