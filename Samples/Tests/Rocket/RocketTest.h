// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2023 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Rocket/SceneLoader.h>
#include <Tests/Rocket/MissileController.h>
#include <Tests/Rocket/RocketController.h>

// This test shows missiles and rockets
class RocketTest : public SceneLoader
{
public:
	JPH_DECLARE_RTTI_VIRTUAL(JPH_NO_EXPORT, RocketTest)

    RocketTest();

	//static bool					sFooValue;
	// Destructor
	virtual						~RocketTest() override;

	// See: Test
	virtual void				Initialize() override;
	virtual void				ProcessInput(const ProcessInputParams &inParams) override;
	virtual void				PrePhysicsUpdate(const PreUpdateParams &inParams) override;
	virtual void				SaveInputState(StateRecorder &inStream) const override;
	virtual void				RestoreInputState(StateRecorder &inStream) override;

	virtual void				GetInitialCamera(CameraState &ioState) const override;
	virtual RMat44				GetCameraPivot(float inCameraHeading, float inCameraPitch) const override { return mCameraPivot; }

	virtual void				CreateSettingsMenu(DebugUI *inUI, UIElement *inSubMenu) override;

private:
	void						UpdateCameraPivot();

	Body*						mMissileBody;								///< Body of missile
	MissileController*			mMissile;									///< Cruising missile

	// TODO: add guided rockets to scene
	Array<Body *>				mRocketBodies;								///< List of guided rocket Bodies in the scene
	using RocketList = Array<RocketController *>;
	RocketList					mRockets;									///< List of rockets in the scene

	RMat44						mCameraPivot = RMat44::sIdentity();			///< The camera pivot, recorded before the physics update to align with the drawn world

	// Player input
	float						mYaw { 0.0f };								///< Yaw throttle in range [-1, 1]
	float						mPitch { 0.0f };							///< Pitch throttle in range [-1, 1]
	float						mRoll { 0.0f };								///< Roll throttle in range [-1, 1]
	float						mForward { 0.0f };							///< Thrust throttle in rage [-1, 1]
};
