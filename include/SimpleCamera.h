#ifndef CAMERA_H

#define CAMERA_H

#include <stdint.h>

namespace RENDER_DEBUG
{
	class RenderDebug;
};

class SimpleCamera
{
public:
	SimpleCamera(RENDER_DEBUG::RenderDebug *renderDebug);
	~SimpleCamera(void);

	void update(float dtime);

	const float *getEyePosition(void) const
	{
		return mEyePosition;
	}

	const float * getEyeDirection(void) const
	{
		return mViewDir;
	}

	bool getSpacebarSemaphore(void)
	{
		bool ret = mSpacebar;
		mSpacebar = false;
		return ret;
	}

private:
	void applyChanges(float dtime);

	bool			mSpacebar{ false };
	uint32_t		mLastRenderFrame;
	uint32_t		mLastCommunicationsFrame;
	float			mCameraSpeed;
	float			mPitchMin;
	float			mPitchMax;
	float			mTargetYaw;
	float			mTargetPitch;

	float			mGamepadPitchInc;
	float			mGamepadYawInc;
	float			mGamepadForwardInc;
	float			mGamepadLateralInc;
	float			mGamepadUpDownInc;

	float			mEyePosition[3];
	float			mViewDir[3];
	RENDER_DEBUG::RenderDebug *mRenderDebug;
};

#endif


