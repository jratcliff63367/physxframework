#include "SimpleCamera.h"
#include "NvRenderDebug.h"
#include <math.h>

static const float NvPi = float(3.141592653589793);
static const float NvHalfPi = float(1.57079632679489661923);

static void EulerToMat33(const float e[3],float matrix[9])
{
	float c1 = cosf(e[2]);
	float s1 = sinf(e[2]);
	float c2 = cosf(e[1]);
	float s2 = sinf(e[1]);
	float c3 = cosf(e[0]);
	float s3 = sinf(e[0]);

	matrix[3*0+0] = c1*c2;
	matrix[3*0+1] = -s1*c2;
	matrix[3*0+2] = s2;

	matrix[3*1+0] = (s1*c3)+(c1*s2*s3);
	matrix[3*1+1] = (c1*c3)-(s1*s2*s3);
	matrix[3*1+2] = -c2*s3;

	matrix[3*2+0] = (s1*s3)-(c1*s2*c3);
	matrix[3*2+1] = (c1*s3)+(s1*s2*c3);
	matrix[3*2+2] = c2*c3;


}

SimpleCamera::SimpleCamera(RENDER_DEBUG::RenderDebug *renderDebug) : 	mGamepadPitchInc(0.0f),
	mGamepadYawInc				(0.0f),
	mGamepadForwardInc			(0.0f),
	mGamepadLateralInc			(0.0f),
	mGamepadUpDownInc			(0.0f)
{
	mLastRenderFrame = 0;
	mCameraSpeed = 4.0f;
	mLastCommunicationsFrame = 0;

	mEyePosition[0] = 10*4;
	mEyePosition[1] = 10*4;
	mEyePosition[2] = -4*4;

	mTargetPitch = -0.697395146f;
	mTargetYaw = -4.47776270;

	mPitchMin = -NvHalfPi*0.99f;
	mPitchMax = NvHalfPi*0.99f;


	mRenderDebug = renderDebug;

	mRenderDebug->registerAnalogInputEvent(RENDER_DEBUG::InputEventIds::CAMERA_MOUSE_LOOK,1.0f,RENDER_DEBUG::InputIds::MOUSE_MOVE);

	mRenderDebug->registerDigitalInputEvent(RENDER_DEBUG::InputEventIds::CAMERA_JUMP, RENDER_DEBUG::InputIds::WKEY_SPACE);

	mRenderDebug->registerDigitalInputEvent(RENDER_DEBUG::InputEventIds::CAMERA_HOME,RENDER_DEBUG::InputIds::WKEY_HOME);
	mRenderDebug->registerDigitalInputEvent(RENDER_DEBUG::InputEventIds::CAMERA_MOVE_FORWARD,RENDER_DEBUG::InputIds::SCAN_CODE_FORWARD);
	mRenderDebug->registerDigitalInputEvent(RENDER_DEBUG::InputEventIds::CAMERA_MOVE_BACKWARD,RENDER_DEBUG::InputIds::SCAN_CODE_BACKWARD);

	mRenderDebug->registerDigitalInputEvent(RENDER_DEBUG::InputEventIds::CAMERA_MOVE_UP,RENDER_DEBUG::InputIds::SCAN_CODE_UP);
	mRenderDebug->registerDigitalInputEvent(RENDER_DEBUG::InputEventIds::CAMERA_MOVE_DOWN,RENDER_DEBUG::InputIds::SCAN_CODE_DOWN);

	mRenderDebug->registerDigitalInputEvent(RENDER_DEBUG::InputEventIds::CAMERA_MOVE_LEFT,RENDER_DEBUG::InputIds::SCAN_CODE_LEFT);
	mRenderDebug->registerDigitalInputEvent(RENDER_DEBUG::InputEventIds::CAMERA_MOVE_RIGHT,RENDER_DEBUG::InputIds::SCAN_CODE_RIGHT);

	mRenderDebug->registerDigitalInputEvent(RENDER_DEBUG::InputEventIds::CAMERA_SHIFT_SPEED,RENDER_DEBUG::InputIds::SCAN_CODE_LEFT_SHIFT);
	mRenderDebug->registerDigitalInputEvent(RENDER_DEBUG::InputEventIds::CAMERA_SPEED_INCREASE,RENDER_DEBUG::InputIds::WKEY_ADD);
	mRenderDebug->registerDigitalInputEvent(RENDER_DEBUG::InputEventIds::CAMERA_SPEED_DECREASE,RENDER_DEBUG::InputIds::WKEY_SUBTRACT);

	mRenderDebug->registerDigitalInputEvent(RENDER_DEBUG::InputEventIds::CAMERA_MOUSE_LOOK,RENDER_DEBUG::InputIds::MOUSE_BUTTON_LEFT);
	mRenderDebug->registerDigitalInputEvent(RENDER_DEBUG::InputEventIds::CAMERA_SPEED_INCREASE,RENDER_DEBUG::InputIds::GAMEPAD_LEFT_STICK);
	mRenderDebug->registerDigitalInputEvent(RENDER_DEBUG::InputEventIds::CAMERA_SPEED_DECREASE,RENDER_DEBUG::InputIds::GAMEPAD_RIGHT_STICK);


	mRenderDebug->registerAnalogInputEvent(RENDER_DEBUG::InputEventIds::CAMERA_GAMEPAD_ROTATE_LEFT_RIGHT, 0.01f, RENDER_DEBUG::InputIds::GAMEPAD_RIGHT_STICK_X);
	mRenderDebug->registerAnalogInputEvent(RENDER_DEBUG::InputEventIds::CAMERA_GAMEPAD_ROTATE_UP_DOWN, 0.01f, RENDER_DEBUG::InputIds::GAMEPAD_RIGHT_STICK_Y);
	mRenderDebug->registerAnalogInputEvent(RENDER_DEBUG::InputEventIds::CAMERA_GAMEPAD_MOVE_LEFT_RIGHT,0.1f, RENDER_DEBUG::InputIds::GAMEPAD_LEFT_STICK_X);
	mRenderDebug->registerAnalogInputEvent(RENDER_DEBUG::InputEventIds::CAMERA_GAMEPAD_MOVE_FORWARD_BACK,0.1f, RENDER_DEBUG::InputIds::GAMEPAD_LEFT_STICK_Y);
}

SimpleCamera::~SimpleCamera(void)
{

}

void SimpleCamera::applyChanges(float dtime)
{
	mTargetYaw+=mGamepadYawInc;
	mTargetPitch+=mGamepadPitchInc;

	if(mTargetPitch<mPitchMin)	
	{
		mTargetPitch = mPitchMin;
	}
	if(mTargetPitch>mPitchMax)	
	{
		mTargetPitch = mPitchMax;
	}

	float matrix[9];
	float euler[3] = { -mTargetPitch,-mTargetYaw,0 };
	EulerToMat33(euler,matrix);
	mViewDir[0] = -matrix[3*2+0];
	mViewDir[1] = -matrix[3*2+1];
	mViewDir[2] = -matrix[3*2+2];

	float lateral[3];
	float forward[3];
	float updown[3];

	float lateralv = mGamepadLateralInc*mCameraSpeed*dtime;
	float updownv = mGamepadUpDownInc*mCameraSpeed*dtime;
	float forwardv = -mGamepadForwardInc*mCameraSpeed*dtime;

	lateral[0] = matrix[3*0+0]*lateralv;
	lateral[1] = matrix[3*0+1]*lateralv;
	lateral[2] = matrix[3*0+2]*lateralv;

	updown[0] = matrix[3*1+0]*updownv;
	updown[1] = matrix[3*1+1]*updownv;
	updown[2] = matrix[3*1+2]*updownv;

	forward[0] = matrix[3*2+0]*forwardv;
	forward[1] = matrix[3*2+1]*forwardv;
	forward[2] = matrix[3*2+2]*forwardv;

	mEyePosition[0] = mEyePosition[0]+lateral[0]+updown[0]+forward[0];
	mEyePosition[1] = mEyePosition[1]+lateral[1]+updown[1]+forward[1];
	mEyePosition[2] = mEyePosition[2]+lateral[2]+updown[2]+forward[2];


}

void SimpleCamera::update(float dtime)
{
	const RENDER_DEBUG::InputEvent *e = mRenderDebug->getInputEvent(true);
	if ( e )
	{
		while ( e )
		{
			switch ( e->mEventType )
			{
				case RENDER_DEBUG::InputEvent::ET_ANALOG:
					switch ( e->mId )
					{
					case RENDER_DEBUG::InputEventIds::CAMERA_GAMEPAD_ROTATE_LEFT_RIGHT:
						mGamepadYawInc = -e->mAnalogValue*0.01f; 
						break;
					case RENDER_DEBUG::InputEventIds::CAMERA_GAMEPAD_ROTATE_UP_DOWN:
						mGamepadPitchInc = e->mAnalogValue*0.01f;
						break;
					case RENDER_DEBUG::InputEventIds::CAMERA_GAMEPAD_MOVE_LEFT_RIGHT:
						mGamepadLateralInc = e->mAnalogValue;
						break;
					case RENDER_DEBUG::InputEventIds::CAMERA_GAMEPAD_MOVE_FORWARD_BACK:
						mGamepadForwardInc = e->mAnalogValue;
						break;
					}
					break;
				case RENDER_DEBUG::InputEvent::ET_DIGITAL:
					switch ( e->mId )
					{
					case RENDER_DEBUG::InputEventIds::CAMERA_JUMP:
						mSpacebar = e->mDigitalValue ? true : false;
						break;
					case RENDER_DEBUG::InputEventIds::CAMERA_MOVE_FORWARD:
						mGamepadForwardInc = e->mDigitalValue ? 1.0f : 0.0f;
						break;
					case RENDER_DEBUG::InputEventIds::CAMERA_MOVE_BACKWARD:
						mGamepadForwardInc = e->mDigitalValue ? -1.0f : 0.0f;
						break;
					case RENDER_DEBUG::InputEventIds::CAMERA_MOVE_LEFT:
						mGamepadLateralInc = e->mDigitalValue ? -1.0f : 0.0f;
						break;
					case RENDER_DEBUG::InputEventIds::CAMERA_MOVE_RIGHT:
						mGamepadLateralInc = e->mDigitalValue ? 1.0f : 0.0f;
						break;
					case RENDER_DEBUG::InputEventIds::CAMERA_MOVE_UP:
						mGamepadUpDownInc = e->mDigitalValue ? 1.0f : 0.0f;
						break;
					case RENDER_DEBUG::InputEventIds::CAMERA_MOVE_DOWN:
						mGamepadUpDownInc = e->mDigitalValue ? -1.0f : 0.0f;
						break;
					case RENDER_DEBUG::InputEventIds::CAMERA_SPEED_DECREASE:
						if ( e->mDigitalValue )
						{
							mCameraSpeed*=0.5f;
						}
						break;
					case RENDER_DEBUG::InputEventIds::CAMERA_SPEED_INCREASE:
						if ( e->mDigitalValue )
						{
							mCameraSpeed*=2.0f;
						}
						break;
					}
					break;
				case RENDER_DEBUG::InputEvent::ET_POINTER:
					switch ( e->mId )
					{
					case RENDER_DEBUG::InputEventIds::CAMERA_MOUSE_LOOK:
						mTargetPitch+=(float)e->mMouseDY*0.001f;
						mTargetYaw+=(float)-e->mMouseDX*0.001f;
						break;
					}
					break;
			}
			if ( e->mRenderFrame != mLastRenderFrame )
			{
				applyChanges(dtime);
				mLastRenderFrame = e->mRenderFrame;
			}
			e = mRenderDebug->getInputEvent(true);
		}
	}
	else
	{
		applyChanges(dtime);
	}

	mRenderDebug->sendRemoteCommand("setcamera %0.9f %0.9f %0.9f %0.9f %0.9f %0.9f",
		mEyePosition[0],mEyePosition[1],mEyePosition[2],
		mTargetPitch, mTargetYaw, 0 );
}


