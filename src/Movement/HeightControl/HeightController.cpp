/*
 * HeightController.cpp
 *
 *  Created on: 18 Apr 2019
 *      Author: David
 */

// TODO entire file changed by Paul
// TODO max frequency is limited to 50 Hz right now, beyond that motion is no longer smooth. It may be because there is essentially no buffer. Would be nice to use higher frequencies

#include "HeightController.h"

#if SUPPORT_ASYNC_MOVES

#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <Tools/Tool.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Heating/Heat.h>
#include <Heating/Sensors/TemperatureSensor.h>
#include <Movement/Move.h>
#include <Platform/TaskPriorities.h>

HeightController::HeightController() noexcept
: heightControllerTask(nullptr), sensorNumber1(-1), sensorNumber2(-1), sampleInterval(DefaultSampleInterval), configuredDrive(-1), currentSpeed(0.0), state(PidState::stopped)
{

}

extern "C" [[noreturn]] void HeightControllerTaskStart(void *p) noexcept
{
	static_cast<HeightController*>(p)->RunTask();
}

GCodeResult HeightController::Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
		{
	// Get first sensor number
	bool seen1 = false;
	uint32_t sn1;
	gb.TryGetUIValue('S', sn1, seen1);
	if (seen1)
	{
		sensorNumber1 = (int)sn1;
	}

	// Get second sensor number
	bool seen2 = false;
	uint32_t sn2;
	gb.TryGetUIValue('H', sn2, seen2);
	if (seen2)
	{
		sensorNumber2 = (int)sn2;
	}

	// Speed control can be configured for an axis, an extruder drive or a tool (only works for single-extruder tools, will default to the tool's first extruder)
	// Get axis if specified
	for (size_t axis = 0; axis < reprap.GetGCodes().GetTotalAxes(); ++axis)
	{
		const char* axisLetters = reprap.GetGCodes().GetAxisLetters();
		if (gb.Seen(axisLetters[axis]))
		{
			configuredDrive = axis;
		}
	}

	// Get extruder if specified
	if (gb.Seen(extrudeLetter))
	{
		configuredDrive = ExtruderToLogicalDrive(gb.GetLimitedUIValue(extrudeLetter, reprap.GetGCodes().GetNumExtruders()));
	}

	// Get tool if specified
	if (gb.Seen('P'))
	{
		const unsigned int toolNumber = gb.GetUIValue();
		auto tool = reprap.GetTool(toolNumber);
		configuredDrive = ExtruderToLogicalDrive(tool.Ptr()->Drive(0));
	}

	// Get frequency
	if (gb.Seen('F'))
	{
		const float freq = gb.GetFValue();
		if (freq >= 0.1 && freq <= 200.0)
		{
			sampleInterval = lrintf(1000/freq);
		}
	}

	if (seen1 && seen2 && configuredDrive >= 0)
	{
		TaskCriticalSectionLocker lock;			// make sure we don't create the task more than once

		if (heightControllerTask == nullptr && sensorNumber1 >= 0 && sensorNumber2 >= 0 && configuredDrive >= 0)
		{
			state = PidState::stopped;
			heightControllerTask = new Task<HeightControllerTaskStackWords>;
			heightControllerTask->Create(HeightControllerTaskStart, "HEIGHT", (void*)this, TaskPriority::HeightFollowingPriority);
		}
	}
	else if (sensorNumber1 < 0 || sensorNumber2 < 0 || configuredDrive < 0)
	{
		reply.copy("Analog speed controller is not configured");
	}
	else
	{
		reply.printf("Analog speed controller is configured for drive %u, uses sensors %u and %u, frequency %.1f",
				configuredDrive, sensorNumber1, sensorNumber2, (double)(1000.0/(float)sampleInterval)); // TODO this could be changed so if it's an extruder drive it says so instead of showing a high drive number
	}
	return GCodeResult::ok;
		}

// Start/stop height following
GCodeResult HeightController::StartHeightFollowing(GCodeBuffer& gb, const StringRef& reply) noexcept
{
	if (gb.Seen('P'))
	{
		if (gb.GetIValue() == 1)
		{
			// Start height following
			if (sensorNumber1 < 0 || sensorNumber2 < 0 || heightControllerTask == nullptr)
			{
				reply.copy("Analog speed controller is not configured");
				return GCodeResult::error;
			}

			if (state == PidState::stopped)
			{
				state = PidState::starting;
				heightControllerTask->Give();
			}
		}
		else
		{
			// Stop height following
			currentSpeed = 0;
			Stop();
		}
	}
	else
	{
		reply.printf("Analog speed-following mode is %sactive", (state == PidState::stopped) ? "in" : "");
	}
	return GCodeResult::ok;
}

// Stop height following mode
void HeightController::Stop() noexcept
{
	state = PidState::stopped;
}

[[noreturn]] void HeightController::RunTask() noexcept
{
	lastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		if (state == PidState::stopped)
		{
			(void)TaskBase::Take();

			// Here when we get woken up again, normally because the state has been changed to 'starting'. So get ready to start.
			lastWakeTime = xTaskGetTickCount();
			lastReadingOk = false;
			float machinePos[MaxAxes];
			reprap.GetMove().GetCurrentMachinePosition(machinePos, false);
			currentPosition = machinePos[configuredDrive];
		}
		else if (sensorNumber1 < 0 || sensorNumber2 < 0)
		{
			state = PidState::stopped;
		}
		else
		{
			TemperatureError err1;
			const float sensorVal1 = reprap.GetHeat().GetSensorTemperature(sensorNumber1, err1);
			TemperatureError err2;
			const float sensorVal2 = reprap.GetHeat().GetSensorTemperature(sensorNumber2, err2);
			if (err1 == TemperatureError::success && err2 == TemperatureError::success)
			{
					AsyncMove * const move = reprap.GetMove().LockAuxMove();
					if (move != nullptr)
					{

						float targetSpeed = min<float>(sensorVal1 * sensorVal2, reprap.GetPlatform().MaxFeedrate(configuredDrive));
						float finalSpeed = targetSpeed; // Calculations below will adjust this value if it isn't possible to reach the target velocity
						const float interval = sampleInterval * MillisToSeconds;
						float adjustment = 0; // Distance to move to reach the final speed at the end of the interval
						float maxAcceleration = reprap.GetPlatform().Acceleration(configuredDrive);
						float maxDeceleration = maxAcceleration;

						if (targetSpeed > currentSpeed)
						{
							if (targetSpeed > currentSpeed + maxAcceleration * interval) // Acceleration-limited case, can't reach targetSpeed
							{
								finalSpeed = currentSpeed + maxAcceleration * interval;
								adjustment = currentSpeed * interval + 0.5 * maxAcceleration * fsquare(interval);
							}
							else // The axis can reach the target speed before the end of the interval
							{
								float rampInterval = (finalSpeed - currentSpeed) / maxAcceleration;
								adjustment = currentSpeed * rampInterval + 0.5 * maxAcceleration * fsquare(rampInterval) + finalSpeed * (interval - rampInterval);
							}
						}
						else if (targetSpeed < currentSpeed)
						{

							if (targetSpeed < currentSpeed - maxDeceleration * interval) // Deceleration-limited case, can't reach targetVelocity
							{
								finalSpeed = currentSpeed - maxDeceleration * interval;
								adjustment = currentSpeed * interval - 0.5 * maxDeceleration * fsquare(interval);
							}
							else // The axis can reach the target speed before the end of the interval, but we don't let it do so. The Duet is designed to have deceleration at the end
								// of a move, not at the beginning, so this would cause issues. Instead, we reduce the deceleration so it reaches the target speed just in time.
							{

								maxDeceleration = (currentSpeed - finalSpeed)/interval;
								adjustment = currentSpeed * interval - 0.5 * maxDeceleration * fsquare(interval);
								//float rampInterval = (currentSpeed - finalSpeed) / maxAcceleration;
								//adjustment = currentSpeed * rampInterval - 0.5 * maxAcceleration * fsquare(rampInterval) + finalSpeed * (interval - rampInterval);
							}
						}
						else // Maintain current speed
						{
							adjustment = currentSpeed * interval;
						}

						currentPosition += adjustment;

						// Schedule an async move for the configured drive

						move->SetDefaults();
						move->movements[configuredDrive] = adjustment;
						move->requestedSpeed = finalSpeed;
						move->acceleration = reprap.GetPlatform().Acceleration(configuredDrive);
						move->deceleration = maxDeceleration;

						move->startSpeed = currentSpeed;
						move->endSpeed = finalSpeed;
						reprap.GetMove().ReleaseAuxMove(true);
						currentSpeed = finalSpeed;
					}

				lastReadingOk = true;

			}
			else
			{
				lastReadingOk = false;
			}

			vTaskDelayUntil(&lastWakeTime, sampleInterval);
		}
	}
}

#endif
