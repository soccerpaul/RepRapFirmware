/*

 * HeightController.cpp
 *
 *  Created on: 18 Apr 2019
 *      Author: David


#include "HeightController.h"

#if SUPPORT_ASYNC_MOVES

#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Heating/Heat.h>
#include <Heating/Sensors/TemperatureSensor.h>
#include <Movement/Move.h>
#include <Platform/TaskPriorities.h>

HeightController::HeightController() noexcept
	: heightControllerTask(nullptr), sensorNumber(-1),
		sampleInterval(DefaultSampleInterval), setPoint(1.0), pidP(1.0), configuredPidI(0.0), configuredPidD(0.0), iAccumulator(0.0),
		zMin(5.0), zMax(10.0), configuredDrive(Z_AXIS), speedControlMode(false), currentSpeed(0.0), state(PidState::stopped)
{
	CalcDerivedValues();
}

extern "C" [[noreturn]] void HeightControllerTaskStart(void *p) noexcept
{
	static_cast<HeightController*>(p)->RunTask();
}

GCodeResult HeightController::Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	// Get sensor number
	bool seen = false;
	uint32_t sn;
	gb.TryGetUIValue('S', sn, seen);
	if (seen)
	{
		sensorNumber = (int)sn;
	}

	// Check which axis we're configuring motion following for
	for (size_t axis = 0; axis < reprap.GetGCodes().GetTotalAxes(); ++axis)
	{
		const char* axisLetters = reprap.GetGCodes().GetAxisLetters();
		if (gb.Seen(axisLetters[axis]))
		{
			configuredDrive = axis;
		}
	}

	//TODO Extruder control doesn't seem to work
	if (gb.Seen(extrudeLetter))
	{
		configuredDrive = ExtruderToLogicalDrive(gb.GetLimitedUIValue(extrudeLetter, reprap.GetGCodes().GetNumExtruders()));
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

	// Get motion limits
	float motionLimits[2];
	bool seenL = false;
	if (gb.TryGetFloatArray('L', 2, motionLimits, reply, seenL, false))
	{
		return GCodeResult::error;
	}
	if (seenL && motionLimits[0] < motionLimits[1])
	{
		zMin = motionLimits[0];
		zMax = motionLimits[1];
	}

	// TODO maybe make speed control automatic when you select an extruder
	if (gb.Seen('Q'))
	{
		speedControlMode = true;
	}
	else
	{
		gb.TryGetFValue('P', pidP, seen);
		gb.TryGetFValue('I', configuredPidI, seen);
		gb.TryGetFValue('D', configuredPidD, seen);

		bool dummy;
		gb.TryGetFValue('H', setPoint, dummy);
	}

	if (seen || seenL)
	{
		CalcDerivedValues();

		TaskCriticalSectionLocker lock;			// make sure we don't create the task more than once

		if (heightControllerTask == nullptr && sensorNumber >= 0)
		{
			state = PidState::stopped;
			heightControllerTask = new Task<HeightControllerTaskStackWords>;
			heightControllerTask->Create(HeightControllerTaskStart, "HEIGHT", (void*)this, TaskPriority::HeightFollowingPriority);
		}
	}
	else if (sensorNumber < 0)
	{
		reply.copy("Analog motion controller is not configured");
	}
	else
	{
		reply.printf("Analog motion controller uses sensor %u, frequency %.1f, P%.1f I%.1f D%.1f, setpoint %.1f, L%.1f to %.1f",
						sensorNumber, (double)(1000.0/(float)sampleInterval), (double)pidP, (double)configuredPidI, (double)configuredPidD, (double)setPoint, (double)zMin, (double)zMax);
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
			if (sensorNumber < 0|| heightControllerTask == nullptr)
			{
				reply.copy("Analog motion controller is not configured");
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
			Stop();
		}
	}
	else
	{
		reply.printf("Analog motion following mode is %sactive", (state == PidState::stopped) ? "in" : "");
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
			currentZ = machinePos[configuredDrive];
			iAccumulator = constrain<float>(currentZ, zMin, zMax);
		}
		else if (sensorNumber < 0)
		{
			state = PidState::stopped;
		}
		else
		{
			TemperatureError err;
			const float sensorVal = reprap.GetHeat().GetSensorTemperature(sensorNumber, err);
			if (err == TemperatureError::success)
			{
				AsyncMove * const move = reprap.GetMove().LockAuxMove();
				if (move != nullptr)
				{
					if (speedControlMode)
					{
						const float interval = sampleInterval * MillisToSeconds;
						// In speed control mode, sensorVal is the speed we're aiming for
						const float requestedSpeedChange = sensorVal - currentSpeed;
						float adjustment = 0;

						if (requestedSpeedChange > 0) // speed increase requested
						{

							// Acceleration limited case
							if (requestedSpeedChange / interval > maxAcceleration)
							{
								acceleration = maxAcceleration;
								endSpeed = currentSpeed + interval * maxAcceleration;
							}
							// Top speed limited case
							else if (sensorVal > maxSpeed)
							{
								acceleration = (maxSpeed - currentSpeed)/interval;
								endSpeed = maxSpeed;
							}
							else
							{
								acceleration = requestedSpeedChange / interval;
								endSpeed = sensorVal;
							}

							adjustment = currentSpeed * interval + 0.5 * acceleration * fsquare(interval);
						}
						else if (requestedSpeedChange < 0) // speed decrease requested
						{
							// Acceleration limited case
							if (abs(requestedSpeedChange / interval) > maxAcceleration)
							{
								acceleration = -maxAcceleration;
								endSpeed = currentSpeed + interval * maxAcceleration;
							}
							else
							{
								acceleration = requestedSpeedChange / interval;
								endSpeed = sensorVal;
							}

							adjustment = currentSpeed * interval + 0.5 * acceleration * fsquare(interval);
						}
						else // no speed change
						{
							endSpeed = currentSpeed;
							acceleration = maxAcceleration;
							adjustment = currentSpeed * interval;
						}

						deceleration = acceleration; // TODO check if this is true or not (ex. should accel be zero on decelerating moves or does it even matter? zero deceleration doesn't work at all, nothing moves)

						// Schedule an async move to adjust Z
						move->SetDefaults();
						move->movements[configuredDrive] = adjustment;
						move->startSpeed = currentSpeed;
						// TODO Moves can't end at a zero speed because they don't know what's coming up next, need to fix this
						move->endSpeed = endSpeed;
						move->requestedSpeed = endSpeed;
						move->acceleration = acceleration;
						move->deceleration = deceleration;
						reprap.GetMove().ReleaseAuxMove(true);
					}
					else
					{
						// Calculate the new target Z height using the PID algorithm
						const float difference = setPoint - sensorVal;
						iAccumulator = constrain<float>(iAccumulator + difference * actualPidI, zMin, zMax);
						float newZ = currentZ + pidP * difference + iAccumulator;
						if (lastReadingOk)
						{
							newZ += actualPidD * (lastReading - sensorVal);
						}


						// Constrain the target Z height to be within the limits
						newZ = constrain<float>(newZ, zMin, zMax);

						// Calculate how far we need to move Z and constrain it to the maximum permissible Z movement per sample interval.
						// During the startup phase, Z may be initial outside the limits, so the resulting Z may be outside the limits too.
						// Note, if the initial Z is well outside the limits, moving it to be within the limits in several small steps like this is non-optimal.
						// It is faster to move Z to be within the limits before engaging height following mode.
						const float adjustment = constrain<float>(newZ - currentZ, -maxZAdjustmentPerSample, maxZAdjustmentPerSample);
						currentZ += adjustment;

						// Schedule an async move to adjust Z
						move->SetDefaults();
						move->movements[configuredDrive] = adjustment;
						move->startSpeed = move->endSpeed = startSpeed;
						move->requestedSpeed = reprap.GetPlatform().MaxFeedrate(configuredDrive);
						move->acceleration = move->deceleration = maxAcceleration;
						reprap.GetMove().ReleaseAuxMove(true);
					}
				}

				lastReading = sensorVal;
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

void HeightController::CalcDerivedValues() noexcept
{
	maxSpeed = reprap.GetPlatform().MaxFeedrate(configuredDrive);
	maxAcceleration = reprap.GetPlatform().Acceleration(configuredDrive);
	const float interval = sampleInterval * MillisToSeconds;

	if (speedControlMode)
	{
		startSpeed = currentSpeed;
		endSpeed = 0;
		acceleration = reprap.GetPlatform().Acceleration(configuredDrive);
		deceleration = acceleration;
	}
	else
	{
		actualPidI = configuredPidI * ((float)sampleInterval * MillisToSeconds);
		actualPidD = configuredPidD * (SecondsToMillis/(float)sampleInterval);

		// Calculate the maximum Z adjustment per sample interval.
		// We always start and end at half the Z jerk speed so that back-to-back Z movements are always possible.
		startSpeed = reprap.GetPlatform().GetInstantDv(configuredDrive) * 0.5;
		if (startSpeed + maxAcceleration * interval * 0.5 < maxSpeed)
		{
			// Acceleration limited case
			// We have s = 2 * ((u * (t/2)) + (0.5 * a *(t/2)^2)) = (u * t) + (0.25 * a * t^2)
			maxZAdjustmentPerSample = startSpeed * interval + maxAcceleration * fsquare(interval) * 0.25;
		}
		else
		{
			// Top-speed limited
			const float accelDecelTime = (maxSpeed - startSpeed)/maxAcceleration;
			maxZAdjustmentPerSample = interval * maxSpeed - accelDecelTime * (maxSpeed - startSpeed);
			}
	}
}

#endif

// End




*/
