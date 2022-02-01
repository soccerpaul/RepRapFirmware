/*
 * HeightController.h
 *
 *  Created on: 18 Apr 2019
 *      Author: David
 */

// TODO entire file changed by Paul

#ifndef SRC_MOVEMENT_HEIGHTCONTROL_HEIGHTCONTROLLER_H_
#define SRC_MOVEMENT_HEIGHTCONTROL_HEIGHTCONTROLLER_H_

#include <RepRapFirmware.h>

#if SUPPORT_ASYNC_MOVES

#include <RTOSIface/RTOSIface.h>

class HeightController
{
public:
	HeightController() noexcept;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult StartHeightFollowing(GCodeBuffer& gb, const StringRef& reply) noexcept;		// Start/stop height following
	void Stop() noexcept;							// stop height following mode

	[[noreturn]] void RunTask() noexcept;

private:

	static constexpr unsigned int HeightControllerTaskStackWords = 100;
	static constexpr uint32_t DefaultSampleInterval = 200;

	Task<HeightControllerTaskStackWords> *heightControllerTask;
	int sensorNumber1;								// which sensor, normally a virtual heater, or -1 if not configured
	int sensorNumber2;								// which sensor, normally a virtual heater, or -1 if not configured
	uint32_t sampleInterval;						// in milliseconds
	uint32_t lastWakeTime;
	int configuredDrive;							// the axis for which motion following is configured
	float currentSpeed;
	float currentPosition;

	enum class PidState : uint8_t
	{
		stopped,
		starting,
		running
	};

	volatile PidState state;						// volatile because it is accessed by more than one task
	bool lastReadingOk;
};

#endif

#endif /* SRC_MOVEMENT_HEIGHTCONTROL_HEIGHTCONTROLLER_H_ */
