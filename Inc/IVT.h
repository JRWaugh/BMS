/*
 * IVT.h
 *
 *  Created on: 14 Mar 2020
 *      Author: Joshua
 */

#ifndef IVT_H_
#define IVT_H_

#include "stm32f4xx_hal.h"
#include "Status.h"

class IVT {
public:
	IVT(Status& status) : status{ status } {}

	int8_t SetCurrent(int32_t raw_current) {
		current = static_cast<float>(raw_current) / 1000;
		received_update = true;
#if TEST_OVERCURRENT
		return status.ErrorHandler(Status::Overcurrent, current > kMaxCurrent);
#endif
		return 0;
	}

	void SetVoltage(int32_t raw_voltage) {
		voltage = static_cast<float>(raw_voltage) / 1000;
		received_update = true;
	}

	// TODO This used to be precharge compare, kinda
	int8_t SetVoltage2(int32_t raw_voltage) {
		voltage2 = static_cast<float>(raw_voltage) / 1000;
		received_update = true;

#if TEST_ACCU_UNDERVOLTAGE
		if (status.ErrorHandler(Status::AccuUndervoltage, voltage2 < kAccuMinVoltage))
			return -1;
#endif

		/* Doing this here because it should hopefully avoid divide by 0 errors... */
		precharge_percentage = voltage * 100 / voltage2;

		if (precharge_percentage >= 95 && CheckVoltageMatch() && voltage > kPrechargeMinStartVoltage && voltage2 > kPrechargeMinStartVoltage)
			status.ClosePRE();
		else if (voltage < kPrechargeMaxEndVoltage || voltage2 < kPrechargeMaxEndVoltage) // TODO think there was a bug here in old system
			status.OpenPRE();

		return 0;
	}

	bool CheckVoltageMatch(void) {
		float percentage = voltage * 100 / (status.sum_of_cells / 10000)- 100;
		return percentage < 10 && percentage > -10;
	}

private:
	Status& status;
	float current = 0;
	float voltage = 0;
	float voltage2 = 0;
	float precharge_percentage = 0;
	bool received_update = false;

private:
	static constexpr float kAccuMinVoltage { 490.0 };
	static constexpr float kMaxCurrent { 180.0 };
	static constexpr float kPrechargeMinStartVoltage = 470.0;
	static constexpr float kPrechargeMaxEndVoltage = 450.0;
	static constexpr uint8_t kErrorLimitLost { 1 };
};

#endif /* IVT_H_ */
