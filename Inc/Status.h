/*
 * Struct.h
 *
 *  Created on: 12 Mar 2020
 *      Author: Joshua
 */

#ifndef STATUS_H_
#define STATUS_H_

#include "stm32f4xx_hal.h"
#include <ctime>

/*** Debug functionality enable/disable ***/
#define CAN_DEBUG					1
#define CAN_ENABLED					1
#define SD_CARD_DEBUG				1
#define FAN_DEBUG					1
#define BMS_RELAY_CTRL_BYPASS		0
#define STOP_CORE_ON_SAFE_STATE		1
#define START_DEBUG_ON_SAFE_STATE	1
#define BYPASS_INITIAL_CHECK		1
#define SKIP_PEC_ERROR_ACTIONS		1
#define LIMP_COUNT_LIMIT			2

/*** Test enable/disable ***/
#define TEST_OVERVOLTAGE					1
#define TEST_UNDERVOLTAGE					1
#define TEST_OVERTEMPERATURE				1
#define TEST_UNDERTEMPERATURE				1
#define TEST_OVERPOWER						1
#define TEST_ACCU_UNDERVOLTAGE				1 // This is for testing undervoltage with IVT
#define CHECK_IVT							1 // To completely disable IVT TEST_ACCU_UNDERVOLTAGE this needs to be set to 0
#define TEST_OVERTEMPERATURE_CHARGING		1
#define TEST_OVERCURRENT					1
#define IVT_TIMEOUT							0

struct Status {
public:
	enum ErrorEvent {
		Underfined, Overvoltage, Undervoltage, Overtemp, Undertemp, Overcurrent,
		Overpower, Extern, PecError, AccuUndervoltage, IVTLost, OvertempCharging,
		NumberOfErrors
	};

	enum OpMode { Core = 1 << 0, Balance = 1 << 1, Charging = 1 << 2, Debug = 1 << 3, Logging = 1 << 4 };

	Status(uint8_t op_mode) : op_mode { op_mode }, manual_mode { false } { // get rid of logging bool and just use op mode
		OpenAIR();
		OpenPRE();
		SetFanDutyCycle(kFanDCDefault);
	};

	uint8_t op_mode;
	bool manual_mode;

	/* Energize AIR (Accumulator Indicator Relay). */
	void CloseAIR(void) const {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // BMSRelay
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); // LED2
	}

	/* De-energize AIR (Accumulator Indicator Relay). */
	void OpenAIR(void) const {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // BMSRelay
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); // LED2
	}

	/* Energize PRE (Pre-charge Relay). */
	void ClosePRE(void) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // PRECHARGE
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET); // LED1
		precharge_flag = true;
	}

	/* De-energize PRE (Pre-charge Relay). */
	void OpenPRE(void) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // PRECHARGE
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); // LED1
		precharge_flag = false;
	}

	void IncreasePecCounter() {
		pec_average = static_cast<float>(++pec_counter) / uptime;
	}

	auto GetPecChange() {
		auto pec_change = pec_counter - pec_counter_last;
		pec_counter_last = pec_counter;

		return pec_change;
	}

	/* Sets BMS into safe state. */
	void GoToSafeState(ErrorEvent e) {
#if BMS_RELAY_CTRL_BYPASS
		// Do nothing.
#elif SKIP_PEC_ERROR_ACTIONS
		if (e != PecError) {
			OpenAIR();
			OpenPRE();
		}
#else
		OpenAIR();
		OpenPRE();
#endif

#if STOP_CORE_ON_SAFE_STATE
		op_mode &= ~Core;
#endif

#if START_DEBUG_ON_SAFE_STATE
		op_mode |= Debug;
#endif
		safe_state_executed = true;
		reason_code = e;
	}

	void SetFanDutyCycle(uint8_t duty_cycle = kFanDCDefault) {
		if (duty_cycle > kFanDCMax)
			duty_cycle = kFanDCMax;
		else if (duty_cycle < kFanDCMin)
			duty_cycle = kFanDCMin;

		if (!manual_mode);
		//pwm_channel_update_duty(PWM, &g_pwm_channel_fan, dc);
	}

	constexpr uint8_t CalcDutyCycle() {
		if (max_temp > kT2DCHighTemp)
			return kFanDCMax;
		else if (max_temp < kT2DCLowTemp)
			return kFanDCDefault;
		else
			return (max_temp * kT2DC_M) + (kT2DCLowDutyCycle - kT2DC_M * kT2DCLowTemp);
	};

	/* Test accumulator data against limit values. (0: OK, -1 FAIL) */
	/* MAYBE WE DON'T WANT 50% ERRORS TO BE ALLOWED */
	int8_t TestLimits() {
#if IVT_TIMEOUT
		if (recieved_IVT != 1) {
			if (++error_counters[IVT_LOST] > kErrorLimitLost || !tested) {
				goto_safe_state(IVT_LOST);
				return -1;
			}
		} else {
			recieved_IVT = 0;
			if (error_counters[ACCU_UNDERVOLTAGE] > 0) //These checks are annoying. Just don't want to decrement a 0.
				--error_counters[IVT_LOST];
		}
#endif
		return 0;
	}

	int8_t ErrorHandler(ErrorEvent e, bool occurred) {
		if(++error_counters[e] > kErrorLimit || !tested) {
			GoToSafeState(e);
			return -1;
		} else if (error_counters[e] > 0)
			--error_counters[e];

		return 0;
	}

	uint32_t uptime { 0 };
	uint32_t pec_counter { 0 };
	bool precharge_flag { false };
	bool safe_state_executed { 0 };
	uint8_t reason_code { 0 };
	uint8_t charger_event_counter = 0;
	struct tm rtc = { 0 };

	uint16_t min_voltage = 0;
	uint8_t min_voltage_id = 0;
	uint16_t max_voltage = 0;
	uint8_t max_voltage_id = 0;
	uint16_t sum_of_cells = 0;

	int16_t min_temp = 0;
	int16_t max_temp = 0;
	uint8_t min_temp_id = 0;
	uint8_t max_temp_id = 0;

private:
	uint32_t error_counters[NumberOfErrors] = { 0 };
	uint32_t pec_counter_last { 0 };
	float pec_average { 0 };
	bool tested { false };

	static constexpr uint16_t kT2DCLowTemp { 2000 };
	static constexpr uint8_t kT2DCLowDutyCycle { 10 };
	static constexpr uint16_t kT2DCHighTemp { 6000 };
	static constexpr uint8_t kT2DCHighDutyCycle { 100 };
	static constexpr uint8_t kErrorLimit { 2 };
	static constexpr uint8_t kFanDCMax { 100 };
	static constexpr uint8_t kFanDCMin { 0 };
	static constexpr uint8_t kFanDCDefault { 10 };
	static constexpr float kT2DC_M { static_cast<float>(kT2DCHighDutyCycle - kT2DCLowDutyCycle) / (kT2DCHighTemp - kT2DCLowTemp) };
};

#endif /* STATUS_H_ */
