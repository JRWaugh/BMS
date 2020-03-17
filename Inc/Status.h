/*
 * Struct.h
 *
 *  Created on: 12 Mar 2020
 *      Author: Joshua
 */

#ifndef STATUS_H_
#define STATUS_H_

#include "stm32f4xx_hal.h"
#include "NLG5.h"
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
#define IVT_TIMEOUT							1

struct Status {
public:
	enum ErrorEvent {
		Undefined, Overvoltage, Undervoltage, Overtemp, Undertemp, Overcurrent,
		Overpower, Extern, PecError, AccuUndervoltage, IVTLost, OvertempCharging,
		NumberOfErrors
	};

	enum OpMode { Core = 1 << 0, Balance = 1 << 1, Charging = 1 << 2, Debug = 1 << 3, Logging = 1 << 4 };

	Status(uint8_t op_mode, NLG5& nlg5) : op_mode { op_mode }, nlg5 { nlg5 } { // get rid of logging bool and just use op mode
		OpenAIR();
		OpenPRE();
		SetFanDutyCycle(kFanLowDutyCycle);
	};

	uint8_t op_mode;
	bool manual_mode { false };

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

	void SetFanDutyCycle(uint8_t duty_cycle = kFanLowDutyCycle) {
		if (duty_cycle > kFanDCMax)
			duty_cycle = kFanDCMax;
		else if (duty_cycle < kFanDCMin)
			duty_cycle = kFanDCMin;

		if (!manual_mode);
		// TODO pwm_channel_update_duty(PWM, &g_pwm_channel_fan, dc);
	}

	constexpr uint8_t CalcDutyCycle() { // TODO could be private if the fan stuff was reworked a bit
		if (max_temp > kT2DCHighTemp)
			return kFanDCMax;
		else if (max_temp < kT2DCLowTemp)
			return kFanLowDutyCycle;
		else
			return (max_temp * kT2DC_M) + (kFanLowDutyCycle - kT2DC_M * kT2DCLowTemp);
	};

	void SetMinVoltage(uint16_t min_voltage, uint8_t min_voltage_id) {
		this->min_voltage = min_voltage;
		this->min_voltage_id = min_voltage_id;
		if (this->min_voltage < kLimpMinVoltage) { // TODO might be checking this too often
			if (++limp_counter > kLimpCountLimit)
				limp_counter += 9;
		} else if (limp_counter > 0)
			--limp_counter;
#if TEST_UNDERVOLTAGE
		ErrorCheck(Status::Undervoltage, min_voltage < kMinVoltage);
#endif
	}

	void SetMaxVoltage(uint16_t max_voltage, uint8_t max_voltage_id) {
		this->max_voltage = max_voltage;
		this->max_voltage_id = max_voltage_id;
		if (this->max_voltage > kChargerDis)
			nlg5.ctrl = 0;
		else if (this->max_voltage < kChargerEn)
			nlg5.ctrl = NLG5::C_C_EN;
#if TEST_OVERVOLTAGE
		ErrorCheck(Status::Overvoltage, this->max_voltage > kMaxVoltage);
#endif
	}

	void SetMinTemp(int16_t min_temp, uint8_t min_temp_id) {
		this->min_temp = min_temp;
		this->min_temp_id = min_temp_id;
#if TEST_UNDERTEMPERATURE
		ErrorCheck(Status::Undertemp, this->min_temp < kMinTemp);
#endif
	}

	void SetMaxTemp(int16_t max_temp, uint8_t max_temp_id) {
		this->max_temp = max_temp;
		this->max_temp_id = max_temp_id;
#if TEST_OVERTEMPERATURE
		ErrorCheck(Status::Overtemp, this->max_temp > kMaxTemp);
#endif
#if TEST_OVERTEMPERATURE_CHARGING
		if (op_mode & Charging)
			ErrorCheck(Status::OvertempCharging, this->max_temp > kMaxChargeTemp);
#endif
	}

	auto GetLimping() const {
		return limp_counter > kLimpCountLimit;
	}

	int8_t SetCurrent(int32_t raw_current) {
		current = static_cast<float>(raw_current) / 1000;
		received_update = true;
#if TEST_OVERCURRENT
		ErrorCheck(Status::Overcurrent, current > kMaxCurrent);
#endif
		return 0;
	}

	void SetAccuVoltage(int32_t raw_voltage) {
		AccuVoltage = static_cast<float>(raw_voltage) / 1000;
		received_update = true;
	}

	// TODO This used to be precharge compare, kinda
	int8_t SetAccuVoltage2(int32_t raw_voltage) {
		AccuVoltage2 = static_cast<float>(raw_voltage) / 1000;
		received_update = true;

#if TEST_ACCU_UNDERVOLTAGE
		ErrorCheck(Status::AccuUndervoltage, AccuVoltage2 < kAccuMinVoltage);
#endif

		float precharge_percentage = AccuVoltage * 100 / AccuVoltage2;
		if (precharge_percentage >= 95 && CheckVoltageMatch() && AccuVoltage > kPrechargeMinStartVoltage && AccuVoltage2 > kPrechargeMinStartVoltage)
			ClosePRE();
		else if (AccuVoltage < kPrechargeMaxEndVoltage || AccuVoltage2 < kPrechargeMaxEndVoltage) // TODO think there was a bug here in old system
			OpenPRE();

		return 0;
	}

	void CalcPower() {
		power = current * sum_of_cells / 10000;
#if TEST_OVERPOWER
		ErrorCheck(Status::Overpower, power > kMaxPower);
#endif
	}

	/* Test accumulator data against limit values. (0: OK, -1 FAIL) */
	/* MAYBE WE DON'T WANT 50% ERRORS TO BE ALLOWED */
	int8_t CheckIVTLost() {
#if IVT_TIMEOUT
		ErrorCheck(Status::IVTLost, received_update);
#endif
		received_update = false;
		return 0;
	}

	uint32_t uptime { 0 };
	uint32_t pec_counter { 0 };
	bool precharge_flag { false };
	bool safe_state_executed { 0 };
	ErrorEvent last_error { Undefined };
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

	int32_t current = 0;
	int32_t power = 0;
	uint32_t limp_counter = 0;

	float AccuVoltage { 0.0 };
	float AccuVoltage2 { 0.0 };

private:
	NLG5& nlg5;
	uint32_t error_counters[NumberOfErrors] = { 0 };
	uint32_t pec_counter_last { 0 };
	bool received_update = false;
	float pec_average { 0 };
	bool tested { false };

	bool CheckVoltageMatch() {
		float percentage = AccuVoltage * 100 / (sum_of_cells / 10000) - 100;
		return percentage < 10 && percentage > -10;
	}

	int8_t ErrorCheck(ErrorEvent e, bool error) {
		if (error) {
			if (++error_counters[e] > kErrorLimit || !tested) {
				GoToSafeState(e);
				return -1;
			}
		} else if (error_counters[e] > 0)
			--error_counters[e];

		return 0;
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
		last_error = e;
	}

	static constexpr uint16_t kT2DCLowTemp { 2000 };
	static constexpr uint16_t kT2DCHighTemp { 6000 };
	static constexpr  uint8_t kFanLowDutyCycle { 10 };
	static constexpr  uint8_t kFanHighDutyCycle { 100 };
	static constexpr  uint8_t kErrorLimit { 2 };
	static constexpr  uint8_t kFanDCMax { 100 };
	static constexpr  uint8_t kFanDCMin { 0 };

	static constexpr int32_t kMaxPower { 8000000 };
	static constexpr uint16_t kMaxVoltage = 42000;
	static constexpr uint16_t kMinVoltage = 31000;
	static constexpr int16_t kMaxTemp = 5900;
	static constexpr int16_t kMinTemp = -1500;
	static constexpr int16_t kMaxChargeTemp = 4400;
	static constexpr uint16_t kLimpMinVoltage = 34000.0;
	static constexpr uint8_t kLimpCountLimit { 2 };
	static constexpr float kMaxCurrent { 180.0 };
	static constexpr uint16_t kChargerDis { 41800 };
	static constexpr uint16_t kChargerEn { 41500 };
	static constexpr float kAccuMinVoltage { 490.0 };
	static constexpr float kPrechargeMinStartVoltage = 470.0;
	static constexpr float kPrechargeMaxEndVoltage = 450.0;
	static constexpr uint8_t kErrorLimitLost { 1 };
	static constexpr float kT2DC_M { static_cast<float>(kFanHighDutyCycle - kFanLowDutyCycle) / (kT2DCHighTemp - kT2DCLowTemp) };
};

#endif /* STATUS_H_ */
