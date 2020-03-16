/*
 * LTC6820.h
 *
 *  Created on: 12 Mar 2020
 *      Author: Joshua
 */

#ifndef LTC6811_H_
#define LTC6811_H_

#include "stm32f4xx_hal.h"
#include "Status.h"
#include "NLG5.h"
#include <array>
#include "Reverse.h"
#include <algorithm>

#define IC_NUM 12

/* Timing of states (in microseconds) */ // TODO need to implement a microsecond delay
#define T_WAKE_MAX		400
#define T_READY			10
#define T_IDLE_MIN		4300
#define T_REFUP_MAX		4400

/* Measurement + Calibration Cycle Time When Starting from the REFUP State in Fast Mode */
#define T_CYCLE_FAST_MAX	1185	// Measure 12 Cells

/* Conversion mode */
enum class Mode { Fast = 1, Normal, Filtered };

/* Conversion channels */
enum class CellCh { All, OneAndSeven, TwoAndEight, ThreeAndNine, FourAndTen, FiveAndEleven, SixAndTwelve };

/* Conversion channels */
enum class AuxCh  { All, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, VREF2 };

/* Conversion channels */
enum class STSCh  { All, SOC, ITMP, VA, VD };

/* Controls if Discharging transistors are enabled or disabled during Cell conversions. */
enum class DCP { Disabled, Enabled };

/* Relevant CAN codes */
#define CAN_ID_VOLT_TOTAL	11
#define CAN_ID_VOLT			1912
#define CAN_ID_TEMP			1948

enum RegisterLength { Aux = 6, CfgTx = 6, CfgRx = 8, Voltage = 12, Temp = 6 };

using VoltageRegisters = uint16_t[IC_NUM][Voltage];
using TempRegisters = int16_t[IC_NUM][Temp];
using CfgTxRegisters = uint8_t[IC_NUM][CfgTx];
using CfgRxRegisters = uint8_t[IC_NUM][CfgRx];

class LTC6820 {
public:
	LTC6820(SPI_HandleTypeDef& hspi,
			Status& status,
			NLG5& nlg5,
			CAN_HandleTypeDef& hcan,
			Mode mode = Mode::Normal, DCP dcp = DCP::Disabled, CellCh cell = CellCh::All, AuxCh aux = AuxCh::All, STSCh sts = STSCh::All)
: 	hspi{ hspi }, status{ status }, nlg5 { nlg5 }, hcan { hcan } {
	uint16_t pec;
	uint8_t md_bits = (static_cast<uint8_t>(mode) & 0x02) >> 1;

	ADCV[0]   = md_bits + 0x02;
	ADAX[0]   = md_bits + 0x04;
	ADSTAT[0] = md_bits + 0x04;

	md_bits   = (static_cast<uint8_t>(mode) & 0x01) << 7;
	ADCV[1]   =	md_bits	+ 0x60 + (static_cast<uint8_t>(dcp) << 4) + static_cast<uint8_t>(cell);
	ADAX[1]   =	md_bits	+ 0x60 + static_cast<uint8_t>(aux);
	ADSTAT[1] = md_bits + 0x68 + static_cast<uint8_t>(sts);

	pec = PEC15Calc(ADCV, 2);
	ADCV[2] = static_cast<uint8_t>(pec >> 8);
	ADCV[3] = static_cast<uint8_t>(pec);

	pec = PEC15Calc(ADAX, 2);
	ADAX[2] = static_cast<uint8_t>(pec >> 8);
	ADAX[3] = static_cast<uint8_t>(pec);

	pec = PEC15Calc(ADAX, 2);
	ADSTAT[2] = static_cast<uint8_t>(pec >> 8);
	ADSTAT[3] = static_cast<uint8_t>(pec);
}

uint8_t discharge_mode { 0 };

/* Wake the LTC SPI from IDLE state. */
void WakeFromIdle(void) {
	uint8_t data = 0xFF;
	HAL_SPI_Transmit(&hspi, &data, 1, 10);
	HAL_Delay(IC_NUM * T_READY);
}

/*  Wake the LTC from SLEEP state.

	Excerpt from 6804 datasheet:
	"If there are ‘N’ devices in the stack, all the devices are powered up
	within the time	N * tWAKE or N * tREADY, depending on the Core State.
	For large stacks, the time N * tWAKE may be equal to or larger
	than tIDLE. In this case, after waiting longer than the time
	of N * tWAKE, the host may send another dummy byte and
	wait for the time N * tREADY, in order to ensure that all
	devices are in the READY state."
 */
void WakeFromSleep(void) {
	uint8_t data = 0xFF;
	HAL_SPI_Transmit(&hspi, &data, 1, 10);
	HAL_Delay(IC_NUM * T_WAKE_MAX);
#if (IC_NUM * T_WAKE_MAX >= T_IDLE_MIN)
	HAL_SPI_Transmit(&hspi, &data, 1, 10);
	HAL_Delay(IC_NUM * T_READY);
#endif
}

/* Write the LTC6804 configuration register. */
void WriteConfigRegister(CfgTxRegisters& cfg_tx) {
	buffer = { 0x00, 0x01, 0x3D, 0x6E };
	uint8_t buffer_index = 4; // One past the elements added in the line above.

	for (auto const& row : reverse(cfg_tx)) {
		for (auto const& byte : row)
			buffer[buffer_index++] = byte;

		auto cfg_pec = PEC15Calc(row, sizeof(row));
		buffer[buffer_index++] = static_cast<uint8_t>(cfg_pec >> 8);
		buffer[buffer_index++] = static_cast<uint8_t>(cfg_pec);
	}

	WakeFromIdle();
	HAL_SPI_Transmit(&hspi, buffer.data(), 100, 10);
}

void ReadWriteConfigRegisters(CfgTxRegisters& cfg_tx, CfgRxRegisters& cfg_rx, const VoltageRegisters& cell_data) {
	uint16_t DCCx = 0;
	uint8_t index = 0;
	uint16_t avg_cell = status.sum_of_cells / 144;

	if (status.op_mode & Status::Balance) {
		switch (discharge_mode) {
		case 0: //Discharge all above (min_voltage + delta)
			for (const auto& row : cell_data) {
				for (const auto& voltage : row )
					if (voltage > status.min_voltage + kDelta)
						DCCx |= 1 << std::distance(row, &voltage); // Set bit

				cfg_tx[index][4] = DCCx & 0xFF;
				cfg_tx[index++][5] = DCCx >> 8 & 0xFF; // TODO All of these used to be 0x0F. Seemed wrong?
				DCCx = 0;
			}
			break;

		case 1: //Discharge only the max_voltage cell.
			// TODO could be wrongly implemented, and id could be used wrong.
			if (status.max_voltage - status.min_voltage > kDelta) {
				DCCx |= 1 << status.max_voltage_id % IC_NUM;
				cfg_tx[status.max_voltage_id / IC_NUM][4] = DCCx & 0xFF;
				cfg_tx[status.max_voltage_id / IC_NUM][5] = DCCx >> 8 & 0xFF;
			}
			break;

		case 2: //Discharge all cells that are above (average cell voltage + delta)
			for (const auto& row : cell_data) {
				for (const auto& voltage : row )
					if (voltage > avg_cell + kDelta)
						DCCx |= 1 << std::distance(row, &voltage); // Set bit

				cfg_tx[index][4] = DCCx & 0xFF;
				cfg_tx[index++][5] = DCCx >> 8 & 0xFF; // TODO All of these used to be 0x0F. Seemed wrong?
				DCCx = 0;
			}
			break;
		}
	} else
		for (auto& row : cfg_tx)
			row[4] = row[5] = 0;

	WakeFromSleep();
	WriteConfigRegister(cfg_tx);
	HAL_Delay(500);
	ReadConfigRegister(cfg_rx);
}

/* Reads configuration registers of a LTC6804 daisy chain. */
int8_t ReadConfigRegister(CfgRxRegisters& cfg_rx) {
	int8_t pec_error;
	buffer = {0x00, 0x02, 0x2b, 0x0a};

	WakeFromIdle();
	HAL_SPI_TransmitReceive(&hspi, buffer.data(), *cfg_rx, 96, 10);

	for (const auto& row : cfg_rx)
		if ((row[6] << 8 | row[7]) != PEC15Calc(row, 6))
			pec_error = -1;

	return pec_error;
}


/* Read all cell voltages from LTC-6811 daisy chain.
 * Up to five consecutive reads are performed in case a CRC (PEC) check fails.
 * -1 on pec error, 0 on successful read. */
int8_t ReadVoltage(VoltageRegisters& cell_data) {
	WakeFromSleep();
	adcv();
	HAL_Delay((T_REFUP_MAX + T_CYCLE_FAST_MAX) / 1000); // Was a microsecond delay on old board.
	WakeFromIdle(); // Make sure isoSPI port is active

	for (uint8_t i = 0; i < 5; ++i)	{
		if (ReadVoltageHelper(cell_data) == Status::PecError)
			status.IncreasePecCounter();
		else
			return 0;
	}

	status.GoToSafeState(Status::PecError);
	return -1;
}

/* Read all auxiliary voltages from LTC-6811 daisy chain.
 * Up to five consecutive reads are performed in case a CRC (PEC) check fails.
 * -1 on pec error, 0 on successful read.
 */
int8_t ReadTemperature(TempRegisters& temp_data) {
	WakeFromSleep();
	adax();
	HAL_Delay((T_REFUP_MAX + T_CYCLE_FAST_MAX) / 1000);
	WakeFromIdle();

	for (uint8_t i = 0; i < 5; i++)	{
		if (ReadTemperatureHelper(temp_data) == Status::PecError)
			status.IncreasePecCounter();
		else
			return 0;
	}

	status.GoToSafeState(Status::PecError);
	return -1;
}

/* Clears the LTC6804 cell voltage registers. */
void ClearVoltageRegisters(void) {
	buffer = { 0x07, 0x11 };
	auto buffer_pec = PEC15Calc(buffer.data(), 2);
	buffer[2] = static_cast<uint8_t>(buffer_pec >> 8);
	buffer[3] = static_cast<uint8_t>(buffer_pec);

	WakeFromIdle();
	HAL_SPI_Transmit(&hspi, buffer.data(), 4, 10);
}


/* Clears the LTC6804 Auxiliary registers. */
void ClearAuxRegisters(void) {
	buffer = { 0x07, 0x12 };
	auto buffer_pec = PEC15Calc(buffer.data(), 2);
	buffer[2] = static_cast<uint8_t>(buffer_pec >> 8);
	buffer[3] = static_cast<uint8_t>(buffer_pec);

	WakeFromIdle();
	HAL_SPI_Transmit(&hspi, buffer.data(), 4, 10);
}

/* Reads status registers A of a LTC6804 daisy chain. */
int8_t ReadStatusRegisterA(uint8_t (&r_config)[IC_NUM][8]) {
	int8_t pec_error = 0;
	buffer = { 0x00, 0x10, 0xED, 0x72 };

	WakeFromIdle();
	HAL_SPI_TransmitReceive(&hspi, buffer.data(), *r_config, 96, 10);

	for (const auto& row : r_config)
		if ((row[6] << 8 | row[7]) != PEC15Calc(row, 6))
			pec_error = -1;

	return pec_error;
}


/* Reads status registers B of a LTC6804 daisy chain. */
int8_t ReadStatusRegisterB(uint8_t (&r_config)[IC_NUM][8]) {
	int8_t pec_error = 0;
	buffer = { 0x00, 0x12, 0x70, 0x24 };

	WakeFromIdle();
	HAL_SPI_TransmitReceive(&hspi, buffer.data(), *r_config, 96, 10);

	for (const auto& row : r_config)
		if ((row[6] << 8 | row[7]) != PEC15Calc(row, 6))
			pec_error = -1;

	return pec_error;
}

auto GetLimping() const {
	return limp_counter > kLimpCountLimit;
}

private:
SPI_HandleTypeDef& hspi;
Status& status;
NLG5& nlg5;
CAN_HandleTypeDef& hcan;
CAN_TxHeaderTypeDef TxHeader;

std::array<uint8_t, 100> buffer;

uint8_t ADCV[4]; 	// Cell Voltage conversion command
uint8_t ADAX[4]; 	// GPIO conversion command
uint8_t ADSTAT[4]; 	// STAT conversion command

int32_t current = 0;
int32_t power = 0;
uint32_t limp_counter = 0;

void adcv(void); /* Starts cell voltage conversion. */
void adax(void); /* Start an GPIO Conversion. */
void adstat(void); /* Start a STATUS register Conversion. */
void ReadVoltageRegister(uint8_t reg); /* Read the raw data from the LTC6804 cell voltage register. */
uint8_t ReadVoltageHelper(VoltageRegisters& cell_data); /* Reads and parses the LTC6804 cell voltage registers. */
void ReadAuxRegister(uint8_t reg); /* Read the raw data from the LTC6804 auxiliary register.*/
int8_t ReadTemperatureHelper(TempRegisters& temp_data); /* Reads and parses the LTC6804 auxiliary registers. */
int16_t CalcTemp(uint16_t ntc_voltage); /* Calculates the temperature from thermistor voltage using lookup table. */
uint16_t PEC15Calc(uint8_t const * const data, size_t data_length);

static constexpr uint8_t kDelta = 100;
static constexpr int32_t kMaxPower { 8000000 };
static constexpr uint16_t kMaxVoltage = 42000;
static constexpr uint16_t kMinVoltage = 31000;
static constexpr int16_t kMaxTemp = 5900;
static constexpr int16_t kMinTemp = -1500;
static constexpr int16_t kMaxChargeTemp = 4400;
static constexpr uint16_t kLimpMinVoltage = 34000.0;
static constexpr uint8_t kLimpCountLimit { 2 };
static constexpr uint16_t kChargerDis { 41800 };
static constexpr uint16_t kChargerEn { 41500 };
};

#endif /* LTC6811_H_ */
