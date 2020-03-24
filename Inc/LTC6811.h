/*
 * LTC6820.h
 *
 *  Created on: 12 Mar 2020
 *      Author: Joshua
 */

#ifndef LTC6811_H_
#define LTC6811_H_

//#include "stm32f4xx_hal.h"
#include "Status.h"
#include <array>
#include <gsl/span>
#include <algorithm>

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

static constexpr uint8_t kBytesPerRegister{ 8 };
static constexpr uint8_t kDaisyChainLength{ 12 };
static constexpr uint8_t kCommandLength{ 4 };

using LTC6811Command = std::array<uint8_t, kCommandLength>;

template <typename T>
using LTC6811Register = std::array<std::array<T, kBytesPerRegister / sizeof(T)>, kDaisyChainLength>;

template <typename T>
auto span(LTC6811Register<T>& data) {
	// This is naughty!
	return gsl::span<T>(data.data()->data(), data.data()->data() + kDaisyChainLength * kBytesPerRegister / sizeof(T));
}

class LTC6811 {
public:
	LTC6811(SPI_HandleTypeDef& hspi,
			Status& status,
			CAN_HandleTypeDef& hcan,
			Mode mode = Mode::Normal, DCP dcp = DCP::Disabled, CellCh cell = CellCh::All, AuxCh aux = AuxCh::All, STSCh sts = STSCh::All)
: 	hspi{ hspi }, status{ status }, hcan { hcan } {
	uint8_t md_bits = (static_cast<uint8_t>(mode) & 0x02) >> 1;
	uint16_t pec{ 0 };
	ADCV[0]   = md_bits + 0x02;
	ADAX[0]   = md_bits + 0x04;
	ADSTAT[0] = md_bits + 0x04;

	md_bits   = (static_cast<uint8_t>(mode) & 0x01) << 7;
	ADCV[1]   =	md_bits	+ 0x60 + (static_cast<uint8_t>(dcp) << 4) + static_cast<uint8_t>(cell);
	ADAX[1]   =	md_bits	+ 0x60 + static_cast<uint8_t>(aux);
	ADSTAT[1] = md_bits + 0x68 + static_cast<uint8_t>(sts);

	pec = PEC15Calc(ADCV);
	ADCV[2] = static_cast<uint8_t>(pec >> 8);
	ADCV[3] = static_cast<uint8_t>(pec);

	pec = PEC15Calc(ADAX);
	ADAX[2] = static_cast<uint8_t>(pec >> 8);
	ADAX[3] = static_cast<uint8_t>(pec);

	pec = PEC15Calc(ADAX);
	ADSTAT[2] = static_cast<uint8_t>(pec >> 8);
	ADSTAT[3] = static_cast<uint8_t>(pec);
}

static constexpr uint8_t kCellsInReg { 3 };
static constexpr uint8_t kDelta = 100;

void WakeFromSleep(void) {
	uint8_t data = 0xFF;

	HAL_SPI_Transmit(&hspi, &data, 1, 10);
	HAL_Delay(kDaisyChainLength * T_WAKE_MAX);

#if (kDaisyChainLength * T_WAKE_MAX >= T_IDLE_MIN)
	HAL_SPI_Transmit(&hspi, &data, 1, 10);
	HAL_Delay(kDaisyChainLength * T_READY);
#endif
}

void WakeFromIdle(void) {
	uint8_t data = 0xFF;

	HAL_SPI_Transmit(&hspi, &data, 1, 10);
	HAL_Delay(kDaisyChainLength * T_READY);
}


/** Write Register Function Overloads **
 ** Return 0 if success, 1 if failure. */

/* Takes 2D LTC6811Register array of T type and serialises the data as  bytes */
template <typename T>
uint8_t WriteRegister(LTC6811Command const& command, LTC6811Register<T>& reg) {
	return WriteRegister(command, gsl::as_bytes(span(reg)));
}

uint8_t WriteRegister(LTC6811Command const& command, gsl::span<const gsl::byte> data) {
	//WakeFromSleep(); probably want this
	WakeFromIdle();

	// NSS PIN (PA4) low
	if (HAL_SPI_Transmit(&hspi, command.data(), kCommandLength, 100) != HAL_ERROR)
		if (HAL_SPI_Transmit(&hspi, (uint8_t *) data.data(), kBytesPerRegister * kDaisyChainLength, 100) == HAL_ERROR) //TODO dunno if this data thing will work...
			return 1;
	return 0;
	// NSS PIN (PA4) high
}

/** Read Register Function Overloads  **
 ** Return 0 if success, 1 if failure. */

/* Takes 2D LTC6811Register array of T type and serialises the data as bytes */
template <typename T>
uint8_t ReadRegister(LTC6811Command const& command, LTC6811Register<T>& reg) {
	return ReadRegister(command, gsl::as_writable_bytes(span(reg)));
}

uint8_t ReadRegister(LTC6811Command const& command, gsl::span<gsl::byte> data) {
	//WakeFromSleep(); probably want this
	WakeFromIdle();
	// NSS PIN (PA4) low
	if (HAL_SPI_Transmit(&hspi, command.data(), kCommandLength, 100) != HAL_ERROR) {
		// NSS PIN (PA4) high
		if (HAL_SPI_Receive(&hspi, (uint8_t *) data.data(), kBytesPerRegister * kDaisyChainLength, 100) == HAL_ERROR)
			return 1;
	}
	return 0;
}

/* Write to the configuration registers of the LTC6811s in the daisy chain.
 * The configuration is written in descending order so the 2D array is reversed before writing. */
void WriteConfigRegister(LTC6811Register<uint8_t>& cfg_tx) {
	LTC6811Command command = { 0x00, 0x01, 0x3D, 0x6E };
	std::reverse(std::begin(cfg_tx), std::end(cfg_tx));
	WriteRegister(command, cfg_tx);
}

/* Read configuration registers of a LTC6811 daisy chain */
void ReadConfigRegister(LTC6811Register<uint8_t>& cfg_rx) {
	LTC6811Command command = { 0x00, 0x02, 0x2B, 0x0A };
	ReadRegister(command, cfg_rx);
}

/* Read all cell voltages from LTC6811 daisy chain.
 * Up to five consecutive reads are performed in case of PEC errors.
 * Return 1 on PEC error, 0 on successful read. */
uint8_t ReadVoltage(std::array<LTC6811Register<uint16_t>, 4>& cell_data) {
	WakeFromSleep();
	adcv();
	HAL_Delay((T_REFUP_MAX + T_CYCLE_FAST_MAX) / 1000); // Was a microsecond delay on old board.
	WakeFromIdle(); // Make sure isoSPI port is active. Probably not necessary?

	for (uint8_t i = 0; i < 5; ++i)	{
		if (ReadVoltageHelper(cell_data))
			status.IncreasePecCounter();
		else
			return 0;
	}
	return 1;
}

/* Read all auxiliary voltages from LTC6811 daisy chain.
 * Up to five consecutive reads are performed in case of PEC errors.
 * Return 1 on PEC error, 0 on successful read. */
uint8_t ReadTemperature(std::array<LTC6811Register<int16_t>, 2>& temp_data) {
	WakeFromSleep();
	adax();
	HAL_Delay((T_REFUP_MAX + T_CYCLE_FAST_MAX) / 1000);
	WakeFromIdle();

	for (uint8_t i = 0; i < 5; ++i)	{
		if (ReadTemperatureHelper(temp_data))
			status.IncreasePecCounter();
		else
			return 0;
	}
	return 1;
}

/* Clear the LTC6811 cell voltage registers. */
void ClearVoltageRegisters(void) {
	LTC6811Command command = { 0x07, 0x11 };
	auto result = PEC15Calc(command);
	command[2] = static_cast<uint8_t>(result >> 8);
	command[3] = static_cast<uint8_t>(result);

	WakeFromIdle();
	HAL_SPI_Transmit(&hspi, command.data(), 4, 10);
}


/* Clear the LTC6811 Auxiliary registers. */
void ClearAuxRegisters(void) {
	LTC6811Command command = { 0x07, 0x12 };
	auto result = PEC15Calc(command);
	command[2] = static_cast<uint8_t>(result >> 8);
	command[3] = static_cast<uint8_t>(result);

	WakeFromIdle();
	HAL_SPI_Transmit(&hspi, command.data(), 4, 10);
}

/* Read status registers A of a LTC6811 daisy chain. */
uint8_t ReadStatusRegisterA(LTC6811Register<uint8_t>& r_config) {
	LTC6811Command command = { 0x00, 0x10, 0xED, 0x72 };
	return ReadRegister(command, r_config);
}


/* Read status registers B of a LTC6811 daisy chain. */
uint8_t ReadStatusRegisterB(LTC6811Register<uint8_t>& r_config) {
	LTC6811Command command = { 0x00, 0x12, 0x70, 0x24 };
	return ReadRegister(command, r_config);
}

private:
SPI_HandleTypeDef& hspi;
Status& status;
CAN_HandleTypeDef& hcan;
CAN_TxHeaderTypeDef TxHeader;

LTC6811Command ADCV; 	// Cell Voltage conversion command
LTC6811Command ADAX; 	// GPIO conversion command
LTC6811Command ADSTAT; 	// STAT conversion command

void adcv(void); /* Starts cell voltage conversion. */
void adax(void); /* Start an GPIO Conversion. */
void adstat(void); /* Start a STATUS register Conversion. */
uint8_t ReadVoltageRegister(uint8_t reg_id, LTC6811Register<uint16_t>& reg); /* Read the raw data from the LTC6804 cell voltage register. */
uint8_t ReadVoltageHelper(std::array<LTC6811Register<uint16_t>, 4>& cell_data); /* Reads and parses the LTC6804 cell voltage registers. */
uint8_t ReadAuxRegister(uint8_t reg_id, LTC6811Register<int16_t>& reg); /* Read the raw data from the LTC6804 auxiliary register.*/
uint8_t ReadTemperatureHelper(std::array<LTC6811Register<int16_t>, 2>& temp_data); /* Reads and parses the LTC6804 auxiliary registers. */
int16_t CalcTemp(uint16_t ntc_voltage); /* Calculates the temperature from thermistor voltage using lookup table. */

static uint16_t crc15Table[256];

/* 	Calculates and returns the CRC15 */
template <typename T, size_t S>
static T PEC15Calc(std::array<T, S> data) {
	uint16_t remainder = 16, addr;
	auto data_as_bytes = gsl::as_bytes(gsl::span<T>(data));

	for (uint8_t i = 0; i < data_as_bytes.size() - 2; ++i) {
		addr = (remainder >> 7 ^ (uint8_t)data_as_bytes[i]) & 0xFF;
		remainder <<= 8 ^ LTC6811::crc15Table[addr];
	}

	return remainder * 2;
}
};

#endif /* LTC6811_H_ */
