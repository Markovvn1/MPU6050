#include "mpu6050.hpp"

#include <cmath>

using namespace std;

MPU6050::MPU6050(const char* fileName, unsigned char addr) : DeviceI2C(addr, fileName)
{
	data = shared_ptr<__MPU6050>(new __MPU6050);
}

MPU6050::~MPU6050()
{
	if (data.use_count() != 1) return;
}

bool MPU6050::begin(mpu6050_dps_t scale, mpu6050_range_t range, int tryTime)
{
    start();

    // Reset calibrate values
    data->dg.x = 0;
    data->dg.y = 0;
    data->dg.z = 0;
    data->useCalibrate = false;

    // Reset threshold values
    data->tg.x = 0;
    data->tg.y = 0;
    data->tg.z = 0;
    data->actualThreshold = 0;

    // Check MPU6050 Who Am I Register
    if (fastRegister8(MPU6050_REG_WHO_AM_I) != 0x68) return false;

    // Set Clock Source
    setClockSource(MPU6050_CLOCK_PLL_XGYRO);

    // Set Scale & Range
    setScale(scale);
    setRange(range);

    // Disable Sleep Mode
    setSleepEnabled(false);

    return true;
}

void MPU6050::setScale(mpu6050_dps_t scale)
{
    uint8_t value;

    switch (scale)
    {
	case MPU6050_SCALE_250DPS:
		data->dpsPerDigit = .007633f;
	    break;
	case MPU6050_SCALE_500DPS:
		data->dpsPerDigit = .015267f;
	    break;
	case MPU6050_SCALE_1000DPS:
		data->dpsPerDigit = .030487f;
	    break;
	case MPU6050_SCALE_2000DPS:
		data->dpsPerDigit = .060975f;
	    break;
	default:
	    break;
    }

    value = readRegister8(MPU6050_REG_GYRO_CONFIG);
    value &= 0b11100111;
    value |= (scale << 3);
    writeRegister8(MPU6050_REG_GYRO_CONFIG, value);
}

mpu6050_dps_t MPU6050::getScale()
{
    uint8_t value;
    value = readRegister8(MPU6050_REG_GYRO_CONFIG);
    value &= 0b00011000;
    value >>= 3;
    return (mpu6050_dps_t)value;
}

void MPU6050::setRange(mpu6050_range_t range)
{
    uint8_t value;

    switch (range)
    {
	case MPU6050_RANGE_2G:
		data->rangePerDigit = .000061f;
	    break;
	case MPU6050_RANGE_4G:
		data->rangePerDigit = .000122f;
	    break;
	case MPU6050_RANGE_8G:
		data->rangePerDigit = .000244f;
	    break;
	case MPU6050_RANGE_16G:
		data->rangePerDigit = .0004882f;
	    break;
	default:
	    break;
    }

    value = readRegister8(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b11100111;
    value |= (range << 3);
    writeRegister8(MPU6050_REG_ACCEL_CONFIG, value);
}

mpu6050_range_t MPU6050::getRange()
{
    uint8_t value;
    value = readRegister8(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b00011000;
    value >>= 3;
    return (mpu6050_range_t)value;
}

void MPU6050::setDHPFMode(mpu6050_dhpf_t dhpf)
{
    uint8_t value;
    value = readRegister8(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b11111000;
    value |= dhpf;
    writeRegister8(MPU6050_REG_ACCEL_CONFIG, value);
}

void MPU6050::setDLPFMode(mpu6050_dlpf_t dlpf)
{
    uint8_t value;
    value = readRegister8(MPU6050_REG_CONFIG);
    value &= 0b11111000;
    value |= dlpf;
    writeRegister8(MPU6050_REG_CONFIG, value);
}

void MPU6050::setClockSource(mpu6050_clockSource_t source)
{
    uint8_t value;
    value = readRegister8(MPU6050_REG_PWR_MGMT_1);
    value &= 0b11111000;
    value |= source;
    writeRegister8(MPU6050_REG_PWR_MGMT_1, value);
}

mpu6050_clockSource_t MPU6050::getClockSource()
{
    uint8_t value;
    value = readRegister8(MPU6050_REG_PWR_MGMT_1);
    value &= 0b00000111;
    return (mpu6050_clockSource_t)value;
}

bool MPU6050::getSleepEnabled()
{
    return readRegisterBit(MPU6050_REG_PWR_MGMT_1, 6);
}

void MPU6050::setSleepEnabled(bool state)
{
    writeRegisterBit(MPU6050_REG_PWR_MGMT_1, 6, state);
}

bool MPU6050::getIntZeroMotionEnabled()
{
    return readRegisterBit(MPU6050_REG_INT_ENABLE, 5);
}

void MPU6050::setIntZeroMotionEnabled(bool state)
{
    writeRegisterBit(MPU6050_REG_INT_ENABLE, 5, state);
}

bool MPU6050::getIntMotionEnabled()
{
    return readRegisterBit(MPU6050_REG_INT_ENABLE, 6);
}

void MPU6050::setIntMotionEnabled(bool state)
{
    writeRegisterBit(MPU6050_REG_INT_ENABLE, 6, state);
}

bool MPU6050::getIntFreeFallEnabled()
{
    return readRegisterBit(MPU6050_REG_INT_ENABLE, 7);
}

void MPU6050::setIntFreeFallEnabled(bool state)
{
    writeRegisterBit(MPU6050_REG_INT_ENABLE, 7, state);
}

uint8_t MPU6050::getMotionDetectionThreshold()
{
    return readRegister8(MPU6050_REG_MOT_THRESHOLD);
}

void MPU6050::setMotionDetectionThreshold(uint8_t threshold)
{
    writeRegister8(MPU6050_REG_MOT_THRESHOLD, threshold);
}

uint8_t MPU6050::getMotionDetectionDuration()
{
    return readRegister8(MPU6050_REG_MOT_DURATION);
}

void MPU6050::setMotionDetectionDuration(uint8_t duration)
{
    writeRegister8(MPU6050_REG_MOT_DURATION, duration);
}

uint8_t MPU6050::getZeroMotionDetectionThreshold()
{
    return readRegister8(MPU6050_REG_ZMOT_THRESHOLD);
}

void MPU6050::setZeroMotionDetectionThreshold(uint8_t threshold)
{
    writeRegister8(MPU6050_REG_ZMOT_THRESHOLD, threshold);
}

uint8_t MPU6050::getZeroMotionDetectionDuration()
{
    return readRegister8(MPU6050_REG_ZMOT_DURATION);
}

void MPU6050::setZeroMotionDetectionDuration(uint8_t duration)
{
    writeRegister8(MPU6050_REG_ZMOT_DURATION, duration);
}

uint8_t MPU6050::getFreeFallDetectionThreshold()
{
    return readRegister8(MPU6050_REG_FF_THRESHOLD);
}

void MPU6050::setFreeFallDetectionThreshold(uint8_t threshold)
{
    writeRegister8(MPU6050_REG_FF_THRESHOLD, threshold);
}

uint8_t MPU6050::getFreeFallDetectionDuration()
{
    return readRegister8(MPU6050_REG_FF_DURATION);
}

void MPU6050::setFreeFallDetectionDuration(uint8_t duration)
{
    writeRegister8(MPU6050_REG_FF_DURATION, duration);
}

bool MPU6050::getI2CMasterModeEnabled()
{
    return readRegisterBit(MPU6050_REG_USER_CTRL, 5);
}

void MPU6050::setI2CMasterModeEnabled(bool state)
{
    writeRegisterBit(MPU6050_REG_USER_CTRL, 5, state);
}

void MPU6050::setI2CBypassEnabled(bool state)
{
    return writeRegisterBit(MPU6050_REG_INT_PIN_CFG, 1, state);
}

bool MPU6050::getI2CBypassEnabled()
{
    return readRegisterBit(MPU6050_REG_INT_PIN_CFG, 1);
}

void MPU6050::setAccelPowerOnDelay(mpu6050_onDelay_t delay)
{
    uint8_t value;
    value = readRegister8(MPU6050_REG_MOT_DETECT_CTRL);
    value &= 0b11001111;
    value |= (delay << 4);
    writeRegister8(MPU6050_REG_MOT_DETECT_CTRL, value);
}

mpu6050_onDelay_t MPU6050::getAccelPowerOnDelay()
{
    uint8_t value;
    value = readRegister8(MPU6050_REG_MOT_DETECT_CTRL);
    value &= 0b00110000;
    return (mpu6050_onDelay_t)(value >> 4);
}

uint8_t MPU6050::getIntStatus()
{
    return readRegister8(MPU6050_REG_INT_STATUS);
}

Activites MPU6050::readActivites()
{
    uint8_t data = readRegister8(MPU6050_REG_INT_STATUS);

    this->data->a.isOverflow = ((data >> 4) & 1);
    this->data->a.isFreeFall = ((data >> 7) & 1);
    this->data->a.isInactivity = ((data >> 5) & 1);
    this->data->a.isActivity = ((data >> 6) & 1);
    this->data->a.isDataReady = ((data >> 0) & 1);

    data = readRegister8(MPU6050_REG_MOT_DETECT_STATUS);

    this->data->a.isNegActivityOnX = ((data >> 7) & 1);
    this->data->a.isPosActivityOnX = ((data >> 6) & 1);

    this->data->a.isNegActivityOnY = ((data >> 5) & 1);
    this->data->a.isPosActivityOnY = ((data >> 4) & 1);

    this->data->a.isNegActivityOnZ = ((data >> 3) & 1);
    this->data->a.isPosActivityOnZ = ((data >> 2) & 1);

    return this->data->a;
}

Vector3d<sint> MPU6050::readRawAccel()
{
	char t1 = MPU6050_REG_ACCEL_XOUT_H; write(&t1, 1);

	char t2[6]; read(t2, 6);

	Vector3d<sint> res;

	res.x = (t2[0] << 8) | (t2[1]);
	res.y = (t2[2] << 8) | (t2[3]);
	res.z = (t2[4] << 8) | (t2[5]);

    return res;
}

Vector3d<double> MPU6050::readNormalizeAccel()
{
	Vector3d<sint> raw = readRawAccel();
	Vector3d<double> res;

	res.x = raw.x * data->rangePerDigit * 9.80665f;
	res.y = raw.y * data->rangePerDigit * 9.80665f;
	res.z = raw.z * data->rangePerDigit * 9.80665f;

    return res;
}

Vector3d<double> MPU6050::readScaledAccel()
{
    Vector3d<sint> raw = readRawAccel();
    Vector3d<double> res;

    res.x = raw.x * data->rangePerDigit;
    res.y = raw.y * data->rangePerDigit;
    res.z = raw.z * data->rangePerDigit;

    return res;
}


Vector3d<sint> MPU6050::readRawGyro()
{
    char t1 = MPU6050_REG_GYRO_XOUT_H; write(&t1, 1);

    char t2[6]; read(t2, 6);

    Vector3d<sint> res;

    res.x = (t2[0] << 8) | (t2[1]);
    res.y = (t2[2] << 8) | (t2[3]);
    res.z = (t2[4] << 8) | (t2[5]);

    return res;
}

Vector3d<int> MPU6050::readCorrectGyro()
{
	Vector3d<sint> raw = readRawGyro();

	Vector3d<int> res;

	if (data->useCalibrate)
	{
		res.x = int(raw.x) - data->dg.x;
		res.y = int(raw.y) - data->dg.y;
		res.z = int(raw.z) - data->dg.z;
	}
	else
	{
		res.x = raw.x;
		res.y = raw.y;
		res.z = raw.z;
	}

	if (data->actualThreshold)
	{
		if (abs(res.x) < data->tg.x) res.x = 0;
		if (abs(res.y) < data->tg.y) res.y = 0;
		if (abs(res.z) < data->tg.z) res.z = 0;
	}

	return res;
}

Vector3d<double> MPU6050::readNormalizeGyro()
{
    Vector3d<int> raw = readCorrectGyro();

    Vector3d<double> res;

	res.x = raw.x * data->dpsPerDigit;
	res.y = raw.y * data->dpsPerDigit;
	res.z = raw.z * data->dpsPerDigit;

    return res;
}

float MPU6050::readTemperature()
{
    int16_t T;
    T = readRegister16(MPU6050_REG_TEMP_OUT_H);
    return (float)T/340 + 36.53;
}

int16_t MPU6050::getGyroOffsetX()
{
    return readRegister16(MPU6050_REG_GYRO_XOFFS_H);
}

int16_t MPU6050::getGyroOffsetY()
{
    return readRegister16(MPU6050_REG_GYRO_YOFFS_H);
}

int16_t MPU6050::getGyroOffsetZ()
{
    return readRegister16(MPU6050_REG_GYRO_ZOFFS_H);
}

void MPU6050::setGyroOffsetX(int16_t offset)
{
    writeRegister16(MPU6050_REG_GYRO_XOFFS_H, offset);
}

void MPU6050::setGyroOffsetY(int16_t offset)
{
    writeRegister16(MPU6050_REG_GYRO_YOFFS_H, offset);
}

void MPU6050::setGyroOffsetZ(int16_t offset)
{
    writeRegister16(MPU6050_REG_GYRO_ZOFFS_H, offset);
}

int16_t MPU6050::getAccelOffsetX()
{
    return readRegister16(MPU6050_REG_ACCEL_XOFFS_H);
}

int16_t MPU6050::getAccelOffsetY()
{
    return readRegister16(MPU6050_REG_ACCEL_YOFFS_H);
}

int16_t MPU6050::getAccelOffsetZ()
{
    return readRegister16(MPU6050_REG_ACCEL_ZOFFS_H);
}

void MPU6050::setAccelOffsetX(int16_t offset)
{
    writeRegister16(MPU6050_REG_ACCEL_XOFFS_H, offset);
}

void MPU6050::setAccelOffsetY(int16_t offset)
{
    writeRegister16(MPU6050_REG_ACCEL_YOFFS_H, offset);
}

void MPU6050::setAccelOffsetZ(int16_t offset)
{
    writeRegister16(MPU6050_REG_ACCEL_ZOFFS_H, offset);
}

// Calibrate algorithm
void MPU6050::calibrateGyro(int samples)
{
    // Set calibrate
	data->useCalibrate = true;

    // Reset values
    int sumX = 0;
    int sumY = 0;
    int sumZ = 0;
    long long sigmaX = 0;
    long long sigmaY = 0;
    long long sigmaZ = 0;

    // Read n-samples
    for (int i = 0; i < samples; i++)
    {
		Vector3d<sint> raw = readRawGyro();
		sumX += raw.x;
		sumY += raw.y;
		sumZ += raw.z;

		sigmaX += raw.x * raw.x;
		sigmaY += raw.y * raw.y;
		sigmaZ += raw.z * raw.z;

		usleep(5 * 1000);
    }

    // Calculate delta vectors
    data->dg.x = double(sumX) / samples;
    data->dg.y = double(sumY) / samples;
    data->dg.z = double(sumZ) / samples;

    // Calculate threshold vectors
    data->th.x = sqrt((double(sigmaX) / samples) - (data->dg.x * data->dg.x));
    data->th.y = sqrt((double(sigmaY) / samples) - (data->dg.y * data->dg.y));
    data->th.z = sqrt((double(sigmaZ) / samples) - (data->dg.z * data->dg.z));

    // If already set threshold, recalculate threshold vectors
    if (data->actualThreshold > 0)
    {
    	setThreshold(data->actualThreshold);
    }
}

// Get current threshold value
uint8_t MPU6050::getThreshold()
{
    return data->actualThreshold;
}

// Set treshold value
void MPU6050::setThreshold(int multiple)
{
    if (multiple > 0)
    {
	// If not calibrated, need calibrate
		if (!data->useCalibrate) calibrateGyro();

		// Calculate threshold vectors
		data->tg.x = data->th.x * multiple;
		data->tg.y = data->th.y * multiple;
		data->tg.z = data->th.z * multiple;
	}
    else
	{
		// No threshold
    	data->tg.x = 0;
    	data->tg.y = 0;
    	data->tg.z = 0;
    }

    // Remember old threshold value
    data->actualThreshold = multiple;
}

// Fast read 8-bit from register
uint8_t MPU6050::fastRegister8(uint8_t reg)
{
    write((char*)&reg, 1);

    uint8_t value; read((char*)&value, 1);

    return value;
}

// Read 8-bit from register
uint8_t MPU6050::readRegister8(uint8_t reg)
{
    write((char*)&reg, 1);

    uint8_t value; read((char*)&value, 1);

    return value;
}

// Write 8-bit to register
void MPU6050::writeRegister8(uint8_t reg, uint8_t value)
{
    char t[2] = {(char)reg, (char)value};
    write(t, 2);
}

int16_t MPU6050::readRegister16(uint8_t reg)
{
	write((char*)&reg, 1);

	int16_t value; read((char*)&value, 2);

	return value;
}

void MPU6050::writeRegister16(uint8_t reg, int16_t value)
{
	char t[3] = {(char)reg, (char)(value >> 8), (char)value};

	write(t, 3);
}

// Read register bit
bool MPU6050::readRegisterBit(uint8_t reg, uint8_t pos)
{
    uint8_t value;
    value = readRegister8(reg);
    return ((value >> pos) & 1);
}

// Write register bit
void MPU6050::writeRegisterBit(uint8_t reg, uint8_t pos, bool state)
{
    uint8_t value;
    value = readRegister8(reg);

    if (state)
    {
        value |= (1 << pos);
    }
    else
    {
        value &= ~(1 << pos);
    }

    writeRegister8(reg, value);
}
