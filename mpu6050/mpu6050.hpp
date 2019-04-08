#pragma once

#include <cstdint>
#include <memory>
#include <unistd.h>

#include "../device/device.hpp"

#define MPU6050_ADDRESS             (0x68) // 0x69 when AD0 pin to Vcc

#define MPU6050_REG_ACCEL_XOFFS_H     (0x06)
#define MPU6050_REG_ACCEL_XOFFS_L     (0x07)
#define MPU6050_REG_ACCEL_YOFFS_H     (0x08)
#define MPU6050_REG_ACCEL_YOFFS_L     (0x09)
#define MPU6050_REG_ACCEL_ZOFFS_H     (0x0A)
#define MPU6050_REG_ACCEL_ZOFFS_L     (0x0B)
#define MPU6050_REG_GYRO_XOFFS_H      (0x13)
#define MPU6050_REG_GYRO_XOFFS_L      (0x14)
#define MPU6050_REG_GYRO_YOFFS_H      (0x15)
#define MPU6050_REG_GYRO_YOFFS_L      (0x16)
#define MPU6050_REG_GYRO_ZOFFS_H      (0x17)
#define MPU6050_REG_GYRO_ZOFFS_L      (0x18)
#define MPU6050_REG_CONFIG            (0x1A)
#define MPU6050_REG_GYRO_CONFIG       (0x1B) // Gyroscope Configuration
#define MPU6050_REG_ACCEL_CONFIG      (0x1C) // Accelerometer Configuration
#define MPU6050_REG_FF_THRESHOLD      (0x1D)
#define MPU6050_REG_FF_DURATION       (0x1E)
#define MPU6050_REG_MOT_THRESHOLD     (0x1F)
#define MPU6050_REG_MOT_DURATION      (0x20)
#define MPU6050_REG_ZMOT_THRESHOLD    (0x21)
#define MPU6050_REG_ZMOT_DURATION     (0x22)
#define MPU6050_REG_INT_PIN_CFG       (0x37) // INT Pin. Bypass Enable Configuration
#define MPU6050_REG_INT_ENABLE        (0x38) // INT Enable
#define MPU6050_REG_INT_STATUS        (0x3A)
#define MPU6050_REG_ACCEL_XOUT_H      (0x3B)
#define MPU6050_REG_ACCEL_XOUT_L      (0x3C)
#define MPU6050_REG_ACCEL_YOUT_H      (0x3D)
#define MPU6050_REG_ACCEL_YOUT_L      (0x3E)
#define MPU6050_REG_ACCEL_ZOUT_H      (0x3F)
#define MPU6050_REG_ACCEL_ZOUT_L      (0x40)
#define MPU6050_REG_TEMP_OUT_H        (0x41)
#define MPU6050_REG_TEMP_OUT_L        (0x42)
#define MPU6050_REG_GYRO_XOUT_H       (0x43)
#define MPU6050_REG_GYRO_XOUT_L       (0x44)
#define MPU6050_REG_GYRO_YOUT_H       (0x45)
#define MPU6050_REG_GYRO_YOUT_L       (0x46)
#define MPU6050_REG_GYRO_ZOUT_H       (0x47)
#define MPU6050_REG_GYRO_ZOUT_L       (0x48)
#define MPU6050_REG_MOT_DETECT_STATUS (0x61)
#define MPU6050_REG_MOT_DETECT_CTRL   (0x69)
#define MPU6050_REG_USER_CTRL         (0x6A) // User Control
#define MPU6050_REG_PWR_MGMT_1        (0x6B) // Power Management 1
#define MPU6050_REG_WHO_AM_I          (0x75) // Who Am I

template <typename T>
struct Vector3d
{
	T x, y, z;
};

struct Activites
{
    bool isOverflow;
    bool isFreeFall;
    bool isInactivity;
    bool isActivity;
    bool isPosActivityOnX;
    bool isPosActivityOnY;
    bool isPosActivityOnZ;
    bool isNegActivityOnX;
    bool isNegActivityOnY;
    bool isNegActivityOnZ;
    bool isDataReady;
};

typedef enum
{
    MPU6050_CLOCK_KEEP_RESET      = 0b111,
    MPU6050_CLOCK_EXTERNAL_19MHZ  = 0b101,
    MPU6050_CLOCK_EXTERNAL_32KHZ  = 0b100,
    MPU6050_CLOCK_PLL_ZGYRO       = 0b011,
    MPU6050_CLOCK_PLL_YGYRO       = 0b010,
    MPU6050_CLOCK_PLL_XGYRO       = 0b001,
    MPU6050_CLOCK_INTERNAL_8MHZ   = 0b000
} mpu6050_clockSource_t;

typedef enum
{
    MPU6050_SCALE_2000DPS         = 0b11,
    MPU6050_SCALE_1000DPS         = 0b10,
    MPU6050_SCALE_500DPS          = 0b01,
    MPU6050_SCALE_250DPS          = 0b00
} mpu6050_dps_t;

typedef enum
{
    MPU6050_RANGE_16G             = 0b11,
    MPU6050_RANGE_8G              = 0b10,
    MPU6050_RANGE_4G              = 0b01,
    MPU6050_RANGE_2G              = 0b00,
} mpu6050_range_t;

typedef enum
{
    MPU6050_DELAY_3MS             = 0b11,
    MPU6050_DELAY_2MS             = 0b10,
    MPU6050_DELAY_1MS             = 0b01,
    MPU6050_NO_DELAY              = 0b00,
} mpu6050_onDelay_t;

typedef enum
{
    MPU6050_DHPF_HOLD             = 0b111,
    MPU6050_DHPF_0_63HZ           = 0b100,
    MPU6050_DHPF_1_25HZ           = 0b011,
    MPU6050_DHPF_2_5HZ            = 0b010,
    MPU6050_DHPF_5HZ              = 0b001,
    MPU6050_DHPF_RESET            = 0b000,
} mpu6050_dhpf_t;

typedef enum
{
    MPU6050_DLPF_6                = 0b110,
    MPU6050_DLPF_5                = 0b101,
    MPU6050_DLPF_4                = 0b100,
    MPU6050_DLPF_3                = 0b011,
    MPU6050_DLPF_2                = 0b010,
    MPU6050_DLPF_1                = 0b001,
    MPU6050_DLPF_0                = 0b000,
} mpu6050_dlpf_t;


struct __MPU6050
{
	// Vector3d<sint> ra, rg; // Raw vectors
	// Vector3d<double> na, ng; // Normalized vectors
	Vector3d<double> tg, dg; // Threshold and Delta for Gyro
	Vector3d<double> th;     // Threshold
	Activites a;   // Activities

	double dpsPerDigit, rangePerDigit;
	double actualThreshold;
	bool useCalibrate;
	int mpuAddress;
};

class MPU6050 : private DeviceI2C
{
private:
	std::shared_ptr<__MPU6050> data;


	uint8_t fastRegister8(uint8_t reg);

	uint8_t readRegister8(uint8_t reg);
	void writeRegister8(uint8_t reg, uint8_t value);

	int16_t readRegister16(uint8_t reg);
	void writeRegister16(uint8_t reg, int16_t value);

	bool readRegisterBit(uint8_t reg, uint8_t pos);
	void writeRegisterBit(uint8_t reg, uint8_t pos, bool state);


public:
	MPU6050(const char* fileName, unsigned char addr = MPU6050_ADDRESS);
	virtual ~MPU6050();

	bool begin(mpu6050_dps_t scale = MPU6050_SCALE_2000DPS, mpu6050_range_t range = MPU6050_RANGE_2G, int tryTime = TRY_NO);

	void setClockSource(mpu6050_clockSource_t source);
	void setScale(mpu6050_dps_t scale);
	void setRange(mpu6050_range_t range);
	mpu6050_clockSource_t getClockSource();
	mpu6050_dps_t getScale();
	mpu6050_range_t getRange();
	void setDHPFMode(mpu6050_dhpf_t dhpf);
	void setDLPFMode(mpu6050_dlpf_t dlpf);
	mpu6050_onDelay_t getAccelPowerOnDelay();
	void setAccelPowerOnDelay(mpu6050_onDelay_t delay);

	uint8_t getIntStatus();

	bool getIntZeroMotionEnabled();
	void setIntZeroMotionEnabled(bool state);
	bool getIntMotionEnabled();
	void setIntMotionEnabled(bool state);
	bool getIntFreeFallEnabled();
	void setIntFreeFallEnabled(bool state);

	uint8_t getMotionDetectionThreshold();
	void setMotionDetectionThreshold(uint8_t threshold);
	uint8_t getMotionDetectionDuration();
	void setMotionDetectionDuration(uint8_t duration);

	uint8_t getZeroMotionDetectionThreshold();
	void setZeroMotionDetectionThreshold(uint8_t threshold);
	uint8_t getZeroMotionDetectionDuration();
	void setZeroMotionDetectionDuration(uint8_t duration);

	uint8_t getFreeFallDetectionThreshold();
	void setFreeFallDetectionThreshold(uint8_t threshold);
	uint8_t getFreeFallDetectionDuration();
	void setFreeFallDetectionDuration(uint8_t duration);

	bool getSleepEnabled();
	void setSleepEnabled(bool state);
	bool getI2CMasterModeEnabled();
	void setI2CMasterModeEnabled(bool state);
	bool getI2CBypassEnabled();
	void setI2CBypassEnabled(bool state);

	float readTemperature();
	Activites readActivites();

	int16_t getGyroOffsetX();
	void setGyroOffsetX(int16_t offset);
	int16_t getGyroOffsetY();
	void setGyroOffsetY(int16_t offset);
	int16_t getGyroOffsetZ();
	void setGyroOffsetZ(int16_t offset);

	int16_t getAccelOffsetX();
	void setAccelOffsetX(int16_t offset);
	int16_t getAccelOffsetY();
	void setAccelOffsetY(int16_t offset);
	int16_t getAccelOffsetZ();
	void setAccelOffsetZ(int16_t offset);

	void calibrateGyro(int samples = 50);
	void setThreshold(int multiple = 1);
	uint8_t getThreshold();

	Vector3d<sint> readRawGyro();
	Vector3d<int> readCorrectGyro();
	Vector3d<double> readNormalizeGyro();

	Vector3d<sint> readRawAccel();
	Vector3d<double> readNormalizeAccel();
	Vector3d<double> readScaledAccel();
};
