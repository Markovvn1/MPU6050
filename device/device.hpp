#pragma once

#include <memory>

#include "../timer/timer.hpp"

typedef short int sint;
typedef int (*DeviceEventOnRead)(unsigned int lenght);

struct __Device;
struct __DeviceI2C;

class Device
{
protected:
	std::shared_ptr<__Device> data;

public:
	Device();
	virtual ~Device();

	bool isActive();

	bool start(int tryTime = TRY_NO);
	bool stop();

	int read(char* d, unsigned int lenght);
	int write(const char* d, unsigned int lenght);

	void setOnRead(DeviceEventOnRead eventOnRead);
};



class DeviceI2C : public Device
{
protected:
	std::shared_ptr<__DeviceI2C> dataI2C;

public:
	DeviceI2C();
	DeviceI2C(const unsigned char addr, const char* fileName = "/dev/i2c-0");
	virtual ~DeviceI2C();
};
