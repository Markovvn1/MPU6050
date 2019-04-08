#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <termios.h>
#include <cstring>
#include <mutex>

#include "../timer/timer.hpp"
#include "../logs/logs.hpp"
#include "device.hpp"

using namespace std;

#define MAX_TIME_WAIT 100 // Максимальное время ожидания получения ответа

typedef bool (*StartF)(__Device* data);
typedef bool (*StopF)(__Device* data);
typedef int (*ReadF)(__Device* data, char* d, unsigned int lenght);
typedef int (*WriteF)(__Device* data, const char* d, unsigned int lenght);

struct __Device
{
	mutex lockData;
	void* data; // I2C или UART

	bool active = false;

	StartF startF = NULL;
	StopF stopF = NULL;
	ReadF readF = NULL;
	WriteF writeF = NULL;
};

struct __DeviceI2C
{
	const char* fileName;
	unsigned char addr;

	int deviceId = 0;
};
