#include "deviceStruct.hpp"



bool deviceI2C_Start(__Device* data)
{
	__DeviceI2C& dataI2C = *((__DeviceI2C*)data->data);

	data->lockData.lock();

	dataI2C.deviceId = open(dataI2C.fileName, O_RDWR | O_NONBLOCK);

	if(dataI2C.deviceId < 0)
	{
		logError("Не удалось подключится к устройству " << dataI2C.fileName);
		data->lockData.unlock();
		return false;
	}

	if (ioctl(dataI2C.deviceId, I2C_SLAVE, dataI2C.addr) < 0)
	{
		logError("Не удалось подключиться к I2C устройству с адресом " << dataI2C.addr);
		data->lockData.unlock();
		return false;
	}

	data->lockData.unlock();

	return true;
}

bool deviceI2C_Stop(__Device* data)
{
	__DeviceI2C& dataI2C = *((__DeviceI2C*)data->data);

	data->lockData.lock();
	close(dataI2C.deviceId);
	dataI2C.deviceId = 0;
	data->lockData.unlock();

	return true;
}

int deviceI2C_Read(__Device* data, char* d, unsigned int lenght)
{
	__DeviceI2C& dataI2C = *((__DeviceI2C*)data->data);

	data->lockData.lock();
	int res = read(dataI2C.deviceId, d, lenght);
	data->lockData.unlock();

	return res;
}

int deviceI2C_Write(__Device* data, const char* d, unsigned int lenght)
{
	__DeviceI2C& dataI2C = *((__DeviceI2C*)data->data);

	data->lockData.lock();
	int res = write(dataI2C.deviceId, d, lenght);
	ioctl(dataI2C.deviceId, TCSBRK, 1);
	data->lockData.unlock();

	return res;
}



DeviceI2C::DeviceI2C()
{
	dataI2C = shared_ptr<__DeviceI2C>(new __DeviceI2C);
	data->data = dataI2C.get();
	data->startF = deviceI2C_Start;
	data->stopF = deviceI2C_Stop;
	data->readF = deviceI2C_Read;
	data->writeF = deviceI2C_Write;
}

DeviceI2C::DeviceI2C(const unsigned char addr, const char* fileName) : DeviceI2C()
{
	dataI2C->addr = addr;
	dataI2C->fileName = fileName;
}

DeviceI2C::~DeviceI2C()
{
	if (dataI2C.use_count() != 1) return;

	// Деструктор
}
