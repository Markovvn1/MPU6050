#include "deviceStruct.hpp"



Device::Device()
{
	data = shared_ptr<__Device>(new __Device);
}

Device::~Device()
{
	if (data.use_count() != 1) return;

	stop();
}

bool Device::isActive()
{
	return data->active;
}

bool Device::start(int tryTime)
{
	if (isActive()) return true;

	long long endTime = getCTimeMillisecond() + tryTime;

	do
	{
		if (data->startF)
			data->active = data->startF(data.get());

		if (!isActive()) continue;

		return true;
	}
	while ((tryTime == TRY_INFINITY || getCTimeMillisecond() < endTime));

	return false;
}

bool Device::stop()
{
	if (!isActive()) return true;

	data->active = !data->stopF(data.get());

	return !isActive();
}

int Device::read(char* d, unsigned int lenght)
{
	return data->readF(data.get(), d, lenght);
}

int Device::write(const char* d, unsigned int lenght)
{
	return data->writeF(data.get(), d, lenght);
}
