// Программа подключается к датчику MPU6050 (/dev/i2c-1), колибрует его, считывает данные
// и, обрабатывая их, получает абсолютный угол наклона

#include <iostream>
#include <cmath>
#include "mpu6050/mpu6050.hpp"

using namespace std;

long double safe_asin(long double x)
{
	if (x > 1) return M_PI / 2;
	if (x < -1) return -M_PI / 2;

	return asinl(x);
}

int main()
{
	MPU6050 mpu6050("/dev/i2c-1"); // Подключение к датчику
	if (!mpu6050.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
	{
		cout << "Сенсор MPU6050 не найден!" << endl;
		return 0;
	}

	cout << "Gyro: Коллибровка" << endl;
	mpu6050.calibrateGyro(100);
	mpu6050.setThreshold(3);
	cout << "Gyro: Успешно" << endl;

	// Далее идет сложный алгоритм, расчитывающий абсолютное положение датчика на
	// основе полученных измерений с гироскопа. Алгоритм использует кватернионы.
	long double quaterion1[4] = {1, 0, 0, 0};
	long double quaterion2[4];

	long double* lastQuat = quaterion1;
	long double* newQuat = quaterion2;

	long long lastTime = getCTimeMicrosecond();

	while (true)
	{
		Vector3d<int> raw = mpu6050.readCorrectGyro();
		Vector3d<long double> norm;
		long long currentTime = getCTimeMicrosecond();
		int dTime = currentTime - lastTime;
		lastTime = currentTime;

		norm.x = raw.x * dTime / 2048.0 / 8000.0 * M_PI / 360.0;
		norm.y = raw.y * dTime / 2048.0 / 8000.0 * M_PI / 360.0;
		norm.z = raw.z * dTime / 2048.0 / 8000.0 * M_PI / 360.0;

		long double c1, c2, c3, s1, s2, s3;
		c1 = cosl(norm.x); s1 = sinl(norm.x);
		c2 = cosl(norm.y); s2 = sinl(norm.y);
		c3 = cosl(norm.z); s3 = sinl(norm.z);

		long double quatDelta[4];
		quatDelta[0] = c1 * c2 * c3 - s1 * s2 * s3;
		quatDelta[1] = c1 * c2 * s3 + s1 * s2 * c3;
		quatDelta[2] = s1 * c2 * c3 + c1 * s2 * s3;
		quatDelta[3] = c1 * s2 * c3 - s1 * c2 * s3;

		newQuat[0] = (lastQuat[0] * quatDelta[0] - lastQuat[1] * quatDelta[1] - lastQuat[2] * quatDelta[2] - lastQuat[3] * quatDelta[3]);
		newQuat[1] = (lastQuat[0] * quatDelta[1] + lastQuat[1] * quatDelta[0] + lastQuat[2] * quatDelta[3] - lastQuat[3] * quatDelta[2]);
		newQuat[2] = (lastQuat[0] * quatDelta[2] - lastQuat[1] * quatDelta[3] + lastQuat[2] * quatDelta[0] + lastQuat[3] * quatDelta[1]);
		newQuat[3] = (lastQuat[0] * quatDelta[3] + lastQuat[1] * quatDelta[2] - lastQuat[2] * quatDelta[1] + lastQuat[3] * quatDelta[0]);

		Vector3d<long double> euler;
		euler.z = atan2(2.0 * (newQuat[0] * newQuat[1] + newQuat[2] * newQuat[3]), 1 - 2.0 * (newQuat[1] * newQuat[1] + newQuat[2] * newQuat[2]));
		euler.y = safe_asin(2.0 * (newQuat[0] * newQuat[2] - newQuat[3] * newQuat[1]));
		euler.x = atan2(2.0 * (newQuat[0] * newQuat[3] + newQuat[1] * newQuat[2]), 1 - 2.0 * (newQuat[2] * newQuat[2] + newQuat[3] * newQuat[3]));

		Vector3d<int> angle;
		angle.x = euler.x * 180 / M_PI;
		angle.y = euler.y * 180 / M_PI;
		angle.z = euler.z * 180 / M_PI;

		printf("Абсолютное положение гироскопа: x: %d    y: %d    z: %d", angle.x, angle.y, angle.z);

		swap(lastQuat, newQuat);
	}

	return 0;
}
