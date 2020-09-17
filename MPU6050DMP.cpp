#include "MPU6050DMP.h"

MPU6050DMPClass MPU6050DMP;

void MPU6050DMPClass::periodicAction()
{
	if (nowRound < targetRound) {
		nowRound++;
		//�����ÿ�ֶ���Ҫ������
		accFixStep();
		return;
	}
	else {
		nowRound = 0;
		//�Զ������
		//display();
	}
	//����Ҫ�������
	lastActionMillis = millis();
}

void MPU6050DMPClass::init()
{
}


//ָ��������������Ϣ
void MPU6050DMPClass::display() {
	//��η�Ҫ��ʾ������
	ZYXRPYDeg angle = getAngle();
	Serial.print("-DF(Hz)=");
	Serial.print(targetRound * 1000.0f / (millis() - lastActionMillis));
	Serial.print("\t Roll:");
	Serial.print(angle.roll);
	Serial.print("\tYaw:");
	Serial.print(angle.yaw);
	Serial.println(";");
}

void MPU6050DMPClass::GyroSetup(int _sampleDelay)
{
	Serial.println("-DGyroSetup...;");
	sampleDelay = _sampleDelay;
	filter.init(3); //���ÿ�����Ƕȣ�3=2000,2=1000,1=500,0=250
	gyr2Deg = filter.gyr2Deg;
	lastAccFixTime = millis();
	lastActionMillis = millis();
	filter.Calibration();
	isAccFixEnabled = false;
	nLastTime = micros();
	gyroCalibration();

}

void MPU6050DMPClass::GyroLoop()
{
	integralUpdateStep();
	periodicAction(); //������ִ�еĺ���
	delay(sampleDelay); //�ӳ�
}
void MPU6050DMPClass::GyroLoopStart(bool _isAccFixEnabled = true)
{
	//�������ٶ����������������м�ʱ��
	isAccFixEnabled = _isAccFixEnabled;
	long ms = millis();
	startLoopMills = ms;
	nLastTime = micros();
	lastActionMillis = ms;
	lastAccFixTime = ms;
}
void MPU6050DMPClass::setAngle(float roll, float pitch, float yaw)
{
	integralAngle = ZYXRPYDeg{ roll,pitch,yaw };
}
ZYXRPYDeg MPU6050DMPClass::getAngle()
{
	float fixCoef = (millis() - startLoopMills) / secondFixTime;
	return ZYXRPYDeg{
		integralAngle.roll - fixCoef * fixAngle.x,
		integralAngle.pitch - fixCoef * fixAngle.y,
		integralAngle.yaw - fixCoef * fixAngle.z,
	};
}

void MPU6050DMPClass::gyroCalibration()
{
#ifdef SERIAL_DEBUG
	Serial.println("-DRunning calibration: second part. Don't move the sensor.;");

#endif // SERIAL_DEBUG
	//�����Ư
	startLoopMills = millis();
	nLastTime = micros();
	fixAngle = vec3{ 0,0,0 };
	long endms = millis() + (long)secondFixTime;
	while (millis() < endms) {
		GyroLoop();
	}
	//��¼��
	ZYXRPYDeg fangle = integralAngle;
	fixAngle = vec3{ fangle.roll,fangle.pitch,fangle.yaw };
#ifdef SERIAL_DEBUG
	Serial.print("-DfixedRoll=");
	Serial.print(fixAngle.x);
	Serial.print("\t fixedPitch=");
	Serial.print(fixAngle.y);
	Serial.print("\t fixedYaw=");
	Serial.print(fixAngle.z);
	Serial.println(";");
#endif // SERIAL_DEBUG

	//����
	integralAngle = ZYXRPYDeg{ 0.0f,0.0f,0.0f };
	setAngle(0, 0, 0);
}

void MPU6050DMPClass::accFixStep()
{
	//У׼��Ҫ�ڵ�����ת�����½���
	if ((!isAccFixEnabled) || (millis() < lastAccFixTime + accFixPeriod)) {
		return;
	}
	else {
		if (abs(filterData[4]) > 5) return;
	}
	//������ٶ�
	filter.ReadAccGyr(filterData);
	if (filterData[2] > localG) return;
	cosG = filterData[2];
	float accY = filterData[1];
	cosG /= localG;
	accY /= localG;
	if (abs(accY * accY + cosG * cosG - 1.0f) < 0.001f) //����������1�ȣ�Ҳ����0.1%
	{
		//У׼��ֻУ׼roll��
		if(accY > 0.0f) integralAngle.roll = acos(cosG);
		else integralAngle.roll = acos(cosG) * -1.0f;
#ifdef SERIAL_DEBUG
		Serial.print("-DAccfixed,roll=");
		Serial.print(integralAngle.roll);
		Serial.print(" accY=");
		Serial.print(filterData[1]);
		Serial.print(";");
#endif // SERIAL_DEBUG

		//���¶��ۼ�����ʱ
		lastAccFixTime = millis();
		startLoopMills = millis();
	}
}
void MPU6050DMPClass::integralUpdateStep()
{
	filter.getOriginData(filterData);
	float h = (micros() - nLastTime) / 1000000.0f;
	nLastTime = micros();
	integralAngle.roll += h * ((float)filterData[4] / gyr2Deg);
}
