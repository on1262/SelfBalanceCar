#include "MPU6050DMP.h"

MPU6050DMPClass MPU6050DMP;

void MPU6050DMPClass::periodicAction()
{
	if (nowRound < targetRound) {
		nowRound++;
		//�����ÿ�ֶ���Ҫ������
		return;
	}
	else {
		nowRound = 0;
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
	SerialBT.print("-DF(Hz)=");
	SerialBT.print(targetRound * 1000.0f / (millis() - lastActionMillis));
	SerialBT.print("\t Roll:");
	SerialBT.print(angle.roll);
	SerialBT.print("\tYaw:");
	SerialBT.print(angle.yaw);
	SerialBT.println(";");
}

void MPU6050DMPClass::GyroSetup(int _sampleDelay)
{
	SerialBT.println("-DGyroSetup...;");
	sampleDelay = _sampleDelay;
	filter.init(3); //���ÿ�����Ƕȣ�3=2000,2=1000,1=500,0=250
	gyr2Deg = filter.gyr2Deg;
	lastActionMillis = millis();
	filter.Calibration();
	nLastTime = micros();
	gyroCalibration();
}

void MPU6050DMPClass::GyroLoop()
{
	integralUpdateStep();
	periodicAction(); //������ִ�еĺ���
	if (accFixOnce == true) {
		accFixStep();
		accFixOnce = false;
	}
	delay(sampleDelay); //�ӳ�
}
void MPU6050DMPClass::GyroLoopStart()
{
	//�������ٶ����������������м�ʱ��
	long ms = millis();
	startLoopMills = ms;
	nLastTime = micros();
	lastActionMillis = ms;
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
	SerialBT.println("-DRunning calibration: second part. Don't move the sensor.;");

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
	SerialBT.print("-DfixedRoll=");
	SerialBT.print(fixAngle.x);
	SerialBT.print("\t fixedPitch=");
	SerialBT.print(fixAngle.y);
	SerialBT.print("\t fixedYaw=");
	SerialBT.print(fixAngle.z);
	SerialBT.println(";");
#endif // SERIAL_DEBUG
	SerialBT.println("-DTouch to Loop.;");
	//����
	integralAngle = ZYXRPYDeg{ 0.0f,0.0f,0.0f };
	setAngle(0, 0, 0);
}

void MPU6050DMPClass::accFixStep()
{
	//У׼��Ҫ�ڵ�����ת�����½���
	//������ٶ�
	filter.ReadAccGyr(filterData);
	cosG = filterData[2];
	float accY = filterData[1];
	cosG /= localG;
	accY /= localG;
	//У׼��ֻУ׼roll��
	if (accY > 0.0f) integralAngle.roll = acos(cosG);
	else integralAngle.roll = acos(cosG) * -1.0f;
	SerialBT.print("-DAccfixed,roll=");
	SerialBT.print(integralAngle.roll);
	SerialBT.print(" accY=");
	SerialBT.print(filterData[1]);
	SerialBT.print(";");
	//���¶��ۼ�����ʱ
	startLoopMills = millis();
}
void MPU6050DMPClass::integralUpdateStep()
{
	filter.getOriginData(filterData);
	float h = (micros() - nLastTime) / 1000000.0f;
	nLastTime = micros();
	integralAngle.roll += h * ((float)filterData[4] / gyr2Deg);
}