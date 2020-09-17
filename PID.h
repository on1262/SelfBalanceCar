// PID.h

#ifndef _PID_h
#define _PID_h
#include "MPU6050DMP.h"

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class PIDClass {
protected:
	struct PIDParameterA // static ���Ǳ���
	{
		long lastTime;           // ǰ��ʱ��
		float left_output, right_output;
		float Input, output, Setpoint, error, errSum, dErr, lastErr, TimeChange;
		float turnLSpeed_Need, turnRSpeed_Need;// ת������
		float Kp, Ki, Kd;                    // ����ϵ��(8.0f)������ϵ��(0.05)��΢��ϵ��(0.26)
	};
	struct PIDParameterS // static ���Ǳ���
	{
		long lastTime;           // ǰ��ʱ��
		float Input, output, Setpoint, error, errSum, dErr, lastErr, TimeChange;
		float Kp, Ki, Kd;                    // ����ϵ��(8.0f)������ϵ��(0.05)��΢��ϵ��(0.26)
	};
	struct PIDParameterV // static ���Ǳ���
	{
		long lastTime;           // ǰ��ʱ��
		float Input, output, Setpoint, error, errSum, dErr, lastErr, TimeChange;
		float Kp, Ki, Kd;                    // ����ϵ��(8.0f)������ϵ��(0.05)��΢��ϵ��(0.26)
	};


	/*�������*/
	int dirpin_left;
	int stepperpin_left;
	int dirpin_right;
	int stepperpin_right;
	int sleep_left;
	int sleep_right;
	int LPreScaler = 1024; //Ԥ���������ڿ���pwm
	int RPreScaler = 1024;
	/*********** PID���������� *********/
	PIDParameterA PIDa;
	PIDParameterS PIDs;
	PIDParameterV PIDv;
	float motorCoef; //������ת������
	float accCoef; //���ٶ�ת������
	float turningSpeed = 100.0f; //ת��ʱ�ĸ����ٶ�
	float lastSpeed = 0.0f; //���ھ��������ļ���
	bool isLoggingDistance = true;
	float distanceDelta = 0.0f;
	float distanceSum = 0.0f;
	float autoBalancePoint = 0.0f;
	float targetDistance = 0.0f;
	float targetRotation = 0.0f;
	const float turnCoef = 10.0f;
	float rotatingTime = 0.2f;
	bool isMoving = false;
	char detectChar = '\0';
	char bufferChar = '\0';
	char chrdir[5];
	int SampleTime = 20; //PID���ƵĲ������ms
	void signalDetect(); //����������͵��ź�
	void PIDAngle();
	void PIDDistance();
	void PIDVelocity();
	void speedControl(float speedL, float speedR);
	void pwm(float leftSpeed, float rightSpeed); //�����speed�����ӵ����ٶȣ���λ��cm/s
public:
	void PIDsetup(int dir1, int dir2, int stp1, int stp2, int slp1, int slp2);
	void PIDLoop();
	void PIDLoopStart(); //���뾡����PIDLoopǰ����
	void resetStatus();
};

extern PIDClass PID;
#endif

