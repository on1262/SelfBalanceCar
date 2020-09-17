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
	struct PIDParameterA // static 不是必须
	{
		long lastTime;           // 前次时间
		float left_output, right_output;
		float Input, output, Setpoint, error, errSum, dErr, lastErr, TimeChange;
		float turnLSpeed_Need, turnRSpeed_Need;// 转弯设置
		float Kp, Ki, Kd;                    // 比例系数(8.0f)、积分系数(0.05)、微分系数(0.26)
	};
	struct PIDParameterS // static 不是必须
	{
		long lastTime;           // 前次时间
		float Input, output, Setpoint, error, errSum, dErr, lastErr, TimeChange;
		float Kp, Ki, Kd;                    // 比例系数(8.0f)、积分系数(0.05)、微分系数(0.26)
	};
	struct PIDParameterV // static 不是必须
	{
		long lastTime;           // 前次时间
		float Input, output, Setpoint, error, errSum, dErr, lastErr, TimeChange;
		float Kp, Ki, Kd;                    // 比例系数(8.0f)、积分系数(0.05)、微分系数(0.26)
	};


	/*电机控制*/
	int dirpin_left;
	int stepperpin_left;
	int dirpin_right;
	int stepperpin_right;
	int sleep_left;
	int sleep_right;
	int LPreScaler = 1024; //预除数，用于控制pwm
	int RPreScaler = 1024;
	/*********** PID控制器参数 *********/
	PIDParameterA PIDa;
	PIDParameterS PIDs;
	PIDParameterV PIDv;
	float motorCoef; //电机输出转换参数
	float accCoef; //加速度转换参数
	float turningSpeed = 100.0f; //转向时的附加速度
	float lastSpeed = 0.0f; //用于距离增量的计算
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
	int SampleTime = 20; //PID控制的采样间隔ms
	void signalDetect(); //检测蓝牙发送的信号
	void PIDAngle();
	void PIDDistance();
	void PIDVelocity();
	void speedControl(float speedL, float speedR);
	void pwm(float leftSpeed, float rightSpeed); //这里的speed是轮子的线速度，单位是cm/s
public:
	void PIDsetup(int dir1, int dir2, int stp1, int stp2, int slp1, int slp2);
	void PIDLoop();
	void PIDLoopStart(); //必须尽早在PIDLoop前调用
	void resetStatus();
};

extern PIDClass PID;
#endif

