#include "PID.h"

void PIDClass::PIDsetup(int dir1, int dir2, int stp1, int stp2, int slp1, int slp2) {
	dirpin_left = dir1;
	dirpin_right = dir2;
	stepperpin_left = stp1;
	stepperpin_right = stp2;
	sleep_left = slp1;
	sleep_right = slp2;
	//电机初始化
	pinMode(dirpin_left, OUTPUT);
	pinMode(stepperpin_left, OUTPUT);
	pinMode(dirpin_right, OUTPUT);
	pinMode(stepperpin_right, OUTPUT);
	//参数初始化
	distanceDelta = 0.0f;
	lastSpeed = 0.0f;
	//电机控制参数
	motorCoef = 0.155f; //电机控制（速度环）的输出系数
	//PID参数
	PIDv.Kp = 26.0f;
	PIDv.Ki = 0.04f;
	PIDv.Kd = 1.5f;

	PIDs.Kp = 10.0f; //8.0f
	PIDs.Ki = 0.5f; //0.08
	PIDs.Kd = 2.5f; //0.32
	//初始化
	PIDv.lastTime = 0.0f;
	PIDv.turnLSpeed_Need = 0.0f;
	PIDv.turnRSpeed_Need = 0.0f;
	PIDv.errSum = 0.0f;
	PIDv.lastErr = 0.0f;
	PIDs.lastTime = 0.0f;
	PIDs.Setpoint = 0.0f;
	PIDs.errSum = 0.0f;
	PIDs.lastErr = 0.0f;
	PIDs.output = 0.0f;
}

//算法执行
void PIDClass::PIDLoop()
{
	PIDv.TimeChange = (millis() - PIDv.lastTime);
	if (PIDv.TimeChange >= SampleTime)
	{
		PIDv.Input = MPU6050DMP.getAngle().roll;
		if (PIDv.Input > 50.0f || PIDv.Input < -50.0f) {
			digitalWrite(sleep_left, LOW);
			digitalWrite(sleep_right, LOW);
			BTSerial.println("-S");
			exit(0);
		}
		signalDetect(); //检测信号
		PIDSpeed();
		PIDDistance();
		//执行
		digitalWrite(sleep_left, HIGH);
		digitalWrite(sleep_right, HIGH);
		speedControl(motorCoef * PIDv.left_output, motorCoef * PIDv.right_output);
	}
}

void PIDClass::PIDLoopStart()
{
	PIDv.lastTime = millis();
	PIDs.lastTime = millis();
}

void PIDClass::resetStatus()
{
	PIDv.turnLSpeed_Need = 0.0f;//转弯
	PIDv.turnRSpeed_Need = 0.0f;
	PIDs.Setpoint = 0.0f;
	PIDv.Setpoint = 0.0f;
	PIDv.Kp = 26.0f;
	PIDv.Ki = 0.04f;
	PIDv.Kd = 1.5f;
	targetDistance = 0.0f;
	targetRotation = 0.0f;
	isLoggingDistance = true;
	isMoving = false;
}

void PIDClass::signalDetect()
{
	BTSerial.println(PIDs.errSum);
	if (BTSerial.available()) {
		detectChar = BTSerial.read();
		char chrdir[5];
		if (detectChar == 'F' || detectChar == 'B' || detectChar == 'R' || detectChar == 'L' || detectChar == 'A' || detectChar == 'S') {
			chrdir[0] = detectChar;
			for (int i = 1; i < 5; i++)
			{
				if (BTSerial.available()) {
					chrdir[i] = (char)BTSerial.read();
				}
			}
			int data3 = (chrdir[2] - '0') * 100 + (chrdir[3] - '0') * 10 + chrdir[4] - '0';

			switch (detectChar)
			{
			case 'L':
				isMoving = true;
				PIDv.turnLSpeed_Need = turningSpeed;
				PIDv.turnRSpeed_Need = -turningSpeed;
				targetRotation = data3;
				rotatingTime = millis() + turnCoef * targetRotation;
				break;
			case 'R':
				isMoving = true;
				PIDv.turnLSpeed_Need = -turningSpeed;
				PIDv.turnRSpeed_Need = turningSpeed;
				targetRotation = data3;
				rotatingTime = millis() + turnCoef * targetRotation;
				break;
			case 'F': //forward
				isMoving = true;
				targetDistance = (float)data3 / 10.0f;
				PIDv.Setpoint = 1.0f;
				isLoggingDistance = false;
				distanceSum = 0.0f;
				break;
			case 'B': //forward
				isMoving = true;
				targetDistance = -1.0f * (float)data3 / 10.0f;
				PIDv.Setpoint = -1.0f;
				isLoggingDistance = false;
				distanceSum = 0.0f;
				break;
			case 'S': //Abort
				if (data3 == 0 && chrdir[1] == '0')
				{
					if (isMoving == true) {
						BTSerial.println("-E1;");
						isMoving = false;
					}
					resetStatus();
					PIDs.errSum = 0;
				}
				else
				{
					digitalWrite(sleep_left, LOW);
					digitalWrite(sleep_right, LOW);
					exit(0);
				}
				break;
			default:
				break;
			}
		}
	}
	if (isMoving == true) {//到目标点或中途停止
		if (detectChar == 'F' || detectChar == 'B')
		{
			//反馈
			BTSerial.print("-");
			BTSerial.print(detectChar);
			BTSerial.print((int)(distanceSum * 10));
			BTSerial.println(";");
			//检测退出条件
			if (abs(targetDistance - distanceSum) < 0.05f * abs(targetDistance)) {
				isMoving = false;
				BTSerial.println("-E0;");
				resetStatus();
				//PIDs.errSum += targetDistance - distanceSum;
			}
		}
		else if (detectChar == 'L' || detectChar == 'R') {
			//反馈
			BTSerial.print("-");
			int deltaRotatingTime = (targetRotation - (rotatingTime - millis()) / (float)turnCoef);
			BTSerial.print(detectChar);
			BTSerial.print(deltaRotatingTime);
			BTSerial.println(";");
			//检测退出条件
			if (rotatingTime - millis() < 0) {
				BTSerial.println("-E0;");
				isMoving = false;
				resetStatus();
			}
		}
	}
}

void PIDClass::PIDSpeed()
{
	PIDv.error = PIDv.Setpoint - PIDv.Input;                     //输入角度
	PIDv.errSum += PIDv.error * PIDv.TimeChange;
	PIDv.dErr = (PIDv.error - PIDv.lastErr) / PIDv.TimeChange;
	PIDv.output = PIDv.Kp * PIDv.error + PIDv.Ki * PIDv.errSum + PIDv.Kd * PIDv.dErr;// 计算输出值
	PIDv.output = -PIDv.output; //角度为正，电机应当正向运转进行补偿
	PIDv.left_output = PIDv.output + PIDv.turnLSpeed_Need;//左电机
	PIDv.right_output = PIDv.output + PIDv.turnRSpeed_Need;//右电机
	PIDv.lastErr = PIDv.error;
	PIDv.lastTime = millis();// 记录本次时间
	distanceDelta = lastSpeed * PIDv.TimeChange; //路程微分
	lastSpeed = PIDv.output; //输出速度
}

void PIDClass::PIDDistance()
{
	PIDs.TimeChange = (millis() - PIDs.lastTime);
	PIDs.Input = distanceDelta / 10000.0f;//路程微分
	if(isLoggingDistance) PIDs.error =  -PIDs.Input;// 偏差值
	distanceSum += PIDs.Input;	
	PIDs.errSum += PIDs.error; //起到控制作用,cm/10
	PIDs.dErr = (PIDs.error - PIDs.lastErr) / PIDs.TimeChange;
	PIDs.output = PIDs.Kp * PIDs.error + PIDs.Ki * PIDs.errSum + PIDs.Kd * PIDs.dErr;
	if (isLoggingDistance) PIDv.Setpoint = PIDs.output;
	PIDs.lastErr = PIDs.error;
	PIDs.lastTime = millis(); // 记录本次时间
}

//电机前进后退控制
void PIDClass::speedControl(float speedL, float speedR) {
	float L, R;
	if (speedL < 0.0f) {
		digitalWrite(dirpin_left, 0);
		L = -speedL;
	}
	else {
		digitalWrite(dirpin_left, 1);
		L = speedL;
	}
	if (speedR < 0.0f) {
		digitalWrite(dirpin_right, 1);
		R = -speedR;
	}
	else {
		digitalWrite(dirpin_right, 0);
		R = speedR;
	}
	pwm(L, R);
}

void PIDClass::pwm(float leftSpeed, float rightSpeed)
{
	//超过限制关闭pwm
	if (leftSpeed < 0.16f || rightSpeed < 0.16f || leftSpeed > 109.1f || rightSpeed > 109.1f) {
		TCCR1A = _BV(WGM11) | _BV(WGM10);
		TCCR2A = _BV(WGM21) | _BV(WGM20);
		return;
	}
	LPreScaler = 1024;
	RPreScaler = 1024;
	//开启pwm
	TCCR1A = _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
	TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
	//左侧电机
	if (leftSpeed > 0.615) { //设计if分支使得低速状态反应比较灵活
		if (leftSpeed < 2.5) {
			LPreScaler = 256;
			TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS12);
		}
		else {
			if (leftSpeed < 22.5) {
				LPreScaler = 64;
				TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
			}
			else {
				LPreScaler = 8;
				TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
			}
		}
	}
	else { //1024
		TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS12) | _BV(CS10);
	}
	//右侧电机
	if (rightSpeed > 0.615) {
		if (rightSpeed < 2.5) {
			RPreScaler = 256;
			TCCR2B = _BV(WGM22) | _BV(CS22) | _BV(CS21);
		}
		else {
			if (rightSpeed < 22.5) {
				RPreScaler = 64;
				TCCR2B = _BV(WGM22) | _BV(CS22);
			}
			else {
				RPreScaler = 8;
				TCCR2B = _BV(WGM22) | _BV(CS21);
			}
		}
	}
	else { //1024
		TCCR2B = _BV(WGM22) | _BV(CS22) | _BV(CS21) | _BV(CS20);
	}
	OCR1A = (39268.7f / ((float)LPreScaler * leftSpeed));
	OCR2A = (39268.7f / ((float)RPreScaler * rightSpeed));
}

PIDClass PID;