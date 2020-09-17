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
	PIDa.Kp = 22.0f; //26.0
	PIDa.Ki = 0.05f;//0.04
	PIDa.Kd = 1.5f; //1.5

	PIDs.Kp = 12.0f; //12.0f
	PIDs.Ki = 0.16f; //0.16
	PIDs.Kd = 0.32f; //0.32

	PIDv.Kp = 0.0124f;//0.0125
	PIDv.Ki = 0.000027f; //0.000030
	PIDv.Kd = 0.0015f; //0.0015
	//初始化
	PIDa.lastTime = 0.0f;
	PIDa.turnLSpeed_Need = 0.0f;
	PIDa.turnRSpeed_Need = 0.0f;
	PIDa.errSum = 0.0f;
	PIDa.lastErr = 0.0f;
	PIDs.lastTime = 0.0f;
	PIDs.Setpoint = 0.0f;
	PIDs.errSum = 0.0f;
	PIDs.lastErr = 0.0f;
	PIDs.output = 0.0f;
	PIDv.lastTime = 0.0f;
	PIDv.errSum = 0.0f;
	PIDv.lastErr = 0.0f;
	PIDv.output = 0.0f;
}

void PIDClass::PIDVelocity()
{
	PIDv.TimeChange = (millis() - PIDv.lastTime);
	PIDv.Input = PIDa.output;
	if (isMoving) {
		PIDv.error = PIDv.Setpoint - PIDv.Input;// 偏差值
		PIDv.errSum += PIDv.error; //起到控制作用,cm/10
		PIDv.dErr = (PIDv.error - PIDv.lastErr) / PIDv.TimeChange;
		PIDv.output = PIDv.Kp * PIDv.error + PIDv.Ki * PIDv.errSum + PIDv.Kd * PIDv.dErr;
		/*假设PIDa在某偏移角平衡，且PIDv控制速度为0，那么输出output就是0，
		如果在非0倾角处平衡且没有用A重置平衡点，那么就会出现抖动*/
		PIDa.Setpoint = PIDv.output; 
		PIDv.lastErr = PIDv.error;
	}
	PIDv.lastTime = millis(); // 记录本次时间
}

//算法执行
void PIDClass::PIDLoop()
{
	PIDa.TimeChange = (millis() - PIDa.lastTime);
	if (PIDa.TimeChange >= SampleTime)
	{
		PIDa.Input = MPU6050DMP.getAngle().roll;
		if (PIDa.Input > 50.0f || PIDa.Input < -50.0f) {
			digitalWrite(sleep_left, LOW);
			digitalWrite(sleep_right, LOW);
			Serial.println("-S;");
			exit(0);
		}
		signalDetect(); //检测信号
		PIDAngle();
		PIDVelocity();
		PIDDistance();
		//执行
		digitalWrite(sleep_left, HIGH);
		digitalWrite(sleep_right, HIGH);
		speedControl(motorCoef * PIDa.left_output, motorCoef * PIDa.right_output);
	}
}

void PIDClass::PIDLoopStart()
{
	PIDa.lastTime = millis();
	PIDs.lastTime = millis();
	PIDv.lastTime = millis();
}

void PIDClass::resetStatus()
{
	PIDa.turnLSpeed_Need = 0.0f;//转弯
	PIDa.turnRSpeed_Need = 0.0f;
	PIDs.Setpoint = 0.0f;
	PIDa.Setpoint = 0.0f;
	targetDistance = 0.0f;
	targetRotation = 0.0f;
	isLoggingDistance = true;
	distanceSum = 0.0f;
	isMoving = false;
}

void PIDClass::signalDetect()
{
	if (Serial.available()) {
		bufferChar = Serial.read();
		//单字母指令检测
		if (bufferChar == 'T') {
			Serial.print("-DUno OK;");
			Serial.flush();
		}
		//字母+数字指令检测
		if (bufferChar == 'F' || bufferChar == 'B' || bufferChar == 'R' || bufferChar == 'L' || bufferChar == 'A' || bufferChar == 'S') {
			detectChar = bufferChar;
			chrdir[0] = bufferChar;
			for (int i = 1; i < 5; i++)
			{
				if (Serial.available()) {
					chrdir[i] = (char)Serial.read();
				}
				else {
					Serial.print("-DUno:Fail to get data.;");
					Serial.flush();
					return;
				}
			}
			int data3 = (chrdir[2] - '0') * 100 + (chrdir[3] - '0') * 10 + chrdir[4] - '0';
			//测试输出
			Serial.print("-D");
			Serial.print(chrdir);
			Serial.print(";");
			Serial.flush();
			switch (bufferChar)
			{
			case 'L':
				isMoving = true;
				PIDa.turnLSpeed_Need = turningSpeed;
				PIDa.turnRSpeed_Need = -turningSpeed;
				targetRotation = data3;
				rotatingTime = millis() + turnCoef * targetRotation;
				break;
			case 'R':
				isMoving = true;
				PIDa.turnLSpeed_Need = -turningSpeed;
				PIDa.turnRSpeed_Need = turningSpeed;
				targetRotation = data3;
				rotatingTime = millis() + turnCoef * targetRotation;
				break;
			case 'F': //forward
				isMoving = true;
				targetDistance = (float)data3;
				if(chrdir[1] == '1') PIDv.Setpoint = 200.0f;
				else PIDv.Setpoint = 120.0f;
				isLoggingDistance = false;
				distanceSum = 0.0f;
				break;
			case 'B': //forward
				isMoving = true;
				targetDistance = -1.0f * (float)data3;
				if (chrdir[1] == '1') PIDv.Setpoint = -200.0f;
				else PIDv.Setpoint = -120.0f;
				isLoggingDistance = false;
				distanceSum = 0.0f;
				break;
			case 'A':
				autoBalancePoint = PIDa.Setpoint;
				PIDa.Setpoint = 0.0f;
				PIDs.errSum = 0.0f;
				Serial.print("-A");
				Serial.print(abs((int)(autoBalancePoint*100.0f)));
				Serial.println(";");
				distanceSum = 0.0f;
				break;
			case 'S': //Abort
				if (data3 == 0 && chrdir[1] == '0')
				{
					if (isMoving == true) {
						Serial.print("-E1;");
						Serial.flush();
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
			Serial.print("-");
			Serial.print(detectChar);
			Serial.print(abs((int)(distanceSum)));
			Serial.println(";");
			//检测退出条件
			if (abs(targetDistance) < abs(distanceSum) && targetDistance*distanceSum >= 0) {
				isMoving = false;
				Serial.print("-E0;");
				Serial.flush();
				resetStatus();
			}
		}
		else if (detectChar == 'L' || detectChar == 'R') {
			//反馈
			Serial.print("-");
			int deltaRotatingTime = (targetRotation - (rotatingTime - millis()) / (float)turnCoef);
			Serial.print(detectChar);
			Serial.print(deltaRotatingTime);
			Serial.println(";");
			//检测退出条件
			if (rotatingTime - millis() < 0) {
				Serial.print("-E0;");
				Serial.flush();
				isMoving = false;
				resetStatus();
			}
		}
	}
}

void PIDClass::PIDAngle()
{
	PIDa.error = PIDa.Setpoint - PIDa.Input + autoBalancePoint;                     //输入角度
	PIDa.errSum += PIDa.error * PIDa.TimeChange;
	PIDa.dErr = (PIDa.error - PIDa.lastErr) / PIDa.TimeChange;
	PIDa.output = PIDa.Kp * PIDa.error + PIDa.Ki * PIDa.errSum + PIDa.Kd * PIDa.dErr;// 计算输出值
	PIDa.output = -PIDa.output; //角度为正，电机应当正向运转进行补偿
	PIDa.left_output = PIDa.output + PIDa.turnLSpeed_Need;//左电机
	PIDa.right_output = PIDa.output + PIDa.turnRSpeed_Need;//右电机
	PIDa.lastErr = PIDa.error;
	PIDa.lastTime = millis();// 记录本次时间
	distanceDelta = lastSpeed * PIDa.TimeChange / 1000.0f; //路程微分,单位厘米
	distanceSum += distanceDelta;
	lastSpeed = PIDa.output; //输出速度
}

void PIDClass::PIDDistance()
{
	PIDs.TimeChange = (millis() - PIDs.lastTime);
	PIDs.Input = distanceDelta / 10.0f;
	if (isLoggingDistance) {
		if (abs(distanceSum) < 70.0f) {
			PIDs.Kp = 15.0f;
			PIDs.Ki = 0.16f + (1 - abs(distanceSum) / 70.0f) * 0.64f;
		}
		else {
			PIDs.Kp = 12.0f;
			PIDs.Ki = 0.16f;
		}
	}
	else {
		PIDs.Ki = 0.16f;
		PIDs.Kp = 12.0f;
	}
	if (isLoggingDistance) {
		PIDs.error = PIDs.Setpoint - PIDs.Input;// 偏差值
		PIDs.errSum += PIDs.error; //起到控制作用,cm/10
		PIDs.dErr = (PIDs.error - PIDs.lastErr) / PIDs.TimeChange;
		PIDs.output = PIDs.Kp * PIDs.error + PIDs.Ki * PIDs.errSum + PIDs.Kd * PIDs.dErr;
		PIDa.Setpoint = PIDs.output;
		PIDs.lastErr = PIDs.error;
	}

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