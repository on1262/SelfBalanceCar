#include "PID.h"

void PIDClass::PIDsetup(int dir1, int dir2, int stp1, int stp2, int slp1, int slp2) {
	dirpin_left = dir1;
	dirpin_right = dir2;
	stepperpin_left = stp1;
	stepperpin_right = stp2;
	sleep_left = slp1;
	sleep_right = slp2;
	//�����ʼ��
	pinMode(dirpin_left, OUTPUT);
	pinMode(stepperpin_left, OUTPUT);
	pinMode(dirpin_right, OUTPUT);
	pinMode(stepperpin_right, OUTPUT);
	//������ʼ��
	distanceDelta = 0.0f;
	lastSpeed = 0.0f;
	//������Ʋ���
	motorCoef = 0.155f; //������ƣ��ٶȻ��������ϵ��
	//PID����
	PIDv.Kp = 26.0f;
	PIDv.Ki = 0.04f;
	PIDv.Kd = 1.5f;

	PIDs.Kp = 10.0f; //8.0f
	PIDs.Ki = 0.5f; //0.08
	PIDs.Kd = 2.5f; //0.32
	//��ʼ��
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

//�㷨ִ��
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
		signalDetect(); //����ź�
		PIDSpeed();
		PIDDistance();
		//ִ��
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
	PIDv.turnLSpeed_Need = 0.0f;//ת��
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
	if (isMoving == true) {//��Ŀ������;ֹͣ
		if (detectChar == 'F' || detectChar == 'B')
		{
			//����
			BTSerial.print("-");
			BTSerial.print(detectChar);
			BTSerial.print((int)(distanceSum * 10));
			BTSerial.println(";");
			//����˳�����
			if (abs(targetDistance - distanceSum) < 0.05f * abs(targetDistance)) {
				isMoving = false;
				BTSerial.println("-E0;");
				resetStatus();
				//PIDs.errSum += targetDistance - distanceSum;
			}
		}
		else if (detectChar == 'L' || detectChar == 'R') {
			//����
			BTSerial.print("-");
			int deltaRotatingTime = (targetRotation - (rotatingTime - millis()) / (float)turnCoef);
			BTSerial.print(detectChar);
			BTSerial.print(deltaRotatingTime);
			BTSerial.println(";");
			//����˳�����
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
	PIDv.error = PIDv.Setpoint - PIDv.Input;                     //����Ƕ�
	PIDv.errSum += PIDv.error * PIDv.TimeChange;
	PIDv.dErr = (PIDv.error - PIDv.lastErr) / PIDv.TimeChange;
	PIDv.output = PIDv.Kp * PIDv.error + PIDv.Ki * PIDv.errSum + PIDv.Kd * PIDv.dErr;// �������ֵ
	PIDv.output = -PIDv.output; //�Ƕ�Ϊ�������Ӧ��������ת���в���
	PIDv.left_output = PIDv.output + PIDv.turnLSpeed_Need;//����
	PIDv.right_output = PIDv.output + PIDv.turnRSpeed_Need;//�ҵ��
	PIDv.lastErr = PIDv.error;
	PIDv.lastTime = millis();// ��¼����ʱ��
	distanceDelta = lastSpeed * PIDv.TimeChange; //·��΢��
	lastSpeed = PIDv.output; //����ٶ�
}

void PIDClass::PIDDistance()
{
	PIDs.TimeChange = (millis() - PIDs.lastTime);
	PIDs.Input = distanceDelta / 10000.0f;//·��΢��
	if(isLoggingDistance) PIDs.error =  -PIDs.Input;// ƫ��ֵ
	distanceSum += PIDs.Input;	
	PIDs.errSum += PIDs.error; //�𵽿�������,cm/10
	PIDs.dErr = (PIDs.error - PIDs.lastErr) / PIDs.TimeChange;
	PIDs.output = PIDs.Kp * PIDs.error + PIDs.Ki * PIDs.errSum + PIDs.Kd * PIDs.dErr;
	if (isLoggingDistance) PIDv.Setpoint = PIDs.output;
	PIDs.lastErr = PIDs.error;
	PIDs.lastTime = millis(); // ��¼����ʱ��
}

//���ǰ�����˿���
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
	//�������ƹر�pwm
	if (leftSpeed < 0.16f || rightSpeed < 0.16f || leftSpeed > 109.1f || rightSpeed > 109.1f) {
		TCCR1A = _BV(WGM11) | _BV(WGM10);
		TCCR2A = _BV(WGM21) | _BV(WGM20);
		return;
	}
	LPreScaler = 1024;
	RPreScaler = 1024;
	//����pwm
	TCCR1A = _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
	TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
	//�����
	if (leftSpeed > 0.615) { //���if��֧ʹ�õ���״̬��Ӧ�Ƚ����
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
	//�Ҳ���
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