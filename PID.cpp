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
	distanceSum = 0.0f;
	lastSpeed = 0.0f;
	//������Ʋ���
	motorCoef = 0.155f; //������ƣ��ٶȻ��������ϵ��
	//PID����
	PIDv.Kp = 28.0f;
	PIDv.Ki = 0.04f;
	PIDv.Kd = 1.0f;

	PIDs.Kp = 4.0f;
	PIDs.Ki = 0.004f;
	PIDs.Kd = 0.16f;
	//��ʼ��
	PIDv.lastTime = 0;
	PIDv.turnLSpeed_Need = 0.0f;
	PIDv.turnRSpeed_Need = 0.0f;
	PIDv.errSum = 0.0f;
	PIDv.lastErr = 0.0f;
	PIDv.Setpoint = 0.5f;

	PIDs.lastTime = 0;
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
		if (PIDv.Input > 30 || PIDv.Input < -30) {
			digitalWrite(sleep_left, LOW);
			digitalWrite(sleep_right, LOW);
			BTSerial.println("-S");
			exit(0);
		}
		PIDv.Input = MPU6050DMP.getAngle().roll;
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

void PIDClass::signalDetect()
{
	if ((isControlling && millis() > sampletime) ||breakJudge) {//��Ŀ������;ֹͣ
		PIDv.turnLSpeed_Need = 0.0f;
		PIDv.turnRSpeed_Need = 0.0f;//ת��
		PIDv.errSum = 0.0f;
		PIDs.Setpoint = 0.0f;
		PIDv.Kp = 28.0f;
		PIDv.Ki = 0.04f;
		PIDv.Kd = 1.0f;
		isControlling = false;
		isDistanceLogging = true;
	}
	else {
		if (BTSerial.available()) {
			char chrdir[5]; char dis[3] = { chrdir[2] , chrdir[3] , chrdir[4] }; breakJudge = false;
			sprintf(chrdir, "%d", BTSerial.read());
			switch (chrdir[0])
			{
			case 'L':
				PIDv.turnLSpeed_Need = -turningSpeed;
				PIDv.turnRSpeed_Need = turningSpeed;
				PIDs.Setpoint = atoi(dis);
				BTSerial.print("-L");
				BTSerial.println(turningSpeed);
				break;
			case 'R':
				PIDv.turnLSpeed_Need = turningSpeed;
				PIDv.turnRSpeed_Need = -turningSpeed;
				PIDs.Setpoint = atoi(dis);
				BTSerial.print("-R");
				BTSerial.println(turningSpeed);
				break;
			case 'F': //forward
				PIDv.turnLSpeed_Need = 0.0f;
				PIDv.turnRSpeed_Need = 0.0f;
				PIDs.Setpoint = atoi(dis);
				isDistanceLogging = false;
				break;
			case 'B': //forward
				PIDv.turnLSpeed_Need = 0.0f;
				PIDv.turnRSpeed_Need = 0.0f;
				PIDs.Setpoint = -atoi(dis);
				isDistanceLogging = false;
				break;
			case 'S': //Abort
				digitalWrite(sleep_left, LOW);
				digitalWrite(sleep_right, LOW);
				char stopNum[4]= { chrdir[1], chrdir[2] , chrdir[3] , chrdir[4] };
				if (stopNum=="0000")
				{
					delay(10000);
					break;
				}
				else
				{
					exit(0);
					break;
				}
			default:
				return;
			}
			switch (chrdir[1])
			{
			case '0'://����
				PIDv.Setpoint = 0.5f;
				PIDv.Kp = 28.0f;
				PIDv.Ki = 0.04f;
				PIDv.Kd = 1.0f;
				break;
			case '1':
				PIDv.Setpoint = 1.0f;
				PIDv.Kp = 36.0f;
				PIDv.Ki = 0.06f;
				PIDv.Kd = 2.0f;
				break;
			default:
				return;
			}
			if (chrdir[0]=='F'|| chrdir[0] == 'B')
			{
				while (PIDs.Setpoint - distanceSum > 0.05*PIDs.Setpoint || distanceSum - PIDs.Setpoint > 0.05* PIDs.Setpoint) {//cm
					if (BTSerial.available())
					{
						BTSerial.print("-E1");
						breakJudge = true;
						break;
					}
					if (PIDs.Setpoint - distanceSum > 0.15*PIDs.Setpoint || distanceSum - PIDs.Setpoint > 0.15*PIDs.Setpoint)
					{
						PIDv.Setpoint = 0.5f;
						PIDv.Kp = 28.0f;
						PIDv.Ki = 0.04f;
						PIDv.Kd = 1.0f;
					}
					if (millis() > sampletime)
					{
						PIDSpeed();
						PIDDistance();
						//ִ��
						digitalWrite(sleep_left, HIGH);
						digitalWrite(sleep_right, HIGH);
						speedControl(motorCoef * PIDv.left_output, motorCoef * PIDv.right_output);
						BTSerial.print(chrdir[0]);
						BTSerial.println(distanceDelta);
						long now = millis();
					}
				}
			}
			else if (chrdir[0] == 'L' || chrdir[0] == 'R')
			{
				while (PIDs.Setpoint - turnSum > 0.05*PIDs.Setpoint) {//cm
					if (BTSerial.available())
					{
						BTSerial.print("-E1");
						breakJudge = true;
						break;
					}
					if (millis() > sampletime)
					{
						PIDSpeed();
						PIDDistance();
						//ִ��
						digitalWrite(sleep_left, HIGH);
						digitalWrite(sleep_right, HIGH);
						speedControl(motorCoef * PIDv.left_output, motorCoef * PIDv.right_output);
						BTSerial.print(chrdir[0]);
						BTSerial.println(turnDelta);
						long now = millis();
					}
				}
			}
			if (!breakJudge)
			{
				BTSerial.print("-E0");
			}
			isControlling = true; //��������
		}
	}
}

void PIDClass::PIDSpeed()
{
	PIDv.error = PIDv.Setpoint - PIDv.Input;                     // ƫ��ֵ
	PIDv.errSum += PIDv.error * PIDv.TimeChange;
	PIDv.dErr = (PIDv.error - PIDv.lastErr) / PIDv.TimeChange;
	PIDv.output = PIDv.Kp * PIDv.error + PIDv.Ki * PIDv.errSum + PIDv.Kd * PIDv.dErr;// �������ֵ
	PIDv.output = -PIDv.output; //�Ƕ�Ϊ�������Ӧ��������ת���в���
	PIDv.left_output = PIDv.output + PIDv.turnLSpeed_Need;//����
	PIDv.right_output = PIDv.output + PIDv.turnRSpeed_Need;//�ҵ��
	PIDv.lastErr = PIDv.error;
	PIDv.lastTime = millis();// ��¼����ʱ��
	turnDelta=turningSpeed * PIDv.TimeChange;
	distanceDelta = lastSpeed * PIDv.TimeChange; //���ֵõ�·��
	turnSum += turnDelta;
	distanceSum += distanceDelta;
	lastSpeed = PIDv.output;
}

void PIDClass::PIDDistance()
{
	PIDs.TimeChange = (millis() - PIDs.lastTime);
	PIDs.Input = distanceDelta / 10000.0f;// ���븳ֵ
	PIDs.error = PIDs.Setpoint - PIDs.Input;// ƫ��ֵ
	if (isDistanceLogging) PIDs.errSum += PIDs.error * PIDs.TimeChange; //���������Ϊ����״̬�Ž��о���У׼
	PIDs.dErr = (PIDs.error - PIDs.lastErr) / PIDs.TimeChange;
	PIDs.output = PIDs.Kp * PIDs.error + PIDs.Ki * PIDs.errSum + PIDs.Kd * PIDs.dErr;
	PIDv.Setpoint = PIDs.output;
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