//#include <stdio.h>
//#include "platform.h"
#include "xil_printf.h"

struct AXIE {
	int cur_position;		//��ĵ�ǰλ��
	int target_position;	//���Ŀ��λ��
	int direction;			//���ת������-1��ʱ��ת��1˳ʱ��ת
};

//���� MIO���ŵ�ַ
#define MIO_PIN_07		(*(volatile unsigned int *)0xF800071C)
#define MIO_PIN_50		(*(volatile unsigned int *)0xF80007C8)
#define MIO_PIN_51		(*(volatile unsigned int *)0xF80007CC)

//���� GPIO�˿ڷ���Ĵ�����ַ
#define DIRM_0			(*(volatile unsigned int *)0xE000A204)
#define DIRM_1			(*(volatile unsigned int *)0xE000A244)
#define DIRM_2			(*(volatile unsigned int *)0xE000A284)
#define DIRM_3			(*(volatile unsigned int *)0xE000A2C4)
//���� GPIO�˿����ʹ�ܼĴ�����ַ
#define OEN_0			(*(volatile unsigned int *)0xE000A208)
#define OEN_1			(*(volatile unsigned int *)0xE000A248)
#define OEN_2			(*(volatile unsigned int *)0xE000A288)
#define OEN_3			(*(volatile unsigned int *)0xE000A2C8)
//���� GPIO�˿�����Ĵ�����ַ
#define DATA_0			(*(volatile unsigned int *)0xE000A040)
#define DATA_1			(*(volatile unsigned int *)0xE000A044)
#define DATA_2			(*(volatile unsigned int *)0xE000A048)
#define DATA_3			(*(volatile unsigned int *)0xE000A04C)
//���� GPIO�˿�����Ĵ�����ַ
#define DATA_0_RO		(*(volatile unsigned int *)0xE000A060)
#define DATA_1_RO		(*(volatile unsigned int *)0xE000A064)
#define DATA_2_RO		(*(volatile unsigned int *)0xE000A068)
#define DATA_3_RO		(*(volatile unsigned int *)0xE000A06C)

//���� UART1���ŵ�ַ�ĺ궨��
#define rMIO_PIN_48		(*(volatile unsigned long*)0xF80007C0)
#define rMIO_PIN_49 	(*(volatile unsigned long*)0xF80007C4)
#define rUART_CLK_CTRL 	(*(volatile unsigned long*)0xF8000154)
#define rControl_reg0 	(*(volatile unsigned long*)0xE0001000)
#define rMode_reg0 		(*(volatile unsigned long*)0xE0001004)
//���� UART1�˿ڲ����ʵȲ�����ַ�Ĵ����ĺ궨��
#define rBaud_rate_gen_reg0 (*(volatile unsigned long*)0xE0001018)
#define rBaud_rate_divider_reg0 (*(volatile unsigned long*)0xE0001034)
#define rTx_Rx_FIFO0 (*(volatile unsigned long*)0xE0001030)
#define rChannel_sts_reg0 (*(volatile unsigned long*)0xE000102C)

void send_Char_9(unsigned char modbus[]);				//9�ֽڴ��ڷ��ͺ���
void send_Char(unsigned char data);						//���ڷ��ͺ�����һ��һ���ֽ�
void RS232_Init();										//���ڳ�ʼ������

void delay(int i, int n, int m);							//��ʱ����

struct AXIE axie_1[3];
int reset_flag = 0;								//�Ƿ�λ��־��=1��ʾ��Ҫ��λ������=0��ʾ����Ҫ��λ
int SW0_flag = 0;								//SW0����������־��=0Ϊ��ʼֵ��=1ʱ���²���=2ʱ���ϲ�
int run_step;											//�Զ�����Ĺ�����
int run_num = 5;										//�Զ�������趨��������ֵΪ5��Ĭ��ֵ��
int runing_num;											//��ǰ����Ĵ���
int conveyorState = 0;
int boxStateFlag = 0;				// ��ʼ��ʱ��
int suckerState = 0;				// ��ʼ��ʱ��û����ס
int count = 0;						// ��¼���˴���
int auto_armState = 0;					// ��е�۵ĳ�ʼλ�ã���ʼλ��Ϊ0��ץȡλ��Ϊ1���ͷ�λ��Ϊ2

int conveyor2flag = 0;				// 2�Ŵ��ʹ���ʼλ��
int box1State = 0;


// �ֶ��ٶ�
unsigned char leftSpeed = 0x32;
unsigned char rightSpeed = 0x36;

unsigned char auto_leftSpeed = 0x32;
unsigned char auto_rightSpeed = 0x36;

int resetArray[7] = {0};		// ��¼��ǰ����λ��

// ����2�Ŵ��ʹ�
void conveyor() {
	u32 stateFlag;	// ��¼��BTNC�İ�����Ϣ
	stateFlag = DATA_2_RO & 0x00000010;
	if (stateFlag == 0x00000010 && conveyorState == 0) {
		unsigned char modbus_open[] = { '#', '6', '1', '0', '0', '0', '0', '0',
				'0' };
		send_Char_9(modbus_open);
		delay(1000, 100, 1000);
		conveyorState = 1;
	} else if (stateFlag == 0x00000010 && conveyorState == 1) {
		unsigned char modbus_close[] = { '#', '6', '2', '0', '0', '0', '0', '0',
				'0' };
		send_Char_9(modbus_close);
		delay(1000, 100, 1000);
		conveyorState = 0;
	}
	return;
}

/**
 * ���̣�
 * 		1) BTNU ץ��
 * 		2) BTND ����
 */
void sucker() {
	unsigned char sucker_modbus[] = { '#', '5', '0', '0', '0', '0', '0', '0',
			'0' };

	u32 BTNUD_FLAG;

	BTNUD_FLAG = DATA_2_RO & 0x00000003;
	switch (BTNUD_FLAG) {
	case 0x00:

		break;
	case 0x01:	// ����BTNU����ס
		if (suckerState == 0) {
			sucker_modbus[2] = '1';
			send_Char_9(sucker_modbus);
			suckerState = 1;
		}
		break;
	case 0x02:	// ����BTND������
		if (suckerState == 1) {
			sucker_modbus[2] = '2';
			send_Char_9(sucker_modbus);
			suckerState = 0;
		}
		break;
	default:
		break;
	}
}

/**
 * �ֶ�ģʽ���ƻ�е��
 * ʹ��sw2sw1sw0����
 */
void maArms() {
	// ָ���ʼ������
	unsigned char arm_modbus[9] =
			{ '#', '1', '0', '0', '0', '0', '0', '0', '0' };

	// ��¼ѡ���е��
	u32 armsFlag;
	armsFlag = DATA_2_RO & 0x00001C00;	// 1.��ȡsw2sw1sw0

//	unsigned char leftSpeed = 0x32;		// ��ת˳ʱ��
//	unsigned char rightSpeed = 0x36;	// ��ת��ʱ��

	u32 direFlag;	// ���Ʒ���
	int Lflag = 0, rflag = 0;

	// ��¼BTNR��BTNL
	direFlag = DATA_2_RO & 0x0000000C;

	switch (armsFlag) {
	case 0x00000000:
		DATA_2 = (DATA_2 | 0x00012000) & 0xFFE13FFF;
		break;

	case 0x00001000:	// 001 1��
		DATA_2 = (DATA_2 | 0x00112000) & 0xFFF13FFF;
		if (direFlag == 0x4) { // ��
			arm_modbus[2] = leftSpeed;
			send_Char_9(arm_modbus);

			resetArray[1] ++;
			delay(1000, 100, 100);
		} else if (direFlag == 0x8) { // ��
			arm_modbus[2] = rightSpeed;
			send_Char_9(arm_modbus);

			resetArray[1] --;
			delay(1000, 100, 100);
		}
		break;
	case 0x00000800:	// 010 2��
		DATA_2 = (DATA_2 | 0x00092000) & 0xFFE93FFF;
		if (direFlag == 0x4) { // ��
			arm_modbus[3] = leftSpeed;
			send_Char_9(arm_modbus);

			resetArray[2] ++;
			delay(1000, 100, 100);
		} else if (direFlag == 0x8) { // ��
			arm_modbus[3] = rightSpeed;
			send_Char_9(arm_modbus);

			resetArray[2] --;
			delay(1000, 100, 100);
		}
		break;
	case 0x00001800:	// 011 3��
		DATA_2 = (DATA_2 | 0x00192000) & 0xFFF93FFF;
		if (direFlag == 0x4) { // ��
			arm_modbus[4] = leftSpeed;
			send_Char_9(arm_modbus);

			resetArray[3] ++;
			delay(1000, 100, 100);
		} else if (direFlag == 0x8) { // ��
			arm_modbus[4] = rightSpeed;
			send_Char_9(arm_modbus);

			resetArray[3] --;
			delay(1000, 100, 100);
		}
		break;
	case 0x00000400:	// 100 4��
		DATA_2 = (DATA_2 | 0x00052000) & 0xFFE53FFF;
		if (direFlag == 0x4) { // ��
			arm_modbus[5] = leftSpeed;
			send_Char_9(arm_modbus);

			resetArray[4] ++;
			delay(1000, 100, 100);
		} else if (direFlag == 0x8) { // ��
			arm_modbus[5] = rightSpeed;
			send_Char_9(arm_modbus);

			resetArray[4] --;
			delay(1000, 100, 100);
		}
		break;
	case 0x00001400:	// 101 5��
		DATA_2 = (DATA_2 | 0x00152000) & 0xFFF53FFF;
		if (direFlag == 0x4) { // ��
			arm_modbus[6] = leftSpeed;
			send_Char_9(arm_modbus);

			resetArray[5] ++;
			delay(1000, 100, 100);
		} else if (direFlag == 0x8) { // ��
			arm_modbus[6] = rightSpeed;
			send_Char_9(arm_modbus);

			resetArray[5] --;
			delay(1000, 100, 100);
		}
		break;
	case 0x00000C00:	// 110 6��
		DATA_2 = (DATA_2 | 0x000D2000) & 0xFFED3FFF;
		if (direFlag == 0x4) { // ��
			arm_modbus[7] = leftSpeed;
			send_Char_9(arm_modbus);

			resetArray[6] ++;
			delay(1000, 100, 100);
		} else if (direFlag == 0x8) { // ��
			arm_modbus[7] = rightSpeed;
			send_Char_9(arm_modbus);

			resetArray[6] --;
			delay(1000, 100, 100);
		}
		break;
	default:
		DATA_2 = (DATA_2 | 0x00012000) & 0xFFE13FFF;
		break;
	}

	reset_flag = 1;
}

/**
 * ʹ��BTN8����1�Ŵ��ʹ��ϵ�����
 */
void boxState() {
	u32 stateFlag;	// ��¼��BTN8�İ�����Ϣ
	stateFlag = DATA_1_RO & 0x00040000;

	if (stateFlag == 0x00040000 && boxStateFlag == 0) {
		unsigned char modbus_open[] = { '#', '4', '1', '0', '0', '0', '0', '0',
				'0' };
		send_Char_9(modbus_open);
		delay(1000, 100, 100);
		boxStateFlag = 1;
	} else if (stateFlag == 0x00040000 && boxStateFlag == 1) {
		unsigned char modbus_close[] = { '#', '4', '0', '0', '0', '0', '0', '0',
				'0' };
		send_Char_9(modbus_close);
		delay(1000, 100, 100);
		boxStateFlag = 0;
	}
	return;
}

/**
 * ����2�Ŵ��ʹ��ϵ�LED��ʾ
 */
void counter() {
	unsigned char counter_modbus[][9] = { { '#', '7', '0', '1', '1', '1', '1',
			'1', '1' },	//0
			{ '#', '7', '0', '0', '0', '0', '1', '1', '0' },	//1
			{ '#', '7', '1', '0', '1', '1', '0', '1', '1' },	//2
			{ '#', '7', '1', '0', '0', '1', '1', '1', '1' },	//3
			{ '#', '7', '1', '1', '0', '0', '1', '1', '0' },	//4
			{ '#', '7', '1', '1', '0', '1', '1', '0', '1' },	//5
			{ '#', '7', '1', '1', '1', '1', '1', '0', '1' },	//6
			};
	// ���ݵ�ǰ�Ĵ�����������
	send_Char_9(counter_modbus[count]);
	delay(100, 10, 10);
}

// ��ʼ״̬ -> ץȡ״̬
void armsState1() {
	// 1.˳ʱ��56��
	axie_1[1].cur_position = 0;
	axie_1[1].target_position = 56;	// 304
	axie_1[1].direction = 1;

	// 2.˳ʱ��22��
	axie_1[2].cur_position = 0;
	axie_1[2].target_position = 22;
	axie_1[2].direction = 1;

	// 3.��ʱ��76��
	axie_1[3].cur_position = 0;
	axie_1[3].target_position = 74; // 287
	axie_1[3].direction = -1;

	// 4.����
	axie_1[4].cur_position = 0;
	axie_1[4].target_position = 0;
	axie_1[4].direction = 0;

	// 5.˳ʱ��142��
	axie_1[5].cur_position = 0;
	axie_1[5].target_position = 138;
	axie_1[5].direction = 1;

	// 6.����
	axie_1[6].cur_position = 0;
	axie_1[6].target_position = 0;
	axie_1[6].direction = 0;
}

// ץȡ״̬ -> �ͷ�״̬
void armsState2() {
	// 1.��ʱ��44��
	axie_1[1].cur_position = -56;
	axie_1[1].target_position = 50;
	axie_1[1].direction = -1;

	// 2.
	axie_1[2].cur_position = 22;
	axie_1[2].target_position = 28;
	axie_1[2].direction = 1;

	// 3.��ʱ��76��
	axie_1[3].cur_position = 266;
	axie_1[3].target_position = 287;
	axie_1[3].direction = -1;

	// 4.��ʱ��
	axie_1[4].cur_position = 0;
	axie_1[4].target_position = 0;
	axie_1[4].direction = -1;

	// 5.˳ʱ��142��
	axie_1[5].cur_position = 138;
	axie_1[5].target_position = 150;
	axie_1[5].direction = 1;

	// 6.˳ʱ��
	axie_1[6].cur_position = 0;
	axie_1[6].target_position = 16;
	axie_1[6].direction = 1;
}

// �ͷ�״̬  -> ץȡ״̬
void armsState3() {
	// 1.��ʱ��44��
		axie_1[1].cur_position = -56;
		axie_1[1].target_position = 50;
		axie_1[1].direction = 1;

		// 2.
		axie_1[2].cur_position = 22;
		axie_1[2].target_position = 28;
		axie_1[2].direction = -1;

		// 3.��ʱ��76��
		axie_1[3].cur_position = 266;
		axie_1[3].target_position = 287;
		axie_1[3].direction = 1;

		// 4.��ʱ��
		axie_1[4].cur_position = 0;
		axie_1[4].target_position = 0;
		axie_1[4].direction = 1;

		// 5.˳ʱ��142��
		axie_1[5].cur_position = 138;
		axie_1[5].target_position = 150;
		axie_1[5].direction = -1;

		// 6.˳ʱ��
		axie_1[6].cur_position = 0;
		axie_1[6].target_position = 16;
		axie_1[6].direction = -1;
}

// �Զ�ģʽ�»�е�۵��ƶ�
void auto_move() {
	int a[] = { 2, 3, 4, 5, 6, 1 };
	int i;
	for (i = 0; i < 6; i++) {
		int index = a[i];
		unsigned char modbus[] = { '#', '1', '0', '0', '0', '0', '0', '0', '0' };
		int dire = axie_1[index].direction;
		int cnt = (axie_1[index].target_position - axie_1[index].cur_position)
				/ 2;
		if (dire == 1) {
			modbus[index + 1] = auto_leftSpeed;
		} else if (dire == -1) {
			modbus[index + 1] = auto_rightSpeed;
		}
		while (cnt--) {
			send_Char_9(modbus);
		}
	}
}
// �Զ�ģʽ
void autoWork() {

	if (count > 6) return ;
	// ���ȿ���2�Ŵ��ʹ�
	if (conveyor2flag == 0) {
		unsigned char modbus_open[] = { '#', '6', '1', '0', '0', '0', '0', '0',
				'0' };
		send_Char_9(modbus_open);
		conveyor2flag = 1;
	}

	if (box1State == 0) {
		// ��е���ڿ�ʼ����֮ǰ�ȳ�����
		unsigned char box_modbus[] = { '#', '4', '1', '0', '0', '0', '0', '0', '0' };
		send_Char_9(box_modbus);
		box1State = 1;
		delay(1000, 1000, 1000);	// ���ӳ�ʱ����Ҫ����
	}


	// ���̿�������
	unsigned char sucker_modbus[] = { '#', '5', '0', '0', '0', '0', '0', '0', '0' };
	switch (auto_armState) {
	case 0:
		armsState1();		// ��ʼ������
		auto_move();
		auto_armState = 1;
		delay(1000, 100, 100);
		break;
	case 1:		// ��1�Ŵ��ʹ��Ϸ�
		// ׼����ס����
		if (suckerState == 0) {
			sucker_modbus[2] = '1';
			send_Char_9(sucker_modbus);
			suckerState = 1;
		}
		armsState2();		// ��ʼ������
		// ��ס���Ӻ����ƶ���2�Ŵ��ʹ�
		auto_move();
		auto_armState = 2;
		break;
	case 2:
		// �Ƚ����ӷ���
		if (suckerState == 1) {
			sucker_modbus[2] = '2';
			send_Char_9(sucker_modbus);
			suckerState = 0;
		}
		// ������++
		delay(1000, 1000, 1000);
		count++;
		box1State = 0;	// û��1������

		// ����е���ƻ�1�Ŵ��ʹ�
		armsState3();		// ��������
		auto_move();
		auto_armState = 1;
		break;
	}
}

// �ֶ�ģʽ�¸�λ��е��
void resetArms() {
	int i;

	for (i = 1; i<= 6; i ++) {
		unsigned char arm_modbus[9] = { '#', '1', '0', '0', '0', '0', '0', '0', '0' };
		if (resetArray[i] >= 0) {
			arm_modbus[i + 1] = rightSpeed;
		} else if (resetArray[i] < 0) {
			arm_modbus[i + 1] = leftSpeed;
		}
		int cnt = resetArray[i] > 0 ? resetArray[i] : 0 - resetArray[i];
		while (cnt --) {
			send_Char_9(arm_modbus);
		}
	}

	for (i = 0; i < 7; i ++) {
		resetArray[i] = 0;
	}

	reset_flag = 0;
}
/**
 * �ı��ƶ����ٶȣ�������Ҫ����ʵ�֣���ǰʵ�ֲ�����
 */
void changeSpeed() {
	u32 btnFlag;
	// ��¼BTNR��BTNL
	btnFlag = DATA_2_RO & 0x0000000C;
	switch(btnFlag) {
	case 0x4:	// -
		if (leftSpeed > '0' && rightSpeed > '5') {
			leftSpeed --;
			rightSpeed --;
			delay(1000,100,100);
		}
		break;

	case 0x8:	// +
		if (leftSpeed < '5' && rightSpeed < '9') {
			leftSpeed ++;
			rightSpeed ++;
			delay(1000,100,100);
		}
		break;
	}
}

int main() {
	u32 flag, flag1;		//����flag���ڼ�¼SW0~SW7����������Ϣ������flag1���ڼ�¼BTN8��BTN9����������Ϣ
	u32 flag_mode1;	// ��¼sw4��sw3

	//ע������MIO���ź�EMIO���ŵ������ͳһ��ŵģ�MIO���Ϊ0~31��32~53��EMIO���Ϊ54~85��86~117
	//���ü���ʼ��MIO07���ŵ���ؼĴ�����MIO07��ΪLED�ƿ��Ƶ��������
	MIO_PIN_07 = 0x00003600;
	DIRM_0 = DIRM_0 | 0x00000080;
	OEN_0 = OEN_0 | 0x00000080;
	//���ü���ʼ��MIO50��MIO51���ŵ���ؼĴ�����MIO50��MIO51��Ϊ������������
	MIO_PIN_50 = 0x00003600;
	MIO_PIN_51 = 0x00003600;
	DIRM_1 = DIRM_1 & 0xFFF3FFFF;
	//��ʼ��EMIO54~EMIO58�����ţ����Ƕ�ӦBTNU��BTND��BTNL��BTNR��BTNC����������
	DIRM_2 = DIRM_2 & 0xFFFFFFE0;
	//��ʼ��EMIO59~EMIO66�����ţ����Ƕ�ӦSW7~SW0�������أ�����
	DIRM_2 = DIRM_2 & 0xFFFFE01F;
	//��ʼ��EMIO67~EMIO74�����ţ����Ƕ�ӦLED7~LED0�����
	DIRM_2 = DIRM_2 | 0x001FE000;
	OEN_2 = OEN_2 | 0x001FE000;

	//��ʼ��UART1
	RS232_Init();

	while (1) {
		//��ģʽ��Ϣ������SW7��SW6��������Ϣ
		flag = DATA_2_RO & 0x00000060;

		boxState();
		conveyor();

		switch (flag) {
		case 0x00:					//��λģʽ
			DATA_2 = DATA_2 & 0xFFE01FFF;		//ģʽָʾ��LED7~LED0��
			if (reset_flag == 1)
				resetArms();
			break;

		case 0x20:   //�ֶ�����ģʽ
			DATA_2 = (DATA_2 | 0x00002000) & 0xFFFFBFFF;	//ģʽָʾ��LED7����LED6��
			/*
			 * ʹ��sw4��sw3��λ�����Ƶ�ǰ�ǲ������ʹ�����е�ۻ�������
			 * sw4sw3: 01 ��λ����������ʵ����������
			 * sw4sw3: 10 ��е��
			 * sw4sw3: 11 ����
			 * 		�������裺
			 * 			1��ʹ�� DATA_2 ��ȡsw3��sw4������
			 * */

			// counter();
			// ��ʱ����¼sw4sw3��Ϣ�����Բ鿴��λ����0x0000_0360
			flag_mode1 = DATA_2_RO & 0x00000300;
			switch (flag_mode1) {
			// ��ʱ����������������ָ��
			case 0x0000:
				DATA_2 = DATA_2 & 0xFFE03FFF;	// �����ֶ�ģʽ��LED7��LED6����
				break;

			case 0x0200:	// ���ʹ� sw4sw3:01	LED7�� LED6�� LED4�� LED3��
				DATA_2 = (DATA_2 | 0x00022000) & 0xFFFEBFFF;
				// ���ٶȣ��������û��ʵ�ֳɹ����Ҳ²���ȫ�ֱ���
				// leftSpeed��rightSpeed�ĳ�ʼֵ����
				// ������ʹ�õ���ʮ�����Ƶ����������Ϊchar���͵ĳ�ʼֵ'2'�����ı�ʾ���Գɹ�
//				changeSpeed();
				break;

			case 0x0100:	// ��е�� sw4sw3:10
				DATA_2 = (DATA_2 | 0x00012000) & 0xFFFDBFFF;
				maArms();
				break;

			case 0x0300:	// ���� sw4sw3: 11
				DATA_2 = (DATA_2 | 0x00032000) & 0xFFFFBFFF;
				sucker();
				break;
			}

			break;
		case 0x40:					//�Զ�����ģʽ
			DATA_2 = (DATA_2 | 0x00004000) & 0xFFFF7FFF;	//LED7��LED6��
//			if (count > 6) break;
			autoWork();
			counter();
			break;
		case 0x60:					//��е��ʾ��ģʽ����ģʽ�ݲ�ʵ�֣�
			DATA_2 = DATA_2 | 0x00006000;					//LED7����LED6��
			break;
		}
	}
	return 0;
}

//9���ֽ����ݵķ��ͺ���
void send_Char_9(unsigned char modbus[]) {
	int i;
	char data;
	for (i = 0; i < 9; i++) {
		data = modbus[i];
		send_Char(data);
		delay(100, 10, 10);		//��ʱ
	}
}

//�����ֽ����ݵķ��ͺ���
void send_Char(unsigned char data) {
	while ((rChannel_sts_reg0 & 0x10) == 0x10)
		;
	rTx_Rx_FIFO0 = data;
}

//UART1�ĳ�ʼ������
void RS232_Init() {
	rMIO_PIN_48 = 0x000026E0;
	rMIO_PIN_49 = 0x000026E0;
	rUART_CLK_CTRL = 0x00001402;
	rControl_reg0 = 0x00000017;
	rMode_reg0 = 0x00000020;
	rBaud_rate_gen_reg0 = 62;
	rBaud_rate_divider_reg0 = 6;
}

//��ʱ����
void delay(int n, int m, int p) {
	int i, j, k;
	for (i = 1; i <= n; i++) {
		for (j = 1; j <= m; j++) {
			for (k = 1; k <= p; k++) {
			}
		}
	}
}

