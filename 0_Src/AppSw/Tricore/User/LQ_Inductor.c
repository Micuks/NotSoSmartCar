*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL

 ����    д��chiusir
 ��E-mail��chiusir@163.com
 ������汾��V1.1 ��Ȩ���У���λʹ��������ϵ��Ȩ
 �������¡�2020��10��28��
 �������Ϣ�ο����е�ַ��
 ����    վ��http:// www.lqist.cn
 ���Ա����̡�http:// longqiu.taobao.com
 ------------------------------------------------
 ��dev.env.��AURIX Development Studio1.2.2�����ϰ汾
 ��Target �� TC264DA/TC264D
 ��Crystal�� 20.000Mhz
 ��SYS PLL�� 200MHz
 ________________________________________________________________
 ����iLLD_1_0_1_11_0�ײ����,

 ʹ�����̵�ʱ�򣬽������û�пո��Ӣ��·����
 ����CIFΪTC264DA�����⣬�����Ĵ������TC264D
 ����Ĭ�ϳ�ʼ����EMEM��512K������û�ʹ��TC264D��ע�͵�EMEM_InitConfig()��ʼ��������
 ������\Libraries\iLLD\TC26B\Tricore\Cpu\CStart\IfxCpu_CStart0.c��164�����ҡ�
 ===============================================================
 ����������Ƶ��ַ��https:// space.bilibili.com/95313236
 =================================================================
 ʹ��˵����
 ����ѧ��ʾ���������ڵ�����ֻ������ֳ���
 ������ԴΪ��
 ģ�飺����TC264DA���İ塢����ĸ�塢˫·ȫ�ŵ��������˫������512������TFT1.8��Ļ�����������·���ģ�飻
 ��ģ�����ֻ������־��ɣ�
 ��зֲ���
 ||----------��------------------------------------��--------------||
 ||---------��--------------------------------------��-------------||
 ||--------��----------------------------------------��------------||
 ||-------1----����2����� -------------�Ҳ��2����� -----1-----------||
 ||------��--------------------------------------------��----------||
 ||-----��----------------------------------------------��---------||
 ||----��------------------------------------------------��--------||
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include <IfxCpu.h>
#include <LQ_ADC.h>
#include <LQ_CCU6.h>
#include <LQ_STM.h>
#include <LQ_TFT18.h>
#include <Main.h>
#include <Platform_Types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <LQ_GPIO_KEY.h>
#include <LQ_MotorServo.h>
#include <LQ_GPIO_LED.h>
#include <LQ_Inductor.h>
#include <LQ_GPT12_ENC.h>
#include <LQ_PID.h>

sint16 TempAngle = 0;        // ���ݵ��ƫ�������������ʱ���ֵ
sint16 LastAngle = 0;        // ����������ǰ����Чƫ�Ʒ���

sint16 Lleft1 = 0, Lleft2 = 0, Lright2 = 0, Lright1 = 0;  // ���ƫ����
sint16 LnowADC[6];           // ��е�ǰADC��ֵ

//sint16 ad_max[6] = {0, 0,0, 0, 0,0}; // �°��ӷŵ������궨���ֵ,�ᱻ����ˢ��
sint16 ad_min[6] = {200, 200, 200, 200, 200, 200}; // �°��Ӿ���Ҫ�궨��Сֵ,�ᱻ����ˢ��

sint16 ad_max[6] = {4000, 4000,4000, 4000, 4000,4000}; // �°��ӷŵ������궨���ֵ,�ᱻ����ˢ��
//sint16 ad_min[6] = {30000, 30000, 30000, 30000, 30000, 30000}; // �°��Ӿ���Ҫ�궨��Сֵ,�ᱻ����ˢ��

uint8 CircleNumber = 4;    // �뻷������0������Ĭ��4�� ;���ĸ���һ���ڱ���ǰ��������ʱ��֪����,����·���������Σ��뻷����Ϊ���ĸ���������
uint8 TangentPoint = 1;    // �е��ж�   0�е������Ĭ��1�����뻷����ȡ����Ϊ�뻷׼��
uint8 EnterCircle = 0;     // �������  Ĭ�� 0���ɽ�����1���Խ���
uint8 OutCircle = 0;       // �������   Ĭ��0���ɳ�����1���Գ���
uint8 LeftRightCircle = 0; // ��໷�����Ҳ໷ Ĭ��0ԭʼ��1�󻷣�2�һ�

sint32 TangentPointpulse = 0; // ���������ʱ����1
sint32 EnterCirclePulse = 0;  // ���������ʱ����2
sint32 OutCirclePulse = 0;    // ���������ʱ����3
sint32 EnterCircleOKPulse = 0;// ���������ʱ����4
sint16 LowSpeed = 0;          // Բ��/ʮ�ֿ�/����·����

uint8 SanNumber = 2;	//����·���������һ������·��������
uint8 SanPoint = 1;	    //�е��жϣ�0�е������Ĭ��1���Խ�����·����ȡ����Ϊ�뻷��׼��
uint8 EnterSan = 0;		//���������·��Ĭ��0���ɽ�����·��1���Խ�����·
uint8 OutSan = 0;		//���������·��Ĭ��0���ɳ�����·��1���Գ�����·
uint8 LeftRightSan = 0;	//�����������Ҳ��������·��Ĭ��1���룬2

float err;
float speederr;
sint32 CurrSpeed;
sint32 ExpectedSpeed = 100000 * 579 / 100;
uint8 state;
char msg[][16]={"Straight    ","Curve       ","Severe_Curve"};

sint32 SanPointpulse = 0; // ���������ʱ���� ����·���
sint32 EnterSanPulse = 0;  	  // ���������ʱ���� ������·
sint32 OutSanPulse = 0;       // ���������ʱ���� ������·
sint32 EnterSanOKPulse = 0;   // ���������ʱ���� �Ƿ���������·

uint16 MagneticField = 0;     // �ų�ǿ��    magnetic field intensity,�ж�Բ��������·�Ƿ����

sint16 OffsetDelta = 0;

pid_param_t curve_pid;// ���
pid_param_t straight_pid;// ֱ��
pid_param_t severe_curve_pid;// ����
pid_param_t speed_pid;
pid_param_t fast_speed_pid;
pid_param_t extreme_speed_pid;

/*************************************************************************
 *  �������ƣ�void InductorInit (void)
 *  ����˵�����˸����ADC��ʼ��������
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��11��18��
 *  ��    ע��
 *************************************************************************/
void InductorInit (void)
{
    ADC_InitConfig(ADC0, 100000); // ��ʼ��,������100kHz
    ADC_InitConfig(ADC1, 100000); // ��ʼ��
    ADC_InitConfig(ADC2, 100000); // ��ʼ��
    ADC_InitConfig(ADC3, 100000); // ��ʼ��
    ADC_InitConfig(ADC7, 80000);
}
void Admin_Control(void) {
    State_Control();
    Mode_Control();
    Angle_Control();
    Speed_Control();
}
void State_Control(void) {
    switch(state) {
        case NORMAL: {
            if(Game_Over) {
                state = IN_GARAGE;
            }
            CircleDetect();
            SanDetect();
        } break;
        case 
    }
} 
/*************************************************************************
 *  �������ƣ�void InductorNormal(void)
 *  ����˵�����ɼ���е�ѹ����һ����
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��11��18��
 *  ��    ע��   ע��Ҫ�ȱ궨�˷ŵķŴ����������ĸ�һ�»��߽ӽ�
 *************************************************************************/
void InductorNormal (void)
{
    LnowADC[0] = ADC_ReadAverage(ADC0, 5);  // ����1����У��������н�Լ30�ȣ��ɼ�������е�ADֵ
    LnowADC[1] = ADC_ReadAverage(ADC1, 5);  // ����2����У���ֱ������
    LnowADC[4] = ADC_ReadAverage(ADC2, 5);  // �Ҳ��2����У���ֱ������
    LnowADC[5] = ADC_ReadAverage(ADC3, 5);  // �Ҳ��1����У��������н�Լ30��
    BatVolt    = ADC_ReadAverage(ADC7, 5);  // ˢ�µ�ص�ѹ
    if (LnowADC[0] < ad_min[0])
		ad_min[0] = LnowADC[0];     // ˢ����Сֵ
	if (LnowADC[0] > ad_max[0])
		ad_max[0] = LnowADC[0];     // ˢ�����ֵ
	if (LnowADC[1] < ad_min[1])
		ad_min[1] = LnowADC[1];
	if (LnowADC[1] > ad_max[1])
		ad_max[1] = LnowADC[1];
	if (LnowADC[4] < ad_min[4])
		ad_min[4] = LnowADC[4];
	if (LnowADC[4] > ad_max[4])
		ad_max[4] = LnowADC[4];
	if (LnowADC[5] < ad_min[5])
		ad_min[5] = LnowADC[5];
	if (LnowADC[5] > ad_max[5])
		ad_max[5] = LnowADC[5];

    Lleft1 = (LnowADC[0] - ad_min[0]) * 100 / (ad_max[0] - ad_min[0]);     // ��ƫ������һ����0--100����
    Lleft2 = (LnowADC[1] - ad_min[1]) * 100 / (ad_max[1] - ad_min[1]);
    Lright2 = (LnowADC[4] - ad_min[4]) * 100 / (ad_max[4] - ad_min[4]);
    Lright1 = (LnowADC[5] - ad_min[5]) * 100 / (ad_max[5] - ad_min[5]);

    MagneticField = Lleft1 + Lleft2 + Lright2 + Lright1;// �ų�����ǿ�� ����һ��150 ʮ��200~300

    err = Lleft1 + Lleft2 - Lright1 - Lright2;// ���Ҵų���

//    if(TangentPointpulse != 0)
//    	TempAngle = Servo_Center_Mid + PidLocCtrl(&curve_pid, err);
    if(MagneticField > 300 && RAllPulse > 3000) {
    	TempAngle = Servo_Center_Mid + PidLocCtrl(&straight_pid, -err);
    	state = 0;
    }
    if(Lleft2 + Lright2<54)      // ����ֵ��С��20����Ÿ�Զ������������ֱ����ͼ���
    {
        TempAngle = Servo_Center_Mid+PidLocCtrl(&severe_curve_pid, -err);
        state = 2;
        LastAngle =TempAngle;// ������Ч����������ƫ�Ʒ���
    }
    else if ((Lleft2 > 27) && (Lright2 > 27))   // С���������������м� ��Ҫ����
    {
        TempAngle = Servo_Center_Mid + PidLocCtrl(&straight_pid, -err); //  ����ƫ������ֵС���ȴ�ǣ���ֱֹ��ҡ�� kp = 2
        state = 0;
    }
    else                                        //  С���������������������һ��һС����Ҫ�ϴ�̶ȿ���ת��
    {
        TempAngle = Servo_Center_Mid + PidLocCtrl(&curve_pid, -err); //  һ�����ת����ƣ���ֵԽ��ת��Խ�� kp = 7
        state = 1;
    }
}

/*************************************************************************
 *  �������ƣ�void CircleDetect void
 *  ����˵����ʶ�𲢽���Բ���ĸ�����
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��11��18��
 *  ��    ע��
 *************************************************************************/
void CircleDetect (void)
{
    if (CircleNumber) // �뻷������0������Ĭ��2��
    {
        // �����е������ж�
    	if(MagneticField > 300 && RAllPulse > 3000)
        {
            if (TangentPoint)
            {
                TangentPointpulse = RAllPulse; // ��ȡ��ǰ������ֵ
                TangentPoint = 0;             // ��ֹ�ٴζ�ȡ��ǰ������ֵ
                state = ENTER_CIRCLE_01;
            }
            if (LeftRightCircle == 0)
            {                                 // ���ǰ������Ϊ220���˴�Ӧ�ô�������������һЩ��̫����߹��е���Ҳ���У�
                if (RAllPulse > TangentPointpulse + 2500)   // �����е����ǰ��3000���壬��Լ50cm��//��Լ5790���� = 1m
                {
                    EnterCircle = 1;      // ͨ���е����򣬿����뻷
                }
            }
        }

        // ��Լ1.2���������Ч������Ҫ����ʶ���е�
        if ((LeftRightCircle == 0) && (RAllPulse > TangentPointpulse + 8000)) // Լ1.2���������Ч
        {
            EnterCircle = 0;   // Լ1.2���������Ч
            TangentPoint = 1;  // ����ʶ���е�
            state = NORMAL;
        }
        if ((EnterCircle) && (MagneticField > 300)) // Լ1.2�����ٴη���ǿ�ų����뻷
        {
            LowSpeed = 500;    // ����
            state = CIRCLE_FOUND;
            // ������뻷����
            int err = Lleft1 + Lleft2 - Lright1 - Lright2;
            if (Lleft1 + Lleft2 > Lright2 + Lright1)     // ����뻷���������У�����
            {
                LeftRightCircle = 1;  // ��໷Ϊ1
                EnterCircle = 0;      // �뻷���ֹ�ٴ��뻷
                EnterCirclePulse = RAllPulse;
                ServoCtrl(Servo_Center_Mid - Servo_Delta/2);     // ���PWM�����ת������ǿ���
                while (RAllPulse < EnterCirclePulse + 2400)
                {
                    delayms(1);       // ���Ƕ�ǰ��1200���壬Լ20cm������512�����������1��5790������
                }
            }
            else // �Ҳ��뻷����
            {
                LeftRightCircle = 2;  // �Ҳ໷Ϊ2
                EnterCircle = 0;  // �뻷���ֹ�ٴ��뻷
                EnterCirclePulse = RAllPulse;
                ServoCtrl(Servo_Center_Mid + Servo_Delta/2);        // ���PWM�����ת������ǿ���
                while (RAllPulse < EnterCirclePulse + 2400) // �õ����Ҳ�ı�������ʵ���ߵľ����һ���
                {
                    delayms(1);   // ���Ƕ�ǰ��1200���壬Լ20cm������512�����������1��5790������
                }
            }
            EnterCircleOKPulse = RAllPulse; // ������
        }
        // ��������
        if ((LeftRightCircle > 0) && (RAllPulse > EnterCircleOKPulse + 3000)) // 3000�����У���Ҫ����
        {
            EnterCircleOKPulse = 10000000; //��ֹ�ٴγ���ʹ��
            OutCircle = 1;  // ��������Գ���
        }
        if ((OutCircle) && (MagneticField > 300)) // ������־Ϊ����ܳ���
        {
            LowSpeed = 500;        // ����
            state = IN_CIRCLE;
            // ����������
            if (LeftRightCircle == 1)   //����뻷
            {
                OutCirclePulse = RAllPulse;
                ServoCtrl(Servo_Center_Mid - Servo_Delta/2);     // ���PWM�����ת������ǿ���
                while (RAllPulse < OutCirclePulse + 2400)
                {
                    delayms(1); // ���Ƕ�ǰ��1200���壬Լ20cm������512�����������1��5790������
                }
                OutCirclePulse = RAllPulse;
                ServoCtrl(Servo_Center_Mid+Servo_Delta*2/3);     // ���PWM�����������
                while (RAllPulse < OutCirclePulse + 700)
                {
                    delayms(1);         // ���Ƕ�ǰ��600���壬Լ10cm������512�����������1��5790������
                }
                CircleNumber--;         // ������
                TangentPoint = 1;       // �е��ж�   0�е������Ĭ��1�����뻷����ȡ����Ϊ�뻷׼��
                EnterCircle = 0;        // �������  Ĭ�� 0���ɽ�����1���Խ���
                OutCircle = 0;          // �������   Ĭ��0���ɳ�����1���Գ���
                LeftRightCircle = 0;    // ��໷�����Ҳ໷ Ĭ��0ԭʼ��1�󻷣�2�һ�
                LowSpeed = 0;           // �ָ��ٶ�
                TangentPointpulse = 0;
                state = NORMAL;
                if(CircleNumber == 0 || SanNumber == 0)
                	Reed_Init();            // �ɻɹ�GPIO���жϳ�ʼ������,Ϊͣ�������׼��
            }
            // �Ҳ��������
            else if (LeftRightCircle == 2)  //�ұ��뻷
            {
                //OutCircle = 0;     // �뻷���ֹ�ٴ��뻷
                OutCirclePulse = RAllPulse;
                ServoCtrl(Servo_Center_Mid + Servo_Delta/2);     // ���PWM�����ת������ǿ���
                while (RAllPulse < OutCirclePulse + 2400)
                {
                    delayms(1);   // ���Ƕ�ǰ��1200���壬Լ20cm������512�����������1��5790������
                }
                OutCirclePulse = RAllPulse;
                ServoCtrl(Servo_Center_Mid-Servo_Delta*2/3);     // ���PWM�����������
                while (RAllPulse < OutCirclePulse + 700)
                {
                    delayms(1);         // ���Ƕ�ǰ��600���壬Լ10cm������512�����������1��5790������
                }
                CircleNumber--;         // ������
                TangentPoint = 1;       // �е��ж�   0�е������Ĭ��1�����뻷����ȡ����Ϊ�뻷׼��
                EnterCircle = 0;        // �������  Ĭ�� 0���ɽ�����1���Խ���
                OutCircle = 0;          // �������   Ĭ��0���ɳ�����1���Գ���
                LeftRightCircle = 0;    // ��໷�����Ҳ໷ Ĭ��0ԭʼ��1�󻷣�2�һ�
                LowSpeed = 0;           // �ָ��ٶ�
                TangentPointpulse = 0;
                if(CircleNumber == 0 || SanNumber == 0)
                	Reed_Init();            // �ɻɹ�GPIO���жϳ�ʼ������,Ϊͣ�������׼��
            }
        }
    }
}


/*************************************************************************
 *  �������ƣ�void SanDetect void
 *  ����˵����ʶ�𲢽�������·�ĸ�����
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��11��18��
 *  ��    ע��
 *************************************************************************/
void SanDetect (void)
{
    if (SanNumber) // ������·������0������Ĭ��1��
    {
        // ������·�е������ж�
        if (MagneticField < 10) // ֱ�����룬��ֵ����̫�����׶�ʧ�е�
        {
            if (SanPoint) //�е�ʶ�𣬴�д
            {
                state = SAN_ENTER_01;
                SanPointpulse = RAllPulse; // ��ȡ��ǰ������ֵ
                SanPoint = 0;             // ��ֹ�ٴζ�ȡ��ǰ������ֵ
            }
            else if (LeftRightSan == 0)
            {                                 // ���ǰ������Ϊ220���˴�Ӧ�ô�������������һЩ��̫����߹��е���Ҳ���У�
                if (RAllPulse > SanPointpulse + 3500)   // �����е����ǰ��3000���壬��Լ50cm��//��Լ5790���� = 1m
                {
                    EnterSan = 1;      // ͨ���е����򣬿���������·
                    state = SAN_ENTER_02;
                }
            }
        }

        // ��Լ1.2���������·��Ч������Ҫ����ʶ���е�
        if ((LeftRightSan == 0) && (RAllPulse > SanPointpulse + 8000)) // Լ1.2���������·��Ч
        {
            state = NORMAL;
            EnterSan = 0;   // Լ1.2���������·��Ч
            SanPoint = 1;  // ����ʶ���е�
        }
        if ((EnterSan) && (MagneticField > 260)) // Լ1.2�����ٴη���ǿ�ų���������·
        {
            state = IN_SAN;
            LowSpeed = 500;    // ����
            // �����������·����
            int  err = Lleft1 + Lleft2 - Lright1 - Lright2;
            if (SanNumber == 2)     //���������·���������У�����
            {
                LeftRightSan = 1;  // �������·Ϊ1
                state = LEFT_SAN;
                EnterSan = 0;      // ������·���ֹ�ٴ�������·
                EnterSanPulse = RAllPulse;
                ServoCtrl(Servo_Left_Max);     // ���PWM�����ת������ǿ���
                while (RAllPulse < EnterSanPulse + 2800)
                {
                    delayms(1);       // ���Ƕ�ǰ��1200���壬Լ20cm������512�����������1��5790������
                }
            }
            else if(SanNumber == 1)// �Ҳ�������·����
            {
                LeftRightSan = 2;  //�Ҳ��������·Ϊ2
                state = RIGHT_SAN;
                EnterSan = 0;  // ������·���ֹ�ٴ�������·
                EnterSanPulse = RAllPulse;
                ServoCtrl(Servo_Right_Min);        // ���PWM�����ת������ǿ���
                while (RAllPulse < EnterSanPulse + 1800) // �õ����Ҳ�ı�������ʵ���ߵľ����һ���
                {
                    delayms(1);   // ���Ƕ�ǰ��1200���壬Լ20cm������512�����������1��5790������
                }
            }
            EnterSanOKPulse = RAllPulse; // ������·��
        }
        // ������·����
        if ((LeftRightSan > 0) && (RAllPulse > EnterSanOKPulse + 3000)) // 3000�����У���Ҫ����
        {
            EnterSanOKPulse = 10000000; //��ֹ�ٴγ�����·ʹ��
            OutSan = 1;  // ������·����Գ�����·
            state = OUT_SAN_01;
        }
        if ((OutSan) && (MagneticField < 10)) // ������·��־Ϊ����ܳ�����·
        {
            state = OUT_SAN_02;
            LowSpeed = 500;        // ����
            // ��������·����
            if (LeftRightSan == 1)   //���������·��LeftRightSan == 1
            {
                OutSanPulse = RAllPulse;
                ServoCtrl(Servo_Left_Max);     // ���PWM�����ת������ǿ���
                while (RAllPulse < OutSanPulse + 2800)
                {
                    delayms(1); // ���Ƕ�ǰ��1200���壬Լ20cm������512�����������1��5790������
                }
                OutSanPulse = RAllPulse;
                ServoCtrl(Servo_Center_Mid-Servo_Delta*2/3);     // ���PWM�����������
                while (RAllPulse < OutSanPulse + 700)
                {
                    delayms(1);         // ���Ƕ�ǰ��600���壬Լ10cm������512�����������1��5790������
                }
                SanNumber--;         // ����·����
                SanPoint = 1;       // �е��ж�   0�е������Ĭ��1����������·����ȡ����Ϊ������·׼��
                EnterSan = 0;        // ���������·  Ĭ�� 0���ɽ�����·��1���Խ�����·
                OutSan = 0;          // ���������·   Ĭ��0���ɳ�����·��1���Գ�����·
                LeftRightSan = 0;    // �������·�����Ҳ�����· Ĭ��0ԭʼ��1������·��2������·
                LowSpeed = 0;           // �ָ��ٶ�
                state = NORMAL;
                if(CircleNumber == 0 || SanNumber == 0)
               	Reed_Init();            // �ɻɹ�GPIO���жϳ�ʼ������,Ϊͣ�������׼��
            }                                                                                                                                                                                                                                                                                                                                                                                                                                           
            / / �Ҳ������·����
            else if (LeftRightSan == 2)  //�ұ�������·�� LeftRightSan == 2
            {
                //OutSan = 0;     // ������·���ֹ�ٴ�������·
                OutSanPulse = RAllPulse;
                ServoCtrl(Servo_Right_Min);     // ���PWM�����ת������ǿ���
                while (RAllPulse < OutSanPulse + 2500)
                {
                    delayms(1);   // ���Ƕ�ǰ��1200���壬Լ20cm������512�����������1��5790������
                }
                OutSanPulse = RAllPulse;
                ServoCtrl(Servo_Center_Mid+Servo_Delta*2/3);     // ���PWM�����������
                while (RAllPulse < OutSanPulse + 1400)
                {
                    delayms(1);         // ���Ƕ�ǰ��600���壬Լ10cm������512�����������1��5790������
                }
                SanNumber--;         // ����·����
                SanPoint = 1;       // �е��ж�   0�е������Ĭ��1����������·����ȡ����Ϊ������·׼��
                EnterSan = 0;        // ���������·  Ĭ�� 0���ɽ�����·��1���Խ�����·
                OutSan = 0;          // ���������·   Ĭ��0���ɳ�����·��1���Գ�����·
                LeftRightSan = 0;    // �������·�����Ҳ�����· Ĭ��0ԭʼ��1������·��2������·
                LowSpeed = 0;           // �ָ��ٶ�
                state = NORMAL;
                if(CircleNumber == 0 || SanNumber == 0)
                	Reed_Init();            // �ɻɹ�GPIO���жϳ�ʼ������,Ϊͣ�������׼��
            }
        }
    }
}

/*************************************************************************
 *  �������ƣ�void TFT_Show_EleMag_Info(void)
 *  ����˵������ʾ����������Ϣ
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��11��18��
 *  ��    ע��
 *************************************************************************/
void TFT_Show_EleMag_Info(void)
{
    char txt[16] = "X:";

    sint16 mps = 0, dmm = 0;    // �ٶȣ�m/s,������ֵ
    sint16 pulse100 = 0;
    uint16 bat=0;

    dmm = (sint16) (RAllPulse * 100 / 579);           // ����512�����������1��5790�����壬��ֵ̫�󣬳���100
    pulse100 = (sint16) (RAllPulse / 100);
    sprintf(txt, "AP:%05d00", pulse100);              //
    TFTSPI_P8X16Str(3, 1, txt, u16RED, u16BLACK);     // ��ʾ����ƫ�����

    NowTime = (STM_GetNowUs(STM0) - NowTime) / 1000;  // ��ȡSTM0 ��ǰʱ�䣬�õ�����
    mps = (sint16) (dmm / (NowTime / 1000));          // �����ٶ�mm/s
    // ������Ϣ
    sprintf(txt, "%04d %04d %04d ", TempAngle, ECPULSE1, ECPULSE2);   // ��ʾ����Ƕ���ֵ�����ռ�ձ���ֵ����������ֵ
    TFTSPI_P8X16Str(1, 0, txt, u16WHITE, u16BLACK);      // �ַ�����ʾ
    //��ʾ����й�һ�����ƫ����  ��ǰ����е�ѹֵ ����п�������ʷ��Сֵ ����п�������ʷ���ֵ
    sprintf(txt, "0:%04d %04d %04d ", Lleft1, LnowADC[0], ad_max[0]);
    TFTSPI_P8X16Str(0, 2, txt, u16WHITE, u16BLACK);
    sprintf(txt, "1:%04d %04d %04d ", Lleft2, LnowADC[1], ad_max[1]);
    TFTSPI_P8X16Str(0, 3, txt, u16WHITE, u16BLACK);
    sprintf(txt, "2:%04d %04d %04d ", Lright2, LnowADC[4], ad_max[4]);
    TFTSPI_P8X16Str(0, 4, txt, u16WHITE, u16BLACK);
    sprintf(txt, "3:%04d %04d %04d ", Lright1, LnowADC[5], ad_max[5]);
    TFTSPI_P8X16Str(0, 5, txt, u16WHITE, u16BLACK);

    sprintf(txt, "Ring num: %d ", CircleNumber);
    TFTSPI_P8X16Str(2, 6, txt, u16GREEN, u16BLACK);
    sprintf(txt, "M:%03d Q:%d J:%d ", MagneticField, TangentPoint, EnterCircle);
    TFTSPI_P8X16Str(0, 7, txt, u16WHITE, u16BLACK);
    if (LeftRightCircle == 1)
        TFTSPI_P8X16Str(0, 8, "Left Ring ", u16WHITE, u16BLACK);
    else if (LeftRightCircle == 2)

    else
        TFTSPI_P8X16Str(0, 8, "No Ring   ", u16WHITE, u16BLACK);

    bat = BatVolt * 11 / 25;  // x/4095*3.3*100*5.7
    sprintf(txt, "B:%d.%02dV %d.%02dm/s", bat / 100, bat % 100, mps / 1000, (mps / 10) % 100);  // *3.3/4095*3
    TFTSPI_P8X16Str(0, 9, txt, u16PURPLE, u16BLACK);   // �ַ�����ʾ
}
/*************************************************************************
 *  �������ƣ�void OLED_Show_EleMag_Info(void)
 *  ����˵������ʾ����������Ϣ
 *                                                                                           ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��11��18��
 *  ��    ע��
 *************************************************************************/
void OLED_Show_EleMag_Info(void)
{                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
    char txt[16];
 
    s  int16 dmm = 0;    // �ٶȣ�m/s,������ֵ
     sint16 pulse100 = 0;
    u          int16 bat=0;
                                          
    d mm = (sint16) (RAllPulse * 100 / 579);           // ����512�����������1��5790�����壬��ֵ̫�󣬳���100
    p ulse100 = (sint16) (RAllPulse / 100);
/  /     sprintf(txt, "AP:%05d00", pulse100);              //
//    OLED_P6x8Str(0, 3, txt);
    N o w T i me   =   ( S T M _GetNowUs(STM0) - NowTime) / 1000;  // ��ȡSTM0 ��ǰʱ�䣬�õ�����
    C                  u rrSpeed = (sint32) (dmm / (NowTime / 1000));          // �����ٶ�mm/s
    // ������Ϣ
    O LED_P6x8Str(0, 0, msg[state]);
    s p  r intf(txt, "M:%03d E:%2.2f Q:%d J:%d ", MagneticField, err, TangentPoint, EnterCircle);
	O LED_P6x8Str(0, 1, txt);
/   /      sprintf(txt,"%02d %02d %02d %02d",Lleft1,Lleft2,Lright2,Lright1);
/ /    OLED_P6x8Str(0, 2,txt);
/ /       sprintf(txt, "Ang: %04d Cod: %04d", TempAngle, ECPULSE1);   // ��ʾ����Ƕ���ֵ�����ռ�ձ���ֵ����������ֵ
//    OLED_P6x8Str(0, 3, txt);
    //��ʾ����й�һ�����ƫ����  ��ǰ����е�ѹֵ ����п�������ʷ��Сֵ ����п�������ʷ���ֵ
    sprintf(txt, "0:%02d %04d %04d %04d ", Lleft1, LnowADC[  0   ],ad_min[0],ad_max[0]);
    O   LED_P6x8Str(0, 2, txt);
    s      printf(txt, "1:%02d %04d %04d %04d ", Lleft2, LnowADC[1], ad_min[1],ad_max[1]);
    O LED_P6x8Str(0, 3, txt);
    s p                                                                                                                                                                                                                                 rintf(txt, "2:%02d %04d %04d %04d ", Lright2, LnowADC[4], ad_min[4],ad_max[4]);
    OLED_P6x8Str(0, 4, txt);
    sprintf(txt, "3:%02d %04d %04d %04d ", Lright1, LnowADC[5], ad_min[5],ad_max[5]);
    O  LED_P6x8Str(0, 5, txt);

//    sprintf(txt,"error = %f",err);
//    OLED_P6x8Str(0, 6, txt);

//    sprintf(txt, "Ring num: %d ", CircleNumber);
//    OLED_P6x8Str(2, 6, txt);
    if (LeftRightCircle == 1)
    	OLED_P6x8Str(0, 6, "Left Ring     ");
    else if (LeftRightCircle == 2)
    	OLED_P6x8Str(0, 6, "Right Ring    ");
    else if(LeftRightSan == 1)
    	OLED_P6x8Str(0,6,"Left San        ");
    else if(LeftRightSan == 2)
    	OLED_P6x8Str(0,6,"Right San       ");
    else
    	OLED_P6x8Str(0, 6, "No Ring or San");


    bat = BatVolt * 11 / 25;  // x/4095*3.3*100*5.7
    sprintf(txt, "B:%d.%02dV %d.%02dm/s", bat / 100, bat % 100, CurrSpeed / 1000, (CurrSpeed / 10) % 100);  // *3.3/4095*3
    OLED_P6x8Str(0, 7, txt);
}
/*************************************************************************
 *  �������ƣ�void ElectroMagneticCar(void)
 *  ����˵������ų�˫������ٿ���
 *  ����˵������
 *  �������أ���
  *  �޸�ʱ�䣺2020��10��28��
 *  ��    ע������2�����
 *************************************************************************/
void ElectroMagneticCar (void)
{
    sint16 bat=0;

    CircleNumber = 2;   // �뻷������0������Ĭ��1��
    TangentPoint = 1;   // �е��ж�   0�е������Ĭ��1�����뻷����ȡ����Ϊ�뻷׼��
    EnterCircle = 0;    // �������  Ĭ�� 0���ɽ�����1���Խ���
    OutCircle = 0;      // �������   Ĭ��0���ɳ�����1���Գ���
    LeftRightCircle = 0;// ��໷�����Ҳ໷ Ĭ��0ԭʼ��1�󻷣�2�һ�
    LowSpeed = 0;       // �ٶȲ�

    SanNumber = 2;	//����·���������һ������·��������
    SanPoint = 1;	    //�е��жϣ�0�е������Ĭ��1���Խ�����·����ȡ����Ϊ�뻷��׼��
    EnterSan = 0;		//���������·��Ĭ��0���ɽ�����·��1���Խ�����·
    OutSan = 0;		//���������·��Ĭ��0���ɳ�����·��1���Գ�����·
    LeftRightSan = 0;	//�����������Ҳ��������·��Ĭ��1���룬2

    // �м�CPU0,CPU1...������ͬʱ������Ļ��ʾ�������ͻ����ʾ
    mutexCpu0TFTIsOk=0;             // CPU1�� 0ռ��/1�ͷ� TFT
    CircleNumber = SetCircleNum();  // ������Ҫ����Բ���ĸ�����

    // ��������Ҫ���ó���⣬�����ǹ̶�ִ�У�
    // �������Ҫ�ɻɹܺ��ⲿ�ж����ʵ��
    // ���������У��ɻɹ���ͨ��Բ������������������ܴ����Ŀ�����
//    OutInGarage(OUT_GARAGE,ReadOutInGarageMode()); // ���Գ��⣬��������������⣬��֮�Ҳ�����
    //OutInGarage(IN_GARAGE,ReadOutInGarageMode());  // �������

//    TFTSPI_CLS(u16BLACK);            // ����
    OLED_CLS();
    // �м�CPU0,CPU1...������ͬʱ������Ļ��ʾ�������ͻ����ʾ
    mutexCpu0TFTIsOk=1;             // CPU1�� 0ռ��/1�ͷ� TFT

    RAllPulse = 0;                  // ȫ�ֱ����������������
    NowTime = STM_GetNowUs(STM0);   // ��ȡSTM0 ��ǰʱ��

    while (1)
    {
        InductorNormal();           // �ɼ���е�ѹ������һ����
        if (MagneticField > 300 && RAllPulse > 3000)    // ֱ�����룬��ֵ����̫�����׶�ʧ�е�
        {
            LowSpeed = 500;         // ����
        }
        else if (MagneticField < 300)
        {
            LowSpeed = 0; // �ָ��ٶ�
        }
        CircleDetect();             // ʶ�𲢽���Բ���ĸ�����
        SanDetect();				//ʶ�𲢽�������·

        ServoDuty = TempAngle;
        ServoCtrl(ServoDuty);       // ���PWM�����ת������ǿ���

//        OffsetDelta = (Lleft2 - Lright2);  // ֱ��ƫ��
        bat=(BatVolt * 11 / 25-750)*Kbat;
//        MotorDuty1 = MtTargetDuty - bat - LowSpeed;  // �в��ٿ��ƣ���תƫ��Ϊ����������
//        MotorDuty2 = MtTargetDuty - bat - LowSpeed;  // �в��ٿ��ƣ���תƫ��Ϊ�����Ҳ����
        CurrSpeed = (sint32) (RAllPulse / NowTime);
        speederr = ExpectedSpeed - speed_pid.kp * (bat + LowSpeed) - CurrSpeed;
        MotorDuty1 = speed_pid.kp * ExpectedSpeed + PidLocCtrl(&speed_pid, speederr);
        if(MotorDuty1 > 5000) MotorDuty1 = 5000;
        if(MotorDuty1 < -5000) MotorDuty1 = -5000;
        if (MagneticField < 30)     // �ж��Ƿ�������
        {
            MotorCtrl(0);        // �������ͣ��
            delayms(200);
        }
        else
        {
            MotorCtrl(MotorDuty1);   // ����˫�������
        }

        if(Game_Over)
        {
            OutInGarage(IN_GARAGE, ReadOutInGarageMode());
        }
    } // WHILE(1)
} // MAIN()

 /*************************************************************************
 *  �������ƣ�void StateElectroMagneticCar(void)
 *  ��    ע������2�����
 *************************************************************************/
void StateElectroMagneticCar (void)
{
    sint16 bat=0;

    CircleNumber = 2;   // �뻷������0������Ĭ��1��
    TangentPoint = 1;   // �е��ж�   0�е������Ĭ��1�����뻷����ȡ����Ϊ�뻷׼��
    EnterCircle = 0;    // �������  Ĭ�� 0���ɽ�����1���Խ���
    OutCircle = 0;      // �������   Ĭ��0���ɳ�����1���Գ���
    LeftRightCircle = 0;// ��໷�����Ҳ໷ Ĭ��0ԭʼ��1�󻷣�2�һ�
    LowSpeed = 0;       // �ٶȲ�

    SanNumber = 2;	//����·���������һ������·��������
    SanPoint = 1;	    //�е��жϣ�0�е������Ĭ��1���Խ�����·����ȡ����Ϊ�뻷��׼��
    EnerSan = 0;		//���������·��Ĭ��0���ɽ�����·��1���Խ�����·
    OutSan = 0;		//���������·��Ĭ��0���ɳ�����·��1���Գ�����·
    LeftRightSan = 0;	//�����������Ҳ��������·��Ĭ��1���룬2

    // �м�CPU0,CPU1...������ͬʱ������Ļ��ʾ�������ͻ����ʾ
    mutexCpu0TFTIsOk=0;             // CPU1�� 0ռ��/1�ͷ� TFT
    CircleNumber = SetCircleNum();  // ������Ҫ����Բ���ĸ�����

    // ��������Ҫ���ó���⣬�����ǹ̶�ִ�У�
    // �������Ҫ�ɻɹܺ��ⲿ�ж����ʵ��
    // ���������У��ɻɹ���ͨ��Բ������������������ܴ����Ŀ�����
//    OutInGarage(OUT_GARAGE,ReadOutInGarageMode()); // ���Գ��⣬��������������⣬��֮�Ҳ�����
    //OutInGarage(IN_GARAGE,ReadOutInGarageMode());  // �������

//    TFTSPI_CLS(u16BLACK);            // ����
    OLED_CLS();
    // �м�CPU0,CPU1...������ͬʱ������Ļ��ʾ�������ͻ����ʾ
    mutexCpu0TFTIsOk=1;             // CPU1�� 0ռ��/1�ͷ� TFT

    RAllPulse = 0;                  // ȫ�ֱ����������������
    NowTime = STM_GetNowUs(STM0);   // ��ȡSTM0 ��ǰʱ��

    while (1)
    {
        InductorNormal();           // �ɼ���е�ѹ������һ����
        Admin_Control();            //�ܿ��ƺ���
        if (MagneticField > 300 && RAllPulse > 3000)    // ֱ�����룬��ֵ����̫�����׶�ʧ�е�
        {
            LowSpeed = 500;         // ����
        }
        else if (MagneticField < 300)
        {
            LowSpeed = 0; // �ָ��ٶ�
        }
        CircleDetect();             // ʶ�𲢽���Բ���ĸ�����
        SanDetect();				//ʶ�𲢽�������·

        ServoDuty = TempAngle;
        ServoCtrl(ServoDuty);       // ���PWM�����ת������ǿ���

//        OffsetDelta = (Lleft2 - Lright2);  // ֱ��ƫ��
        bat=(BatVolt * 11 / 25-750)*Kbat;
//        MotorDuty1 = MtTargetDuty - bat - LowSpeed;  // �в��ٿ��ƣ���תƫ��Ϊ����������
//        MotorDuty2 = MtTargetDuty - bat - LowSpeed;  // �в��ٿ��ƣ���תƫ��Ϊ�����Ҳ����
        CurrSpeed = (sint32) (RAllPulse / NowTime);
        speederr = ExpectedSpeed - speed_pid.kp * (bat + LowSpeed) - CurrSpeed;
        MotorDuty1 = speed_pid.kp * ExpectedSpeed + PidLocCtrl(&speed_pid, speederr);
        if(MotorDuty1 > 5000) MotorDuty1 = 5000;
        if(MotorDuty1 < -5000) MotorDuty1 = -5000;
        if (MagneticField < 30)     // �ж��Ƿ�������
       {
            MotorCtrl(0);        // �������ͣ��
            delayms(200);
        }
        else
        {
            MotorCtrl(MotorDuty1);   // ����˫�������
        }

        if(Game_Over)
        {
            OutInGarage(IN_GARAGE, ReadOutInGarageMode());
        }
    } // WHILE(1)
