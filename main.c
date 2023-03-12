/*
 * main.c
 */

#include"DSP2833x_Project.h"
#include"DSP2833x_device.h"
#include "C28x_FPU_FastRTS.h"
#include"Park.h"
#include"Inv_park.h"
#include"spwm.h"
#include<math.h>
#include"adc.h"
#include"Solar_F.h"
#include"Power_calc.h"




#define FLASH_RUN 1
#define SRAM_RUN 2
#define RUN_TYPE FLASH_RUN
#if RUN_TYPE==FLASH_RUN
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
#endif

#define pai   3.141592
#define F     50.0        //电压频率50Hz
#define Ld     0.05     //直轴电感 H
#define Lq     0.05     //交轴电感 H

interrupt void epwm2_isr(void);
ADC_GET Voltage_get=ADC_GET_DEFAULTS;
PARK Vabc_to_Vdq=PARK_DEFAULTS;
PARK Iabc_to_Idq=PARK_DEFAULTS;
INV_PARK Vdq_to_Vabc=INV_PARK_DEFAULTS;
SPLL_3ph_SRF_F splll;

CNTL_PI_F active_power_controller;
CNTL_PI_F reactive_power_controller;

CNTL_PI_F dcurrent_controller;
CNTL_PI_F qcurrent_controller;


POWER_GET power_cal=POWER_GET_DEFAULTS;

volatile float P_REF=10.0,Q_REF=0.0;

volatile float32 Theta_out=0.0;
volatile float Ud=0.0,Uq=0.0,Ud0=0.0,Uq0=0.0,Id0=0.0,Iq0=0.0;

/**********Power PI 参数***************/
volatile float KPp=0.001,KPi=0.1;         //有功功率调节PI参数
volatile float KQp=-0.001,KQi=-0.1;       //无功功率调节PI参数

/**********Current PI 参数***************/
volatile float Id_P=5,Id_I=0.0001;     //Id电流调节PI参数
volatile float Iq_P=5,Iq_I=0.0001;   //Iq电流调节PI参数



unsigned char i=0;

float sin_a[100]={0};
float sin_b[100]={0};
float sin_c[100]={0};


void main(void) {

    InitSysCtrl();

    InitEPwm1Gpio();//GPIO0(EPWM1A),GPIO1(EPWM1B)
    InitEPwm2Gpio();//GPIO2(EPWM2A),GPIO3(EPWM2B)
    InitEPwm3Gpio();//GPIO3(EPWM3A),GPIO3(EPWM3B)
    InitEPwm4Gpio();//GPIO4(EPWM4A),GPIO5(EPWM4B)


    EPWMIO_GPIO10();
    EPWMIO_GPIO11();
    EPWMIO_GPIO12();
    EPWMIO_GPIO13();
    EPWMIO_GPIO14();
    EPWMIO_GPIO15();

    DINT;


    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();


#if RUN_TYPE==FLASH_RUN
    MemCopy(&RamfuncsLoadStart,&RamfuncsLoadEnd,&RamfuncsRunStart);
    InitFlash();
#endif



    EALLOW;
    PieVectTable.EPWM2_INT = &epwm2_isr; //     registration interruption
    EDIS;


    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;


    ADC_get_Init();

    SPLL_3ph_SRF_F_init(F,((float)(1.0/10000.0)),&splll);
    CNTL_PI_F_init(&active_power_controller);
    active_power_controller.Kp=KPp;
    active_power_controller.Ki=KPi;

    CNTL_PI_F_init(&reactive_power_controller);
    reactive_power_controller.Kp=KQp;
    reactive_power_controller.Ki=KQi;

    CNTL_PI_F_init(&dcurrent_controller);
    dcurrent_controller.Kp=Id_P;
    dcurrent_controller.Ki=Id_I;

    CNTL_PI_F_init(&qcurrent_controller);
    qcurrent_controller.Kp=Iq_P;
    qcurrent_controller.Ki=Iq_I;

    InitEPwm1Example();
    InitEPwm2Example();
    InitEPwm3Example();
    InitEPwm4Example();

    collectsina();
    collectsinb();
    collectsinc();

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;



    IER |= M_INT3;

    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;

    EINT;
    ERTM;


    for(;;)
    {

    }


}

interrupt void epwm2_isr(void)
{

    Voltage_get.read(&Voltage_get);

    //Voltage PARK
    Vabc_to_Vdq.As=Voltage_get.Ua;
    Vabc_to_Vdq.Bs=Voltage_get.Ub;
    Vabc_to_Vdq.Cs=Voltage_get.Uc;
    Vabc_to_Vdq.Theta=Theta_out;
    Vabc_to_Vdq.calc(&Vabc_to_Vdq);


/*
    /////////////Current PARK//////////////
    Iabc_to_Idq.As=Voltage_get.Ia;
    Iabc_to_Idq.Bs=Voltage_get.Ib;
    Iabc_to_Idq.Cs=Voltage_get.Ic;
    Iabc_to_Idq.Theta=Theta_out;
    Iabc_to_Idq.calc(&Iabc_to_Idq);
*/

/*
    ///////Power calculator//////////////
    power_cal.Id=Iabc_to_Idq.Ds;
    power_cal.Iq=Iabc_to_Idq.Qs;
    power_cal.Ud=Vabc_to_Vdq.Ds;
    power_cal.calc(&power_cal);

    ///////////Active Power PI////////////////
    active_power_controller.Ref=P_REF;
    active_power_controller.Fbk=power_cal.P;
    active_power_controller.Kp=KPp;
    active_power_controller.Ki=KPi;
    CNTL_PI_F_FUNC(&active_power_controller);   //启动有功功率调节

    ///////////////D axis Current PI/////////////////
    dcurrent_controller.Ref=active_power_controller.Out;
    dcurrent_controller.Fbk=Iabc_to_Idq.Ds;
    dcurrent_controller.Kp=Id_P;
    dcurrent_controller.Ki=Id_I;
    CNTL_PI_F_FUNC(&dcurrent_controller);

    ////////////Reactive Power PI////////////////
    reactive_power_controller.Ref=Q_REF;
    reactive_power_controller.Fbk=power_cal.Q;
    reactive_power_controller.Kp=KQp;
    reactive_power_controller.Ki=KQi;
    CNTL_PI_F_FUNC(&reactive_power_controller);


    ///////////////Q axis Current PI////////////////////
    qcurrent_controller.Ref=reactive_power_controller.Out;
    qcurrent_controller.Fbk=Iabc_to_Idq.Qs;
    qcurrent_controller.Kp=Iq_P;
    qcurrent_controller.Ki=Iq_I;
    CNTL_PI_F_FUNC(&qcurrent_controller);

    ///////////DQ axis Voltage output/////////////////
    Ud=Vabc_to_Vdq.Ds+dcurrent_controller.Out-2*pai*F*Lq*Iq0;
    Uq=Vabc_to_Vdq.Qs+qcurrent_controller.Out+2*pai*F*Lq*Id0;
    */

    Ud=Vabc_to_Vdq.Ds;
    Uq=Vabc_to_Vdq.Qs;

    splll.v_q[0]=Uq;
    SPLL_3ph_SRF_F_FUNC(&splll);
    Theta_out=splll.theta[0];


    Vdq_to_Vabc.Ds=Ud;
    Vdq_to_Vabc.Qs=Uq;
    Vdq_to_Vabc.Theta=Theta_out;
    Vdq_to_Vabc.calc(&Vdq_to_Vabc);


    if(Vdq_to_Vabc.As>=0)
    {
        GpioDataRegs.GPASET.bit.GPIO10=1;
        DELAY_US(5);//dead time 50us
        GpioDataRegs.GPACLEAR.bit.GPIO11=1;

    }
    else
    {
        GpioDataRegs.GPACLEAR.bit.GPIO10=1;
        DELAY_US(50); //dead time 50us
        GpioDataRegs.GPASET.bit.GPIO11=1;
    }

    if(Vdq_to_Vabc.Bs>=0)
    {
        GpioDataRegs.GPASET.bit.GPIO12=1;
        DELAY_US(50);//dead time 50us
        GpioDataRegs.GPACLEAR.bit.GPIO13=1;
    }
    else
    {
        GpioDataRegs.GPACLEAR.bit.GPIO12=1;
        DELAY_US(50);//dead time 50us
        GpioDataRegs.GPASET.bit.GPIO13=1;
    }
    if(Vdq_to_Vabc.Cs>=0)
    {
        GpioDataRegs.GPASET.bit.GPIO14=1;
        DELAY_US(50);//dead time 50us
        GpioDataRegs.GPACLEAR.bit.GPIO15=1;
    }
    else
    {
        GpioDataRegs.GPACLEAR.bit.GPIO14=1;
        DELAY_US(50);//dead time 50us
        GpioDataRegs.GPASET.bit.GPIO15=1;
    }
   i++;
   if(i>=100)
   {
       i=0;
   }


     // Clear INT flag for this timer
     EPwm2Regs.ETCLR.bit.INT = 1;
     // Acknowledge this interrupt to receive more interrupts from group 3
     PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

