struct MtoC_Message {
	/*
int16 Gui_wTurnOn,Gui_wTurnOff;  //MtoC From PC
int16 Gui_wMPPT_Status;      //MtoC From DCDC
int16 Gui_GridTie_Enable;    //MtoC From PC
int16 Gui_DC_Board_Connect;  //MtoC From M3
int16 Gui_wLoadDefault_Enable,Gui_wSetting_Enable;  //MtoC From PC
int16 Gui_wUtility_Setting;    //MtoC  From PC
int16 Gui_wPanelVolt,Gui_wPanelCurrent,Gui_wBoostOutputVolt,Gui_wPanelOutputPower; //MtoC From DCDC
*/
short int PC_Gui_wTurnOn;
short int PC_Gui_wTurnOff;
short int PC_Gui_GridTie_Enable;
short int PC_Gui_wLoadDefault_Enable;
short int PC_Gui_wSetting_Enable;
short int PC_Gui_wUtility_Setting;
short int PC_Gui_wLogEnable;		//Vieri/20111123/For buffer data get
short int PC_Gui_wLogStatus;
short int Gui_wSampleLength;
short int Gui_wSampleRate;
short int Gui_CH1_ID;
short int Gui_CH2_ID;
short int Gui_CH3_ID;
short int Gui_CH4_ID;

short int M3_Gui_wInvVolt_High;
short int M3_Gui_wInvVolt_Low;
short int M3_Gui_wFreq_High;
short int M3_Gui_wFreq_Low; //MtoC From M3

short int DCDC_Gui_wMPPT_Status;
short int DCDC_Gui_wPanelVolt;
short int DCDC_Gui_wPanelCurrent;
short int DCDC_Gui_wBoostOutputVolt;
short int DCDC_Gui_wPanelOutputPower;

short int C28_wSendTurnOnCmd_Clear;
short int C28_wSendTurnOffCmd_Clear;

short int M3_Gui_DC_Board_Connect;


};

extern struct MtoC_Message MtoC_Message1;

//C28x owned memory region 
struct CtoM_Message {
	/*
int16 Gui_wInvVoltRms;  //CtoM From C28x
int16 Gui_wInvCurrRms;  //CtoM From C28x
int16 Gui_wDcBusVoltAvg;  //CtoM From C28x
int16 Gui_wActivePower;  //CtoM From C28x
int16 Gui_wInvFreq;    //CtoM From C28x 
int16 Gui_wFaultCode;   //CtoM From C28x
int16 Gui_wSysMode; //CtoM From C28x
int16 Gui_wGridTie_Status;   //CtoM From C28x
int16 Gui_wLLCOutputVolt;   //CtoM From C28x
int16 Gui_wInvVolt_High,Gui_wInvVolt_Low,Gui_wFreq_High,Gui_wFreq_Low; //CtoM From C28
*/

short int C28_Gui_wInvVoltRms;  //CtoM From C28x
short int C28_Gui_wInvCurrRms;  //CtoM From C28x
short int C28_Gui_wDcBusVoltAvg;  //CtoM From C28x
short int C28_Gui_wActivePower;  //CtoM From C28x
short int C28_Gui_wInvFreq;    //CtoM From C28x 
short int C28_Gui_wFaultCode;   //CtoM From C28x
short int C28_Gui_wSysMode; //CtoM From C28x
short int C28_Gui_wGridTie_Status;   //CtoM From C28x
short int C28_Gui_wLLCOutputVolt;   //CtoM From C28x

short int C28_Gui_wInvVolt_High;
short int C28_Gui_wInvVolt_Low;
short int C28_Gui_wFreq_High;
short int C28_Gui_wFreq_Low; //CtoM From C28

short int C28_wSendTurnOnCmd;
short int C28_wSendTurnOffCmd;

short int PC_Gui_wTurnOn_Clear;
short int PC_Gui_wTurnOff_Clear;
short int PC_Gui_wLoadDefault_Enable_Clear;
short int PC_Gui_wSetting_Enable_Clear;
short int PC_Gui_wLogEnable_Clear;
short int C28_Gui_wLogStatus;
short int C28_wDataLogEnable;

};

extern struct CtoM_Message CtoM_Message1;
