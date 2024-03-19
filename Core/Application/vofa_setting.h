#ifndef _VOFA_SETTING_H_
#define _VOFA_SETTING_H_

/*
文件功能&使用方法：
    定义是否启用USB虚拟串口
    修改VOFAClearTime选择超时清空的时间（根据定时调用的时间更改）
    修改VOFAClearLen选择超长丢弃的长度（一般不用改）
    定义是否启用各项模块
*/

extern float VofaData[16];//需要赋给控件的变量extern过来
extern float tempFloat[20];


//#define USB_TRANSMIT     //USB虚拟串口模式 普通串口模式请注释
#define VOFAClearTime 50 //定义超时清空时间  = VOFAOutTime*一次中断时间(0~255)
#define VOFAClearLen 20  //定义超长丢弃长度  (0~63)

//#define UseVOFASlider1
//#define UseVOFASlider2
//#define UseVOFASlider3
//#define UseVOFASlider4
//#define UseVOFASlider5
//#define UseVOFASlider6
//#define UseVOFASlider7

//#define UseVOFAButton1
//#define UseVOFAButton2
//#define UseVOFAButton3
//#define UseVOFAButton4
//#define UseVOFAButton5
//#define UseVOFAButton6
//#define UseVOFAButton7
//#define UseVOFAButton8

//#define UseVOFAKey1
//#define UseVOFAKey2
//#define UseVOFAKey3
//#define UseVOFAKey4

//#define UseVOFABar1
//#define UseVOFABar2

#ifdef UseVOFASlider1
#define Vofa_Slider1
#endif
#ifdef UseVOFASlider2 
#define Vofa_Slider2
#endif
#ifdef UseVOFASlider3
#define Vofa_Slider3
#endif
#ifdef UseVOFASlider4
#define Vofa_Slider4
#endif
#ifdef UseVOFASlider5
#define Vofa_Slider5
#endif
#ifdef UseVOFASlider6
#define Vofa_Slider6
#endif
#ifdef UseVOFASlider7
#define Vofa_Slider7
#endif

#ifdef UseVOFAButton1
#define Vofa_Button1 
#endif
#ifdef UseVOFAButton2
#define Vofa_Button2
#endif
#ifdef UseVOFAButton3
#define Vofa_Button3
#endif
#ifdef UseVOFAButton4
#define Vofa_Button4
#endif
#ifdef UseVOFAButton5
#define Vofa_Button5
#endif
#ifdef UseVOFAButton6
#define Vofa_Button6
#endif
#ifdef UseVOFAButton7
#define Vofa_Button7
#endif
#ifdef UseVOFAButton8
#define Vofa_Button8
#endif

#ifdef UseVOFAKey1
#define Vofa_Key1
#endif
#ifdef UseVOFAKey2
#define Vofa_Key2
#endif
#ifdef UseVOFAKey3
#define Vofa_Key3
#endif
#ifdef UseVOFAKey4
#define Vofa_Key4
#endif

#ifdef UseVOFABar1
#define Vofa_Bar_x1
#define Vofa_Bar_y1
#endif
#ifdef UseVOFABar2
#define Vofa_Bar_x2
#define Vofa_Bar_y2
#endif

#endif
