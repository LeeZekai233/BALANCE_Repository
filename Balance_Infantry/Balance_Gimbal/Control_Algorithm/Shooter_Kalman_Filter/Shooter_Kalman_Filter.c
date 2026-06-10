#include "main.h"


float First_Order_Kalman_Filter_Cal
	(First_Order_Kalman_Filter_t *_First_Order_Kalman_Filter,float _Z/*测量值*/)
{
	//更新上一次的值
	_First_Order_Kalman_Filter->Error_Est_Last=_First_Order_Kalman_Filter->Error_Est;
	_First_Order_Kalman_Filter->X_hat_Last=_First_Order_Kalman_Filter->X_hat;
	//更新上一次的值
	_First_Order_Kalman_Filter->Error_Est_Last=_First_Order_Kalman_Filter->Error_Est;
	_First_Order_Kalman_Filter->X_hat_Last=_First_Order_Kalman_Filter->X_hat;
	//Step1:Kalman_Gain计算
	_First_Order_Kalman_Filter->Kalman_Gain=
	_First_Order_Kalman_Filter->Error_Est_Last
	/(_First_Order_Kalman_Filter->Error_Est_Last+_First_Order_Kalman_Filter->Error_Mea);
	//Step2:计算预测值
	_First_Order_Kalman_Filter->X_hat=
	_First_Order_Kalman_Filter->X_hat_Last
	+_First_Order_Kalman_Filter->Kalman_Gain*(_Z-_First_Order_Kalman_Filter->X_hat_Last);
	//Step3:预测误差更新
	_First_Order_Kalman_Filter->Error_Est=(1-_First_Order_Kalman_Filter->Kalman_Gain)
	*_First_Order_Kalman_Filter->Error_Est_Last;
	//返回预测值
	return _First_Order_Kalman_Filter->X_hat;
}


