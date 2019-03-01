#include<math.h>
#include "IMU.h" 

#include "SEGGER_RTT.h"
#include "stm32f4xx.h"
#include "I2CDriver.h"
#include "delay.h"
#include "I2CDriver.h"


void InitMPU6050(void)
{
	i2c::Instance()->I2C_ByteWrite(PWR_MGMT_1,0x80);			//初始化之前一定要复位以下，再唤醒，延时时间还要够长，否则没用。
	
	delay_ms(2000);
	
	i2c::Instance()->I2C_ByteWrite(PWR_MGMT_1,0x00);

	i2c::Instance()->I2C_ByteWrite(SMPLRT_DIV,0x07);
	
	i2c::Instance()->I2C_ByteWrite(0x1A, 0x06);	//打开低通滤波

	i2c::Instance()->I2C_ByteWrite(CONFIG,0x06);

	i2c::Instance()->I2C_ByteWrite(GYRO_CONFIG,0x18);

	i2c::Instance()->I2C_ByteWrite(ACCEL_CONFIG,0x00);
}

unsigned int GetData(unsigned char REG_Address)
{
	char H,L;
	H=i2c::Instance()->I2C_ByteRead(REG_Address);
	L=i2c::Instance()->I2C_ByteRead(REG_Address+1);
	return (H<<8)+L;   
}

Q4 Q_es; // sensor to earth
Q4 Q_bs; // sensor to base 
volatile uint32_t lastUpdate, now; // 采样周期计数 单位 us
//extern int16_t Gx_offset;
//extern int16_t Gy_offset;
//extern int16_t Gz_offset;
int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;
#define HISTORY_YAW_SIZE 10   //yaw length, short time update
#define _HISTORY_ERROR_TIME 20 //long time update 
#define GET_EZ -100 //get ez
#define TOTNUM 400
#define DEG2RAD 0.017453293f

static float LSB = 16.03556f; //32.8f
static float LSB2RAD = DEG2RAD/16.03556f;

static float Gx_Buf[TOTNUM];
static float Gy_Buf[TOTNUM];
static float Gz_Buf[TOTNUM];
static float Ax_Buf[TOTNUM];
static float Ay_Buf[TOTNUM];
static float Az_Buf[TOTNUM];

static float sum_Gx;
static float sum_Gy;
static float sum_Gz;
static float sum_Input;
static float Last_input_gz;
static int16_t offset_Cnt = 0;
static int16_t ram_234;
static bool isPureRotateFlag = false;

// Fast inverse square-root
/**************************实现函数********************************************
*函数原型:	   float invSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x) 	
输入参数： 要计算的值
输出参数： 结果
*******************************************************************************/
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void toQuaternion(Q4* q, float yaw, float pitch, float roll)// yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    q->w = cy * cp * cr + sy * sp * sr;
    q->x = cy * cp * sr - sy * sp * cr;
    q->y = sy * cp * sr + cy * sp * cr;
    q->z = sy * cp * cr - cy * sp * sr;
}

void getRotationMatrix(float R[9], Q4 q) //row storage
{
	float s = 2.0;

	float xs = q.x * s,   ys = q.y * s,   zs = q.z * s;
	float wx = q.w * xs,  wy = q.w * ys,  wz = q.w * zs;
	float xx = q.x * xs,  xy = q.x * ys,  xz = q.x * zs;
	float yy = q.y * ys,  yz = q.y * zs,  zz = q.y * zs;
	//1.0 - (yy + zz), xy - wz,           xz + wy,
	//xy + wz,         1.0 - (xx + zz),   yz - wx,
	//xz - wy,         yz + wx,           1.0 - (xx + yy);

	R[0] = 1.0 - yy - zz;
	R[1] = xy + wz;
	R[2] = xz - wy;
	R[3] = xy - wz;
	R[4] = 1 - xx - zz;
	R[5] = yz + wx;
	R[6] = xz + wy;
	R[7] = yz - wx;
	R[8] = 1 - xx - yy;
}

void quaternionInverse(Q4 q_in, Q4* q_out)
{
	q_out->w = q_in.w;
	q_out->x = -q_in.x;
	q_out->y = -q_in.y;
	q_out->z = -q_in.z;
}

float quaternionDot(Q4 q1, Q4 q2)
{
	return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
}

void quaternionMulti(Q4 q1, Q4 q2, Q4* q_out)
{
	q_out->w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
	q_out->x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
	q_out->y = q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z;
	q_out->z = q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x; 
}

float getAxisAngle(int axis, Q4 q) //0 means x, 1 means y, 2 means z
{

	float R[9] , x, y , z;
	getRotationMatrix(R, q);
	x = R[0 + axis * 3];
	y = R[1 + axis * 3];
	z = R[2 + axis * 3];
	return atan2(sqrt(x * x + y * y), z);
}

float vector3_dot(V3 in1, V3 in2)
{
	return in1.x * in2.x + in1.y * in2.y + in1.z * in2.z;
}

void vector3_cross(V3 in1, V3 in2, V3* out)
{
	out->x = in1.y * in2.z - in1.z * in2.y;
	out->y = in1.z * in2.x - in1.x * in2.z;
	out->z = in1.x * in2.y - in1.y * in2.x;

}

void vector3_norm(V3* in)
{
	float norm = invSqrt(vector3_dot(*in, *in));
	in->x *= norm;
	in->y *= norm;
	in->z *= norm;
}

float vector3_length(V3 in)
{
	return sqrt(vector3_dot(in, in));
}

void setAxisAngle(V3 axis, float rad, Q4* out) {
    rad = rad * 0.5;
    out->w = cos(rad);
    float s = sin(rad);
    out->x = s * axis.x;
    out->y = s * axis.y;
    out->z = s * axis.z;
}

void quaternion_norm(Q4* in)
{
	float norm = invSqrt(quaternionDot(*in, *in));
	in->w *= norm;
	in->x *= norm;
	in->y *= norm;
	in->z *= norm;
}

void quaternionFromRotate(V3 a, V3 b, Q4* q)
{
    float dot = vector3_dot(a, b);
    V3 xUnitVec3 = {1,0,0};
    V3 yUnitVec3 = {0,1,0};
	V3 tmpvec3;
    if (dot < -0.999999) {
        vector3_cross(xUnitVec3, a, &tmpvec3);
        float length = vector3_length(tmpvec3);
        if(length < 0.000001) {
            vector3_cross(yUnitVec3, a, &tmpvec3);
		}
        vector3_norm(&tmpvec3);
        setAxisAngle(tmpvec3, M_PI, q);
	} else if(dot > 0.999999){
        q->w = 1;
        q->x = 0;
        q->y = 0;
        q->z = 0;
	} else {
        vector3_cross(a, b, &tmpvec3);
        q->w = 1 + dot;
        q->x = tmpvec3.x;
        q->y = tmpvec3.y;
        q->z = tmpvec3.z;
	}
	quaternion_norm(q);
}

void quaternion_rotation(V3 in, Q4 q, V3* out)
{
	Q4 in4 = {0.0, in.x, in.y, in.z};
	Q4 q_star;
	Q4 tmp;
	Q4 out4;
	quaternionInverse(q,&q_star);
	quaternionMulti(q, in4, &tmp);
	quaternionMulti(tmp, q_star, &out4);
	out->x = out4.x;
	out->y = out4.y;
	out->z = out4.z;
}

void toEulerAngle(Q4 q, float* roll, float* pitch, float* yaw)
{
	// roll (x-axis rotation)
	float sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
	float cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	*roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	float sinp = 2.0 * (q.w * q.y - q.z * q.x);
	if (fabs(sinp) >= 1)
		*pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		*pitch = asin(sinp);
	// yaw (z-axis rotation)
	float siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
	float cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
	*yaw = atan2(siny_cosp, cosy_cosp);
	if (*yaw < 0) {
		*yaw += 2 * M_PI;
	}
}

/**************************实现函数********************************************
*函数原型:		void Initial_Timer3(void)
*功　　能:	  初始化Tim2  Tim3 将两个定时器级联，以产生一个32位的定时器来提供系统us 级的计时	
输入参数：无
输出参数：没有	
*******************************************************************************/
void Initial_Timer3(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM3, ENABLE); 
	/* TIM2 configuration*/ 
  /* Time Base configuration 基本配置 配置定时器的时基单元*/
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Period = 0xffff; //自动重装值         
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;       
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
  
  TIM_PrescalerConfig(TIM4, 0, TIM_PSCReloadMode_Update);
  /* Disable the TIM2 Update event */
  TIM_UpdateDisableConfig(TIM4, ENABLE);
  /* ----------------------TIM2 Configuration as slave for the TIM3 ----------*/
  /* Select the TIM2 Input Trigger: TIM3 TRGO used as Input Trigger for TIM2*/
  TIM_SelectInputTrigger(TIM4, TIM_TS_ITR2);
  /* Use the External Clock as TIM2 Slave Mode */
  TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_External1);
  /* Enable the TIM2 Master Slave Mode */
  TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
  TIM_ARRPreloadConfig(TIM4, ENABLE);	
	/* 定时器配置:
	1.设置定时器最大计数值 50000
	2.设置时钟分频系数：TIM_CKD_DIV1
	3. 设置预分频：  1Mhz/50000= 1hz 
	4.定时器计数模式  向上计数模式
	*/		 
  	TIM_TimeBaseStructure.TIM_Period = 0xffff;     
  	TIM_TimeBaseStructure.TIM_Prescaler = 72;	 //1M 的时钟  
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	//应用配置到TIM3 
  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	// 使能TIM3重载寄存器ARR
  	TIM_ARRPreloadConfig(TIM3, ENABLE);	

	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
	TIM_UpdateRequestConfig(TIM3, TIM_UpdateSource_Regular);
	/* ----------------------TIM3 Configuration as Master for the TIM2 -----------*/
  	/* Use the TIM3 Update event  as TIM3 Trigger Output(TRGO) */
  	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
  	/* Enable the TIM3 Master Slave Mode */
  	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);

  	//启动定时器
	TIM_Cmd(TIM3, ENABLE); 
  	TIM_Cmd(TIM4, ENABLE);                  
}

/**************************实现函数********************************************
*函数原型:		uint32_t micros(void)
*功　　能:	  读取系统运行的时间 ，返回单位为us 的时间数。	
输入参数：无
输出参数：处理器当前时间，从上电开始计时  单位 us
*******************************************************************************/
uint32_t micros(void)
{
 	uint32_t temp=0 ;
 	temp = TIM4->CNT; //读高16位时间
 	temp = temp<<16;
 	temp += TIM3->CNT; //读低16位时间
 	return temp;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_init(void)
*功　　能:	  初始化IMU相关	
			  初始化各个传感器
			  初始化四元数
			  将积分清零
			  更新系统时间
输入参数：无
输出参数：没有
*******************************************************************************/
void IMU_init(void)
{	 
	//MPU6050_initialize();
	InitMPU6050();
	//HMC5883L_SetUp();
//	BMP180_init();
	delay_ms(50);
	//MPU6050_initialize();
	InitMPU6050();
	//HMC5883L_SetUp();
//	BMP180_init();
	Initial_Timer3();
	// initialize quaternion
  	Q_es.w = 1.0f;  //初始化四元数
  	Q_es.x = 0.0f;
  	Q_es.y = 0.0f;
  	Q_es.z = 0.0f;
	Q_bs = Q_es;
  	lastUpdate = micros();//更新时间
  	now = micros();
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getValues(float * values)
*功　　能:	 读取加速度 陀螺仪 磁力计 的当前值  
输入参数： 将结果存放的数组首地址
输出参数：没有
*******************************************************************************/
void IMU_getValues(float * values) {  
	int16_t accgyroval[6];
	int i;
	//读取加速度和陀螺仪的当前ADC
	//MPU6050_getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]);

	accgyroval[0] = GetData(ACCEL_XOUT_H);
	accgyroval[1] = GetData(ACCEL_YOUT_H);
	accgyroval[2] = GetData(ACCEL_ZOUT_H);
	accgyroval[3] = GetData(GYRO_XOUT_H);
	accgyroval[4] = GetData(GYRO_YOUT_H);
	accgyroval[5] = GetData(GYRO_ZOUT_H);
	for(i = 0; i<6; i++) {
		if(i < 3) {
			values[i] = (float) accgyroval[i];
		}
		else {
				values[i] = ((float) accgyroval[i]) / LSB; //转成度每秒
	//这里已经将量程改成了 360度每秒  LSB 对应 1度每秒
		}
	}
}


float getMaxMinDiff(float* data, uint16_t number){
	uint16_t i = 0;
	float max_val, min_val;
	max_val = *data;
	min_val = *data;
	for(i = 1; i < number; i++) {
		if(max_val < *(data+i)){
			max_val = *(data+i);
		} else if(min_val > *(data+i) ){
			min_val = *(data+i);
		}
	}
	return (max_val - min_val);
}

void correct_drift()
{
	float dgx,dgy,dgz;
	float dax,day,daz;
	float dift_gx_offset = 0;
	float dift_gy_offset = 0;
	float dift_gz_offset = 0;
	dgx = getMaxMinDiff(Gx_Buf, TOTNUM);
	dgy = getMaxMinDiff(Gy_Buf, TOTNUM);
	dgz = getMaxMinDiff(Gz_Buf, TOTNUM);
	dax = getMaxMinDiff(Ax_Buf, TOTNUM);
	day = getMaxMinDiff(Ay_Buf, TOTNUM);
	daz = getMaxMinDiff(Az_Buf, TOTNUM);
	if ( dgx > 5 * LSB2RAD || dgy > 5 * LSB2RAD ||  dgz > 5 * LSB2RAD )
	{
		ram_234 = 0;
		return;
	}
	if ( dax > 100 || day > 100 ||daz > 100 )
	{
		return;
	}
	//考虑小车在匀速旋转时稳飘	
	V3 rot_body = {0,0,sum_Input};
	V3 rot_sensor = {0,0,0};
	Q4 q_sb = {1,0,0,0};
	Q4 q_bs = Q_bs;
	quaternionInverse(q_bs, &q_sb);
	quaternion_rotation(rot_body, q_sb, &rot_sensor);
	dift_gx_offset = (sum_Gx - rot_sensor.x)/TOTNUM/LSB2RAD;
	dift_gy_offset = (sum_Gy - rot_sensor.y)/TOTNUM/LSB2RAD;
	dift_gz_offset = (sum_Gz - rot_sensor.z)/TOTNUM/LSB2RAD;

	if ( ram_234++ > 2 )
	{
		ram_234 = 0;
		if(fabs(dift_gx_offset) < 10 && fabs(dift_gy_offset) < 10 && fabs(dift_gz_offset) < 10) {
			Gx_offset +=  (int16_t)dift_gx_offset;	
			Gy_offset +=  (int16_t)dift_gy_offset;	
			Gz_offset +=  (int16_t)dift_gz_offset;	
		}
	}
}

void correction_gz(float gx, float gy, float gz, float ax, float ay, float az)
{
	//float input_gz = get_InputGz();
	if(/*fabs(input_gz - Last_input_gz) > 0.001*/1) {
		sum_Gx = 0;
		sum_Gy = 0;
		sum_Gz = 0;
		sum_Input = 0;
		offset_Cnt = 0;
	}
	if ( offset_Cnt < TOTNUM )
	{
		Gx_Buf[offset_Cnt] = gx;
		Gy_Buf[offset_Cnt] = gy;
		Gz_Buf[offset_Cnt] = gz;
			
		Ax_Buf[offset_Cnt] = ax;
		Ay_Buf[offset_Cnt] = ay;
		Az_Buf[offset_Cnt] = az;
			
		sum_Gx += gx;
		sum_Gy += gy;
		sum_Gz += gz;
		//sum_Input += get_InputGz();
		++offset_Cnt;
	} else if ( offset_Cnt == TOTNUM )
    {
		correct_drift();
		offset_Cnt = 0;
		sum_Gx =0;
		sum_Gy =0;
		sum_Gz =0;
		sum_Input = 0;
	} else {
		offset_Cnt = 0;
	}
}	

//input output are all x,y,z,w form
void filterUpdate(Q4* SEq, float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float deltat, float beta)
{
    float SEq_1, SEq_2, SEq_3, SEq_4;
    SEq_1 = SEq->w;
    SEq_2 = SEq->x;
    SEq_3 = SEq->y;
    SEq_4 = SEq->z;
    float norm;
// vector norm
    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
    float f_1, f_2, f_3;
// objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;
// objective function Jacobian elements
    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;
// estimated direction of the gyroscope error
// Axulirary variables to avoid reapeated calcualtions
    float halfSEq_1 = 0.5f * SEq_1;
    float halfSEq_2 = 0.5f * SEq_2;
    float halfSEq_3 = 0.5f * SEq_3;
    float halfSEq_4 = 0.5f * SEq_4;
    float twoSEq_1 = 2.0f * SEq_1;
    float twoSEq_2 = 2.0f * SEq_2;
    float twoSEq_3 = 2.0f * SEq_3;


// Normalise the accelerometer measurement
    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x /= norm; a_y /= norm; a_z /= norm;

    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
    f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
    J_11or24 = twoSEq_3;
    J_12or23 = 2.0f * SEq_4;
    J_13or22 = twoSEq_1;
    J_14or21 = twoSEq_2; J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;

    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;

    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    if(norm > 1e-16){
		SEqHatDot_1 /= norm;
		SEqHatDot_2 /= norm; 
		SEqHatDot_3 /= norm; 
		SEqHatDot_4 /= norm;
	}

	if (w_z > -3*LSB2RAD && w_z < 3*LSB2RAD){
		w_z = 0;
	}
	if (w_x > -3*LSB2RAD && w_x < 3*LSB2RAD){
		w_x = 0;
	}
	if (w_y > -3*LSB2RAD && w_y < 3*LSB2RAD){
		w_y = 0;
	}

    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;


    SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
    SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
    SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
    SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;

    norm = invSqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    SEq_1 *= norm;
    SEq_2 *= norm;
    SEq_3 *= norm;
    SEq_4 *= norm;

    SEq->w = SEq_1;
    SEq->x = SEq_2;
    SEq->y = SEq_3;
    SEq->z = SEq_4;
}

/**************************实现函数**************************************EG2RAD
*函数原型:	   void IMU_getQ(float * q)
*功　　能:	 更新四元数 返回当前的四元数组值
输入参数： 将要存放四元数的数组首地址
输出参数：没有
*******************************************************************************/
void IMU_getQ(Q4 * q) {
	float mygetqval[9];	//用于存放传感器转换结果的数组
	IMU_getValues(mygetqval);	 
	float gx = mygetqval[3] * DEG2RAD;
	float gy = mygetqval[4] * DEG2RAD;
	float gz = mygetqval[5] * DEG2RAD;
	float ax = mygetqval[0];
	float ay = mygetqval[1];
	float az = mygetqval[2];
	correction_gz(gx,gy,gz,ax,ay,az); //gz : rad/s
	now = micros();  //读取时间
	float deltat;
	if(now<lastUpdate){ //定时器溢出过了。
		deltat =  ((float)(now + (0xffff- lastUpdate)) / 1000000.0f);
	}
	else {
		deltat =  ((float)(now - lastUpdate) / 1000000.0f);
	}
	lastUpdate = now;	//更新时间
	filterUpdate(q, gx, gy, gz, ax, ay, az, deltat, 0.05);
}


/**************************实现函数********************************************
*函数原型:	   void IMU_getYawPitchRoll(float * angles)
*功　　能:	 更新四元数 返回当前解算后的姿态数据
输入参数： 将要存放姿态角的数组首地址
输出参数：没有
*******************************************************************************/
void IMU_getYawPitchRoll(float * angles) {
	Q4 q_es = Q_es;
	IMU_getQ(&q_es); //更新全局四元数
	Q_es = q_es;
	Q4 q_eb;
	Q4 q_sb;
	Q4 q_bs = Q_bs;
	quaternionInverse(q_bs, &q_sb);
	quaternionMulti(q_es,q_sb,&q_eb);
	toEulerAngle(q_eb, angles+2, angles+1, angles);
	angles[2] = getAxisAngle(2, q_eb) * 180.0 /M_PI;
	angles[0] = angles[0] * 180.0/M_PI;
	angles[1] = angles[2];
}


bool isPureRotate()
{
	return isPureRotateFlag;
}

void repowerMpu(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_11);
	delay_ms(500);
	NVIC_SystemReset();
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_InitGyro_Offset(void)
*功　　能:	    读取 MPU6050的陀螺仪偏置
此时模块应该被静止放置。以测试静止时的陀螺仪输出
*******************************************************************************/
#define INIT_NUM 200
#define THRESHOLD 4
void MPU6050_InitGyro_Offset(void)
{
	unsigned char i;
	int16_t temp[6];
	float tempgx=0,tempgy=0,tempgz=0;
	float tmpax = 0, tmpay = 0, tmpaz = 0;
	for(i=0;i<INIT_NUM;i++)
	{
		delay_us(100);
		temp[0] = GetData(ACCEL_XOUT_H);
		temp[1] = GetData(ACCEL_YOUT_H);
		temp[2] = GetData(ACCEL_ZOUT_H);
		temp[3] = GetData(GYRO_XOUT_H);
		temp[4] = GetData(GYRO_YOUT_H);
		temp[5] = GetData(GYRO_ZOUT_H);
		tempgx+= (float)temp[3];
		tempgy+= (float)temp[4];
		tempgz+= (float)temp[5];
		tmpax += (float)temp[0];
		tmpay += (float)temp[1];
		tmpaz += (float)temp[2];
	}
	Gx_offset += (int16_t)tempgx/INIT_NUM;
	Gy_offset += (int16_t)tempgy/INIT_NUM;
	Gz_offset += (int16_t)tempgz/INIT_NUM;
	float avgax = tmpax/INIT_NUM;
	float avgay = tmpay/INIT_NUM;
	float avgaz = tmpaz/INIT_NUM;

	V3 eb = {0,0,1};
	V3 es = {avgax,avgay,avgaz};
	vector3_norm(&es);
	Q4 q_bs;
	quaternionFromRotate(es,eb,&q_bs);
	Q_bs = q_bs;
	Q_es = Q_bs;
	return;
}


