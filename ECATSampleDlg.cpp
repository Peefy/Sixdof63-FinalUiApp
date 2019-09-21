
#include "StdAfx.h"
#include "ECATSample.h"
#include "ECATSampleDlg.h"

#include <math.h>

#include <fstream>
#include <deque>

#include "DialogRegister.h"

#include "chart.h"

#include "Sixdofdll2010.h"

#include "communication/sixdof.h"
#include "communication/phasemotioncontrol.h"
#include "communication/SerialPort.h"

#include "control/sensor.h"
#include "control/pid.h"
#include "control/kalman_filter.h"
#include "control/inertialnavigation.h"
#include "control/landvision.h"
#include "control/illusion.h"

#include "ui/uiconfig.h"

#include "util/model.h"

#include "register\register.h"

#include "hardware\SixdofDioAndPulseCount.h"

#include "Sixdofdll2010.h"

#include "glut.h"
#include "opengl/sixdofopenglhelper.h"

#include "cwnds/panel.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

using namespace std;

#define COLOR_RED     RGB(255, 123, 123)
#define COLOR_GREEN   RGB(123, 255, 123)

#define TIMER_MS 10

#define SIXDOF_CONTROL_DELEY 1
#define SCENE_THREAD_DELAY 15
#define SENSOR_THREAD_DELAY 20
#define DATA_BUFFER_THREAD_DELAY 20

#define CHIRP_TIME       2.5
#define ZERO_POS_TIME    2.5
#define ENABLE_CHIRP  false
#define ENABLE_SHOCK  false
#define ENABLE_ZERO_POS true
//#define ENABLE_OPENGL       1
//#define ENABLE_LINE_GRAPH   1

#define IS_USE_NAVIGATION 0
#define IS_USE_KALMAN_FILTER 0

bool enableShock = ENABLE_SHOCK;
bool enableChirp = ENABLE_CHIRP;
bool enableZeroPos = ENABLE_ZERO_POS;

void SixdofControl();
void SensorRead();

UINT8 InitialFlag = 0;

bool closeDataThread = true;
bool stopAndMiddle = true;

volatile HANDLE DataThread;
volatile HANDLE SensorThread;
volatile HANDLE SceneThread;
volatile HANDLE DataBufferThread;

// 六自由度平台逻辑控制
PhaseMotionControl delta;
// 六自由度数据
DataPackage data = {0};
IllusionDataAdapter vision;

// 六自由度平台状态
double pulse_cal[AXES_COUNT];
double poleLength[AXES_COUNT];
double lastStartPulse[AXES_COUNT];
SixDofPlatformStatus status = SIXDOF_STATUS_BOTTOM;
SixDofPlatformStatus lastStartStatus = SIXDOF_STATUS_BOTTOM;

// 图表
CChartCtrl m_ChartCtrl1; 
CChartLineSerie *pLineSerie1;
CChartLineSerie *pLineSerie2;
CChartLineSerie *pLineSerie3;
CChartLineSerie *pLineSerie4;
CChartLineSerie *pLineSerie5;
CChartLineSerie *pLineSerie6;

bool isAutoInit = true;
bool isTest = true;
bool isCosMode = false;
bool stopSCurve = false;

double testVal[FREEDOM_NUM] = {0};
double testHz[FREEDOM_NUM] = {0};
double testPhase[FREEDOM_NUM] = {0};
double testZeroPos[FREEDOM_NUM] = {0};

double chartBottomAxisPoint[CHART_POINT_NUM] = { 0 };
double chartXValPoint[CHART_POINT_NUM] = { 0 };
double chartYValPoint[CHART_POINT_NUM] = { 0 };
double chartZValPoint[CHART_POINT_NUM] = { 0 };
double chartRollValPoint[CHART_POINT_NUM] = { 0 };
double chartPitchValPoint[CHART_POINT_NUM] = { 0 };
double chartYawValPoint[CHART_POINT_NUM] = { 0 };

int runTime = 0;
int netTime = 0;
double chartTime = 0;
double controltime = 0;

kalman1_state kalman_rollFilter;
kalman1_state kalman_yawFilter;
kalman1_state kalman_pitchFilter;
kalman1_state kalman_xFilter;
kalman1_state kalman_yFilter;
kalman1_state kalman_zFilter;

MovingAverageFilterType rollFiter = {6, 0.08};
MovingAverageFilterType yawFiter = {6, 0.08};
MovingAverageFilterType pitchFiter = {6, -0.08};

MovingAverageFilterType xFiter = {8, 0.01};
MovingAverageFilterType yFiter = {8, 0.01};
MovingAverageFilterType zFiter = {8, 0.01};

double hpfAccWn = 1.0;
double lpfAccWn = 0.5; 
double hpfAngleSpdWn = 0.2;

int visionCtrlComand = 0;
int isRecieveData = 0;

double ShockVal = 10.0;
double ShockHz = 8.0;
double pitchPositiveScale = 1.2;

mutex cs;
mutex ctrlCommandLockobj;
DataPackageDouble vData = {0};
DataPackageDouble visionData = {0};
DataPackageDouble lastData = {0};

DWORD WINAPI DataTransThread(LPVOID pParam)
{
	while (true)
	{
		SixdofControl();
	}
	return 0;
}

DWORD WINAPI SensorInfoThread(LPVOID pParam)
{
	while (true)
	{
		vision.SendData(false, status, data.X / 10.0, data.Y / 10.0, 
			data.Z / 10.0, data.Roll / 100.0, data.Yaw / 100.0, data.Pitch / 100.0);	
		Sleep(SENSOR_THREAD_DELAY);
	}
	return 0;
}

void CECATSampleDlg::JudgeControlCommand()
{
	if (ctrlCommandLockobj.try_lock())
	{
		switch (visionCtrlComand)
		{
		case ILLUSION_CTL_CMD_START_INT32:
			// Rise
			if (status == SIXDOF_STATUS_BOTTOM || status == SIXDOF_STATUS_ISFALLING)
			{
				OnBnClickedBtnRise();
			}
			else if (status == SIXDOF_STATUS_MIDDLE || status == SIXDOF_STATUS_RUN)
			{
				OnCommandStopme();
			}
			break;
		case ILLUSION_CTL_CMD_STOP_INT32:
			// Down
			OnBnClickedBtnDown();
			break;
		case ILLUSION_CTL_CMD_CONNECT_INT32:
			// Run
			OnBnClickedBtnStart();
			break;
		case ILLUSION_CTL_CMD_DISCONNECT_INT32:
			// StopAndMiddle
			OnCommandStopme();
			break;
		case ILLUSION_CTL_CMD_PAUSE_INT32:
			// StopAndMiddle
			OnCommandStopme();
			break;
		case ILLUSION_CTL_CMD_RECOVER_INT32:
			// Run
			OnBnClickedBtnStart();
			break;
		default:
			break;
		}
		visionCtrlComand = 0;
		ctrlCommandLockobj.unlock();
	}
}

void VisionDataDeal()
{	
	vision.RenewData();	
	if(cs.try_lock())
	{
		vData.X = vision.X;
		vData.Y = vision.Y;
		vData.Z = vision.Z;
		vData.Roll = vision.Roll;
		vData.Pitch = vision.Pitch;
		vData.Yaw = vision.Yaw;
		vision.SetPoseAngle(vData.Roll, vData.Pitch, vData.Yaw);
#if IS_USE_KALMAN_FILTER 1
		vision.X = kalman1_filter(&kalman_xFilter, vision.X);
		vision.Y = kalman1_filter(&kalman_yFilter, vision.Y);
		vision.Z = kalman1_filter(&kalman_zFilter, vision.Z);
		vision.Roll = kalman1_filter(&kalman_rollFilter, vision.Roll);
		vision.Pitch = kalman1_filter(&kalman_pitchFilter, vision.Pitch);
		vision.Yaw = kalman1_filter(&kalman_yawFilter, vision.Yaw);
#endif
		cs.unlock();
	}
	isRecieveData = 1;
	if (vision.IsRecievedData == false)
		return;
	if (ctrlCommandLockobj.try_lock())
	{
		visionCtrlComand = vision.GetControlCommand();
		enableShock = vision.IsEanbleShock();
		ShockHz = vision.GetShockHz();
		ctrlCommandLockobj.unlock();
	}
}

DWORD WINAPI SceneInfoThread(LPVOID pParam)
{
	while (true)
	{	
		DWORD starttime = GetTickCount();
		VisionDataDeal();
		Sleep(SCENE_THREAD_DELAY);
		DWORD endtime = GetTickCount();
		netTime = endtime - starttime;
	}
	return 0;
}

double t = 0;
int bufferLength = 0;

DWORD WINAPI DataBufferInfoThread(LPVOID pParam)
{
	// 上升、下降、中立位的逻辑控制
	delta.DDAControlThread();
	return 0;
}

// 释放所有线程资源
void CloseThread()
{
	try
	{
		if (DataThread != INVALID_HANDLE_VALUE)
		{
			CloseHandle(DataThread);
			DataThread = INVALID_HANDLE_VALUE;
		}
	}
	catch (CException* e)
	{
		printf("%d\r\n", e->ReportError());
	}
	try
	{
		if (SensorThread != INVALID_HANDLE_VALUE)
		{
			CloseHandle(SensorThread);
			SensorThread = INVALID_HANDLE_VALUE;
		}
	}
	catch (CException* e)
	{
		printf("%d\r\n", e->ReportError());
	}
	try
	{
		if (SceneThread != INVALID_HANDLE_VALUE)
		{
			CloseHandle(SceneThread);
			SceneThread = INVALID_HANDLE_VALUE;
		}
	}
	catch (CException* e)
	{
		printf("%d\r\n", e->ReportError());
	}
	try
	{
		if (DataBufferThread != INVALID_HANDLE_VALUE)
		{
			CloseHandle(DataBufferThread);
			DataBufferThread = INVALID_HANDLE_VALUE;
		}
	}
	catch (CException* e)
	{
		printf("%d\r\n", e->ReportError());
	}
}

void OpenThread()
{
	DataThread = (HANDLE)CreateThread(NULL, 0, DataTransThread, NULL, 0, NULL);
	SensorThread = (HANDLE)CreateThread(NULL, 0, SensorInfoThread, NULL, 0, NULL);
	SceneThread = (HANDLE)CreateThread(NULL, 0, SceneInfoThread, NULL, 0, NULL);
	DataBufferThread = (HANDLE)CreateThread(NULL, 0, DataBufferInfoThread, NULL, 0, NULL);
}

void SensorRead()
{

}

void SixdofControl()
{
	static double deltat = 0.031;
	DWORD start_time = 0;
	start_time = GetTickCount();
	delta.RenewNowPulse();
	Sleep(5);
	if(closeDataThread == false)
	{	
		auto delay = SIXDOF_CONTROL_DELEY;
		double dis[AXES_COUNT] = {0};
		// 正弦测试运动
		if (isTest == true)
		{
			double nowHz[AXES_COUNT] = {testHz[0], testHz[1], testHz[2], testHz[3], testHz[4], testHz[5]};
			double chirpTime = CHIRP_TIME;
			double zeroposTime = ZERO_POS_TIME;
			double pi = 3.1415926;
			double nowt = t;
			double x = 0;
			double y = 0;
			double z = 0;
			double roll = 0;
			double pitch = 0;
			double yaw = 0;
			double stopTime = 0;
			// 开始正弦运动时加一个chrip信号缓冲
			if (t <= chirpTime && enableChirp == true)
			{
				for (auto i = 0; i < AXES_COUNT; ++i)
				{
					nowHz[i] = t * testHz[i] / chirpTime / chirpTime; 
				}
				x = sin(2 * pi * nowHz[0] * nowt) * testVal[0];
				y = sin(2 * pi * nowHz[1] * nowt) * testVal[1];
				z = sin(2 * pi * nowHz[2] * nowt) * testVal[2];
				roll = sin(2 * pi * nowHz[3] * nowt) * testVal[3];
				pitch = sin(2 * pi * nowHz[4] * nowt) * testVal[4];
				yaw = sin(2 * pi * nowHz[5] * nowt) * testVal[5];
			}		
			else if (enableChirp == true)
			{
				nowt = t - chirpTime;
				x = sin(2 * pi * nowHz[0] * nowt + 2 * pi * testHz[0]) * testVal[0];
				y = sin(2 * pi * nowHz[1] * nowt + 2 * pi * testHz[1]) * testVal[1];
				z = sin(2 * pi * nowHz[2] * nowt + 2 * pi * testHz[2]) * testVal[2];
				roll = sin(2 * pi * nowHz[3] * nowt + 2 * pi * testHz[3]) * testVal[3];
				pitch = sin(2 * pi * nowHz[4] * nowt + 2 * pi * testHz[4]) * testVal[4];
				yaw = sin(2 * pi * nowHz[5] * nowt + 2 * pi * testHz[5]) * testVal[5];
			}
			else if(isCosMode == false)
			{
				x = sin(2 * pi * nowHz[0] * nowt) * testVal[0];
				y = sin(2 * pi * nowHz[1] * nowt) * testVal[1];
				z = sin(2 * pi * nowHz[2] * nowt) * testVal[2];
				roll = sin(2 * pi * nowHz[3] * nowt) * testVal[3];
				pitch = sin(2 * pi * nowHz[4] * nowt) * testVal[4];
				yaw = sin(2 * pi * nowHz[5] * nowt) * testVal[5];
			}
			else
			{

			}
			if (isCosMode = true)
			{
				double nowphase_t[AXES_COUNT] = {t,t,t,t,t,t};
				for (int i = 0;i < AXES_COUNT; ++i)
				{
					if (nowHz[i] != 0)
					{
						auto phase_t = t - 1.0 / nowHz[i] * (testPhase[i] / 360.0);
						nowphase_t[i] = DOWN_RANGE(phase_t, 0);
					}			
					else
					{
						nowphase_t[i] = t;
					}					
				}
				x = sin(2 * pi * nowHz[0] * nowphase_t[0]) * testVal[0];
				y = sin(2 * pi * nowHz[1] * nowphase_t[1]) * testVal[1];
				z = sin(2 * pi * nowHz[2] * nowphase_t[2]) * testVal[2];
				roll = sin(2 * pi * nowHz[3] * nowphase_t[3]) * testVal[3];
				pitch = sin(2 * pi * nowHz[4] * nowphase_t[4]) * testVal[4];
				yaw = sin(2 * pi * nowHz[5] * nowphase_t[5]) * testVal[5];
			}
			if (enableZeroPos == true)
			{
				auto minZeroPosTime = min(t, zeroposTime) / zeroposTime;
				x += testZeroPos[0] * minZeroPosTime;
				y += testZeroPos[1] * minZeroPosTime;
				z += testZeroPos[2] * minZeroPosTime;
				roll += testZeroPos[3] * minZeroPosTime;
				pitch += testZeroPos[4] * minZeroPosTime;
				yaw += testZeroPos[5] * minZeroPosTime;
			}
			data.X = (int16_t)(x * 10);
			data.Y = (int16_t)(y * 10);
			data.Z = (int16_t)(z * 10);
			data.Roll = (int16_t)(roll * 100);
			data.Pitch = (int16_t)(pitch * 100);
			data.Yaw = (int16_t)(yaw * 100);
			double* pulse_dugu = Control(x, y, z, roll, yaw, pitch);
			for (auto ii = 0; ii < AXES_COUNT; ++ii)
			{
				pulse_cal[ii] = pulse_dugu[ii];
				poleLength[ii] = pulse_dugu[ii];
				pulse_cal[ii] *= MM_TO_PULSE_COUNT_SCALE;
				auto pulse = pulse_cal[ii];
				dis[ii] = pulse;
			}
			t += deltat;
			delta.PidCsp(dis);
		}
		// 模拟视景
		else
		{
			if (cs.try_lock())
			{
				visionData = vData;
				cs.unlock();
			}
			double pi = 3.1415926;
			double shockVal = ShockVal;
			double shockHz = ShockHz;
			auto shockz = sin(2 * pi * shockHz * t) * shockVal;
			auto x = RANGE(MyMAFilter(&xFiter, visionData.X), -VISION_MAX_XYZ, VISION_MAX_XYZ);
			auto y = RANGE(MyMAFilter(&yFiter, visionData.Y), -VISION_MAX_XYZ, VISION_MAX_XYZ);
			auto z = RANGE(MyMAFilter(&zFiter, visionData.Z), -VISION_MAX_XYZ, VISION_MAX_XYZ);
			auto roll = RANGE(MyMAFilter(&rollFiter, visionData.Roll), -VISION_MAX_DEG, VISION_MAX_DEG);
			auto pitch = RANGE(MyMAFilter(&pitchFiter, visionData.Pitch), -VISION_MAX_DEG, VISION_MAX_DEG);
			auto yaw = RANGE(MyMAFilter(&yawFiter, visionData.Yaw), -VISION_MAX_DEG, VISION_MAX_DEG);
			if (pitch < 0){
				pitch = pitch * pitchPositiveScale;
			}
			z += (enableShock == true ? shockz : 0);
			double* pulse_dugu = Control(x, y, z, roll, yaw, pitch);
			for (auto ii = 0; ii < AXES_COUNT; ++ii)
			{
				pulse_cal[ii] = pulse_dugu[ii];
				poleLength[ii] = pulse_dugu[ii];
				pulse_cal[ii] *= MM_TO_PULSE_COUNT_SCALE;
				auto pulse = pulse_cal[ii];
				dis[ii] = pulse;
			}
			data.X = (int16_t)(x * 10);
			data.Y = (int16_t)(y * 10);
			data.Z = (int16_t)(z * 10);
			data.Roll = (int16_t)(roll * 100);
			data.Yaw = (int16_t)(yaw * 100);
			data.Pitch = (int16_t)(pitch * 100);
			t += deltat;
			delta.PidCsp(dis);
		}

		Sleep(delay);
		DWORD end_time = GetTickCount();
		runTime = end_time - start_time;
	}
}

void MoveValPoint()
{
	chartTime += 0.067;
	for (auto i = 0; i < CHART_POINT_NUM; ++i)
	{
		chartBottomAxisPoint[i] = chartBottomAxisPoint[i + 1];

		chartXValPoint[i] = chartXValPoint[i + 1];
		chartYValPoint[i] = chartYValPoint[i + 1];
		chartZValPoint[i] = chartZValPoint[i + 1];
		chartRollValPoint[i] = chartRollValPoint[i + 1];
		chartPitchValPoint[i] = chartPitchValPoint[i + 1];
		chartYawValPoint[i] = chartYawValPoint[i + 1];
	}

	chartBottomAxisPoint[CHART_POINT_NUM - 1] = chartTime;
	chartXValPoint[CHART_POINT_NUM - 1] = data.X * XYZ_SCALE;
	chartYValPoint[CHART_POINT_NUM - 1] = data.Y * XYZ_SCALE; 
	chartZValPoint[CHART_POINT_NUM - 1] = data.Z * XYZ_SCALE;
	chartRollValPoint[CHART_POINT_NUM - 1] = data.Roll * DEG_SCALE;
	chartPitchValPoint[CHART_POINT_NUM - 1] = data.Pitch * DEG_SCALE;
	chartYawValPoint[CHART_POINT_NUM - 1] = data.Yaw * DEG_SCALE;

	m_ChartCtrl1.EnableRefresh(false);

	pLineSerie1->ClearSerie();
	pLineSerie2->ClearSerie();
	pLineSerie3->ClearSerie();
	pLineSerie4->ClearSerie();
	pLineSerie5->ClearSerie();
	pLineSerie6->ClearSerie();

	pLineSerie1->AddPoints(chartBottomAxisPoint, chartXValPoint, CHART_POINT_NUM);
	pLineSerie2->AddPoints(chartBottomAxisPoint, chartYValPoint, CHART_POINT_NUM);
	pLineSerie3->AddPoints(chartBottomAxisPoint, chartZValPoint, CHART_POINT_NUM);
	pLineSerie4->AddPoints(chartBottomAxisPoint, chartRollValPoint, CHART_POINT_NUM);
	pLineSerie5->AddPoints(chartBottomAxisPoint, chartPitchValPoint, CHART_POINT_NUM);
	pLineSerie6->AddPoints(chartBottomAxisPoint, chartYawValPoint, CHART_POINT_NUM);

	m_ChartCtrl1.EnableRefresh(true);

}

CECATSampleDlg::CECATSampleDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CECATSampleDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDI_ICON1);
}

void CECATSampleDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_CHART_CTRL, m_ChartCtrl1);
}

BEGIN_MESSAGE_MAP(CECATSampleDlg, CDialog)
	//{{AFX_MSG_MAP(CECATSampleDlg)
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BTN_InitialCard, OnBTNInitialCard)
	ON_BN_CLICKED(IDC_BTN_FindSlave, OnBTNFindSlave)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_CHK_SVON, OnChkSvon)
	ON_BN_CLICKED(IDC_CHK_ABS, OnChkAbs)
	//}}AFX_MSG_MAP
	ON_BN_CLICKED(IDC_BTN_Rise, &CECATSampleDlg::OnBnClickedBtnRise)
	ON_BN_CLICKED(IDC_BTN_Middle, &CECATSampleDlg::OnBnClickedBtnMiddle)
	ON_BN_CLICKED(IDC_BTN_Start, &CECATSampleDlg::OnBnClickedBtnStart)
	ON_BN_CLICKED(IDC_BTN_StopMe, &CECATSampleDlg::OnBnClickedBtnStopme)
	ON_BN_CLICKED(IDC_BTN_Down, &CECATSampleDlg::OnBnClickedBtnDown)
	ON_BN_CLICKED(IDOK, &CECATSampleDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDC_BTN_CONNECT, &CECATSampleDlg::OnBnClickedBtnConnect)
	ON_BN_CLICKED(IDC_BTN_Resetme, &CECATSampleDlg::OnBnClickedBtnResetme)
	ON_BN_CLICKED(IDC_BTN_DISCONNECT, &CECATSampleDlg::OnBnClickedBtnDisconnect)
	ON_WM_SHOWWINDOW()
	ON_BN_CLICKED(IDC_BTN_SINGLE_UP, &CECATSampleDlg::OnBnClickedBtnSingleUp)
	ON_BN_CLICKED(IDC_BTN_SINGLE_DOWN, &CECATSampleDlg::OnBnClickedBtnSingleDown)
	ON_WM_KEYDOWN()
	ON_WM_CHAR()
	ON_BN_CLICKED(IDC_BUTTON_TEST, &CECATSampleDlg::OnBnClickedButtonTest)
	ON_BN_CLICKED(IDC_BUTTON_EXIT, &CECATSampleDlg::OnBnClickedButtonExit)
	ON_BN_CLICKED(IDC_BUTTON_TEST3, &CECATSampleDlg::OnBnClickedButtonTest3)
	ON_BN_CLICKED(IDC_BUTTON_STOP_TEST, &CECATSampleDlg::OnBnClickedButtonStopTest)
END_MESSAGE_MAP()

void CECATSampleDlg::AppConfigInit()
{
	double result = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_HPF_ACC_WN_KEY);
	if (result != NULL)
	{
		hpfAccWn = result;
	}
	result = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_LPF_ACC_WN_KEY);
	if (result != NULL)
	{
		lpfAccWn = result;
	}
	result = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_ANG_SPD_WN_KEY);
	if (result != NULL)
	{
		hpfAngleSpdWn = result;
	}
	result = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_ROLL_SCALE_KEY);
	if (result != NULL)
	{
		rollFiter.Scale = result;
	}
	result = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_YAW_SCALE_KEY);
	if (result != NULL)
	{
		yawFiter.Scale = result;
	}
	result = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_PITCH_SCALE_KEY);
	if (result != NULL)
	{
		pitchFiter.Scale = result;
	}
	result = config::ParseIntJsonFromFile(JSON_PARA_FILE_NAME, JSON_ROLL_FILTER_LEVEL_KEY);
	if (result != NULL)
	{
		rollFiter.FilterLevel = (unsigned char)result;
	}
	result = config::ParseIntJsonFromFile(JSON_PARA_FILE_NAME, JSON_YAW_FILTER_LEVEL_KEY);
	if (result != NULL)
	{
		yawFiter.FilterLevel = (unsigned char)result;
	}
	result = config::ParseIntJsonFromFile(JSON_PARA_FILE_NAME, JSON_PITCH_FILTER_LEVEL_KEY);
	if (result != NULL)
	{
		pitchFiter.FilterLevel = (unsigned char)result;
	}

	result = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_X_SCALE_KEY);
	xFiter.Scale = result;
	result = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_Y_SCALE_KEY);
	yFiter.Scale = result;
	result = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_Z_SCALE_KEY);
	zFiter.Scale = result;
	result = config::ParseIntJsonFromFile(JSON_PARA_FILE_NAME, JSON_X_FILTER_LEVEL_KEY);
	if (result != NULL)
	{
		xFiter.FilterLevel = (unsigned char)result;
	}
	result = config::ParseIntJsonFromFile(JSON_PARA_FILE_NAME, JSON_Y_FILTER_LEVEL_KEY);
	if (result != NULL)
	{
		yFiter.FilterLevel = (unsigned char)result;
	}
	result = config::ParseIntJsonFromFile(JSON_PARA_FILE_NAME, JSON_Z_FILTER_LEVEL_KEY);
	if (result != NULL)
	{
		zFiter.FilterLevel = (unsigned char)result;
	}
}

void CECATSampleDlg::KalmanFilterInit()
{
	kalman1_init(&kalman_rollFilter, 0, 0.01);
	kalman1_init(&kalman_yawFilter, 0, 0.01);
	kalman1_init(&kalman_pitchFilter, 0, 0.01);
	kalman1_init(&kalman_xFilter, 0, 0.01);
	kalman1_init(&kalman_yFilter, 0, 0.01);
	kalman1_init(&kalman_zFilter, 0, 0.01);
}

void CECATSampleDlg::ChartInit()
{
	CRect rect, rectChart;
	GetDlgItem(IDC_CHART_CTRL)->GetWindowRect(&rect);
	ScreenToClient(rect);
	rectChart = rect;
	rectChart.top = rect.bottom + 3;
	rectChart.bottom = rectChart.top + rect.Height();

	CChartAxis *pAxis = NULL;
	pAxis = m_ChartCtrl1.CreateStandardAxis(CChartCtrl::BottomAxis);
	pAxis->SetAutomatic(true);
	pAxis = m_ChartCtrl1.CreateStandardAxis(CChartCtrl::LeftAxis);
	pAxis->SetAutomatic(true);

	m_ChartCtrl1.GetTitle()->AddString(_T(CHART_TITLE));

	m_ChartCtrl1.
		GetLeftAxis()->
		GetLabel()->
		SetText(_T(CHART_LEFT_AXIS_TITLE));

	m_ChartCtrl1.EnableRefresh(false);

	m_ChartCtrl1.RemoveAllSeries();
	m_ChartCtrl1.GetLegend()->SetVisible(true);

	pLineSerie1 = m_ChartCtrl1.CreateLineSerie();
	pLineSerie1->SetSeriesOrdering(poNoOrdering);
	pLineSerie1->SetWidth(2);
	pLineSerie1->SetName(_T(IDC_STATIC_X_VAL_SHOW_TEXT));
	
	pLineSerie2 = m_ChartCtrl1.CreateLineSerie();
	pLineSerie2->SetSeriesOrdering(poNoOrdering);
	
	pLineSerie2->SetWidth(2);
	pLineSerie2->SetName(_T(IDC_STATIC_Y_VAL_SHOW_TEXT)); 

	pLineSerie3 = m_ChartCtrl1.CreateLineSerie();
	pLineSerie3->SetSeriesOrdering(poNoOrdering);
	pLineSerie3->SetWidth(2);
	pLineSerie3->SetName(_T(IDC_STATIC_Z_VAL_SHOW_TEXT));

	pLineSerie4 = m_ChartCtrl1.CreateLineSerie();
	pLineSerie4->SetSeriesOrdering(poNoOrdering);
	pLineSerie4->SetWidth(2);
	pLineSerie4->SetName(_T(IDC_STATIC_ROLL_VAL_SHOW_TEXT));

	pLineSerie5 = m_ChartCtrl1.CreateLineSerie();
	pLineSerie5->SetSeriesOrdering(poNoOrdering); 
	pLineSerie5->SetWidth(2);
	pLineSerie5->SetName(_T(IDC_STATIC_PITCH_VAL_SHOW_TEXT));

	pLineSerie6 = m_ChartCtrl1.CreateLineSerie();
	pLineSerie6->SetSeriesOrdering(poNoOrdering);  
	pLineSerie6->SetWidth(2);
	pLineSerie6->SetName(_T(IDC_STATIC_YAW_VAL_SHOW_TEXT));

	m_ChartCtrl1.EnableRefresh(true);

}

void CECATSampleDlg::AppInit()
{
	int statusTemp = 0;
	// 检测应用是否注册
	auto isRegister = TestAppIsRegister();
	if (isRegister == false)
	{
		DialogRegister * dlg = new DialogRegister(this);
		dlg->DoModal();
		dlg->Create(IDD_DIALOG_REGISTER, this);
		dlg->ShowWindow(SW_SHOW);
		delete dlg;
		if (DialogRegister::IsRegister == false)
		{
			// 不注册就退出应用
			OnBnClickedOk();
		}
		else
		{
			MessageBox(_T("注册成功！"));
		}
	}
	// 读取上次应用退出时平台的状态
	config::ReadStatusAndPulse(statusTemp, lastStartPulse);
	// 读取停止并回中模式
	//stopAndMiddle = config::ReadIsAutoStopAndMiddle();
	// 上次应用退出时平台的状态
	lastStartStatus = (SixDofPlatformStatus)statusTemp;

	SetDlgItemText(IDC_EDIT_X_VAL, _T("0"));
	SetDlgItemText(IDC_EDIT_Y_VAL, _T("0"));
	SetDlgItemText(IDC_EDIT_Z_VAL, _T("0"));
	SetDlgItemText(IDC_EDIT_ROLL_VAL, _T("0"));
	SetDlgItemText(IDC_EDIT_PITCH_VAL, _T("0"));
	SetDlgItemText(IDC_EDIT_YAW_VAL, _T("0"));

	SetDlgItemText(IDC_EDIT_X_HZ, _T("0"));
	SetDlgItemText(IDC_EDIT_Y_HZ, _T("0"));
	SetDlgItemText(IDC_EDIT_Z_HZ, _T("0"));
	SetDlgItemText(IDC_EDIT_ROLL_HZ, _T("0"));
	SetDlgItemText(IDC_EDIT_PITCH_HZ, _T("0"));
	SetDlgItemText(IDC_EDIT_YAW_HZ, _T("0"));

	SetDlgItemText(IDC_EDIT_X_PHASE, _T("0"));
	SetDlgItemText(IDC_EDIT_Y_PHASE, _T("0"));
	SetDlgItemText(IDC_EDIT_Z_PHASE, _T("0"));
	SetDlgItemText(IDC_EDIT_ROLL_PHASE, _T("0"));
	SetDlgItemText(IDC_EDIT_PITCH_PHASE, _T("0"));
	SetDlgItemText(IDC_EDIT_YAW_PHASE, _T("0"));

	SetDlgItemText(IDC_EDIT_X_ZERO_POS, _T("0"));
	SetDlgItemText(IDC_EDIT_Y_ZERO_POS, _T("0"));
	SetDlgItemText(IDC_EDIT_Z_ZERO_POS, _T("0"));
	SetDlgItemText(IDC_EDIT_ROLL_ZERO_POS, _T("0"));
	SetDlgItemText(IDC_EDIT_PITCH_ZERO_POS, _T("0"));
	SetDlgItemText(IDC_EDIT_YAW_ZERO_POS, _T("0"));

	CDialog::SetWindowTextW(_T(WINDOW_TITLE));
	GetDlgItem(IDC_BTN_Start)->SetWindowTextW(_T(IDC_BTN_START_SHOW_TEXT));
	GetDlgItem(IDC_BTN_SINGLE_UP)->SetWindowTextW(_T(IDC_BTN_SINGLE_UP_SHOW_TEXT));
	GetDlgItem(IDC_BTN_SINGLE_DOWN)->SetWindowTextW(_T(IDC_BTN_SINGLE_DOWN_SHOW_TEXT));

	GetDlgItem(IDC_BTN_CONNECT)->SetWindowTextW(_T(IDC_BTN_CONNECT_SHOW_TEXT));
	GetDlgItem(IDC_BTN_DISCONNECT)->SetWindowTextW(_T(IDC_BTN_DISCONNECT_SHOW_TEXT));

	GetDlgItem(IDC_STATIC_POSE)->SetWindowTextW(_T(IDC_STATIC_POSE_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_LENGTH)->SetWindowTextW(_T(IDC_STATIC_LENGTH_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_SENSOR)->SetWindowTextW(_T(IDC_STATIC_SENSOR_SHOW_TEXT));

	GetDlgItem(IDC_STATIC_X_VAL)->SetWindowTextW(_T(IDC_STATIC_X_VAL_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_Y_VAL)->SetWindowTextW(_T(IDC_STATIC_Y_VAL_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_Z_VAL)->SetWindowTextW(_T(IDC_STATIC_Z_VAL_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_ROLL_VAL)->SetWindowTextW(_T(IDC_STATIC_ROLL_VAL_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_PITCH_VAL)->SetWindowTextW(_T(IDC_STATIC_PITCH_VAL_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_YAW_VAL)->SetWindowTextW(_T(IDC_STATIC_YAW_VAL_SHOW_TEXT));

	GetDlgItem(IDC_STATIC_X_HZ)->SetWindowTextW(_T(IDC_STATIC_X_HZ_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_Y_HZ)->SetWindowTextW(_T(IDC_STATIC_Y_HZ_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_Z_HZ)->SetWindowTextW(_T(IDC_STATIC_Z_HZ_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_ROLL_HZ)->SetWindowTextW(_T(IDC_STATIC_ROLL_HZ_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_PITCH_HZ)->SetWindowTextW(_T(IDC_STATIC_PITCH_HZ_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_YAW_HZ)->SetWindowTextW(_T(IDC_STATIC_YAW_HZ_SHOW_TEXT));

	GetDlgItem(IDC_STATIC_X_PHASE)->SetWindowTextW(_T(IDC_STATIC_X_PHASE_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_Y_PHASE)->SetWindowTextW(_T(IDC_STATIC_Y_PHASE_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_Z_PHASE)->SetWindowTextW(_T(IDC_STATIC_Z_PHASE_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_ROLL_PHASE)->SetWindowTextW(_T(IDC_STATIC_ROLL_PHASE_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_PITCH_PHASE)->SetWindowTextW(_T(IDC_STATIC_PITCH_PHASE_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_YAW_PHASE)->SetWindowTextW(_T(IDC_STATIC_YAW_PHASE_SHOW_TEXT));

	GetDlgItem(IDC_STATIC_X_ZERO_POS)->SetWindowTextW(_T(IDC_STATIC_X_ZERO_POS_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_Y_ZERO_POS)->SetWindowTextW(_T(IDC_STATIC_Y_ZERO_POS_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_Z_ZERO_POS)->SetWindowTextW(_T(IDC_STATIC_Z_ZERO_POS_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_ROLL_ZERO_POS)->SetWindowTextW(_T(IDC_STATIC_ROLL_ZERO_POS_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_PITCH_ZERO_POS)->SetWindowTextW(_T(IDC_STATIC_PITCH_ZERO_POS_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_YAW_ZERO_POS)->SetWindowTextW(_T(IDC_STATIC_YAW_ZERO_POS_SHOW_TEXT));

	GetDlgItem(IDC_STATIC_TEST)->SetWindowTextW(_T(IDC_STATIC_TEST_SHOW_TEXT));
	GetDlgItem(IDC_BUTTON_TEST)->SetWindowTextW(_T(IDC_BUTTON_TEST_SHOW_TEXT));

	GetDlgItem(IDC_STATIC_PLATFORM)->SetWindowTextW(_T(IDC_STATIC_PLATFORM_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_SWITCH_STATUS)->SetWindowTextW(_T(IDC_STATIC_SWITCH_STATUS_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_POLE_LENGTH)->SetWindowTextW(_T(IDC_STATIC_POLE_LENGTH_SHOW_TEXT));

	GetDlgItem(IDC_STATIC_APP_STATUS)->SetWindowTextW(_T(CORPORATION_NAME));
	GetDlgItem(IDC_STATIC_APP_TITLE)->SetWindowTextW(_T(APP_TITLE));
	CFont* font = new CFont();
	font->CreatePointFont(APP_TITLE_FONT_SIZE, _T("Times New Roman"));
	GetDlgItem(IDC_STATIC_APP_TITLE)->SetFont(font);
	ChartInit();
	AppConfigInit();
	for (auto i = 1; i <= AXES_COUNT; ++i)
	{
		CString xx;
		xx.Format(_T("%d"), i);
		((CComboBox*)GetDlgItem(IDC_CBO_SingleNo))->AddString(xx);
	}
	((CComboBox*)GetDlgItem(IDC_CBO_SingleNo))->SetCurSel(0);
	KalmanFilterInit();
	SetPlatformPara(PlaneAboveHingeLength, PlaneAboveBottomLength, 
		CircleTopRadius, CircleBottomRadius, DistanceBetweenHingeTop,
		DistanceBetweenHingeBottom);
	SetWashOutFilterPara(hpfAccWn, lpfAccWn, hpfAngleSpdWn, ILLUSION_DT);
	OpenThread();
}

double CECATSampleDlg::GetCEditNumber(int cEditId)
{
	CString str;
	GetDlgItemText(cEditId, str);
	auto val = _tstof(str);
	return val;
}

void CECATSampleDlg::OnShowWindow(BOOL bShow, UINT nStatus)
{
	CDialog::OnShowWindow(bShow, nStatus);
}

BOOL CECATSampleDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	SetIcon(m_hIcon, TRUE);			
	SetIcon(m_hIcon, FALSE);	
	// 应用初始化
	AppInit();
	// openGl初始化
	InitOpenGlControl();
	SetTimer(0, TIMER_MS, NULL);
	return TRUE;  
}

int isShowSingleUpDown = 0;
BOOL CECATSampleDlg::PreTranslateMessage(MSG* pMsg)
{
	if (pMsg->message == WM_KEYDOWN)
	{
		if (pMsg->wParam == VK_SHIFT)
		{
			isShowSingleUpDown = !isShowSingleUpDown;
			((CButton*)GetDlgItem(IDC_BTN_SINGLE_UP))->ShowWindow(isShowSingleUpDown);
			((CButton*)GetDlgItem(IDC_BTN_SINGLE_DOWN))->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_BTN_FindSlave)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_EDIT_SlaveNum)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_CBO_SingleNo)->ShowWindow(isShowSingleUpDown);

			GetDlgItem(IDC_CHK_SVON)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_BTN_Resetme)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_BTN_CONNECT)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_BTN_DISCONNECT)->ShowWindow(isShowSingleUpDown);

			GetDlgItem(IDC_BTN_InitialCard)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_STATIC)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_CBO_CardNo)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_STATIC_SHOW_INIT)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_EDIT_InitialStatus)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_SHOW)->ShowWindow(!isShowSingleUpDown);
		}
		if (pMsg->wParam == 'X')
		{
			closeDataThread = true;
			delta.ServoStop();
		}
	}
	else if (pMsg->message == WM_KEYUP)
	{

	}
	return CDialog::PreTranslateMessage(pMsg);
}

BOOL CECATSampleDlg::PreCreateWindow(CREATESTRUCT& cs)
{
	cs.style |= WS_CLIPSIBLINGS | WS_CLIPCHILDREN;
	return CDialog::PreCreateWindow(cs);
}

void CECATSampleDlg::InitOpenGlControl()
{
	CWnd *pWnd = GetDlgItem(IDC_SHOW);
	hrenderDC = ::GetDC(pWnd->GetSafeHwnd());
	if(SetWindowPixelFormat(hrenderDC)==FALSE) 
		return; 
	if(CreateViewGLContext(hrenderDC)==FALSE) 
		return; 
	OpenGlLightInit();
}

BOOL CECATSampleDlg::SetWindowPixelFormat(HDC hDC) 
{ 
	PIXELFORMATDESCRIPTOR pixelDesc; 
	pixelDesc.nSize = sizeof(PIXELFORMATDESCRIPTOR); 
	pixelDesc.nVersion = 1; 
	pixelDesc.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER | PFD_STEREO_DONTCARE; 
	pixelDesc.iPixelType = PFD_TYPE_RGBA; 
	pixelDesc.cColorBits = 32; 
	pixelDesc.cRedBits = 8; 
	pixelDesc.cRedShift = 16; 
	pixelDesc.cGreenBits = 8; 
	pixelDesc.cGreenShift = 8; 
	pixelDesc.cBlueBits = 8; 
	pixelDesc.cBlueShift = 0; 
	pixelDesc.cAlphaBits = 0; 
	pixelDesc.cAlphaShift = 0; 
	pixelDesc.cAccumBits = 64; 
	pixelDesc.cAccumRedBits = 16; 
	pixelDesc.cAccumGreenBits = 16; 
	pixelDesc.cAccumBlueBits = 16; 
	pixelDesc.cAccumAlphaBits = 0; 
	pixelDesc.cDepthBits = 32; 
	pixelDesc.cStencilBits = 8; 
	pixelDesc.cAuxBuffers = 0; 
	pixelDesc.iLayerType = PFD_MAIN_PLANE; 
	pixelDesc.bReserved = 0; 
	pixelDesc.dwLayerMask = 0; 
	pixelDesc.dwVisibleMask = 0; 
	pixelDesc.dwDamageMask = 0; 
	PixelFormat = ChoosePixelFormat(hDC,&pixelDesc); 
	if(PixelFormat==0) 
	{ 
		PixelFormat = 1; 
		if(DescribePixelFormat(hDC,PixelFormat, 
			sizeof(PIXELFORMATDESCRIPTOR),&pixelDesc)==0) 
		{ 
			return FALSE; 
		} 
	} 
	if(SetPixelFormat(hDC,PixelFormat,&pixelDesc)==FALSE) 
	{ 
		return FALSE; 
	} 
	return TRUE; 
} 

BOOL CECATSampleDlg::CreateViewGLContext(HDC hDC) 
{ 
	hrenderRC = wglCreateContext(hDC); 
	if(hrenderRC==NULL) 
		return FALSE; 
	if(wglMakeCurrent(hDC,hrenderRC)==FALSE) 
		return FALSE; 
	return TRUE; 
} 

void CECATSampleDlg::RenderScene()   
{ 
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	OpenGL_SetData(data.X * XYZ_SCALE, data.Y * XYZ_SCALE, data.Z * XYZ_SCALE, 
		data.Roll * DEG_SCALE, data.Yaw * DEG_SCALE, data.Pitch * DEG_SCALE);
	RenderSixdofImage();
	SwapBuffers(hrenderDC); 
} 

void CECATSampleDlg::FillCtlColor(CWnd* cwnd, COLORREF color)
{
	CDC * pDC = cwnd->GetDC();
	CRect rct;
	cwnd->GetWindowRect(&rct);
	CBrush brs;
	brs.CreateSolidBrush(color);
	CRect picrct;
	picrct.top = 0;
	picrct.left = 0;
	picrct.bottom = rct.Height();
	picrct.right = rct.Width();
	pDC->FillRect(&picrct, &brs);
	pDC->ReleaseAttribDC();
	cwnd->ReleaseDC(pDC);
}

void CECATSampleDlg::ShowSingleInitImage(int ctlId)
{
	CRect rect; 
	GetDlgItem(ctlId)->GetClientRect(&rect);
	Gdiplus::Graphics g(GetDlgItem(ctlId)->GetDC()->m_hDC);   
	g.Clear(Gdiplus::Color::White);
	g.DrawImage(GetPumpImage(0, _T("mm")), 0, 0, rect.Width(), rect.Height());
}

void CECATSampleDlg::ShowSingleInitImage(CWnd* pic, float value)
{
	CRect rect; 
	pic->GetClientRect(&rect);
	Gdiplus::Graphics g(pic->GetDC()->m_hDC);   
	g.Clear(Gdiplus::Color::White);
	g.DrawImage(GetPumpImage(value, _T("mm")), 0, 0, rect.Width(), rect.Height());
}

void CECATSampleDlg::ShowInitImage()
{
	ShowSingleInitImage(IDC_STATIC_PIC_POLE1);
	ShowSingleInitImage(IDC_STATIC_PIC_POLE2);
	ShowSingleInitImage(IDC_STATIC_PIC_POLE3);
	ShowSingleInitImage(IDC_STATIC_PIC_POLE4);
	ShowSingleInitImage(IDC_STATIC_PIC_POLE5);
	ShowSingleInitImage(IDC_STATIC_PIC_POLE6);
}

CPen drawLinePen(PS_SOLID, 2, RGB(0, 0, 0));
CPen clearLinePen(PS_SOLID, 10, RGB(244, 244, 244));
CBrush clearBrush(RGB(244, 244, 244));
double ptCenterX = 52;
double ptCenterY = 52;
double f0;
double p0X;
double p0Y;
double p1X;
double p1Y;
//	张开的弧度
float radian=60;
//	同心圆半径
int radius[]=
{
	50,
	24,
	1
};

double fMax = 100;
double fMin = -100;

void CECATSampleDlg::ShowSingleImage(CWnd* pic, float value)
{
	value *= 2;
	CDC* dc = pic->GetDC();   
	f0 = ((180-radian)/2+radian/(fMax-fMin)*(-value + fMax)) / 180 * PI;
	dc->SelectObject(&clearBrush);
	dc->FillSolidRect(2,28,100,32, RGB(255, 255, 255));
	p0X = ptCenterX+(radius[2]*cos(f0));
	p0Y = ptCenterY-(radius[2]*sin(f0));
	p1X = ptCenterX+((radius[1])*cos(f0));
	p1Y = ptCenterY-((radius[1])*sin(f0));
	dc->SelectObject(&drawLinePen);
	dc->MoveTo(p0X, p0Y);
	dc->LineTo(p1X, p1Y);
	pic->ReleaseDC(dc);
}

void CECATSampleDlg::ShowImage()
{
	ShowSingleImage(GetDlgItem(IDC_STATIC_PIC_POLE1), poleLength[0]);
	ShowSingleImage(GetDlgItem(IDC_STATIC_PIC_POLE2), poleLength[1]);
	ShowSingleImage(GetDlgItem(IDC_STATIC_PIC_POLE3), poleLength[2]);
	ShowSingleImage(GetDlgItem(IDC_STATIC_PIC_POLE4), poleLength[3]);
	ShowSingleImage(GetDlgItem(IDC_STATIC_PIC_POLE5), poleLength[4]);
	ShowSingleImage(GetDlgItem(IDC_STATIC_PIC_POLE6), poleLength[5]);
}

void CECATSampleDlg::RenderSwitchStatus()
{
	FillCtlColor(GetDlgItem(IDC_STATIC_STATUS1), delta.IsAtBottoms[0] ? COLOR_GREEN : COLOR_RED);
	FillCtlColor(GetDlgItem(IDC_STATIC_STATUS2), delta.IsAtBottoms[1] ? COLOR_GREEN : COLOR_RED);
	FillCtlColor(GetDlgItem(IDC_STATIC_STATUS3), delta.IsAtBottoms[2] ? COLOR_GREEN : COLOR_RED);
	FillCtlColor(GetDlgItem(IDC_STATIC_STATUS4), delta.IsAtBottoms[3] ? COLOR_GREEN : COLOR_RED);
	FillCtlColor(GetDlgItem(IDC_STATIC_STATUS5), delta.IsAtBottoms[4] ? COLOR_GREEN : COLOR_RED);
	FillCtlColor(GetDlgItem(IDC_STATIC_STATUS6), delta.IsAtBottoms[5] ? COLOR_GREEN : COLOR_RED);
}

void CECATSampleDlg::OnPaint() 
{
	if (IsIconic())
	{
		CPaintDC dc(this); 

		SendMessage(WM_ICONERASEBKGND, (WPARAM) dc.GetSafeHdc(), 0);

		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		dc.DrawIcon(x, y, m_hIcon);		
	}
	else
	{
		CDialog::OnPaint();
	}
}

HCURSOR CECATSampleDlg::OnQueryDragIcon()
{
	return (HCURSOR) m_hIcon;
}

void CECATSampleDlg::OnBTNInitialCard() 
{
	CString xx;
	InitialFlag = true;
	if (InitialFlag)
	{
		xx.Format(_T("%d"), 1);
		((CComboBox*)GetDlgItem(IDC_CBO_CardNo))->AddString(xx);
		((CComboBox*)GetDlgItem(IDC_CBO_CardNo))->SetCurSel(0);
		((CButton *)GetDlgItem(IDC_BTN_FindSlave))->EnableWindow(TRUE);
	}	
	else
		MessageBox(_T(INIT_FAIL_MESSAGE));
}

void CECATSampleDlg::OnBTNFindSlave() 
{
	
}

void CECATSampleDlg::EanbleButton(int isenable)
{	
	((CButton*) GetDlgItem(IDC_CHK_SVON))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BTN_Rise))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BTN_Middle))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BTN_Start))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BTN_StopMe))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BTN_Down))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BTN_Resetme))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BUTTON_TEST))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BUTTON_STOP_TEST))->EnableWindow(isenable);
}

void CECATSampleDlg::OnTimer(UINT nIDEvent) 
{
	I16 rt;
	U16 InitialDone;
	CString statusStr;

	if (isAutoInit == true)
	{
		isAutoInit = false;
		OnBTNInitialCard();	
	}
	MoveValPoint();
	JudgeControlCommand();
	//RenderScene();
	//RenderSwitchStatus();
	//ShowImage();
	statusStr.Format(_T("x:%d y:%d z:%d y:%d a:%d b:%d net:%d run:%d"), data.X, data.Y, data.Z,
		data.Yaw, data.Pitch, data.Roll, netTime, runTime);
	SetDlgItemText(IDC_EDIT_Pose, statusStr);

	statusStr.Format(_T("1:%.2f 2:%.2f 3:%.2f 4:%.2f 5:%.2f 6:%.2f"), 
		poleLength[0], poleLength[1], poleLength[2],
		poleLength[3], poleLength[4], poleLength[5]);
	SetDlgItemText(IDC_EDIT_Pulse, statusStr);

	statusStr.Format(_T("1:%.2f 2:%.2f 3:%.2f 4:%.2f 5:%.2f 6:%.2f"),
		vision.X, vision.Y, vision.Z,
		vision.Roll, vision.Pitch, vision.Yaw);
	SetDlgItemText(IDC_EDIT_Sensor, statusStr);

	delta.CheckStatus(status);
	if(InitialFlag == 0)
	{
		EanbleButton(0);
	}
	if(InitialFlag != 2)
	{
		rt = 0;
		InitialDone = 0;
		if (rt == 0)
		{
			if (InitialDone == 0 && InitialFlag != 2)
			{
				SetDlgItemText(IDC_EDIT_InitialStatus, _T(STATUS_INIT_FINISH));
				EanbleButton(1);
				InitialFlag = 2;
				Sleep(100);
				if (lastStartStatus == 0){
					//delta.ResetStatus();
					//delta.RiseLittle();
				}		
				else
				{
					delta.PowerOnSelfTest(lastStartStatus, lastStartPulse);
				}			
			}		
			else if (InitialDone == 1)
			{
				SetDlgItemText(IDC_EDIT_InitialStatus, _T(STATUS_INIT_ING));
				EanbleButton(0);
			}	
			else if (InitialDone == 99)
			{
				EanbleButton(0);
				SetDlgItemText(IDC_EDIT_InitialStatus, _T(STATUS_INIT_FAIL));
			}
			else
			{

			}			
		}
	}	
	CDialog::OnTimer(nIDEvent);
}

void CECATSampleDlg::OnChkSvon() 
{
	delta.ServoAllOnOff(true);
}

void CECATSampleDlg::OnOK() 
{
	delta.ServoStop();
	Sleep(100);
	delta.Close(status);
	CDialog::OnOK();
}

void CECATSampleDlg::OnChkAbs() 
{
	CButton *lpBtnPtr = (CButton *)GetDlgItem(IDC_CHK_ABS);
}

void CECATSampleDlg::OnBnClickedBtnRise()
{	
	delta.ReadAllSwitchStatus();
	// 所有开关触碰到了才能上升
	if (delta.IsAllAtBottom() == false)
	{
		//MessageBox(_T(SIXDOF_NOT_BOTTOM_AND_RISE_MESSAGE));
		return;
	}	
	if (status != SIXDOF_STATUS_BOTTOM)
	{
		//MessageBox(_T(SIXDOF_NOT_BOTTOM_MESSAGE));
		return;
	}
	delta.ResetStatus();
	Sleep(20);
	status = SIXDOF_STATUS_ISRISING;	
	delta.Rise();
}

void CECATSampleDlg::OnBnClickedBtnMiddle()
{
	if (stopAndMiddle == true)
	{
		if (status != SIXDOF_STATUS_READY)
		{
			//MessageBox(_T(SIXDOF_NOT_MIDDLE_MESSAGE));
			return;
		}
	}
	else
	{
		if (status != SIXDOF_STATUS_MIDDLE)
		{
			//MessageBox(_T(SIXDOF_NOT_MIDDLE_MESSAGE));
			return;
		}
	}
	status = SIXDOF_STATUS_READY;
	delta.MoveToZeroPulseNumber();
}

void CECATSampleDlg::OnBnClickedBtnStart()
{
	if (status != SIXDOF_STATUS_READY)
	{
	    //MessageBox(_T(SIXDOF_NOT_BEGIN_MESSAGE));
		return;
	}
	status = SIXDOF_STATUS_RUN;
	//delta.ServoStop();
	delta.StopRiseDownMove();
	Sleep(100);
	delta.RenewNowPulse();
	delta.GetMotionAveragePulse();
	delta.UnlockServo();
	delta.PidControllerInit();
	// 正常使用模式
	isTest = false;
	isCosMode = false;
	t = 0;
	closeDataThread = false;
}

void CECATSampleDlg::OnCommandStopme()
{
	if (status != SIXDOF_STATUS_RUN)
	{
		return;
	}
	status = SIXDOF_STATUS_READY;
	closeDataThread = true;
	Sleep(100);
	delta.UnlockServo();
//	delta.ServoStop();
	delta.StopRiseDownMove();
	Sleep(100);
	delta.MoveToZeroPulseNumber();
	ResetDefaultData(&data);
}

void CECATSampleDlg::OnBnClickedBtnStopme()
{
	// 停止Csp运动
	stopSCurve = true;
	Sleep(100);
	closeDataThread = true;
	delta.StopRiseDownMove();
	Sleep(100);
	if (status == SIXDOF_STATUS_RUN)
	{
		if (stopAndMiddle == true)
		{
			delta.MoveToZeroPulseNumber();
			status = SIXDOF_STATUS_READY;
		}
		else
		{
			status = SIXDOF_STATUS_MIDDLE;
		}
	}
	ResetDefaultData(&data);
}

void CECATSampleDlg::OnBnClickedBtnDown()
{	
	if (status == SIXDOF_STATUS_ISFALLING || status == SIXDOF_STATUS_BOTTOM)
	{
		return;
	}
	delta.ReadAllSwitchStatus();
	if (delta.IsAllAtBottom() == true)
	{
		return;
	}	
	status = SIXDOF_STATUS_ISFALLING;
	delta.StopRiseDownMove();
	Sleep(100);
	delta.Down();
}

void CECATSampleDlg::OnBnClickedOk()
{
	CloseThread();
	delta.ServoStop();
	Sleep(100);
	delta.Close(status);
	CDialog::OnOK();
}

void CECATSampleDlg::OnBnClickedBtnConnect()
{
	delta.AllTestUp();
}

void CECATSampleDlg::OnBnClickedBtnResetme()
{
	//delta.ResetStatus();
}

void CECATSampleDlg::OnBnClickedBtnDisconnect()
{
	delta.AllTestDown();
}

void CECATSampleDlg::OnBnClickedBtnSingleUp()
{
	auto index = ((CComboBox*)GetDlgItem(IDC_CBO_SingleNo))->GetCurSel();
	delta.SingleUp(index);
}

void CECATSampleDlg::OnBnClickedBtnSingleDown()
{
	auto index = ((CComboBox*)GetDlgItem(IDC_CBO_SingleNo))->GetCurSel();
	delta.SingleDown(index);
}

void CECATSampleDlg::OnBnClickedButtonTest()
{
	if (status != SIXDOF_STATUS_READY)
	{
		//MessageBox(_T(SIXDOF_NOT_BEGIN_MESSAGE));
		return;
	}
	status = SIXDOF_STATUS_RUN;

	memset(testPhase, 0, sizeof(double) * AXES_COUNT);
	//位移单位mm 角度单位 度
	auto xval = RANGE(GetCEditNumber(IDC_EDIT_X_VAL), -MAX_XYZ_X, MAX_XYZ_X); 
	auto yval = RANGE(GetCEditNumber(IDC_EDIT_Y_VAL), -MAX_XYZ_Y, MAX_XYZ_Y);
	auto zval = RANGE(GetCEditNumber(IDC_EDIT_Z_VAL), -MAX_XYZ_Z, MAX_XYZ_Z);
	auto rollval = RANGE(GetCEditNumber(IDC_EDIT_ROLL_VAL), -MAX_DEG_ROLL, MAX_DEG_ROLL);
	auto pitchval = RANGE(GetCEditNumber(IDC_EDIT_PITCH_VAL), -MAX_DEG_PITCH, MAX_DEG_PITCH);
	auto yawval = RANGE(GetCEditNumber(IDC_EDIT_YAW_VAL), -MAX_DEG_YAW, MAX_DEG_YAW);
	//频率单位1hz
	auto xhz = RANGE(GetCEditNumber(IDC_EDIT_X_HZ), 0, MAX_HZ);
	auto yhz = RANGE(GetCEditNumber(IDC_EDIT_Y_HZ), 0, MAX_HZ);
	auto zhz = RANGE(GetCEditNumber(IDC_EDIT_Z_HZ), 0, MAX_HZ);
	auto rollhz = RANGE(GetCEditNumber(IDC_EDIT_ROLL_HZ), 0, MAX_HZ);
	auto pitchhz = RANGE(GetCEditNumber(IDC_EDIT_PITCH_HZ), 0, MAX_HZ);
	auto yawhz = RANGE(GetCEditNumber(IDC_EDIT_YAW_HZ), 0, MAX_HZ);

	auto xphase = RANGE(GetCEditNumber(IDC_EDIT_X_PHASE), 0, MAX_PHASE);
	auto yphase = RANGE(GetCEditNumber(IDC_EDIT_Y_PHASE), 0, MAX_PHASE);
	auto zphase = RANGE(GetCEditNumber(IDC_EDIT_Z_PHASE), 0, MAX_PHASE);
	auto rollphase = RANGE(GetCEditNumber(IDC_EDIT_ROLL_PHASE), 0, MAX_PHASE);
	auto pitchphase = RANGE(GetCEditNumber(IDC_EDIT_PITCH_PHASE), 0, MAX_PHASE);
	auto yawphase = RANGE(GetCEditNumber(IDC_EDIT_YAW_PHASE), 0, MAX_PHASE);

	auto xzeropos = RANGE(GetCEditNumber(IDC_EDIT_X_ZERO_POS), 0, MAX_XYZ_ZERO_POS);
	auto yzeropos = RANGE(GetCEditNumber(IDC_EDIT_Y_ZERO_POS), 0, MAX_XYZ_ZERO_POS);
	auto zzeropos = RANGE(GetCEditNumber(IDC_EDIT_Z_ZERO_POS), 0, MAX_XYZ_ZERO_POS);
	auto rollzeropos = RANGE(GetCEditNumber(IDC_EDIT_ROLL_ZERO_POS), 0, MAX_DEG_ZERO_POS);
	auto pitchzeropos = RANGE(GetCEditNumber(IDC_EDIT_PITCH_ZERO_POS), 0, MAX_DEG_ZERO_POS);
	auto yawzeropos = RANGE(GetCEditNumber(IDC_EDIT_YAW_ZERO_POS), 0, MAX_DEG_ZERO_POS);

	testVal[0] = xval;
	testVal[1] = yval;
	testVal[2] = zval;
	testVal[3] = rollval;
	testVal[4] = pitchval;
	testVal[5] = yawval;

	testHz[0] = xhz;
	testHz[1] = yhz;
	testHz[2] = zhz;
	testHz[3] = rollhz;
	testHz[4] = pitchhz;
	testHz[5] = yawhz;

	testPhase[0] = xphase;
	testPhase[1] = yphase;
	testPhase[2] = zphase;
	testPhase[3] = rollphase;
	testPhase[4] = pitchphase;
	testPhase[5] = yawphase;

	testPhase[0] = xzeropos;
	testPhase[1] = yzeropos;
	testPhase[2] = zzeropos;
	testPhase[3] = rollzeropos;
	testPhase[4] = pitchzeropos;
	testPhase[5] = yawzeropos;

	if (xphase != 0 || yphase != 0 || zphase != 0 || 
		rollphase != 0 || pitchphase != 0 || yawphase != 0)
	{
		enableChirp = false;
		isCosMode = true;
	}
	else
	{
		enableChirp = ENABLE_CHIRP;
		isCosMode = false;
	}
	stopSCurve = false;

	// 电机先停后启动
	delta.StopRiseDownMove();
	delta.RenewNowPulse();
	delta.GetMotionAveragePulse();
	delta.PidControllerInit();
	delta.UnlockServo();
	// 正弦测试运动模式
	isTest = true;
	// 正弦时间清0
	t = 0;
	// 允许运动
	closeDataThread = false;
}

void CECATSampleDlg::OnBnClickedButtonExit()
{
	OnBnClickedOk();
}

void CECATSampleDlg::OnBnClickedButtonTest3()
{
	// 所有电机上锁停转
	delta.ServoStop();
}

void CECATSampleDlg::OnBnClickedButtonStopTest()
{
	OnBnClickedBtnStopme();
}

