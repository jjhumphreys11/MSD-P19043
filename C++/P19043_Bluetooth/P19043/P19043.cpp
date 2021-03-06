// P19043.cpp : Defines the entry point for the application.
//

#pragma comment(lib, "comct132.lib")
#pragma comment(lib, "comdlg32.lib")

#include "stdafx.h"
#include "P19043.h"
#include <windows.h>
#include "commctrl.h"
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <atlstr.h>
#include <cstring>
#include <cstdlib>
#include <memory>
#include <Commdlg.h>
#include <cmath>
#include "SerialPort.h"

using namespace std;

#define MAX_LOADSTRING 100

// Global Variables:
HINSTANCE hInst;                                // current instance
WCHAR szTitle[MAX_LOADSTRING];                  // The title bar text
WCHAR szWindowClass[MAX_LOADSTRING];            // the main window class name

// Forward declarations of functions included in this code module:
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	NewPatientWindow(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam);
INT_PTR CALLBACK	SetComPort(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam);
INT_PTR CALLBACK	SetBaudRate(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam);
void				FillComboBox();
bool				FillPatientsList();
bool				SavePatientsList();
DWORD WINAPI		RecordData(LPVOID lpParam);
TCHAR*				GetThisPath(TCHAR* dest, size_t destSize);
bool				is_file_exist(const char *fileName);

HWND MainWindow;
HWND ConnectToDeviceButton;
HWND StartRecordingButton;
HWND StopRecordingButton;

HWND TitleStatic;
HWND GraphSelectionListView;
HWND StatusStatic;
HWND TimeStatic;
HWND TimerStatic;

HWND HandStatic;
HWND WristStatic;
HWND EMGStatic;

HWND AddCustomPlotsWindow;
HWND NoEMGButton;
HWND DefaultPlotsButton;
HWND AddSubWindow;
HWND onlyOneIMUButton;

HWND PatientID;
HWND PatientAge;
HWND PatientHasTremors;
HWND PatientPlaysSport;
HWND PatientJob;
HWND PatientGender;
HWND PatientHand;
HWND PatientSport;
HWND PatientSportDur;
HWND PatientSportStatic;
HWND PatientSportDurStatic;
HWND PatientDiagnosed;
HWND PatientEssTremor;
HWND PatientEssTremorStatic;
HWND PatientSeverity;
HWND PatientSeverityStatic;

HWND newComPort;
HWND newBaudRate;

HWND ChoosePlotBox;


struct Patient {
	TCHAR name[150];
	int age;
	bool hasTremors;
	TCHAR job[100];
	int gender = 2;// 0:male, 1:female, 2:other
	int handed = 2;// 0:right, 1:left, 2:either/other

	bool tremDiag;// they have been diagnosed with a movement disorder
	bool essTrem;// diagnosed with essential tremor
	int magTrem; // magnitude of tremor,  0:minor, 1:significant, 2:excessive

	bool inSport;// sport participation
	TCHAR sport[50]; // type of sport
	int durSport; // duration of participation (years)
};

Patient CurrPatient;

HANDLE recordingData;

int currentTime = 0;
bool recording = false;
bool connected = false;

char output[MAX_DATA_LENGTH];
char output_r[MAX_DATA_LENGTH];
char output_c[5];

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
int baudRate = 115200;

char temp_port_name[20] = "\\\\.\\COM4";
char *port_name = temp_port_name;

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
	_In_opt_ HINSTANCE hPrevInstance,
	_In_ LPWSTR    lpCmdLine,
	_In_ int       nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	// Initialize global strings
	LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadStringW(hInstance, IDC_P19043, szWindowClass, MAX_LOADSTRING);
	MyRegisterClass(hInstance);

	// Perform application initialization:
	if (!InitInstance(hInstance, nCmdShow))
	{
		return FALSE;
	}

	HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_P19043));

	MSG msg;

	// Main message loop:
	while (GetMessage(&msg, nullptr, 0, 0))
	{
		if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}
	return (int)msg.wParam;
}

//
//  FUNCTION: MyRegisterClass()
//
//  PURPOSE: Registers the window class.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
	WNDCLASSEXW wcex;

	wcex.cbSize = sizeof(WNDCLASSEX);

	wcex.style = CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc = WndProc;
	wcex.cbClsExtra = 0;
	wcex.cbWndExtra = 0;
	wcex.hInstance = hInstance;
	wcex.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_P19043));
	wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
	wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
	wcex.lpszMenuName = MAKEINTRESOURCEW(IDC_P19043);
	wcex.lpszClassName = szWindowClass;
	wcex.hIconSm = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

	return RegisterClassExW(&wcex);
}

//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
	hInst = hInstance; // Store instance handle in our global variable

	MainWindow = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
		20, 50, 240, 285, nullptr, nullptr, hInstance, nullptr); // last number is 360

	TitleStatic = CreateWindow(L"STATIC", TEXT("Essential Tremor Data Acquisition"),
		WS_VISIBLE | WS_CHILD,
		2, 10, 220, 25, MainWindow, (HMENU)TITLE_STATIC, NULL, NULL);

	ConnectToDeviceButton = CreateWindow(WC_BUTTON, TEXT("Check Connection"),
		WS_VISIBLE | WS_CHILD,
		10, 50, 200, 25, MainWindow, (HMENU)CONNECT_TO_DEVICE_BUTTON, NULL, NULL);

	StartRecordingButton = CreateWindow(WC_BUTTON, TEXT("Start Recording"),
		WS_VISIBLE | WS_CHILD,
		10, 90, 200, 25, MainWindow, (HMENU)START_RECORDING_BUTTON, NULL, NULL);

	StopRecordingButton = CreateWindow(WC_BUTTON, TEXT("Stop Recording"),
		WS_VISIBLE | WS_CHILD,
		10, 120, 200, 25, MainWindow, (HMENU)STOP_RECORDING_BUTTON, NULL, NULL);

	DefaultPlotsButton = CreateWindow(WC_BUTTON, TEXT("Real-Time Plots"),
		WS_VISIBLE | WS_CHILD,
		10, 150, 200, 25, MainWindow, (HMENU)DEFAULT_PLOTS_BUTTON, NULL, NULL);

	StatusStatic = CreateWindow(L"STATIC", TEXT(""),
		WS_VISIBLE | WS_CHILD,
		10, 190, 200, 25, MainWindow, (HMENU)STATUS_STATIC, NULL, NULL);

	FillPatientsList();

	if (!MainWindow)
	{
		return FALSE;
	}

	ShowWindow(MainWindow, nCmdShow);
	UpdateWindow(MainWindow);

	return TRUE;
}

//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE:  Processes messages for the main window.
//
//  WM_COMMAND  - process the application menu
//  WM_PAINT    - Paint the main window
//  WM_DESTROY  - post a quit message and return
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	//TCHAR tempName[100] = TEXT("");
	//int selectedPlots[16];
	//int numPlots = 0;

	switch (message)
	{
	case WM_COMMAND:
	{
		int wmId = LOWORD(wParam);
		// Parse the menu selections:
		switch (wmId)
		{
		case CONNECT_TO_DEVICE_BUTTON:
		{
			char test_string = 'C';
			output_c[0] = '\0';
			SetWindowText(StatusStatic, TEXT("Attempting to Connect"));

			// create new thread with timer
			// after 3 seconds without connection, connection fails

			SerialPort arduino(port_name, baudRate);
			if (arduino.isConnected())
			{
				if (!arduino.writeSerialPort(&test_string, MAX_DATA_LENGTH))
				{
					SetWindowText(StatusStatic, TEXT("Connection Write Failed"));
					connected = false;
					return NULL;
				}
				else
				{
					int trys = 0;
					while (trys < 10)
					{
						Sleep(100);
						arduino.readSerialPort(output_c, MAX_DATA_LENGTH);
						trys++;

						if (output_c[0] == 'S')
						{
							SetWindowText(StatusStatic, TEXT("Connected to Device"));
							connected = true;
							break;
						}
						else
						{
							if (trys == 10)
							{
								SetWindowText(StatusStatic, TEXT("Connection Read Failed"));
								connected = false;
								break;
							}
						}
					}
					break;
				}
			}
			else
			{
				SetWindowText(StatusStatic, TEXT("Error: Check COM Port"));
				connected = false;
			}
			break;
		}
		case START_RECORDING_BUTTON:
		{
			// creates seperate thread to run the "Data Recording"
			// allows for GUI to still be used while data is being collected
			if (recording)
			{
				SetWindowText(StatusStatic, TEXT("Already Started"));
				break;
			}

			SetWindowText(StatusStatic, TEXT(""));
			DWORD myThreadID;
			//recording = true;
			recordingData = CreateThread(0, 0, RecordData, NULL, 0, &myThreadID);

			break;
		}
		case STOP_RECORDING_BUTTON:
		{
			// closes the "data recording" thread
			if (recording)
			{
				currentTime = 0;
				recording = false;
				recordingData = false;
				Sleep(100);
				CloseHandle(recordingData);
			}
			else
				SetWindowText(StatusStatic, TEXT("Already Stopped"));
			break;
		}
		case DEFAULT_PLOTS_BUTTON:
		{
			string str = "\"..\\Debug\\kst_1\\kst_2\\bin\\kst2.exe\" \"..\\Debug\\kst_1\\kst_2\\bin\\kst_default.kst\"";
			const char *command = str.c_str();
			if (recording)
			{
				// Opens real-time plotting
				WinExec(command, 1);
				SetActiveWindow(MainWindow);
			}
			else
			{
				//opens plots with no data
				SetWindowText(StatusStatic, TEXT("No Real-Time Data"));
				WinExec(command, 1);
				SetActiveWindow(MainWindow);
			}
			break;
			
		}
		case DEFAULT_PLOT_OK:
		{
			// i don't know if we need an OK button here

			break;
		}
		case NEW_PATIENT:
		{
			DialogBox(hInst, MAKEINTRESOURCE(NEW_PATIENT_WINDOW), hWnd, NewPatientWindow);
			break;
		}
		case ADD_ANOTHER_PLOT:
		{
			break;
			//selectedPlots[numPlots++] = SendMessage(ChoosePlotBox, CB_GETCURSEL, (WPARAM)0, (LPARAM)0);

			SendMessage(ChoosePlotBox, CB_SETCURSEL, (WPARAM)0, (LPARAM)0);
			break;
		}
		case SET_COM_PORT:
		{
			DialogBox(hInst, MAKEINTRESOURCE(NEW_COM_PORT), hWnd, SetComPort);
			break;
		}
		case SET_BAUD_RATE:
		{
			SetWindowText(StatusStatic, TEXT("Access Denied"));
			break;
			DialogBox(hInst, MAKEINTRESOURCE(NEW_BAUD_RATE), hWnd, SetBaudRate);
			break;
		}
		case IDM_ABOUT:
		{
			DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
			break;
		}	
		case SAVE_DATA:
		{

			break;
		}
		case LOAD_DATA:
		{
			
			break;
		}
		case CURSOR_MODE:
		{
			break;
			POINT curCurs;
			int xpos = 0;
			int ypos = 0;
			int addx = 0;
			int addy = 0;



			double currx = 0;
			double curry = 0;
			output[0] = '\0';
			char output2[MAX_DATA_LENGTH];
			char c_string = 'C';
			TCHAR fromArduino[200] = TEXT("");

			SerialPort arduino(port_name, baudRate);
			if (!arduino.isConnected())
			{
				SetWindowText(StatusStatic, TEXT("Error: Check COM Port"));
				break;
			}
			//arduino_out.writeSerialPort(&c_string, MAX_DATA_LENGTH);
			Sleep(10);
			while (arduino.isConnected()) // check if stop button pressed
			{
				output[0] = '\0';
				arduino.readSerialPort(output, MAX_DATA_LENGTH);
				if (output[0] == '\0')	continue;

				currx = roundf(atof(output)*100)/100.0;
				curry = roundf(atof(output2)*100)/100.0;

				addx = -currx * 1;
				addy = -curry * 1;
				if (abs(addx) < 2) addx = 0;
				if (abs(addy) < 2) addy = 0;


				GetCursorPos(&curCurs);
				xpos = curCurs.x + addx;
				ypos = curCurs.y + addy;
				SetCursorPos(xpos, ypos);

				Sleep(10);
			}
			break;
		}
		case GAME_MODE:
		{

			break;
		}
		case IDM_EXIT:
		{
			if (hWnd == MainWindow)
			{
				DestroyWindow(hWnd);
			}
			else
			{
				ShowWindow(hWnd, HIDE_WINDOW);
			}
			break;
		}
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
		break;
	}
	case WM_TIMER:
	{
		currentTime++;
		stringstream strs;
		strs << currentTime;
		string temp_str = strs.str();
		char* char_type = (char*)temp_str.c_str();
		TCHAR temp2[10];
		for (int t = 0; t < 5; t++)
		{
			temp2[t] = *(char_type + t);
		}
		SetWindowText(TimerStatic, temp2);
		break;
	}
	case WM_PAINT:
	{
		PAINTSTRUCT ps;
		HDC hdc = BeginPaint(hWnd, &ps);
		// TODO: Add any drawing code that uses hdc here...
		EndPaint(hWnd, &ps);
	}
	break;
	case WM_DESTROY:
	{
		if (hWnd == MainWindow)
		{
			SavePatientsList();
			//DestroyWindow(hWnd);
			PostQuitMessage(0);
		}
		else
		{
			ShowWindow(hWnd, HIDE_WINDOW);
		}
		break;
	}
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}

// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}

INT_PTR CALLBACK SetComPort(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
	{
		newComPort = CreateWindow(WC_COMBOBOX, TEXT(""),
			WS_VISIBLE | WS_CHILD | CBS_DROPDOWNLIST | CBS_HASSTRINGS,
			90, 13, 100, 350, hDlg, (HMENU)NEW_COM_PORT_EDIT, NULL, NULL);
		SendMessage(newComPort, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("COM1")));
		SendMessage(newComPort, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("COM2")));
		SendMessage(newComPort, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("COM3")));
		SendMessage(newComPort, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("COM4")));
		SendMessage(newComPort, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("COM5")));
		SendMessage(newComPort, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("COM6")));
		SendMessage(newComPort, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("COM7")));
		SendMessage(newComPort, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("COM8")));
		SendMessage(newComPort, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("COM9")));
		SendMessage(newComPort, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("COM10")));
		SendMessage(newComPort, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("COM11")));
		SendMessage(newComPort, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("COM12")));
		SendMessage(newComPort, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("COM13")));
		SendMessage(newComPort, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("COM14")));
		SendMessage(newComPort, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("COM15")));
		SendMessage(newComPort, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("COM16")));
		SendMessage(newComPort, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("COM17")));
		SendMessage(newComPort, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("COM18")));

		return (INT_PTR)TRUE;
	}
	case WM_COMMAND:
	{
		switch (LOWORD(wParam))
		{
		case IDOK:
		{
			int selected = SendMessage(newComPort, CB_GETCURSEL, 0, 0);
			switch (selected)
			{
			case 0:
				temp_port_name[7] = '1';
				temp_port_name[8] = '\0';
				temp_port_name[9] = '\0';
				break;
			case 1:
				temp_port_name[7] = '2';
				temp_port_name[8] = '\0';
				temp_port_name[9] = '\0';
				break;
			case 2:
				temp_port_name[7] = '3';
				temp_port_name[8] = '\0';
				temp_port_name[9] = '\0';
				break;
			case 3:
				temp_port_name[7] = '4';
				temp_port_name[8] = '\0';
				temp_port_name[9] = '\0';
				break;
			case 4:
				temp_port_name[7] = '5';
				temp_port_name[8] = '\0';
				temp_port_name[9] = '\0';
				break;
			case 5:
				temp_port_name[7] = '6';
				temp_port_name[8] = '\0';
				temp_port_name[9] = '\0';
				break;
			case 6:
				temp_port_name[7] = '7';
				temp_port_name[8] = '\0';
				temp_port_name[9] = '\0';
				break;
			case 7:
				temp_port_name[7] = '8';
				temp_port_name[8] = '\0';
				temp_port_name[9] = '\0';
				break;
			case 8:
				temp_port_name[7] = '9';
				temp_port_name[8] = '\0';
				temp_port_name[9] = '\0';
				break;
			case 9:
				temp_port_name[7] = '1';
				temp_port_name[8] = '0';
				temp_port_name[9] = '\0';
				break;
			case 10:
				temp_port_name[7] = '1';
				temp_port_name[8] = '1';
				temp_port_name[9] = '\0';
				break;
			case 11:
				temp_port_name[7] = '1';
				temp_port_name[8] = '2';
				temp_port_name[9] = '\0';
				break;
			case 12:
				temp_port_name[7] = '1';
				temp_port_name[8] = '3';
				temp_port_name[9] = '\0';
				break;
			case 13:
				temp_port_name[7] = '1';
				temp_port_name[8] = '4';
				temp_port_name[9] = '\0';
				break;
			case 14:
				temp_port_name[7] = '1';
				temp_port_name[8] = '5';
				temp_port_name[9] = '\0';
				break;
			case 15:
				temp_port_name[7] = '1';
				temp_port_name[8] = '6';
				temp_port_name[9] = '\0';
				break;
			case 16:
				temp_port_name[7] = '1';
				temp_port_name[8] = '7';
				temp_port_name[9] = '\0';
				break;
			case 17:
				temp_port_name[7] = '1';
				temp_port_name[8] = '8';
				temp_port_name[9] = '\0';
				break;
			default:
				temp_port_name[7] = '8';
			}

			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		case IDCANCEL:
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		}
	}
	}
	return (INT_PTR)FALSE;
}

INT_PTR CALLBACK SetBaudRate(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
	{
		newBaudRate = CreateWindow(WC_EDIT, TEXT(""),
			WS_VISIBLE | WS_CHILD | WS_BORDER,
			90, 13, 100, 25, hDlg, (HMENU)NULL, NULL, NULL);

		return (INT_PTR)TRUE;
	}
	case WM_COMMAND:
	{
		switch (LOWORD(wParam))
		{
		case IDOK:
		{
			TCHAR tempBaudRate[10];
			int tempNew = 0;
			GetWindowText(newBaudRate, tempBaudRate, 10);
			if (tempBaudRate != TEXT(""))
			{
				int fullLen = sizeof(tempBaudRate) / sizeof(*tempBaudRate);
				int len = 0;
				for (int r = 0; r < fullLen; r++)
				{
					if (tempBaudRate[r] == '\0')
					{
						break;
					}
					len++;
				}
				len--;
				int pow1 = 0;
				for (int t = 0; t < len; t++)
				{
					pow1 = pow(10, len-t);
					tempNew = tempNew + (tempBaudRate[t] - '0')*pow1;
				}
				baudRate = tempNew;
			}

			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		case IDCANCEL:
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		}
	}
	}
	return (INT_PTR)FALSE;
}

INT_PTR CALLBACK NewPatientWindow(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
	{
		PatientID = CreateWindow(WC_EDIT, TEXT(""),
			WS_VISIBLE | WS_CHILD | WS_BORDER,
			90, 13, 100, 25, hDlg, (HMENU)PATIENT_ID_EDIT, NULL, NULL);

		PatientAge = CreateWindow(WC_EDIT, TEXT(""),
			WS_VISIBLE | WS_CHILD | WS_BORDER,
			90, 46, 100, 25, hDlg, (HMENU)PATIENT_AGE_EDIT, NULL, NULL);

		PatientHasTremors = CreateWindow(WC_BUTTON, TEXT(""),
			WS_VISIBLE | WS_CHILD | BS_CHECKBOX,
			100, 149, 25, 25, hDlg, (HMENU)PATIENT_HAS_TREMORS, NULL, NULL);

		PatientPlaysSport = CreateWindow(WC_BUTTON, TEXT(""),
			WS_VISIBLE | WS_CHILD | BS_CHECKBOX,
			100, 215, 25, 25, hDlg, (HMENU)PATIENT_PLAYS_SPORT, NULL, NULL);
		PatientSport = CreateWindow(WC_EDIT, TEXT(""),
			WS_CHILD | WS_BORDER,
			110, 245, 150, 25, hDlg, (HMENU)PATIENT_SPORT_TYPE, NULL, NULL);
		PatientSportStatic = CreateWindow(WC_STATIC, TEXT("Sport?"),
			WS_CHILD,
			25, 248, 80, 25, hDlg, (HMENU)NULL, NULL, NULL);
		PatientSportDur = CreateWindow(WC_EDIT, TEXT(""),
			WS_CHILD | WS_BORDER,
			110, 280, 100, 25, hDlg, (HMENU)PATIENT_SPORT_DUR, NULL, NULL);
		PatientSportDurStatic = CreateWindow(WC_STATIC, TEXT("Duration?"),
			WS_CHILD,
			25, 283, 80, 25, hDlg, (HMENU)NULL, NULL, NULL);

		PatientDiagnosed = CreateWindow(WC_BUTTON, TEXT(""),
			WS_VISIBLE | WS_CHILD | BS_CHECKBOX,
			124, 312, 25, 25, hDlg, (HMENU)PATIENT_DIAGNOSED, NULL, NULL);
		PatientEssTremor = CreateWindow(WC_BUTTON, TEXT(""),
			WS_CHILD | BS_CHECKBOX,
			120, 343, 25, 25, hDlg, (HMENU)PATIENT_ESS_TREM, NULL, NULL);
		PatientEssTremorStatic = CreateWindow(WC_STATIC, TEXT("Essential Tremor?"),
			WS_CHILD,
			25, 340, 80, 30, hDlg, (HMENU)NULL, NULL, NULL);
		PatientSeverity = CreateWindow(WC_COMBOBOX, TEXT(""),
			WS_CHILD | CBS_DROPDOWNLIST | CBS_HASSTRINGS,
			120, 381, 130, 100, hDlg, (HMENU)PATIENT_SEVERITY, NULL, NULL);
		SendMessage(PatientSeverity, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("Minor")));
		SendMessage(PatientSeverity, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("Significant")));
		SendMessage(PatientSeverity, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("Excessive")));
		PatientSeverityStatic = CreateWindow(WC_STATIC, TEXT("Severity?"),
			WS_CHILD,
			25, 380, 80, 25, hDlg, (HMENU)NULL, NULL, NULL);

		PatientJob = CreateWindow(WC_EDIT, TEXT(""),
			WS_VISIBLE | WS_CHILD | WS_BORDER,
			90, 114, 150, 25, hDlg, (HMENU)PATIENT_JOB_EDIT, NULL, NULL);

		PatientGender = CreateWindow(WC_COMBOBOX, TEXT(""),
			WS_VISIBLE | WS_CHILD | CBS_DROPDOWNLIST | CBS_HASSTRINGS,
			90, 80, 150, 100, hDlg, (HMENU)NEW_COM_PORT_EDIT, NULL, NULL);
		SendMessage(PatientGender, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("Male")));
		SendMessage(PatientGender, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("Female")));
		SendMessage(PatientGender, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("Other")));

		PatientHand = CreateWindow(WC_COMBOBOX, TEXT(""),
			WS_VISIBLE | WS_CHILD | CBS_DROPDOWNLIST | CBS_HASSTRINGS,
			100, 182, 140, 100, hDlg, (HMENU)NEW_COM_PORT_EDIT, NULL, NULL);
		SendMessage(PatientHand, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("Right")));
		SendMessage(PatientHand, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("Left")));
		SendMessage(PatientHand, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("Neither/Other")));


		return (INT_PTR)TRUE;
	}
	case WM_COMMAND:
	{
		switch (LOWORD(wParam))
		{
		case IDOK:
		{
			TCHAR tempName[20];
			TCHAR tempJob[100];
			TCHAR tempSport[100];
			TCHAR tempSportDur[10];
			TCHAR age[3];
			int age2;
			GetWindowText(PatientID, tempName, 20);
			GetWindowText(PatientAge, age, 3);
			GetWindowText(PatientAge, age, 3);
			GetWindowText(PatientSport, age, 100);

			if (age[2] == '\0')
			{
				if (age[1] == '\0')
				{
					age2 = (age[0] - '0');
				}
				else
				{
					age2 = (age[0] - '0') * 10 + (age[1] - '0');
				}
			}
			else
			{
				age2 = (age[0] - '0') * 100 + (age[1] - '0') * 10 + (age[2] - '0');
			}

			if (SendMessage(PatientHasTremors, BM_GETCHECK, 0, 0) == BST_CHECKED)
			{
				CurrPatient.hasTremors = true;
			}
			else
			{
				CurrPatient.hasTremors = false;
			}

			for (int t = 0; t < 50; t++)
			{
				CurrPatient.name[t] = tempName[t];
			}
			for (int t = 0; t < 100; t++)
			{
				CurrPatient.job[t] = tempJob[t];
			}
			CurrPatient.age = age2;

			int selected = SendMessage(PatientGender, CB_GETCURSEL, 0, 0);
			if(selected >= 0) CurrPatient.gender = selected;

			selected = SendMessage(PatientHand, CB_GETCURSEL, 0, 0);
			if (selected >= 0) CurrPatient.handed = selected;

			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		case PATIENT_HAS_TREMORS:
		{
			if (SendMessage(PatientHasTremors, BM_GETCHECK, 0, 0) == BST_UNCHECKED)
			{
				SendMessage(PatientHasTremors, BM_SETCHECK, (WPARAM)BST_CHECKED, 0);
			}
			else
			{
				SendMessage(PatientHasTremors, BM_SETCHECK, (WPARAM)BST_UNCHECKED, 0);
			}
			break;
		}
		case PATIENT_PLAYS_SPORT:
		{
			if (SendMessage(PatientPlaysSport, BM_GETCHECK, 0, 0) == BST_UNCHECKED)
			{
				SendMessage(PatientPlaysSport, BM_SETCHECK, (WPARAM)BST_CHECKED, 0);
				ShowWindow(PatientSport, SW_SHOWNORMAL);
				ShowWindow(PatientSportDur, SW_SHOWNORMAL);
				ShowWindow(PatientSportStatic, SW_SHOWNORMAL);
				ShowWindow(PatientSportDurStatic, SW_SHOWNORMAL);

			}
			else
			{
				SendMessage(PatientPlaysSport, BM_SETCHECK, (WPARAM)BST_UNCHECKED, 0);
				SetWindowText(PatientSport, TEXT(""));
				SetWindowText(PatientSportDur, TEXT(""));
				ShowWindow(PatientSport, SW_HIDE);
				ShowWindow(PatientSportDur, SW_HIDE);
				ShowWindow(PatientSportStatic, SW_HIDE);
				ShowWindow(PatientSportDurStatic, SW_HIDE);
			}
			break;
		}
		case PATIENT_DIAGNOSED:
		{
			if (SendMessage(PatientDiagnosed, BM_GETCHECK, 0, 0) == BST_UNCHECKED)
			{
				SendMessage(PatientDiagnosed, BM_SETCHECK, (WPARAM)BST_CHECKED, 0);
				ShowWindow(PatientEssTremor, SW_SHOWNORMAL);
				ShowWindow(PatientEssTremorStatic, SW_SHOWNORMAL);
				ShowWindow(PatientSeverity, SW_SHOWNORMAL);
				ShowWindow(PatientSeverityStatic, SW_SHOWNORMAL);

			}
			else
			{
				SendMessage(PatientDiagnosed, BM_SETCHECK, (WPARAM)BST_UNCHECKED, 0);
				SendMessage(PatientEssTremor, BM_SETCHECK, (WPARAM)BST_UNCHECKED, 0);
				SendMessage(PatientSeverity, CB_SETCURSEL, (WPARAM)-1, 0);
				
				ShowWindow(PatientEssTremor, SW_HIDE);
				ShowWindow(PatientEssTremorStatic, SW_HIDE);
				ShowWindow(PatientSeverity, SW_HIDE);
				ShowWindow(PatientSeverityStatic, SW_HIDE);
			}
			break;
		}
		case PATIENT_ESS_TREM:
		{
			if (SendMessage(PatientEssTremor, BM_GETCHECK, 0, 0) == BST_UNCHECKED)
			{
				SendMessage(PatientEssTremor, BM_SETCHECK, (WPARAM)BST_CHECKED, 0);
			}
			else
			{
				SendMessage(PatientEssTremor, BM_SETCHECK, (WPARAM)BST_UNCHECKED, 0);
			}
			break;
		}
		case IDCANCEL:
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		}
	}
	}
	return (INT_PTR)FALSE;
}

// fills the Graph Selection ComboBox with the different possible graphs
void FillComboBox()
{
	SendMessage(ChoosePlotBox, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("Hand Acceleration (X)")));
	SendMessage(ChoosePlotBox, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("Hand Acceleration (Y)")));
	SendMessage(ChoosePlotBox, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("Hand Acceleration (Z)")));
	SendMessage(ChoosePlotBox, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("Wrist Acceleration (X)")));
	SendMessage(ChoosePlotBox, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("Wrist Acceleration (Y)")));
	SendMessage(ChoosePlotBox, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("Wrist Acceleration (Z)")));
	SendMessage(ChoosePlotBox, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("Hand Rotation (X)")));
	SendMessage(ChoosePlotBox, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("Hand Rotation (Y)")));
	SendMessage(ChoosePlotBox, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("Hand Rotation (Z)")));
	SendMessage(ChoosePlotBox, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("Wrist Rotation (X)")));
	SendMessage(ChoosePlotBox, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("Wrist Rotation (Y)")));
	SendMessage(ChoosePlotBox, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("Wrist Rotation (Z)")));
	SendMessage(ChoosePlotBox, (UINT)CB_ADDSTRING, (WPARAM)0, (LPARAM)(TEXT("All EMG Signals")));

	// Send the CB_SETCURSEL message to display an initial item 
	//  in the selection field  
	SendMessage(ChoosePlotBox, CB_SETCURSEL, (WPARAM)0, (LPARAM)0);
}

bool FillPatientsList()
{
	//open a .csv file or something and read in the 

	return TRUE;
}

bool is_file_exist(const char *fileName)
{
	std::ifstream infile(fileName);
	return infile.good();
}

bool SavePatientsList()
{
	// save all patient info at end

	return TRUE;
}

// reads in data and saves it directly to a output .csv file
DWORD WINAPI RecordData(LPVOID lpParam)
{
	SetWindowText(StatusStatic, TEXT("Checking Connection..."));
	char start_string = 'R';
	char end_string = 'D';
	char test_string = 'C';

	SerialPort arduino(port_name, baudRate);

	TCHAR curr_path[MAX_DATA_LENGTH];
	char curr_path2[MAX_DATA_LENGTH];
	TCHAR file_name[MAX_DATA_LENGTH] = TEXT("kst_1/kst_2/bin/temp_data.csv");
	char new_file_name[MAX_DATA_LENGTH];
	char final_save[MAX_DATA_LENGTH];
	ofstream outputFile;
	output_r[0] = '\0';

	// Check if the arduino is connected
	//
	// connected = false:
	//				display a message and exit the function
	// connected = true:
	//				create the temp_data.csv file used to save data
	if (!arduino.isConnected())
	{
		SetWindowText(StatusStatic, TEXT("Error: Check COM Port"));
		recording = false;
		connected = false;
		return 0;
	}
	else
	{
		if (!arduino.writeSerialPort(&test_string, MAX_DATA_LENGTH))
		{
			SetWindowText(StatusStatic, TEXT("Connection Failed"));
			connected = false;
			return NULL;
		}
		else
		{
			int trys = 0;
			while (trys < 10)
			{
				Sleep(100);
				arduino.readSerialPort(output_c, MAX_DATA_LENGTH);
				trys++;

				if (output_c[0] == 'S')
				{
					SetWindowText(StatusStatic, TEXT("Connected to Device"));
					connected = true;
					Sleep(500);
					break;
				}
				else
				{
					if (trys == 10)
					{
						SetWindowText(StatusStatic, TEXT("Connection Read Failed"));
						connected = false;
						return NULL;
					}
				}
			}
		}
		GetThisPath(curr_path, MAX_DATA_LENGTH);
		for (int y = 0; y < MAX_DATA_LENGTH; y++)
		{
			if (curr_path[y] == '.')
			{
				curr_path[y + 3] = '\0';
				curr_path[y + 2] = '\0';
				curr_path[y + 1] = '\0';
				curr_path[y] = '\0';
				for (int u = 0; u < 20; u++)
				{
					if (curr_path[y - u] != '/')
					{
						curr_path[y - u] = '\0';
					}
					else
					{
						y = y - u + 1;
						for (int w = 0; w < 30; w++)
						{
							curr_path[y + w] = file_name[w];
							if (file_name[w] == '\0')
							{
								y = 9999999;
								break;
							}
						}
						break;
					}
				}

			}
			if (y > MAX_DATA_LENGTH)
			{
				break;
			}
			if (curr_path[y] == '\\')
			{
				curr_path[y] = '/';
			}
		}
		for (int q = 0; q < MAX_DATA_LENGTH; q++)
		{

			curr_path2[q] = curr_path[q];
			if (curr_path2[q] == '\0')
			{
				break;
			}
		}
		for (int q = 0; q < MAX_DATA_LENGTH; q++)
		{

			new_file_name[q] = curr_path[q];
			final_save[q] = curr_path[q];
			if (new_file_name[q] == '.')
			{
				new_file_name[q] = '_';
				final_save[q++] = '_';
				new_file_name[q] = 'o';
				final_save[q++] = 's';
				new_file_name[q] = 'l';
				final_save[q++] = 'a';
				new_file_name[q] = 'd';
				final_save[q++] = 'v';
				new_file_name[q] = '.';
				final_save[q++] = 'e';
				new_file_name[q] = 'c';
				final_save[q++] = '.';
				new_file_name[q] = 's';
				final_save[q++] = 'c';
				new_file_name[q] = 'v';
				final_save[q++] = 's';
				new_file_name[q] = '\0';
				final_save[q++] = 'v';
				final_save[q++] = '\0';
				break;
			}
		}
	}

	// manage the temp_data.csv and a temp_data_old.csv file
	{
		Sleep(200);
		if (is_file_exist(curr_path2))// if temp_data.csv exists...
		{
			if (is_file_exist(new_file_name))// if temp_data_old.csv exists...
			{
				remove(new_file_name);// remove temp_data_old.csv
			}
			rename(curr_path2, new_file_name);// rename temp_data to temp_data_old
			Sleep(100);
		}

		// create the temp_data.csv file and populate it
		outputFile.open(curr_path, std::ios_base::app);
		outputFile << "Time,IMU1_xt,IMU1_yt,IMU1_zt,IMU1_xr,IMU1_yr,IMU1_zr,";
		outputFile << "IMU2_xt,IMU2_yt,IMU2_zt,IMU2_xr,IMU2_yr,IMU2_zr,";
		outputFile << "EMG1,EMG2,EMG3,EMG4\n";

//		double negTime = -15.0;
//		while (negTime < -0.005)
//		{
//			outputFile << negTime;
//			outputFile << ",0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
//			outputFile << "\n";
//			negTime = negTime + 0.01;
//		}
		outputFile.close();
		Sleep(200);
	}

	recording = true;
	connected = true;
	if (!arduino.writeSerialPort(&start_string, MAX_DATA_LENGTH))
	{
		SetWindowText(StatusStatic, TEXT("Connection to Device Failed"));
		recording = false;
		connected = false;
		return NULL;
	}

	SetWindowText(StatusStatic, TEXT("Recording Data..."));

	//Sleep(1);
	int failed = 0;
	char out_check = '\0';
	while (out_check != '\n')
	{
		arduino.readSerialPort(&out_check, 1);
	}

	while (arduino.isConnected() && recording)
	{
		output_r[0] = '\0';
		arduino.readSerialPort(output_r, MAX_DATA_LENGTH);
			
		if (output_r[0] == '\0')
		{
			if (++failed > 10000)
			{
				break;
			}
			continue;
		}
	
		//save all values to temp_data.csv file
		failed = 0;
		outputFile.open(curr_path, std::ios_base::app);
		outputFile << output_r;
		outputFile.close();
	}
	Sleep(100);
	arduino.readSerialPort(output_r, MAX_DATA_LENGTH);
	Sleep(10);
	arduino.writeSerialPort(&end_string, MAX_DATA_LENGTH);
	Sleep(10);
	SetWindowText(StatusStatic, TEXT("Stopped Recording"));
	recording = false;

	if (is_file_exist(final_save))
	{
		remove(final_save);
	}
	rename(curr_path2, final_save);
	CopyFile((LPCWSTR)curr_path2, (LPCWSTR)final_save, false);

	// if temp_data.csv does NOT exist, it is created with headers
	if (!is_file_exist(curr_path2))
	{
		outputFile.open(curr_path, std::ios_base::app);
		outputFile << "Time,IMU1_xt,IMU1_yt,IMU1_zt,IMU1_xr,IMU1_yr,IMU1_zr,";
		outputFile << "IMU2_xt,IMU2_yt,IMU2_zt,IMU2_xr,IMU2_yr,IMU2_zr,";
		outputFile << "EMG1,EMG2,EMG3,EMG4\n";
		outputFile.close();
	}
	
	return NULL;
}

TCHAR* GetThisPath(TCHAR* dest, size_t destSize)
{
	if (!dest) return NULL;

	DWORD length = GetModuleFileName(NULL, dest, destSize);
	if (MAX_PATH > destSize) return NULL;
	PathRemoveFileSpec(dest);
	return dest;
}

