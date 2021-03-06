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
HWND AddCustomPlotButton;
HWND DefaultPlotsButton;
HWND AddSubWindow;

HWND PatientID;
HWND PatientAge;
HWND PatientHasTremors;

HWND newComPort;
HWND newBaudRate;

HWND ChoosePlotBox;

HWND CustomPlotWindow2;
HWND DefaultPlotWindow2;


struct Patient {
	TCHAR name[150];
	int age;
	bool hasTremors;
};

Patient allPatients[50];
int numPatients = 0;

bool stopped = true;
HANDLE recordingData;


int currentTime = 0;
bool recording = false;
bool connected = false;
int baudRate = 115200;
ofstream outputFile;

char temp_port_name[20] = "\\\\.\\COM4";
char *port_name = temp_port_name;

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
	_In_opt_ HINSTANCE hPrevInstance,
	_In_ LPWSTR    lpCmdLine,
	_In_ int       nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	// TODO: Place code here.

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
		20, 50, 240, 245, nullptr, nullptr, hInstance, nullptr); // last number is 360

	ConnectToDeviceButton = CreateWindow(WC_BUTTON, TEXT("Connect to Device"),
		WS_VISIBLE | WS_CHILD,
		10, 230, 200, 25, MainWindow, (HMENU)CONNECT_TO_DEVICE_BUTTON, NULL, NULL);

	StartRecordingButton = CreateWindow(WC_BUTTON, TEXT("Start Recording"),
		WS_VISIBLE | WS_CHILD,
		10, 50, 200, 25, MainWindow, (HMENU)START_RECORDING_BUTTON, NULL, NULL);

	StopRecordingButton = CreateWindow(WC_BUTTON, TEXT("Stop Recording"),
		WS_VISIBLE | WS_CHILD,
		10, 80, 200, 25, MainWindow, (HMENU)STOP_RECORDING_BUTTON, NULL, NULL);

	TitleStatic = CreateWindow(L"STATIC", TEXT("Essential Tremor Data Acquisition"),
		WS_VISIBLE | WS_CHILD,
		2, 10, 220, 25, MainWindow, (HMENU)TITLE_STATIC, NULL, NULL);

	StatusStatic = CreateWindow(L"STATIC", TEXT(""),
		WS_VISIBLE | WS_CHILD,
		10, 150, 200, 25, MainWindow, (HMENU)STATUS_STATIC, NULL, NULL);

	TimeStatic = CreateWindow(L"STATIC", TEXT("Time:"),
		WS_VISIBLE | WS_CHILD,
		10, 190, 50, 25, MainWindow, (HMENU)NULL, NULL, NULL);
	
	TimerStatic = CreateWindow(L"STATIC", TEXT("0"),
		WS_VISIBLE | WS_CHILD,
		60, 190, 150, 25, MainWindow, (HMENU)TIMER_STATIC, NULL, NULL);
	
	DefaultPlotsButton = CreateWindow(WC_BUTTON, TEXT("Real-Time Plots"),
		WS_VISIBLE | WS_CHILD,
		10, 110, 200, 25, MainWindow, (HMENU)DEFAULT_PLOTS_BUTTON, NULL, NULL);

	AddCustomPlotButton = CreateWindow(WC_BUTTON, TEXT("Add Custom Plot"),
		WS_VISIBLE | WS_CHILD,
		10, 260, 200, 25, MainWindow, (HMENU)ADD_CUSTOM_PLOT_BUTTON, NULL, NULL);

	FillPatientsList();

	// Custom Plot Window Windows
	{
		CustomPlotWindow2 = CreateWindowW(szWindowClass, TEXT("Add Custom Plot"), WS_OVERLAPPEDWINDOW,
			500, 410, 350, 370, nullptr, nullptr, hInstance, nullptr);

		ChoosePlotBox = CreateWindow(WC_COMBOBOX, TEXT(""),
			WS_VISIBLE | WS_CHILD | CBS_DROPDOWNLIST | CBS_HASSTRINGS,
			10, 10, 200, 300, CustomPlotWindow2, (HMENU)CUSTOM_PLOT_COMBOBOX, NULL, NULL);
		FillComboBox();

		HWND CustomPlotOK = CreateWindow(WC_BUTTON, TEXT("OK"),
			WS_VISIBLE | WS_CHILD,
			220, 260, 100, 25, CustomPlotWindow2, (HMENU)CUSTOM_PLOT_OK, NULL, NULL);

		HWND CustomPlotCancel = CreateWindow(WC_BUTTON, TEXT("Cancel"),
			WS_VISIBLE | WS_CHILD,
			100, 260, 100, 25, CustomPlotWindow2, (HMENU)CUSTOM_PLOT_CANCEL, NULL, NULL);

		HWND AddAnotherPlot = CreateWindow(WC_BUTTON, TEXT("Add Another Plot"),
			WS_VISIBLE | WS_CHILD,
			220, 230, 100, 25, CustomPlotWindow2, (HMENU)ADD_ANOTHER_PLOT, NULL, NULL);
	}

	// Default Plot Window Windows
	{
		DefaultPlotWindow2 = CreateWindowW(szWindowClass, TEXT("Default Plots"), WS_OVERLAPPEDWINDOW,
			750, 50, 700, 700, nullptr, nullptr, hInstance, nullptr);

		HWND DefaultPlotText = CreateWindow(L"STATIC", TEXT("Will Hold All Seperate Plots."),
			WS_VISIBLE | WS_CHILD | SS_CENTER,
			250, 20, 200, 25, DefaultPlotWindow2, (HMENU)NULL, NULL, NULL);
	}
	

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
	TCHAR tempName[100] = TEXT("");
	int selectedPlots[16];
	int numPlots = 0;

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
			break;
			SetWindowText(StatusStatic, TEXT(""));
			SerialPort arduino(port_name);
			// in isConnected(), figure out some way to check if it's the arduino
			if (arduino.isConnected())
			{
				SetWindowText(StatusStatic, TEXT("Connection Made."));
				if (false)
				{
					TCHAR curr_path[MAX_DATA_LENGTH];
					GetThisPath(curr_path, MAX_DATA_LENGTH);
					for (int y = 0; y < MAX_DATA_LENGTH; y++)
					{
						if (curr_path[y] == '.')
						{
							curr_path[y + 3] = '\0';
							curr_path[y + 2] = '\0';
							curr_path[y + 3] = '\0';
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
									TCHAR file_name[MAX_DATA_LENGTH] = TEXT("kst_1/kst_2/bin/temp_data.csv");
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
					outputFile.open(curr_path, std::ios_base::app);
					outputFile << "Time,IMU1_xt,IMU1_yt,IMU1_zt,IMU1_xr,IMU1_yr,IMU1_zr,";
					outputFile << "IMU2_xt,IMU2_yt,IMU2_zt,IMU2_xr,IMU2_yr,IMU2_zr,";
					outputFile << "EMG1,EMG2,EMG3,EMG4\n";
					outputFile.close();
				}
				Sleep(500);
			}
			else
			{
				SetWindowText(StatusStatic, TEXT("ERROR, check port name"));
			}
			break;
		}
		case START_RECORDING_BUTTON:
		{
			// creates seperate thread to run the "Data Recording"
			// allows for GUI to still be used while data is being collected
			if (recording && !stopped)
			{
				SetWindowText(StatusStatic, TEXT("Already Recording"));
				break;
			}
			SetWindowText(StatusStatic, TEXT("Started"));
			recording = true;
			DWORD myThreadID;
			stopped = false;
			recordingData = CreateThread(0, 0, RecordData, NULL, 0, &myThreadID);

			break;
		}
		case STOP_RECORDING_BUTTON:
		{
			// closes the "data recording" thread
			if (recording && !stopped)
			{
				currentTime = 0;
				stopped = true;
				recording = false;
				Sleep(100);
				CloseHandle(recordingData);
				SetWindowText(StatusStatic, TEXT("Stopped"));
			}
			else
				SetWindowText(StatusStatic, TEXT("Already Stopped"));
			break;
		}
		case DEFAULT_PLOTS_BUTTON:
		{
			string str = "\".\\kst_1\\kst_2\\bin\\kst2.exe\" \".\\kst_1\\kst_2\\bin\\kst_default.kst\"";
			const char *command = str.c_str();
			if (recording && !stopped)
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
		case DEFAULT_PLOT_CANCEL:
		{
			ShowWindow(DefaultPlotWindow2, HIDE_WINDOW);
			return (INT_PTR)TRUE;
		}
		case NEW_PATIENT:
		{
			DialogBox(hInst, MAKEINTRESOURCE(NEW_PATIENT_WINDOW), hWnd, NewPatientWindow);
			break;
		}
		case ADD_CUSTOM_PLOT_BUTTON:
		{
			ShowWindow(CustomPlotWindow2, 10);
			UpdateWindow(CustomPlotWindow2);
			break;
		}
		case ADD_ANOTHER_PLOT:
		{
			selectedPlots[numPlots++] = SendMessage(ChoosePlotBox, CB_GETCURSEL, (WPARAM)0, (LPARAM)0);

			SendMessage(ChoosePlotBox, CB_SETCURSEL, (WPARAM)0, (LPARAM)0);
			break;
		}
		case CUSTOM_PLOT_OK:
		{
			// create new plot in another window
			// will use the plotting software that Sean is working with

			numPlots = 0;
			//break;
		}
		case CUSTOM_PLOT_CANCEL:
		{
			ShowWindow(CustomPlotWindow2, HIDE_WINDOW);
			return (INT_PTR)TRUE;
		}
		case SET_COM_PORT:
		{
			DialogBox(hInst, MAKEINTRESOURCE(NEW_COM_PORT), hWnd, SetComPort);
			break;
		}
		case SET_BAUD_RATE:
		{
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
			POINT curCurs;
			int xpos = 0;
			int ypos = 0;
			int addx = 0;
			int addy = 0;

			double currx = 0;
			double curry = 0;

			char output[MAX_DATA_LENGTH];
			char output2[MAX_DATA_LENGTH];
			char c_string = 'C';
			TCHAR fromArduino[200] = TEXT("");

			SerialPort arduino(port_name);
			if (!arduino.isConnected())
			{
				SetWindowText(StatusStatic, TEXT("ERROR, check port name"));
				break;
			}
			arduino.writeSerialPort(&c_string, MAX_DATA_LENGTH);
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
			DestroyWindow(hWnd);
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
			90, 13, 100, 200, hDlg, (HMENU)NEW_COM_PORT_EDIT, NULL, NULL);
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
				break;
			case 1:
				temp_port_name[7] = '2';
				break;
			case 2:
				temp_port_name[7] = '3';
				break;
			case 3:
				temp_port_name[7] = '4';
				break;
			case 4:
				temp_port_name[7] = '5';
				break;
			case 5:
				temp_port_name[7] = '6';
				break;
			case 6:
				temp_port_name[7] = '7';
				break;
			case 7:
				temp_port_name[7] = '8';
				break;
			case 8:
				temp_port_name[7] = '9';
				break;
			case 9:
				temp_port_name[7] = '10\0';
				break;
			case 10:
				temp_port_name[7] = '11\0';
				break;
			case 11:
				temp_port_name[7] = '12\0';
				break;
			case 12:
				temp_port_name[7] = '13\0';
				break;
			case 13:
				temp_port_name[7] = '14\0';
				break;
			case 14:
				temp_port_name[7] = '15\0';
				break;
			case 15:
				temp_port_name[7] = '16\0';
				break;
			case 16:
				temp_port_name[7] = '17\0';
				break;
			case 17:
				temp_port_name[7] = '18\0';
				break;
			default:
				temp_port_name[7] = '4';
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
			90, 75, 25, 25, hDlg, (HMENU)PATIENT_HAS_TREMORS, NULL, NULL);

		return (INT_PTR)TRUE;
	}
	case WM_COMMAND:
	{
		switch (LOWORD(wParam))
		{
		case IDOK:
		{
			TCHAR tempName[10];
			TCHAR age[3];
			int age2;
			bool tempHasTremors;
			GetWindowText(PatientID, tempName, 10);
			GetWindowText(PatientAge, age, 3);

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
				tempHasTremors = true;
			}
			else
			{
				tempHasTremors = false;
			}

			for (int t = 0; t < 50; t++)
			{
				allPatients[0].name[t] = tempName[t];
			}
			allPatients[numPatients].age = age2;
			allPatients[numPatients].hasTremors = tempHasTremors;
			numPatients++;

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
	char output[MAX_DATA_LENGTH];
	char c_string = 'R';
	char end_string = 'D';

	SerialPort arduino(port_name);

	TCHAR curr_path[MAX_DATA_LENGTH];
	char curr_path2[MAX_DATA_LENGTH];
	TCHAR file_name[MAX_DATA_LENGTH] = TEXT("kst_1/kst_2/bin/temp_data.csv");
	char new_file_name[MAX_DATA_LENGTH];
	ofstream outputFile;
	if (!arduino.isConnected())
	{
		SetWindowText(StatusStatic, TEXT("ERROR, check port name"));
		return NULL;
	}
	else
	{
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
			if (new_file_name[q] == '.')
			{
				new_file_name[q++] = '_';
				new_file_name[q++] = 'o';
				new_file_name[q++] = 'l';
				new_file_name[q++] = 'd';
				new_file_name[q++] = '.';
				new_file_name[q++] = 'c';
				new_file_name[q++] = 's';
				new_file_name[q++] = 'v';
				new_file_name[q++] = '\0';
				break;
			}
		}
	}
	//remove(curr_path2);
	Sleep(200);
	if (is_file_exist(curr_path2))
	{
		if (is_file_exist(new_file_name))
		{
			remove(new_file_name);
		}
		rename(curr_path2, new_file_name);
		Sleep(100);
	}
	
	outputFile.open(curr_path, std::ios_base::app);
	outputFile << "Time,IMU1_xt,IMU1_yt,IMU1_zt,IMU1_xr,IMU1_yr,IMU1_zr,";
	outputFile << "IMU2_xt,IMU2_yt,IMU2_zt,IMU2_xr,IMU2_yr,IMU2_zr,";
	outputFile << "EMG1,EMG2,EMG3,EMG4\n";

	outputFile.close();
	Sleep(200);

	arduino.writeSerialPort(&end_string, MAX_DATA_LENGTH);
	Sleep(10);

	if (!arduino.writeSerialPort(&c_string, MAX_DATA_LENGTH))
	{
		SetWindowText(StatusStatic, TEXT("Write Failed"));
		return NULL;
	}
	
	SetWindowText(StatusStatic, TEXT("Recording Data..."));
	Sleep(10);

	while (arduino.isConnected() && !stopped)
	{
		output[0] = '\0';
		arduino.readSerialPort(output, MAX_DATA_LENGTH);
			
		if (output[0] == '\0')
		{
			continue;
		}
	
		//save all values to temp_data.csv file
		outputFile.open(curr_path, std::ios_base::app);
		outputFile << output;
		outputFile.close();
	}
	Sleep(100);
	arduino.readSerialPort(output, MAX_DATA_LENGTH);
	Sleep(10);
	arduino.writeSerialPort(&end_string, MAX_DATA_LENGTH);
	Sleep(10);
	SetWindowText(StatusStatic, TEXT("Stopped Recording"));
	recording = false;
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

