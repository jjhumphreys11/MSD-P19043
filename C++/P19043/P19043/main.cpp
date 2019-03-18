//This code snippet will help you to read data from arduino

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "SerialPort.h"

using std::cout;
using std::endl;

/*Portname must contain these backslashes, and remember to
replace the following com port*/
char temp_port_name[20] = "\\\\.\\COM4";
char *port_name = temp_port_name;

//String for incoming data
char incomingData[MAX_DATA_LENGTH];

int main2()
{
	SerialPort arduino(port_name);
	if (arduino.isConnected()) cout << "Connection Established" << endl;
	else cout << "ERROR, check port name";

	while (arduino.isConnected()) {
		//Check if data has been read or not
		int read_result = arduino.readSerialPort(incomingData, MAX_DATA_LENGTH);
		//prints out data
		puts(incomingData);
		//wait a bit
		Sleep(10);
	}
	return 0;
}