#pragma once


#include<iostream>
#include <windows.h>
#include "Ctime"
#include <mutex>
#include <thread>

using	namespace std;


/*

Edition 1.2

"Timmer" func might make some mistake with "Sleep" func

Edition 1.3

support string to be the parameters, and support the COM>10

Edition 1.4

add clone timmer func to allows timmer runs independently
add individual func to solve the cose of mutex.

Edition 1.5

mutex synchronize has been optimized


Edition 1.6
reduce mutex, mult-thread cost of timmer.
Serial mult-thread read&write protect
*/

class Serial
{
	/*
	此函数仅仅用于快速的发送与读取串口内容， 以同步的方式发送数据

	*/
public:

	bool IsSerialOpen = 1;//串口是否成功打开

	Serial()
	{}

	Serial(string HandName, int SerialSpeed)
	{
		string H_Name = "\\\\.\\";
		string N4 = H_Name + HandName;
		char hanName[20] = { 0 };
		strcpy_s(hanName, N4.c_str());

		STM32 = CreateFileA(hanName,//COM口
			GENERIC_READ | GENERIC_WRITE, //允许读和写
			0, //独占方式
			NULL,
			OPEN_EXISTING, //打开而不是创建
			0, //同步方式
			NULL);
		if (STM32 == (HANDLE)-1)
		{
			cout << "打开COM失败!" << endl;
			IsSerialOpen = FALSE;
			//return 0;
		}

		else cout << "打开COM成功!" << endl;

		SetupComm(STM32, 1024, 1024);//输入缓冲区和输出缓冲区的大小都是1024
		COMMTIMEOUTS TimeOuts;//设定读超时
		TimeOuts.ReadIntervalTimeout = 2;
		TimeOuts.ReadTotalTimeoutMultiplier = 2;
		TimeOuts.ReadTotalTimeoutConstant = 2;//设定写超时
		TimeOuts.WriteTotalTimeoutMultiplier = 2;
		TimeOuts.WriteTotalTimeoutConstant = 2;
		SetCommTimeouts(STM32, &TimeOuts);//设置超时
		DCB dcb;
		GetCommState(STM32, &dcb);
		dcb.BaudRate = SerialSpeed;//波特率
		dcb.ByteSize = 8;//每个字节有8位
		dcb.Parity = NOPARITY;//无奇偶校验位
		dcb.StopBits = 0;//两个停止位
		SetCommState(STM32, &dcb);
		PurgeComm(STM32, PURGE_TXCLEAR | PURGE_RXCLEAR);//清理串口缓存，但是这里暂时不用
		//return 1;
	}


	bool Serial_Init(string HandName, int SerialSpeed)
	{
		string H_Name = "\\\\.\\";
		string N4 = H_Name + HandName;
		char hanName[20] = { 0 };
		strcpy_s(hanName, N4.c_str());

		STM32 = CreateFileA(hanName,//COM口
			GENERIC_READ | GENERIC_WRITE, //允许读和写
			0, //独占方式
			NULL,
			OPEN_EXISTING, //打开而不是创建
			0, //同步方式
			NULL);
		if (STM32 == (HANDLE)-1)
		{
			cout << "打开COM失败!" << endl;
			IsSerialOpen = FALSE;
			return 0;
		}

		else cout << "打开COM成功!" << endl;

		SetupComm(STM32, 1024, 1024);//输入缓冲区和输出缓冲区的大小都是1024
		COMMTIMEOUTS TimeOuts;//设定读超时
		TimeOuts.ReadIntervalTimeout = 2;
		TimeOuts.ReadTotalTimeoutMultiplier = 2;
		TimeOuts.ReadTotalTimeoutConstant = 2;//设定写超时
		TimeOuts.WriteTotalTimeoutMultiplier = 2;
		TimeOuts.WriteTotalTimeoutConstant = 2;
		SetCommTimeouts(STM32, &TimeOuts);//设置超时
		DCB dcb;
		GetCommState(STM32, &dcb);
		dcb.BaudRate = SerialSpeed;//波特率
		dcb.ByteSize = 8;//每个字节有8位
		dcb.Parity = NOPARITY;//无奇偶校验位
		dcb.StopBits = 0;//两个停止位
		SetCommState(STM32, &dcb);
		PurgeComm(STM32, PURGE_TXCLEAR | PURGE_RXCLEAR);
		return 1;
	}

	DWORD wCount = 0;//实际写的字节数
	DWORD RCount = 0;//实际读取字节数

	bool Write(LPCVOID a, DWORD NumWrite)
	{
		bool b = WriteFile(STM32, a, NumWrite, &wCount, NULL);
		//PurgeComm(STM32, PURGE_TXCLEAR);//清理串口缓存，但是这里暂时不用
		return b;
	}

	bool Protect_Write(LPCVOID a, DWORD NumWrite)
	{
		lock_guard<mutex> lck(write_L);//冗余保护
		bool b = WriteFile(STM32, a, NumWrite, &wCount, NULL);
		//PurgeComm(STM32, PURGE_TXCLEAR);//清理串口缓存，但是这里暂时不用
		return b;
	}

	bool Write(LPCVOID a, DWORD NumWrite, DWORD* Count)
	{
		bool b = WriteFile(STM32, a, NumWrite, Count, NULL);
		//PurgeComm(STM32, PURGE_TXCLEAR);
		return b;
	}




	bool Read(LPVOID a, DWORD NumRead)
	{
		bool b = ReadFile(STM32, a, NumRead, &RCount, NULL);
		//PurgeComm(STM32, PURGE_RXCLEAR);
		return b;
	}
	bool Protect_Read(LPVOID a, DWORD NumRead)
	{
		lock_guard<mutex> lck(read_L);//冗余保护
		bool b = ReadFile(STM32, a, NumRead, &RCount, NULL);
		//PurgeComm(STM32, PURGE_RXCLEAR);
		return b;
	}
	bool Read(LPVOID a, DWORD NumRead, DWORD* Count)
	{
		bool b = ReadFile(STM32, a, NumRead, Count, NULL);
		//PurgeComm(STM32, PURGE_RXCLEAR);
		return b;
	}

	bool Clear_Buffer()
	{
		return PurgeComm(STM32, PURGE_TXCLEAR | PURGE_RXCLEAR);
	}
private:
	HANDLE STM32;
	mutex read_L;
	mutex write_L;
};
class timmer_inS
{

private:
	LONGLONG Time_Start = 0;
	LONG dfFreq = 0;
	mutex timmer_locker;

	LONGLONG copy()
	{
		lock_guard<mutex> lck(timmer_locker);//保护
		return Time_Start;
	}

public:
	timmer_inS()
	{
		LARGE_INTEGER Storage;
		QueryPerformanceFrequency(&Storage);
		dfFreq = Storage.QuadPart;// 获得计数器的时钟频率
	}
	void T_start()//开始计时
	{
		LARGE_INTEGER Storage;
		QueryPerformanceCounter(&Storage);
		lock_guard<mutex> lck(timmer_locker);//保护
		Time_Start = Storage.QuadPart;// 获得初始值
	}

	void operator=(timmer_inS& T)//复制时钟初始值（需要在同一系统下）
	{
		lock_guard<mutex> lck(timmer_locker);//保护
		Time_Start = T.copy();
		return;
	}
	double T_now()//输出当前时间间隔（s）
	{
		LARGE_INTEGER L_now;//
		QueryPerformanceCounter(&L_now);//获取时间
		return (double)(L_now.QuadPart - Time_Start) / dfFreq;// 获得对应的时间值，单位为秒

	}





};

