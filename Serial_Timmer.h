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
	�˺����������ڿ��ٵķ������ȡ�������ݣ� ��ͬ���ķ�ʽ��������

	*/
public:

	bool IsSerialOpen = 1;//�����Ƿ�ɹ���

	Serial()
	{}

	Serial(string HandName, int SerialSpeed)
	{
		string H_Name = "\\\\.\\";
		string N4 = H_Name + HandName;
		char hanName[20] = { 0 };
		strcpy_s(hanName, N4.c_str());

		STM32 = CreateFileA(hanName,//COM��
			GENERIC_READ | GENERIC_WRITE, //�������д
			0, //��ռ��ʽ
			NULL,
			OPEN_EXISTING, //�򿪶����Ǵ���
			0, //ͬ����ʽ
			NULL);
		if (STM32 == (HANDLE)-1)
		{
			cout << "��COMʧ��!" << endl;
			IsSerialOpen = FALSE;
			//return 0;
		}

		else cout << "��COM�ɹ�!" << endl;

		SetupComm(STM32, 1024, 1024);//���뻺����������������Ĵ�С����1024
		COMMTIMEOUTS TimeOuts;//�趨����ʱ
		TimeOuts.ReadIntervalTimeout = 2;
		TimeOuts.ReadTotalTimeoutMultiplier = 2;
		TimeOuts.ReadTotalTimeoutConstant = 2;//�趨д��ʱ
		TimeOuts.WriteTotalTimeoutMultiplier = 2;
		TimeOuts.WriteTotalTimeoutConstant = 2;
		SetCommTimeouts(STM32, &TimeOuts);//���ó�ʱ
		DCB dcb;
		GetCommState(STM32, &dcb);
		dcb.BaudRate = SerialSpeed;//������
		dcb.ByteSize = 8;//ÿ���ֽ���8λ
		dcb.Parity = NOPARITY;//����żУ��λ
		dcb.StopBits = 0;//����ֹͣλ
		SetCommState(STM32, &dcb);
		PurgeComm(STM32, PURGE_TXCLEAR | PURGE_RXCLEAR);//�����ڻ��棬����������ʱ����
		//return 1;
	}


	bool Serial_Init(string HandName, int SerialSpeed)
	{
		string H_Name = "\\\\.\\";
		string N4 = H_Name + HandName;
		char hanName[20] = { 0 };
		strcpy_s(hanName, N4.c_str());

		STM32 = CreateFileA(hanName,//COM��
			GENERIC_READ | GENERIC_WRITE, //�������д
			0, //��ռ��ʽ
			NULL,
			OPEN_EXISTING, //�򿪶����Ǵ���
			0, //ͬ����ʽ
			NULL);
		if (STM32 == (HANDLE)-1)
		{
			cout << "��COMʧ��!" << endl;
			IsSerialOpen = FALSE;
			return 0;
		}

		else cout << "��COM�ɹ�!" << endl;

		SetupComm(STM32, 1024, 1024);//���뻺����������������Ĵ�С����1024
		COMMTIMEOUTS TimeOuts;//�趨����ʱ
		TimeOuts.ReadIntervalTimeout = 2;
		TimeOuts.ReadTotalTimeoutMultiplier = 2;
		TimeOuts.ReadTotalTimeoutConstant = 2;//�趨д��ʱ
		TimeOuts.WriteTotalTimeoutMultiplier = 2;
		TimeOuts.WriteTotalTimeoutConstant = 2;
		SetCommTimeouts(STM32, &TimeOuts);//���ó�ʱ
		DCB dcb;
		GetCommState(STM32, &dcb);
		dcb.BaudRate = SerialSpeed;//������
		dcb.ByteSize = 8;//ÿ���ֽ���8λ
		dcb.Parity = NOPARITY;//����żУ��λ
		dcb.StopBits = 0;//����ֹͣλ
		SetCommState(STM32, &dcb);
		PurgeComm(STM32, PURGE_TXCLEAR | PURGE_RXCLEAR);
		return 1;
	}

	DWORD wCount = 0;//ʵ��д���ֽ���
	DWORD RCount = 0;//ʵ�ʶ�ȡ�ֽ���

	bool Write(LPCVOID a, DWORD NumWrite)
	{
		bool b = WriteFile(STM32, a, NumWrite, &wCount, NULL);
		//PurgeComm(STM32, PURGE_TXCLEAR);//�����ڻ��棬����������ʱ����
		return b;
	}

	bool Protect_Write(LPCVOID a, DWORD NumWrite)
	{
		lock_guard<mutex> lck(write_L);//���ౣ��
		bool b = WriteFile(STM32, a, NumWrite, &wCount, NULL);
		//PurgeComm(STM32, PURGE_TXCLEAR);//�����ڻ��棬����������ʱ����
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
		lock_guard<mutex> lck(read_L);//���ౣ��
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
		lock_guard<mutex> lck(timmer_locker);//����
		return Time_Start;
	}

public:
	timmer_inS()
	{
		LARGE_INTEGER Storage;
		QueryPerformanceFrequency(&Storage);
		dfFreq = Storage.QuadPart;// ��ü�������ʱ��Ƶ��
	}
	void T_start()//��ʼ��ʱ
	{
		LARGE_INTEGER Storage;
		QueryPerformanceCounter(&Storage);
		lock_guard<mutex> lck(timmer_locker);//����
		Time_Start = Storage.QuadPart;// ��ó�ʼֵ
	}

	void operator=(timmer_inS& T)//����ʱ�ӳ�ʼֵ����Ҫ��ͬһϵͳ�£�
	{
		lock_guard<mutex> lck(timmer_locker);//����
		Time_Start = T.copy();
		return;
	}
	double T_now()//�����ǰʱ������s��
	{
		LARGE_INTEGER L_now;//
		QueryPerformanceCounter(&L_now);//��ȡʱ��
		return (double)(L_now.QuadPart - Time_Start) / dfFreq;// ��ö�Ӧ��ʱ��ֵ����λΪ��

	}





};

