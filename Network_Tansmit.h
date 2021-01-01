#pragma once
#include <stdio.h>  
#include <winsock2.h>
#include <iostream>
#include<mutex>

#pragma comment(lib,"ws2_32.lib")  
using namespace std;



/*
Base:
https://blog.csdn.net/xiaoquantouer/article/details/58001960?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.control&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.control

error code:
https://docs.microsoft.com/en-us/windows/win32/winsock/windows-sockets-error-codes-2

VS2019 无法使用'inet_ntoa': Use inet_ntop() or InetNtop() instead or define

属性->配置属性->C/C++ -> 预处理器 -> 预处理器定义 ->将“_CRT_SECURE_NO_WARNINGS”加上
文件的属性页----->c/c+±----->常规，将SDL检查改为否

use "win+R>cmd > ipconfig/all" to get ip address.
////////////////////////////////////////////////////////////////////////////////////////

version 1.0

version 1.1
add mutex locker


*/






namespace NETWORK
{








	class N_Serial_Server
	{
	public:
		N_Serial_Server(u_short portNum)
		{
			if (WSAStartup(sockVersion, &wsaData) != 0)
			{
				cout << "ERR: port num" << portNum << endl;
				MessageBoxA(NULL, "Server start mistake", "Warning", NULL);
			}
			slisten = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

			if (slisten == INVALID_SOCKET)
			{
				cout << "ERR: port num" << portNum << endl;
				MessageBoxA(NULL, "Server socket mistake", "Warning", NULL);
			}
			sin.sin_family = AF_INET;
			sin.sin_port = htons(portNum);
			sin.sin_addr.S_un.S_addr = INADDR_ANY;
			if (bind(slisten, (LPSOCKADDR)& sin, sizeof(sin)) == SOCKET_ERROR)
			{
				cout << "ERR: port num" << portNum << endl;
				printf("bind error !\n");
				MessageBoxA(NULL, "bind error !", "Warning", NULL);
			}
			if (listen(slisten, 5) == SOCKET_ERROR)
			{
				cout << "ERR: port num" << portNum << endl;
				printf("listen error !");
				MessageBoxA(NULL, "listen error", "Warning", NULL);
			}


		}

		int WaitConnect()
		{
			start:
			printf("等待连接...\n");
			sClient = accept(slisten, (SOCKADDR*)& remoteAddr, &nAddrlen);
			if (sClient == INVALID_SOCKET)
			{
				if(MessageBoxA(NULL, "Can not connent client, want try again?", "Warning", MB_YESNO)== IDYES)goto start;
				MessageBoxA(NULL, "connect failed", "Warning", NULL);
				return -1;
			}
			printf("接受到一个连接：%s \r\n", inet_ntoa(remoteAddr.sin_addr));
			return 1;
		}

		bool receiveData(byte* buffer,int bufferLen,UINT &lenRecv)
		{
			lock_guard<mutex> lck(R_lock);//保护
			int ret = recv(sClient, (char*)buffer, 255, 0);
			if (ret >= 0)
			{
				lenRecv = ret;
				return 1;
			}
			cout << "recive error, code: " <<ret<< endl;
			return 0;

		}
		bool sendData(byte* buffer, int bufferLen)
		{
			lock_guard<mutex> lck(S_lock);//保护
			int code = send(sClient, (const char*)buffer, bufferLen, 0);
			if(code>=0)return 1;
			cout << "send error, code: " <<code<< endl;
		}

		void disconnect()
		{
			closesocket(sClient);
		}

		~N_Serial_Server()
		{
			closesocket(sClient);
			closesocket(slisten);
			WSACleanup();
		}
	private:
		WORD sockVersion = MAKEWORD(2, 2);
		WSADATA wsaData;
		SOCKET slisten = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		sockaddr_in sin;
		
		SOCKET sClient;
		sockaddr_in remoteAddr;
		int nAddrlen = sizeof(remoteAddr);
		
		mutex R_lock;
		mutex S_lock;
	};


	class N_Serial_Client
	{
	public:
		N_Serial_Client(u_short portNum,string Server_addr)
		{

			
			if (WSAStartup(sockVersion, &wsaData) != 0)
			{
				cout << "ERR: port num" << portNum << endl;
				MessageBoxA(NULL, "Client start mistake", "Warning", NULL);
			}

			sClient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
			sin.sin_family = AF_INET;
			sin.sin_port = htons(portNum);
			sin.sin_addr.S_un.S_addr = inet_addr(Server_addr.c_str());




		}

		int try_Connect()
		{
			start:
			if (connect(sClient, (sockaddr*)& sin, sizeof(sin)) == SOCKET_ERROR)
			{  //连接失败 
				if (MessageBoxA(NULL, "Can not connent server, want try again?", "Warning", MB_YESNO) == IDYES)goto start;
				MessageBoxA(NULL, "failed to connect server", "Warning", NULL);
				closesocket(sClient);
				return 0;
			}
			return 1;
		}

		bool receiveData(byte* buffer, int bufferLen, UINT& lenRecv)
		{
			lock_guard<mutex> lck(R_lock);//保护
			int ret = recv(sClient, (char*)buffer, 255, 0);
			if (ret >= 0)
			{
				lenRecv = ret;
				return 1;
			}
			cout << "recive error, code: " << ret << endl;
			return 0;

		}
		bool sendData(byte* buffer, int bufferLen)
		{
			lock_guard<mutex> lck(S_lock);//保护
			int code = send(sClient, (const char*)buffer, bufferLen, 0);
			if (code >= 0)return 1;
			cout << "send error, code: " << code << endl;
		}

		void disconnect()
		{
			closesocket(sClient);
		}

		~N_Serial_Client()
		{
			closesocket(sClient);
			WSACleanup();
		}
	private:
		WORD sockVersion = MAKEWORD(2, 2);
		WSADATA wsaData;

		sockaddr_in sin;

		SOCKET sClient;
		sockaddr_in remoteAddr;
		int nAddrlen = sizeof(remoteAddr);

		mutex R_lock;
		mutex S_lock;
	};




}