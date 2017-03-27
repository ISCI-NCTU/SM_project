/**
 * Copyright 2017 National Chiao Tung University, Intelligent System and Control Integration Laboratory
 * Author: Cheng-Hei Wu 
 * Maintainer : Howard Chen 
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <winsock2.h>
#include <string>
using namespace std;
typedef long wglong;


class WgSocket
{
	private:
		SOCKET m_Socket;
	public:
		static bool Initialize(void);
		static void Terminate(void);
		static bool IsLocalHost(const char* hostname);
		static bool GetHostIP(const char* hostname, int &ip1, int &ip2, int &ip3, int &ip4);
		WgSocket(void);
		~WgSocket();
		void GetClientIP(SOCKET, string&);
		bool IsOpened(void) const;
		void SetSocket(SOCKET socket);
		bool Open(const char* hostname, int port);
		void Close(void);
		bool WaitInputData(int seconds);
		bool Read(SOCKET m_Socket, void* buffer, wglong len, wglong &ret_len);
		bool Write(SOCKET m_Socket, const void* buffer, wglong len);
		bool Listen(int port);
		bool Accept(SOCKET &socket);
		bool SetNoDelay(void);
		SOCKET getMySocket();
		
};