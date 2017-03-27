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

#include "MySocket.h"

static bool m_Init_Flag=false;


//����::��l��Socket����
bool WgSocket::Initialize(void)
{
	if (!m_Init_Flag)
	{
		WSAData wsa_data;
		try
		{
			if (WSAStartup(0x202,&wsa_data) != 0)
				return false;
		}
		catch (...)
		{
			return false;
		}
		m_Init_Flag = true;
	}
	return true;
}

//����:����Socket����
void WgSocket::Terminate(void)
{
	if(m_Init_Flag)
	{
		try
		{
			WSACleanup();
		}
		catch(...)
		{
		}
		m_Init_Flag = false;
	}
}

//����:�غcSocket����
WgSocket::WgSocket(void)
{
	m_Socket = INVALID_SOCKET; 
}

//�����G�ѺcSocket����
WgSocket::~WgSocket()
{
   Close();
}



//�����G�ˬd�O�_��localhost�I�s
bool WgSocket::IsLocalHost(const char* hostname)
{
   if (hostname == NULL) 
	   return true;
   if (*hostname == 0) 
	   return true;
   if (stricmp(hostname,"localhost") == 0) 
	   return true;
   if (strcmp(hostname,"127.0.0.1") == 0) 
	   return true;
   return false;
}


//�����G�˴�Socket�O�_�w�}��
bool WgSocket::IsOpened(void) const
{
   if (m_Socket == INVALID_SOCKET) 
	   return false;
   return true;
}


//�����G�}�һPServer���s�u
bool WgSocket::Open(const char* hostname, int port)
{
   Close();
   if (!Initialize()) 
	   return false;

   struct sockaddr_in sock_addr;

   // �ѥXsocket address //
   if (IsLocalHost(hostname)) 
	   hostname = "127.0.0.1";
   sock_addr.sin_family = AF_INET;
   sock_addr.sin_port = htons(port);
   struct hostent *hostinfo = gethostbyname(hostname);
   if (hostinfo == NULL) 
	   return false;
   sock_addr.sin_addr = *(struct in_addr *) hostinfo->h_addr;
   // �إ�socket //
   try
   {
     m_Socket = socket(AF_INET,SOCK_STREAM,0);
   }
   catch(...)
   {
     m_Socket = INVALID_SOCKET;
     return false;
   }

   if (m_Socket == INVALID_SOCKET) 
	   return false;
   // �}�l�s�u //
   try
   {
     if (connect(m_Socket,(struct sockaddr*)&sock_addr,sizeof(sock_addr)) >= 0) 
		 return true;
   }
   catch(...)
   {

   }

   // ���B�i�H�[�J�@�ǿ��~�B�z... //
   Close();
   return false;
}


//�����G�����PServer���s�u
void WgSocket::Close(void)
{
   if (!IsOpened()) 
	   return;
   try
   {
     shutdown(m_Socket,SD_SEND);
   }
   catch(...)
   {

   }
   try
   {
     closesocket(m_Socket);
   }
   catch(...)
   {
   }
   m_Socket = INVALID_SOCKET;
}


bool WgSocket::Listen(int port)
{
	Close();
	if (!Initialize())
		return false;
	struct sockaddr_in sock_addr;
	sock_addr.sin_family = AF_INET;
	sock_addr.sin_addr.s_addr = INADDR_ANY;
	sock_addr.sin_port = htons(port);

	//�إ�Socket
	try
	{
		m_Socket = socket(AF_INET,SOCK_STREAM,0);
	}
	catch(...)
	{
		m_Socket = INVALID_SOCKET;
		return false;
	}
	//Bind Socket
	int on = 1;
	int rc;
	setsockopt(m_Socket,SOL_SOCKET,SO_REUSEADDR,(char*)&on,sizeof(on));
	try
	{
		rc = bind(m_Socket,(struct sockaddr*)&sock_addr,sizeof(sock_addr));
	}
	catch(...)
	{
		rc = SOCKET_ERROR;
		Close();
		return false;
	}

	//Listen Socket
	try
	{
		rc = listen(m_Socket,SOMAXCONN);
	}
	catch(...)
	{
		rc = SOCKET_ERROR;
		Close();
		return false;
	}
	return true;
}

//����:���ݱ����s�u
bool WgSocket::Accept(SOCKET &socket)
{
	socket = INVALID_SOCKET;
	if (!IsOpened())
		return false;
	struct sockaddr_in from;
	int fromlen = sizeof(from);
	try
	{
		socket = accept(m_Socket,(struct sockaddr*)&from,&fromlen);
	}
	catch(...)
	{
		socket = INVALID_SOCKET;
		return false;
	}
	return true;
}

//����:�]�w�s�u��Socket
void WgSocket::SetSocket(SOCKET socket)
{
	Close();
	m_Socket = socket;
}


//����:���ݹ��e�Ӹ��
bool WgSocket::WaitInputData(int seconds)
{
	if(!IsOpened())
		return false;

	//�]�wdescriptor sets
	fd_set socket_set;
	FD_ZERO(&socket_set);
	FD_SET((unsigned int)m_Socket,&socket_set);

	//�]�wTimeOut�ɶ�
	struct timeval timeout;
	timeout.tv_sec = seconds;
	timeout.tv_usec = 0;


	//�����O�_�����
	try
	{
		if(select(FD_SETSIZE,&socket_set,NULL,NULL,&timeout) <= 0)
			return false;
	}
	catch(...)
	{
		return false;
	}
	return true;
}


//����:Ū�����
//��J:data, len = ��ƽw�İϤj�p
//��X:data = Ū�������, ret_len = ���Ū������Ƥj�p, 0��ܹ��H�_�u
//����Ʒ|�@�����즳Ū����Ʃε����s�u�ɤ~�Ǧ^
bool WgSocket::Read(SOCKET m_Socket ,void *data, wglong len, wglong &ret_len)
{
	ret_len = 0;
	if(!IsOpened())
		return false;
	try
	{
		ret_len = recv(m_Socket,(char*)data,len,0);
	}
	catch(...)
	{
		ret_len = SOCKET_ERROR;
	}

	if (ret_len < 0)
	{
		ret_len = 0;
		return false;
	}
	return true;
}



//����:�ǰe���
//��J:data,len = ��ƽw�İϤj�p
bool WgSocket::Write(SOCKET m_Socket, const void *data, wglong len)
{
	if(!IsOpened())
		return false;
	if(len <= 0)
		return true;
	int write_len;
	try
	{
		write_len = send(m_Socket,(const char*)data,len,0);
	}
	catch(...)
	{
		write_len = SOCKET_ERROR;
	}
	if(write_len != len)
		return false;
	return true;
}



//�����G���o���whost��ip
//��J�Ghostname = host��}
//��X�Gip1-4 = ip��}
//�Ǧ^�G���ѶǦ^false
bool WgSocket::GetHostIP(const char* hostname, int &ip1, int &ip2, int &ip3, int &ip4)
{
	if (IsLocalHost(hostname))
	{
		// �����X��ڪ�hostname //
		struct hostent *hostinfo = gethostbyname("localhost");
		if (hostinfo == NULL) 
			return false;
		hostname = hostinfo->h_name;
	}
	struct hostent* hostinfo = gethostbyname(hostname);
	if (hostinfo == NULL) 
		return false;
	char* addr = hostinfo->h_addr_list[0];
	ip1 = (unsigned char) addr[0];
	ip2 = (unsigned char) addr[1];
	ip3 = (unsigned char) addr[2];
	ip4 = (unsigned char) addr[3];
	return true;
}

//�����G�]�w������ǰe (����Nagle Algorithm)
//�Ǧ^�G�]�w���ѶǦ^false
bool WgSocket::SetNoDelay(void)
{
   if (!IsOpened()) 
	   return false;
   int on = 1;
   if (setsockopt(m_Socket,IPPROTO_TCP,TCP_NODELAY,(char*)&on,sizeof(on)) != 0) 
	   return false;
   return true;
}

//�����G���oClient��IP
//�Ǧ^�G�]�w���ѶǦ^false
void WgSocket::GetClientIP(SOCKET ClientSocket, string &_ClientIP)
{ 
	struct sockaddr_in peeraddr;
	int iaddr = sizeof(peeraddr);
	//���oClient��T
	getpeername(ClientSocket,(struct sockaddr*)&peeraddr,&iaddr);
	//���oClient IP
	_ClientIP = inet_ntoa(peeraddr.sin_addr);
}

SOCKET WgSocket::getMySocket()
{
	return m_Socket;
}