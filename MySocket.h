
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