/*******************************************************************************

  Copyright (c) by Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include "HoloNetworkConnection.h"

#include <Rcs_macros.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_timer.h>

#include <thread>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/types.h>

#ifdef _WIN32
#pragma comment(lib, "Ws2_32.lib")
#include <iphlpapi.h>
#include <WinSock2.h>
#include <WS2tcpip.h>
typedef SSIZE_T ssize_t;
#else
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#endif


#define MAXPACKETSIZE (4096)



HoloNetworkConnection::HoloNetworkConnection() :
  executeThreads(false), broadcastSocket(-1), receiveSocket(-1),
  messagesSent(0), desiredSendFrequency(50.0)
{

}

void HoloNetworkConnection::setExecuteThreads(bool value)
{
  executeThreads = value;
}

// creates and initializes a socket for UDP broadcasting
socklen_t HoloNetworkConnection::createSocket(unsigned int port)
{

#ifdef _WIN32
  char broadcast = 1;
  char reuseport = 1;

  // must init WinSock before using sockets on Windows
  WSADATA wsaData = { 0 };
  int res = WSAStartup(MAKEWORD(2, 2), &wsaData);
  if (res != 0)
  {
    RFATAL(0, "Could not init WinSock! Networking will fail!");
  }

  SOCKET s;
#else
  int s;
  int broadcast = 1;
  int reuseport = 1;
#endif

  // create socket
  if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    RWARNING(0, "Socket creation failed: %s", strerror(errno));
  }

  if (setsockopt(s, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) != 0)
  {
    RWARNING(0, "Setting broadcast flag on socket failed: %s", strerror(errno));
  }
  if (setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &reuseport, sizeof(reuseport)) != 0)
  {
    RWARNING(0, "Setting reuseaddr flag on socket failed: %s", strerror(errno));
  }

  return s;
}

bool HoloNetworkConnection::startServer(int port, std::string outgoingInterface)
{
  this->broadcastSocket = createSocket(port);
  this->broadcastInterface = std::move(outgoingInterface);

  RLOG(0, "Server setup succeeded - port is %d", port);
  setExecuteThreads(true);
  std::thread t(&HoloNetworkConnection::startServerThreadFunc, this, port);
  t.detach();

  return true;
}

bool HoloNetworkConnection::startClient(int port)
{
  this->receiveSocket = createSocket(port);
  struct sockaddr_in si_me;
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(port);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(this->receiveSocket, (sockaddr*)&si_me, sizeof(si_me)) == -1)
  {
    RFATAL("Could not bind receiving socket!");
  }

  RLOG(0, "Client setup succeeded - port is %d", port);
  setExecuteThreads(true);

  // start listening thread
  std::thread t(&HoloNetworkConnection::startClientThreadFunc, this);
  t.detach();

  return true;
}

void HoloNetworkConnection::stopServer()
{
  setExecuteThreads(false);
}

bool HoloNetworkConnection::sendMessage(const std::string& msg)
{
  this->sendMtx.lock();
  this->sendQueue.push(msg);
  this->sendMtx.unlock();

  return true;
}

bool HoloNetworkConnection::send(const Hologram& hologram)
{
  this->sendMtx.lock();
  this->sendQueue.push(hologram);
  this->sendMtx.unlock();

  return true;
}

bool HoloNetworkConnection::send(const std::string& name, const std::string& parent, const HTr* transform)
{
  this->sendMtx.lock();
  this->sendQueue.push(updateTransformMsg(name, parent, transform));
  this->sendMtx.unlock();

  return true;
}

size_t HoloNetworkConnection::getMessagesSent() const
{
  return this->messagesSent;
}


void HoloNetworkConnection::setSendFrequency(double frequency)
{
  settingsMtx.lock();
  this->desiredSendFrequency = frequency;
  settingsMtx.unlock();
}

double HoloNetworkConnection::getDesiredSendFrequency() const
{
  double freq;

  settingsMtx.lock();
  freq = this->desiredSendFrequency;
  settingsMtx.unlock();

  return freq;
}

size_t HoloNetworkConnection::getSendQueueSize() const
{
  size_t queueSize;

  this->sendMtx.lock();
  queueSize = sendQueue.size();
  this->sendMtx.unlock();

  return queueSize;
}

size_t HoloNetworkConnection::getReceiveQueueSize() const
{
  size_t queueSize;

  this->recvMtx.lock();
  queueSize = recvQueue.size();
  this->recvMtx.unlock();

  return queueSize;
}


std::string HoloNetworkConnection::getMessage()
{
  std::string msg;

  this->recvMtx.lock();
  if (!recvQueue.empty())
  {
    msg = recvQueue.front();
    recvQueue.pop();
  }
  this->recvMtx.unlock();

  return msg;
}

/*******************************************************************************
 * Thread functions
 ******************************************************************************/
void HoloNetworkConnection::startServerThreadFunc(unsigned int port)
{
  struct sockaddr_in si_other;

#ifdef _WIN32
  PIP_ADAPTER_ADDRESSES pAddresses = NULL;
  ULONG outBufLen = 0;

  auto res = GetAdaptersAddresses(AF_INET, GAA_FLAG_INCLUDE_PREFIX, NULL, pAddresses, &outBufLen);
  if (res == ERROR_BUFFER_OVERFLOW)
  {
    free(pAddresses);
    pAddresses = (IP_ADAPTER_ADDRESSES*)malloc(outBufLen);
    res = GetAdaptersAddresses(AF_INET, GAA_FLAG_INCLUDE_PREFIX, NULL, pAddresses, &outBufLen);
  }

  if (res != NO_ERROR)
  {
    RWARNING(0, "Could not enumerate network interfaces. Will only broadcast on the default interface.");
    pAddresses = NULL;
  }
#else
  struct ifaddrs* ifap;
  if (getifaddrs(&ifap) != 0)
  {
    RWARNING(0, "Could not enumerate network interfaces! Will only broadcast on the default interface.");
  }
#endif

  si_other.sin_family = AF_INET;
  si_other.sin_port = htons(port);
  si_other.sin_addr.s_addr = htonl(INADDR_BROADCAST);

  while (executeThreads)
  {
    ssize_t sent = 0;

    while (!this->sendMtx.try_lock())
    {
      continue;
    }

    //    this->sendMtx.lock();
    while (!sendQueue.empty())
    {
      std::string msg = sendQueue.front();
      sendQueue.pop();
      if (msg.length() > 0)
      {
        // only broadcast on default interface
#ifdef _WIN32
        if (pAddresses == NULL)
#else
        if (ifap == nullptr)
#endif
        {
          if ((sent = sendto(this->broadcastSocket, msg.c_str(), msg.length(), 0, (struct sockaddr*) &si_other,
                             sizeof(si_other))) < 0)
          {
            RWARNING(0, "Failed to send message: '%s'", msg.c_str());
          }
          else
          {
            messagesSent++;
          }
        }
        // scan ALL interfaces for our desired interface and broadcast on that
        else
        {
#ifdef _WIN32
          PIP_ADAPTER_ADDRESSES currAddress = pAddresses;
          while (currAddress != NULL)
          {
            RLOG_CPP(6, "Sending '" << msg << "' on interface: " << currAddress->Description);
            si_other.sin_addr = ((sockaddr_in*)(currAddress->FirstUnicastAddress->Address.lpSockaddr))->sin_addr;
            if ((sent = sendto(this->broadcastSocket, msg.c_str(), msg.length(), 0, (struct sockaddr*)&si_other, sizeof(si_other))) < 0)
            {
              RLOG_CPP(1, "Failed to send message: '" << msg << "' on interface " << currAddress->Description);
            }
            else
            {
              //RLOG_CPP(0, "SUCCESS to send message: '" << msg << "' on interface " << currAddress->AdapterName);
            }

            currAddress = currAddress->Next;
          }
#else
          struct ifaddrs* iface = ifap;
          while (iface)
          {
            if (SockAddrToUint32(iface->ifa_addr) > 0)
            {
              // restrict sending to ONLY the specified broadcast interface (to avoid spamming other networks without listening devices)
              if (strcmp(iface->ifa_name, broadcastInterface.c_str()) == 0)
              {
                RLOG(6, "Sending '%s' on interface: %s", msg.c_str(), iface->ifa_name);
                si_other.sin_addr = ((sockaddr_in*)(iface->ifa_dstaddr))->sin_addr;
                if ((sent = sendto(this->broadcastSocket, msg.c_str(), msg.length(), 0, (struct sockaddr*) &si_other,
                                   sizeof(si_other))) < 0)
                {
                  RWARNING(0, "Failed to send message: '%s' with %zu bytes on interface %s: %s", msg.c_str(), msg.length(), iface->ifa_name, strerror(errno));
                }
                break; // we hit our target interface, no need to continue searching others
              }
            }
            iface = iface->ifa_next;
          }
#endif
        }
      }

      RLOG(6, "%zd messages remain in queue", sendQueue.size());
      if (!sendQueue.empty())
      {
        std::this_thread::sleep_for(std::chrono::milliseconds((long)(1000.0 / this->desiredSendFrequency)));
      }
    }
    this->sendMtx.unlock();
  }

  /// TODO: Is this the right place for this?
#ifdef _WIN32
  if (WSACleanup() != 0)
  {
    RLOG(1, "Failed to cleanly shut down WinSock :(");
  }
#endif

  RLOG(0, "Quitting network re-starting thread");
}

void HoloNetworkConnection::startClientThreadFunc()
{
  std::vector<char> buf;
  buf.resize(4096);
  while (executeThreads)
  {
    sockaddr_storage src_addr{};
    socklen_t src_addr_len = sizeof(src_addr);

    ssize_t nRecvd = recvfrom(this->receiveSocket, buf.data(), buf.size(), 0, (struct sockaddr*) &src_addr, &src_addr_len);

    if (nRecvd == -1)
    {
      RWARNING(0, "Unrecoverable error occurred with receiving socket. Thread exiting...");
      break;
    }
    if (nRecvd == (ssize_t)buf.size())
    {
      /// TODO? Peek socket and resize buffer if necessary? May not be necessary here, though.
      // buffer was too small
      RWARNING(0, "Received datagram was larger than recv buffer. Message has been truncated...");
    }

    if (nRecvd > 0)
    {
      recvMtx.lock();
      recvQueue.emplace(std::string(buf.begin(), buf.begin() + nRecvd));
      recvMtx.unlock();
    }
  }
  RLOG(0, "Quitting client listener thread");
}

unsigned long HoloNetworkConnection::SockAddrToUint32(struct sockaddr* a)
{
  return ((a) && (a->sa_family == AF_INET)) ? ntohl(((struct sockaddr_in*)a)->sin_addr.s_addr) : 0;
}
