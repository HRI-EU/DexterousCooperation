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

#ifndef RCS_HOLONETWORKCONNECTION_H
#define RCS_HOLONETWORKCONNECTION_H

#include "HoloParser.h"

#include <string>
#include <mutex>
#include <queue>

#ifndef _WIN32
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#else
#include <WS2tcpip.h>
#endif


/*! \brief Networking class to connect to the Microsoft Holo-Lens
 *
 *         Currently, this class has a number of assumptions:
 *         - In UDP mode, messages are sent only to the first client that
 *           sends a message.
 */
class HoloNetworkConnection
{
public:

  /*! \brief Initializes all members with default values, nothing else. The
   *         default send frequency is set to 60 Hz.
   */
  HoloNetworkConnection();

  /*! \brief Returns the number of sent network messages.
   */
  size_t getMessagesSent() const;

  /*! \brief Adds a message to the send queue. It will only be added if the
   *         connection has been established. The function returns true for
   *         success, false if the message was rejected. The function also
   *         completes the message with "HONDA|" at the beginning, and with
   *         the terminal separator "#" at the end of the message. The
   *         function will return with failure when receiving an empty
   *         message.
   */
  bool sendMessage(const std::string& msg);
  bool send(const Hologram& hologram);
  bool send(const std::string& name, const std::string& parent, const HTr* transform);


  /*! \brief Starts the sending network thread. The sending
   *         The function returns true for success, and false
   *         in case the networking interface could not be configured. In this
   *         case, no networking thread has been started.
   */
  bool startServer(int port, std::string outgoingInterface = "eth1");

  /*! \brief Starts the receiving network thread.
   *         The function returns true for success, and false
   *         in case the networking interface could not be configured. In this
   *         case, no networking thread had been started.
   */
  bool startClient(int port);

  /*! \brief Indicates the sending and receiving thread to leave their loop.
   *         Since there are blocking calls in the thread loops, it is not
   *         guaranteed that the threads finish. You cannot assume that the
   *         server threads have been terminated when this function returns.
   */
  void stopServer();

  /*! \brief Sets the frequency the send queue is sent to the client side. The
   *         frequency can be set at any point in time, and will change the
   *         networking also during an established connection. If the frequency
   *         is equal or smaller zero, no messages will be sent.
   */
  void setSendFrequency(double frequency);

  /*! \brief Returns the desired frequency of the send calls.
   */
  double getDesiredSendFrequency() const;

  /*! \brief Returns the number of messages in the send queue.
   */
  size_t getSendQueueSize() const;

  /*! \brief Returns the number of messages in the receive queue.
   */
  size_t getReceiveQueueSize() const;

  /*! \brief Returns the next message in the receive queue or an empty string if the queue is empty
   */
  std::string getMessage();

private:

  void setExecuteThreads(bool value);
  void startServerThreadFunc(unsigned int port);
  void startClientThreadFunc();
  socklen_t createSocket(unsigned int port);
  unsigned long SockAddrToUint32(struct sockaddr* a);

  std::string broadcastInterface = "eth1";
  bool executeThreads;
  int broadcastSocket;
  int receiveSocket;
  size_t messagesSent;
  double desiredSendFrequency;

  mutable std::mutex sendMtx;
  mutable std::mutex recvMtx;
  mutable std::mutex settingsMtx;
  std::queue<std::string> sendQueue;
  std::queue<std::string> recvQueue;
};

#endif   // RCS_HOLONETWORKCONNECTION_H
