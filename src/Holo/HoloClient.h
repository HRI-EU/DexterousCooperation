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

#ifndef RCS_HOLOCLIENT_H
#define RCS_HOLOCLIENT_H

#if !defined (_MSC_VER)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <signal.h>

#if !defined (_MSC_VER)
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#else
#include <WinSock2.h>
#endif

#define PORT 26794
#define SA struct sockaddr




void serverDown(int /*sig*/)
{
  fprintf(stderr, "SIGPIPE\n");
}


void testHoloClient()
{
  signal(SIGPIPE, serverDown);

  int sockfd;
  struct sockaddr_in servaddr;

  // socket create and varification
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd == -1)
  {
    RFATAL("Socket creation failed");
  }
  else
  {
    RLOG(0, "Socket successfully created");
  }

  int enable = 1;
  int result = setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int));

  if (result<0)
  {
    RLOG(0, "setsockopt(SO_REUSEADDR) failed: %s", strerror(errno));
  }

  bzero(&servaddr, sizeof(servaddr));

  // assign IP, PORT
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
  servaddr.sin_port = htons(PORT);

  // connect the client socket to server socket
  if (connect(sockfd, (SA*)&servaddr, sizeof(servaddr)) != 0)
  {
    RFATAL("connection with the server failed...");
  }
  else
  {
    RLOG(0, "Connected to the server");
  }

  // function for chat
  std::string nwMessage;
  int n=0;

  for (;;)
  {
    nwMessage = "HONDA|message" + std::to_string(n++) + "#";
    //ssize_t len = send(broadcastSocket, nwMessage.c_str(), nwMessage.length(), MSG_NOSIGNAL);
    ssize_t len = send(sockfd, nwMessage.c_str(), nwMessage.length(), 0);

    RLOG(0, "Sending %d bytes: %s", len, nwMessage.c_str());
    if (len == -1)   // some other error has occured
    {
      RLOG(0, "Failed to send data from Holo-Lens: %s", strerror(errno));

      if (connect(sockfd, (SA*)&servaddr, sizeof(servaddr)) != 0)
      {
        printf("reconnect with the server failed...\n");
      }
    }


    Timer_waitDT(0.2);
  }

  RLOG(0, "Closing socket");

  // close the socket
#if defined (_MSC_VER)
  closesocket(socketfd);
#else
  close(sockfd);
#endif
}



#else

void testHoloClient() {}

#endif

#endif   // RCS_HOLOCLIENT_H
