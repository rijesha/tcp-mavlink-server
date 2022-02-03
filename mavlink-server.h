
#include <iostream>
#include <signal.h>
#include <thread>
#include <mutex>

#include "tcp_server_client/tcp_server.h"
#include <ardupilotmega/mavlink.h>

typedef void (*MavlinkSubscriber)(mavlink_message_t *);

class MavlinkServer
{
public:
    MavlinkServer();
    void sendMessage(mavlink_message_t *msg);
    void bindMessageSubscriber(MavlinkSubscriber subscriber);

    static void onIncomingMsg(const Client &client, const char *msg, size_t size);
    static void onClientDisconnected(const Client &client);
    bool init = false;

private:
    char sendBuffer[300];

    // declare the server
    static TcpServer server;

    // declare a server observer which will receive incoming messages.
    // the server supports multiple observers
    server_observer_t observer;

    static MavlinkSubscriber subscriber_;

    std::thread clientThread;
    void clientRunner();
    std::mutex mtx;
};
