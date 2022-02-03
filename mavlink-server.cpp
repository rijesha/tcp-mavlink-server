#include "mavlink-server.h"

typedef unsigned char uchar;
static const std::string b = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"; //=
static std::string base64_encode(const std::string &in)
{
    std::string out;

    int val = 0, valb = -6;
    for (uchar c : in)
    {
        val = (val << 8) + c;
        valb += 8;
        while (valb >= 0)
        {
            out.push_back(b[(val >> valb) & 0x3F]);
            valb -= 6;
        }
    }
    if (valb > -6)
        out.push_back(b[((val << 8) >> (valb + 8)) & 0x3F]);
    while (out.size() % 4)
        out.push_back('=');
    return out;
}

static std::string base64_decode(const std::string &in)
{

    std::string out;

    std::vector<int> T(256, -1);
    for (int i = 0; i < 64; i++)
        T[b[i]] = i;

    int val = 0, valb = -8;
    for (uchar c : in)
    {
        if (T[c] == -1)
            break;
        val = (val << 6) + T[c];
        valb += 6;
        if (valb >= 0)
        {
            out.push_back(char((val >> valb) & 0xFF));
            valb -= 8;
        }
    }
    return out;
}

mavlink_status_t lastStatus;
// observer callback. will be called for every new message received by clients
// with the requested IP address
void MavlinkServer::onIncomingMsg(const Client &client, const char *msg, size_t size)
{
    std::string msgStr = msg;
    // print the message content

    mavlink_message_t message;
    mavlink_status_t status;
    uint8_t msgReceived = false;

    std::string newstr = base64_decode(msgStr);

    for (unsigned i = 0; i < newstr.length(); ++i)
    {
        // the parsing
        msgReceived = mavlink_parse_char(MAVLINK_COMM_1, newstr.at(i), &message, &status);

        // check for dropped packets
        if ((lastStatus.packet_rx_drop_count != status.packet_rx_drop_count))
        {
            printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
        }
        lastStatus = status;
    }

    if (msgReceived && subscriber_)
    {
        subscriber_(&message);
    } else {
        std::cout << "failed \n";
    }
}

// observer callback. will be called when client disconnects
void MavlinkServer::onClientDisconnected(const Client &client)
{
    std::cout << "Client: " << client.getIp() << " disconnected: " << client.getInfoMessage() << std::endl;
}

TcpServer MavlinkServer::server;
MavlinkSubscriber MavlinkServer::subscriber_ = nullptr;

MavlinkServer::MavlinkServer()
{
    // start server on port 65123
    pipe_ret_t startRet = server.start(65123);
    if (startRet.success)
    {
        std::cout << "Server setup succeeded" << std::endl;
    }
    else
    {
        std::cout << "Server setup failed: " << startRet.msg << std::endl;
        return;
    }

    // configure and register observer1
    observer.incoming_packet_func = onIncomingMsg;
    ;
    observer.disconnected_func = onClientDisconnected;
    observer.wantedIp = "127.0.0.1";
    server.subscribe(observer);

    clientThread = std::thread([=]() {
        clientRunner();
    });
    // receive clients
}
void MavlinkServer::clientRunner()
{
    while (1)
    {
        Client client = server.acceptClient(0);
        if (client.isConnected())
        {
            std::cout << "Got client with IP: " << client.getIp() << std::endl;
            server.printClients();
        }
        else
        {
            std::cout << "Accepting client failed: " << client.getInfoMessage() << std::endl;
        }
        sleep(1);
    }
}

void MavlinkServer::sendMessage(mavlink_message_t *msg)
{
    mtx.lock();
    size_t len = mavlink_msg_to_send_buffer((uint8_t *)sendBuffer, msg);
    std::string data = base64_encode(std::string(sendBuffer, len));
    data.append("=");
    server.sendToAllClients(data.c_str(), data.length());
    mtx.unlock();
}

void MavlinkServer::bindMessageSubscriber(MavlinkSubscriber subscriber)
{
    subscriber_ = subscriber;
}
