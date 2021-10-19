#ifndef REPLACERCLIENT_H
#define REPLACERCLIENT_H

#ifndef EXTERNAL_COMPILATION
    #include "Entity/Entity.h"
    #include "Utils/Utils.h"
#endif



#include <QObject>
#include <QUdpSocket>
#include <QString>

#include "Network/pb/command.pb.h"
#include "Network/pb/common.pb.h"
#include "Network/pb/packet.pb.h"

class replacerClient
{
public:
    replacerClient(QString address, int port);
    ~replacerClient();
    void sendCommand(std::vector<Entity> &entities);

    QHostAddress _addr;
    quint16 _port;

private:
    QUdpSocket *socket;

};
#endif // REPLACERCLIENT_H
