#include "replacerclient.h"
#include<QUdpSocket>

replacerClient::replacerClient(QString address, int port){

    // create a QUDP socket
    socket = new QUdpSocket();

    if(socket->isOpen())
        socket->close();

    this->_addr.setAddress(address);
    this->_port = quint16(port);
    socket->connectToHost(address,port, QIODevice::WriteOnly, QAbstractSocket::IPv4Protocol);
    //socket->bind(this->_addr, this->_port);
}

replacerClient::~replacerClient(){
    socket->close();
}

void replacerClient::sendCommand(std::vector<Entity> &entities)
{
    fira_message::sim_to_ref::Environment packet;

    fira_message::Ball ball = packet.mutable_frame()->ball();

    ball.set_x(entities[0].position().x);
    ball.set_y(entities[0].position().x);
    ball.set_vx(entities[0].velocity().x);
    ball.set_vy(entities[0].velocity().y);

    for(int i=0;i<1;i++){
            fira_message::Robot* robot = packet.mutable_frame()->add_robots_blue();
            robot->set_robot_id(i);
            robot->set_x(entities[i+1].position().x);
            robot->set_y(entities[i+1].position().y);
            robot->set_orientation(entities[i+1].angle());
            robot->set_vx(entities[i+1].velocity().x);
            robot->set_vy(entities[i+1].velocity().y);
            robot->set_vorientation(entities[i+1].angleVelocity());
    }

    QByteArray dgram;
    dgram.resize(packet.ByteSize());
    packet.SerializeToArray(dgram.data(), dgram.size());
    if(socket->writeDatagram(dgram, this->_addr, this->_port) > -1){
        //printf("send data\n");
    }
}


