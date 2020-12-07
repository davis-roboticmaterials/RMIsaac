#include "Pong.hpp"

#include <cstdio>
#include <iostream>

void Pong::start() {
    tickOnMessage(rx_trigger());
}

void Pong::tick() {
    // Parse received message
    using std::chrono::system_clock;
    auto currentTime = std::chrono::system_clock::now();
    char buffer[80];

    auto transformed = currentTime.time_since_epoch().count() / 1000;
    auto millis = transformed % 1000000;

    std::time_t tt;
    tt = system_clock::to_time_t ( currentTime );
    auto timeinfo = localtime (&tt);
    strftime (buffer,80,"%S",timeinfo);
    sprintf(buffer, "%s.%06d",buffer,(int)millis);
    float end_time = std::stof(std::string(buffer));

    // Recieve message
    auto proto = rx_trigger().getProto();
    auto message = proto.getMessage().cStr();

    float start_time = std::stof(message);

    // std::printf("%s\n", message);
    // std::printf("%06f -> %06f\n", start_time, end_time);
    std::printf("Ping: time: %03fms\n", 1000*(end_time - start_time));
}