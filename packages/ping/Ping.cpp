#include "Ping.hpp"

#include <stdio.h>      // for sprintf()
#include <iostream>     // for console output
#include <string>       // for std::string

void Ping::start() {
    tickPeriodically();
}

void Ping::tick() {
    // Create and publish a ping message
    auto proto = tx_ping().initProto();

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
    
    
    proto.setMessage(std::string(buffer));
    tx_ping().publish();
}

void Ping::stop() {

}

// char buffer[27];
// int millisec;
// struct tm* tm_info;
// struct timeval tv;

// gettimeofday(&tv, NULL);

// millisec = tv.tv_usec; // Round to nearest millisec
// if (millisec >= 1) { // Allow for rounding up to nearest second
//     millisec -= 1;
//     tv.tv_sec++;
// }

// tm_info = localtime(&tv.tv_usec);

// strftime(buffer, sizeof(buffer), "%Y:%m:%d %H:%M:%S", tm_info);
// sprintf(buffer, "%s.%06d", buffer, millisec);

// std::string str(buffer);

// proto.setMessage(str);