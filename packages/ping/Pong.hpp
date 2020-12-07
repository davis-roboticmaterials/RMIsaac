#pragma once

#include "engine/alice/alice.hpp"
#include "messages/messages.hpp"

class Pong : public isaac::alice::Codelet {
    
    public:
        void start() override;
        void tick() override;

        // Incoming message channel
        ISAAC_PROTO_RX(PingProto, trigger);
};

ISAAC_ALICE_REGISTER_CODELET(Pong)