// -*- c++ -*-
/*
 * Copyright (C) 2019 ifm electronics GmbH
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RESULTTRANSPORTCLIENT_H
#define RESULTTRANSPORTCLIENT_H

#include <map>
#include <queue>

#include "resulttransport_common.hpp"

namespace ifm {

class ResultTransportClient
{
public:
    struct Frame
    {
        std::vector<uint8_t>        frameData;
        std::vector<ChannelType>    incompleteChannels;
        uint32_t                    frameId;
        bool                        complete;
    };

    ResultTransportClient();
    virtual ~ResultTransportClient() = default;

    Error handlePacket(const uint8_t *data, size_t dataLen, std::unique_ptr<Frame>& frame);

private:
    struct PacketCompare
    {
        bool operator()(const std::vector<uint8_t>& lhs, const std::vector<uint8_t>& rhs)
        {
            auto lhsHeader = reinterpret_cast<const PacketHeader*>(lhs.data());
            auto rhsHeader = reinterpret_cast<const PacketHeader*>(rhs.data());
            return lhsHeader->packet_counter > rhsHeader->packet_counter;
        }
    };
    using PacketQueue
        = std::priority_queue<std::vector<std::uint8_t>, std::vector<std::vector<std::uint8_t>>, PacketCompare>;

    struct ReceiveChannel
    {
        uint16_t    numOfPacketsInChannel;
        uint32_t    dataLength;
        PacketQueue packets;
    };

    std::unique_ptr<Frame> finishCurrentFrame(bool forceIncomplete = false);

    std::map<ChannelType, ReceiveChannel> m_channels;
    int64_t                               m_currentFrameId = -1;
};


}

#endif  // RESULTTRANSPORTCLIENT_H
