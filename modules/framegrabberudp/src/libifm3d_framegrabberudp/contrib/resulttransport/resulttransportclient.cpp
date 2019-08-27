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

#include <ifm3d/fg_udp/contrib/resulttransport/resulttransportclient.hpp>

#include <iostream>
#include <cstring>
#include <memory>

ifm::ResultTransportClient::ResultTransportClient() {}

ifm::Error ifm::ResultTransportClient::handlePacket(const uint8_t*                data,
                                                    size_t                        dataLen,
                                                    std::unique_ptr<Frame>& frame)
{
    frame = nullptr;

    if (dataLen >= sizeof(ifm::PacketHeader))
    {
        const PacketHeader* header = (reinterpret_cast<const PacketHeader*>(data));

        if (header->magic != PACKET_MAGIC_NUMBER)
        {
            return { Error::INVALID_PACKET_MAGIC, "Received packet has invalid MAGIC." };
        }

        if (header->channel_id == ChannelType::FRAME)
        {
            if (header->total_channel_length == sizeof(FRAME_HEADER) && memcmp(data + sizeof(PacketHeader), FRAME_HEADER, sizeof(FRAME_HEADER)) == 0)
            {
                std::cout << "Received start frame " << header->frame_counter << std::endl;

                if (!m_channels.empty())
                {
                    frame = finishCurrentFrame(true);
                }

                m_currentFrameId = header->frame_counter;
            }
            else if (header->total_channel_length == sizeof(FRAME_HEADER) && memcmp(data + sizeof(PacketHeader), FRAME_FOOTER, sizeof(FRAME_FOOTER)) == 0)
            {
                std::cout << "Received complete frame " << header->frame_counter << std::endl;

                frame = finishCurrentFrame();

                return { Error::OK };
            }
            else
            {
                return { Error::INVALID_PACKET_DATA,
                         "Package has FRAME channel type, but is neither a FRAME_HEADER nor a "
                         "FRAME_FOOTER" };
            }
        }
        else
        {
            if (m_currentFrameId != -1 && header->frame_counter != m_currentFrameId && !m_channels.empty())
            {
                frame = finishCurrentFrame(true);
            }

            ChannelType channelType = static_cast<ChannelType>(header->channel_id);

            m_channels[channelType].numOfPacketsInChannel = header->number_of_packets_in_channel;
            m_channels[channelType].dataLength            = header->total_channel_length;
            m_channels[channelType].packets.push(std::vector<uint8_t>(data, data + dataLen));
        }

        return { Error::OK };
    }

    return { Error::INVALID_PACKET_DATA, "The received packet is too small." };
}

std::unique_ptr<ifm::ResultTransportClient::Frame> ifm::ResultTransportClient::finishCurrentFrame(bool forceIncomplete)
{
    auto frame      = std::unique_ptr<Frame>(new Frame());
    frame->frameId  = static_cast<uint32_t>(m_currentFrameId);
    frame->complete = !forceIncomplete;

    for (auto& it : m_channels)
    {
        if (it.second.numOfPacketsInChannel == it.second.packets.size())
        {
            frame->frameData.reserve(frame->frameData.size() + it.second.dataLength);

            while (!it.second.packets.empty())
            {
                frame->frameData.insert(frame->frameData.end(), it.second.packets.top().begin()+sizeof(PacketHeader), it.second.packets.top().end());
                it.second.packets.pop();
            }
        }
        else
        {
            frame->complete = false;
            frame->incompleteChannels.push_back(it.first);
        }
    }

    m_channels.clear();

    return frame;
}
