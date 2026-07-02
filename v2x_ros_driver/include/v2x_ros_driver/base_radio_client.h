#pragma once
/*
 * Copyright (C) 2022-2026 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <boost/asio.hpp>
#include <boost/signals2/signal.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace V2XDriverApplication
{

/**
 * @brief BaseRadioClient is a concrete base class that provides shared validation/utilities
 * (as protected virtual functions) and defines abstract transport methods that subclasses
 * (UdpRadioClient, MqttRadioClient) must implement.
 *
 * This design allows runtime selection of the communication protocol while sharing
 * message parsing and validation logic across all implementations.
 */
class BaseRadioClient
{
public:
    BaseRadioClient();
    virtual ~BaseRadioClient();

    //  Abstract transport methods

    /**
     * @brief Connects the driver to the radio/broker
     * @param remote_address Address of radio or broker
     * @param remote_port Port of remote service
     * @param local_port Local port to receive incoming data (unused for MQTT)
     * @param ec Error code set during connect
     * @return True on successful connect, false otherwise
     */
    virtual bool connect(const std::string &remote_address, unsigned short remote_port,
                         unsigned short local_port, boost::system::error_code &ec) = 0;

    /**
     * @brief Convenience overload without error code
     */
    virtual bool connect(const std::string &remote_address, unsigned short remote_port,
                         unsigned short local_port);

    /**
     * @brief Closes connection — closes all sockets/clients, stops all threads
     */
    virtual void close() = 0;

    /**
     * @brief Returns connected state
     */
    virtual bool connected() const = 0;

    /**
     * @brief Sends a V2X message via the underlying transport
     * @param message Message vector to be sent
     * @return True on success
     */
    virtual bool sendV2xMessage(const std::shared_ptr<std::vector<uint8_t>> &message) = 0;

    //  Signals

    /** @brief Signaled when client is connected to a V2X radio device */
    boost::signals2::signal<void()> onConnect;

    /** @brief Signaled when client is disconnected from hardware */
    boost::signals2::signal<void()> onDisconnect;

    /** @brief Signaled when client experiences a fatal error */
    boost::signals2::signal<void(const boost::system::error_code&)> onError;

    /** @brief Signaled when message received */
    boost::signals2::signal<void(std::vector<uint8_t> const &, uint16_t id)> onMessageReceived;

    //  Shared validation

    /** @brief Validate possible DSRCmsgID against loaded wave config */
    bool IsValidMsgID(const std::string &msg_id);

    /** @brief Load list of valid dsrc_msg_ids and PSIDs from wave.json */
    void loadWaveConfigIds(const std::string &fileName);

    /** @brief Set wave config file path */
    void set_wave_file_path(const std::string &path);

    /** @brief Checks actual message vector size against size specified in MessageFrame */
    bool isValidMsgSize(const std::vector<uint8_t> &msg_vec, size_t start_index,
                        const std::vector<uint8_t> &entry);

    /**
     * @brief Initial check for msg_id against PSID list to avoid treating a WSMP
     * T-Header with valid PSID as a MessageFrame.
     */
    bool isPossiblePSID(const std::string &msg_id);

    /**
     * @brief Final check to ensure a found BSM PSID (0x0020) is not actually
     * PSM DSRCmsgID 32 (0x0020).
     */
    bool isValidMsgAssumingBSMPSID(size_t start_index, const std::vector<uint8_t> &entry);

    /**
     * @brief Look up PSID string for a given decimal DSRCmsgID string.
     * @return PSID hex string, or empty string if not found.
     */
    std::string psidForDsrcMsgId(const std::string &dsrc_msg_id) const;

protected:

    /**
     * @brief Processes incoming data — finds and validates J2735 messages.
     * Subclasses call this from their receive callbacks.
     */
    virtual void process(const std::shared_ptr<const std::vector<uint8_t>> &data);

    /**
     * @brief Interpret buf[s:] as an IEEE 1609.2 Ieee1609Dot2Data and, if it ultimately
     * carries a J2735 MessageFrame as its unsecuredData, return that MessageFrame.
     */
    bool stripIeee1609Dot2Header(const std::vector<uint8_t> &in, std::vector<uint8_t> &out);

    rclcpp::Logger logger_{rclcpp::get_logger("v2x_ros_driver")};

    std::vector<std::string> wave_cfg_dsrc_msg_ids_;
    std::vector<std::string> wave_cfg_psids_;
    std::string wave_file_path;

    /** @brief Frame overhead for payloads > 127 bytes (2-byte length + 2-byte DSRCmsgID) */
    static const int long_frame_ = 4;
    /** @brief Frame overhead for payloads < 128 bytes (1-byte length + 2-byte DSRCmsgID) */
    static const int short_frame_ = 3;

    // Latency accumulators for stripIeee1609Dot2Header — reported via DEBUG throttle
    uint64_t strip_call_count_{0};
    uint64_t strip_total_ns_{0};
    uint64_t strip_max_ns_{0};
};

}
