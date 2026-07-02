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

#include "v2x_ros_driver/mqtt_radio_client.h"

#include <iostream>
#include <iomanip>
#include <sstream>

namespace V2XDriverApplication
{

//  Static library reference count for mosquitto_lib_init/cleanup

std::atomic<int> MqttRadioClient::lib_ref_count_{0};

//  Construction / Destruction

MqttRadioClient::MqttRadioClient()
{
    logger_ = rclcpp::get_logger("v2x_ros_driver");

    // Thread-safe one-time library init
    if (lib_ref_count_.fetch_add(1) == 0)
    {
        mosquitto_lib_init();
    }
}

MqttRadioClient::~MqttRadioClient()
{
    try { close(); } catch (...) {}

    if (lib_ref_count_.fetch_sub(1) == 1)
    {
        mosquitto_lib_cleanup();
    }
}

//  MQTT-specific configuration

void MqttRadioClient::setMqttConfig(const std::string &client_id,
                                    int qos,
                                    int reconnect_interval_sec,
                                    const std::string &mqtt_topic_prefix)
{
    client_id_ = client_id;
    qos_ = qos;
    reconnect_interval_sec_ = reconnect_interval_sec;
    mqtt_topic_prefix_ = mqtt_topic_prefix;
}

//  Connect

bool MqttRadioClient::connect(const std::string &remote_address,
                              unsigned short remote_port,
                              unsigned short local_port,
                              boost::system::error_code &ec)
{
    (void)local_port; // unused for MQTT

    if (connected_.load()) return false;

    broker_address_ = remote_address;
    broker_port_ = static_cast<int>(remote_port);

    ec.clear();

    // Create mosquitto client instance
    const char *id = client_id_.empty() ? nullptr : client_id_.c_str();
    mosq_ = mosquitto_new(id, true /*clean_session*/, this /*userdata*/);
    if (!mosq_)
    {
        RCLCPP_ERROR(logger_, "MqttRadioClient: mosquitto_new() failed");
        ec = boost::asio::error::no_memory;
        return false;
    }

    // Set callbacks
    mosquitto_connect_callback_set(mosq_, &MqttRadioClient::onConnectCb);
    mosquitto_disconnect_callback_set(mosq_, &MqttRadioClient::onDisconnectCb);
    mosquitto_message_callback_set(mosq_, &MqttRadioClient::onMessageCb);

    // Configure automatic reconnection
    mosquitto_reconnect_delay_set(mosq_,
                                  static_cast<unsigned int>(reconnect_interval_sec_),
                                  static_cast<unsigned int>(reconnect_interval_sec_ * 10),
                                  true);

    // Start the threaded network loop (handles reconnects automatically)
    int rc = mosquitto_loop_start(mosq_);
    if (rc != MOSQ_ERR_SUCCESS)
    {
        RCLCPP_ERROR_STREAM(logger_, "MqttRadioClient: mosquitto_loop_start() failed: "
                                     << mosquitto_strerror(rc));
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        ec = boost::asio::error::connection_refused;
        return false;
    }

    // Initiate connection to broker (non-blocking with threaded loop)
    int keepalive = 60; // seconds
    rc = mosquitto_connect(mosq_, broker_address_.c_str(), broker_port_, keepalive);
    if (rc != MOSQ_ERR_SUCCESS)
    {
        RCLCPP_ERROR_STREAM(logger_, "MqttRadioClient: mosquitto_connect() failed: "
                                     << mosquitto_strerror(rc));
        mosquitto_loop_stop(mosq_, true /*force*/);
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        ec = boost::asio::error::connection_refused;
        return false;
    }

    RCLCPP_INFO_STREAM(logger_, "MqttRadioClient: connecting to broker at "
                                << broker_address_ << ":" << broker_port_);
    return true;
}

//  Close

void MqttRadioClient::close()
{
    if (!mosq_) return;

    connected_.store(false);
    mosquitto_disconnect(mosq_);
    mosquitto_loop_stop(mosq_, false /*force*/);
    mosquitto_destroy(mosq_);
    mosq_ = nullptr;

    onDisconnect();
}

//  Send (publish)

bool MqttRadioClient::sendV2xMessage(const std::shared_ptr<std::vector<uint8_t>> &message)
{
    if (!connected_.load() || !mosq_) return false;
    if (!message || message->size() < 2) return false;

    // Extract DSRCmsgID from first two bytes
    auto dsrc_msg_id = (static_cast<uint16_t>((*message)[0]) << 8) |
                        static_cast<uint16_t>((*message)[1]);
    std::string dsrc_msg_id_str = std::to_string(dsrc_msg_id);

    // Look up PSID for this DSRCmsgID
    std::string psid = psidForDsrcMsgId(dsrc_msg_id_str);
    if (psid.empty())
    {
        RCLCPP_WARN_STREAM(logger_, "MqttRadioClient: no PSID mapping for DSRCmsgID "
                                    << dsrc_msg_id_str << ", using raw id");
        psid = dsrc_msg_id_str;
    }

    std::string topic = buildPublishTopic(psid);

    int rc = mosquitto_publish(mosq_,
                               nullptr,                          // message id (let lib assign)
                               topic.c_str(),
                               static_cast<int>(message->size()),
                               message->data(),
                               qos_,
                               false);                           // retain
    if (rc != MOSQ_ERR_SUCCESS)
    {
        RCLCPP_WARN_STREAM(logger_, "MqttRadioClient: publish failed on topic '"
                                    << topic << "': " << mosquitto_strerror(rc));
        return false;
    }

    RCLCPP_DEBUG_STREAM(logger_, "MqttRadioClient: published " << message->size()
                                 << " bytes to " << topic);
    return true;
}

//  Mosquitto callbacks (static trampolines)

void MqttRadioClient::onConnectCb(struct mosquitto * /*mosq*/, void *userdata, int rc)
{
    auto *self = static_cast<MqttRadioClient *>(userdata);
    self->handleConnect(rc);
}

void MqttRadioClient::onDisconnectCb(struct mosquitto * /*mosq*/, void *userdata, int rc)
{
    auto *self = static_cast<MqttRadioClient *>(userdata);
    self->handleDisconnect(rc);
}

void MqttRadioClient::onMessageCb(struct mosquitto * /*mosq*/, void *userdata,
                                   const struct mosquitto_message *msg)
{
    auto *self = static_cast<MqttRadioClient *>(userdata);
    self->handleMessage(msg);
}

//  Instance-level callback handlers

void MqttRadioClient::handleConnect(int rc)
{
    if (rc == 0)
    {
        RCLCPP_INFO_STREAM(logger_, "MqttRadioClient: connected to broker at "
                                    << broker_address_ << ":" << broker_port_);
        connected_.store(true);
        subscribeToTopics();
        onConnect();
    }
    else
    {
        RCLCPP_WARN_STREAM(logger_, "MqttRadioClient: broker connection failed, rc="
                                    << rc << " (" << mosquitto_connack_string(rc) << ")");
    }
}

void MqttRadioClient::handleDisconnect(int rc)
{
    connected_.store(false);
    if (rc != 0)
    {
        RCLCPP_WARN_STREAM(logger_, "MqttRadioClient: unexpected disconnect, rc=" << rc
                                    << ". Mosquitto threaded loop will attempt auto-reconnect.");
    }
    else
    {
        RCLCPP_INFO(logger_, "MqttRadioClient: cleanly disconnected from broker.");
    }
}

void MqttRadioClient::handleMessage(const struct mosquitto_message *msg)
{
    if (!msg || !msg->payload || msg->payloadlen <= 0) return;

    RCLCPP_DEBUG_STREAM(logger_, "MqttRadioClient: received " << msg->payloadlen
                                 << " bytes on topic '" << msg->topic << "'");

    // Wrap payload in shared_ptr<vector<uint8_t>> and pass to base class process()
    auto data = std::make_shared<std::vector<uint8_t>>(
        static_cast<uint8_t *>(msg->payload),
        static_cast<uint8_t *>(msg->payload) + msg->payloadlen);

    process(data);
}

//  Topic helpers

void MqttRadioClient::subscribeToTopics()
{
    // Subscribe to wildcard topic covering all J2735 PSIDs
    std::string topic = buildSubscribeTopic();
    int rc = mosquitto_subscribe(mosq_, nullptr, topic.c_str(), qos_);
    if (rc != MOSQ_ERR_SUCCESS)
    {
        RCLCPP_ERROR_STREAM(logger_, "MqttRadioClient: subscribe failed for '"
                                     << topic << "': " << mosquitto_strerror(rc));
    }
    else
    {
        RCLCPP_INFO_STREAM(logger_, "MqttRadioClient: subscribed to '" << topic << "'");
    }
}

// Override map — only for PSID where Ettifos differs from current wave.json mapping
static const std::unordered_map<std::string, int> kEttifosPsidOverrides = {

    {"0020", 32},  // BSM
    {"0083", 131},  // TIM
    {"0082", 2113687},  // MAP
    {"0082", 130},  //SPAT
    {"8010", 144},  //SDSM
    {"0027", 39},  //PSM
    /* Test Messages */
    {"BFEE", 16494},  //Mobility Messages
    //TODO for future: SRM ID: 2113686
};

std::string MqttRadioClient::buildPublishTopic(const std::string &psid_hex) const {
    int decimal_psid;
    auto it = kEttifosPsidOverrides.find(psid_hex);
    if (it != kEttifosPsidOverrides.end()) {
        decimal_psid = it->second;
    } else {
        decimal_psid = std::stoi(psid_hex, nullptr, 16);
    }
    return mqtt_topic_prefix_ + "/req/J2735/" + std::to_string(decimal_psid);
}

std::string MqttRadioClient::buildSubscribeTopic() const
{
    // Ettifos/V2X/ind/J2735/#
    return mqtt_topic_prefix_ + "/ind/J2735/#";
}

}
