/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Torc Robotics, LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Torc Robotics, LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros2_dsrc_driver/ros2_dsrc_driver_node.h"
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/schema.h>
#include <fstream>

namespace DSRCApplication
{

namespace std_ph = std::placeholders;

Node::Node(const rclcpp::NodeOptions &options)
    : carma_ros2_utils::CarmaLifecycleNode(options)
{
}

rcl_interfaces::msg::SetParametersResult Node::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
{

    auto error = update_params<int>({{"listening_port", config_.listening_port}}, parameters);
    error = update_params<int>({{"dsrc_listening_port", config_.dsrc_listening_port}}, parameters);
    error = update_params<std::string>({{"dsrc_address", config_.dsrc_address}}, parameters);

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error;

    return result;
}


std::string Node::uint8_vector_to_hex_string(const std::vector<uint8_t>& v) {
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    std::vector<uint8_t>::const_iterator it;
    for (it = v.begin(); it != v.end(); it++) {
        ss << std::setw(2) << static_cast<unsigned>(*it);
    }

    return ss.str();
}

carma_ros2_utils::CallbackReturn Node::handle_on_configure(const rclcpp_lifecycle::State &) {

    pre_spin();

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("ros2_dsrc_driver");

    std::string wave_cfg_file = package_share_directory + "/config/wave.json";
    loadWaveConfig(wave_cfg_file);

    config_.dsrc_address = declare_parameter<std::string>("dsrc_address", config_.dsrc_address);
    config_.dsrc_listening_port = declare_parameter<int>("dsrc_listening_port", config_.dsrc_listening_port);
    config_.listening_port = declare_parameter<int>("listening_port", config_.listening_port);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&Node::parameter_update_callback, this, std_ph::_1));

    //set wave file path in dsrc_client
    dsrc_client_.set_wave_file_path(wave_cfg_file);
    //Setup connection handlers
    dsrc_client_error_.clear();
    dsrc_client_.onConnect.connect([this](){});
    dsrc_client_.onDisconnect.connect([this](){});
    dsrc_client_.onError.connect([this](const boost::system::error_code& err){dsrc_client_error_ = err;});

    //Comms Subscriber
    comms_sub_ = create_subscription<carma_driver_msgs::msg::ByteArray>("outbound_binary_msg", queue_size_, std::bind(&Node::onOutboundMessage, this, std_ph::_1));

    //Comms Publisher
    comms_pub_ = create_publisher<carma_driver_msgs::msg::ByteArray>("inbound_binary_msg", queue_size_);

    //Comms Service
    // Setup service servers
    comms_srv_ = create_service<carma_driver_msgs::srv::SendMessage>("send", std::bind(&Node::sendMessageSrv, this, std_ph::_1, std_ph::_2, std_ph::_3));

    dsrc_client_.onMessageReceived.connect([this](std::vector<uint8_t> const &msg, uint16_t id) {onMessageReceivedHandler(msg, id); });

    sendMessageFromQueue();

    return CallbackReturn::SUCCESS;
}


/**
* @brief Handles messages received from the DSRCOBUClient
*
* Populates a ROS message with the contents of the incoming OBU message, and
* publishes to the ROS 'recv' topic.
*/
void Node::onMessageReceivedHandler(const std::vector<uint8_t> &data, uint16_t id) {
    // Create and populate the message
    auto it = std::find_if(wave_cfg_items_.begin(),wave_cfg_items_.end(),[id](const WaveConfigStruct& entry)
                                                                            {
                                                                               return entry.dsrc_id == std::to_string(id);
                                                                            });

    carma_driver_msgs::msg::ByteArray msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "";
    msg.message_type = it != wave_cfg_items_.end() ? it->name : "Unknown";
    msg.content = data;
    // Publish it
    comms_pub_->publish(msg);

    RCLCPP_DEBUG_STREAM(this->get_logger(),"Application received Data: " << data.size() << " bytes, message: " << uint8_vector_to_hex_string(data));
}

/**
 * @brief Packs an outgoing message into J2375 standard.
 * @param message
 *
 * This processes an incoming ByteArray message, and packs it according to the
 * J2735 standard.
 *
 * TODO: Right now it doesn't do anything except return the message data (ignoring message type).
 * Depending on what the input is, that might be all that's necessary, but possibly more.
 * Depending on what the input is, that might be all that's necessary, but possibly more.
 */
std::vector<uint8_t> Node::packMessage(carma_driver_msgs::msg::ByteArray message) {
    std::stringstream ss;
    auto wave_item = std::find_if(wave_cfg_items_.begin(),wave_cfg_items_.end(),[&message](const WaveConfigStruct& entry)
    {
        return entry.name == message.message_type;
    });

    WaveConfigStruct cfg;
    if(wave_item == wave_cfg_items_.end())
    {
        RCLCPP_WARN_STREAM(this->get_logger(),"No wave config entry for type: " << message.message_type << ", using defaults");
        cfg.name = message.message_type;
        cfg.channel = "CCH";  //Assuming the Default channel is not the safety related info that would be in a BSM message
        cfg.priority = "1";
        cfg.dsrc_id = std::to_string((message.content[0] << 8 ) | message.content[1]);
        cfg.psid = cfg.dsrc_id;
    }
    else
    {
        cfg = *wave_item;
    }

    ss << "Version=0.7" << std::endl;
    ss << "Type=" << cfg.name << std::endl;
    ss << "PSID=" << cfg.psid << std::endl;
    ss << "Priority=" << cfg.priority << std::endl;
    ss << "TxMode=ALT" << std::endl;
    ss << "TxChannel=" << cfg.channel << std::endl;
    ss << "TxInterval=0" << std::endl;
    ss << "DeliveryStart=" << std::endl;
    ss << "DeliveryStop=" << std::endl;
    ss << "Signature=False" << std::endl;
    ss << "Encryption=False" << std::endl;

    ss << "Payload=" <<  uint8_vector_to_hex_string(message.content) << std::endl;

    std::string str = ss.str();

    return std::vector<uint8_t>(str.begin(), str.end());
}

/**
* @brief Handles outbound messages from the ROS network
* @param message
*
* This method receives a message from the ROS network, and adds it to the send queue.
*/
void Node::onOutboundMessage(carma_driver_msgs::msg::ByteArray::UniquePtr message) {
    if(!dsrc_client_.connected())
    {
        RCLCPP_WARN_STREAM(this->get_logger(),"Outbound message received but node is not connected to DSRC Radio");
        return;
    }
    std::shared_ptr<std::vector<uint8_t>> message_content = std::make_shared<std::vector<uint8_t>>(std::move(packMessage(*message)));
    send_msg_queue_.push_back(std::move(message_content));
}

/**
* @brief Sends a message from the queue of outbound messages
*/
void Node::sendMessageFromQueue() {
    if (!send_msg_queue_.empty()) {
        RCLCPP_DEBUG_STREAM(this->get_logger(),"Sending message: " << std::string(send_msg_queue_.front()->begin(),send_msg_queue_.front()->end()));
        bool success = dsrc_client_.sendDsrcMessage(send_msg_queue_.front());
        send_msg_queue_.pop_front();
        if (!success) {
            RCLCPP_WARN_STREAM(this->get_logger(),"Message send failed");
        }
        else {
            RCLCPP_DEBUG_STREAM(this->get_logger(),"Message successfully sent from queue");
        }
    }
}

// /**
// * @brief Message sending service
// * @param req
// * @param res
// */

void Node::sendMessageSrv(const std::shared_ptr<rmw_request_id_t> header,
                                  const std::shared_ptr<carma_driver_msgs::srv::SendMessage::Request> req,
                                  const std::shared_ptr<carma_driver_msgs::srv::SendMessage::Response> res) {

    if(!dsrc_client_.connected())
    {
        RCLCPP_WARN_STREAM(this->get_logger(),"Outbound message received but node is not connected to DSRC Radio");
        res->error_status = 1;
    }

    // Package data into a message shared pointer; this lets packMessage have
    // the same interface for outgoing messages from the topic and service.
    const carma_driver_msgs::msg::ByteArray::UniquePtr message = carma_driver_msgs::msg::ByteArray::UniquePtr(new carma_driver_msgs::msg::ByteArray(req->message_to_send));

    std::shared_ptr<std::vector<uint8_t>> message_data = std::make_shared<std::vector<uint8_t>>(std::move(packMessage(*message)));

    bool success = dsrc_client_.sendDsrcMessage(message_data);
    if (success) {
        RCLCPP_DEBUG_STREAM(this->get_logger(),"SendMessage service returned success");
        res->error_status = 0;
    }
    else {
        RCLCPP_WARN_STREAM(this->get_logger(),"SendMessage service returned failure");
        res->error_status = 1;
    }
}

void Node::pre_spin()
{
    // Adjust output queue size if config changed.
    if(dsrc_client_error_)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(),"DSRC Client Error : " << dsrc_client_error_.message());
        dsrc_client_.close();
        dsrc_client_error_.clear();
    }
    //If we are not connected
    if (!connecting_ && !dsrc_client_.connected())
    {
        connecting_ = true;
        if (connect_thread_)
            connect_thread_->join();

        //We don't want to block the spin thread because the driver
        //application maintains driver status topic
        connect_thread_.reset(new std::thread([this]()
        {
            RCLCPP_INFO_STREAM(this->get_logger(),"Attempting to connect to OBU");
            boost::system::error_code ec;
            RCLCPP_INFO_STREAM(this->get_logger(),"Connecting to: " << config_.dsrc_address.c_str() << ", " << config_.dsrc_listening_port);
            RCLCPP_INFO_STREAM(this->get_logger(),"Local port: " << config_.listening_port);
            try {
                if (!dsrc_client_.connect(config_.dsrc_address, config_.dsrc_listening_port,
                                          config_.listening_port, ec))
                {
                    RCLCPP_WARN_STREAM(this->get_logger(),"Failed to connect, err: " << ec.message());
                }
            }catch(std::exception e)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(),"Exception connecting to dsrc radio: " << e.what() << " error_code: " << ec.message());
                RCLCPP_ERROR_STREAM(this->get_logger(),"Config:\n\tdsrc_address:" << config_.dsrc_address
                                         << "\n\tdsrc_listening_port:" << config_.dsrc_listening_port
                                         << "\n\tlistening_port:" << config_.listening_port);
            }


            connecting_ = false;
        }));
    }
}

void Node::loadWaveConfig(const std::string &fileName)
{
    RCLCPP_DEBUG_STREAM(this->get_logger(),"Loading wave config");

    const char* schema = "{\n"
                        " \"$schema\":\"http://json-schema.org/draft-06/schema\",\n"
                        " \"title\":\"Wave Config Schema\",\n"
                        " \"description\":\"A simple schema to describe DSRC/Wave messages\",\n"
                        "  \"type\": \"array\",\n"
                        "  \"items\": {\n"
                        "    \"type\": \"object\",\n"
                        "    \"properties\": {\n"
                        "      \"name\": {\n"
                        "        \"description\": \"message type - abbreviated name\",\n"
                        "        \"type\": \"string\"\n"
                        "      },\n"
                        "      \"psid\": {\n"
                        "        \"description\": \"psid assigned to message type in decimal\",\n"
                        "        \"type\": \"string\"\n"
                        "      },\n"
                        "      \"dsrc_id\": {\n"
                        "        \"description\": \"J2735 DSRC id assigned to message type in decimal\",\n"
                        "        \"type\": \"string\"\n"
                        "      },\n"
                        "      \"channel\": {\n"
                        "        \"description\": \"DSRC radio channel assigned to message type in decimal\",\n"
                        "        \"type\": \"string\"\n"
                        "      },\n"
                        "      \"priority\": {\n"
                        "        \"description\": \"WSM Priotiy to use assigned to message type in decimal\",\n"
                        "        \"type\":\"string\"\n"
                        "      }\n"
                        "    },\n"
                        "    \"required\":[\"name\",\"psid\",\"dsrc_id\",\"channel\",\"priority\"]"
                        "  }\n"
                        "}\n";

    std::ifstream file;
    try
    {
        file.open(fileName);
    }
    catch (std::exception& e)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(),"Unable to open file : " << fileName << ", exception: " << e.what());
        return;
    }

    rapidjson::Document sd;
    if(sd.Parse(schema).HasParseError())
    {
        RCLCPP_ERROR_STREAM(this->get_logger(),"Invalid Wave Config Schema");
        return;
    }

    rapidjson::SchemaDocument schemaDocument(sd);
    rapidjson::Document doc;
    rapidjson::IStreamWrapper isw(file);
    if(doc.ParseStream(isw).HasParseError())
    {
        RCLCPP_ERROR_STREAM(this->get_logger(),"Error Parsing Wave Config");
        return;
    }

    rapidjson::SchemaValidator validator(schemaDocument);
    if(!doc.Accept(validator))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(),"Wave Config improperly formatted");
        return;
    }

    for(auto& it : doc.GetArray())
    {
        auto entry = it.GetObject();
        wave_cfg_items_.emplace_back(entry["name"].GetString(),
                                     entry["psid"].GetString(),
                                     entry["dsrc_id"].GetString(),
                                     entry["channel"].GetString(),
                                     entry["priority"].GetString());

    }
}

void Node::handle_on_shutdown()
{
    RCLCPP_INFO_STREAM(this->get_logger(),"Shutdown signal received from DriverApplication");
    if (connect_thread_)
    {
        RCLCPP_INFO_STREAM(this->get_logger(),"Cleaning up connection thread");
        connect_thread_->join();
        connect_thread_.reset();
    }

    RCLCPP_INFO_STREAM(this->get_logger(),"Closing connection to radio");
    dsrc_client_.close();
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(DSRCApplication::Node)
