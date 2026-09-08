/*------------------------------------------------------------------------------
* Copyright (C) 2020-2025 LEIDOS.
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

------------------------------------------------------------------------------*/

#include <iostream>
#include <functional>
#include "v2x_ros_driver/udp_radio_client.h"

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/schema.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "v2x_ros_driver/mqtt_radio_client.h"
#include "v2x_ros_driver/v2x_ros_driver_config.h"
#include <mosquitto.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>

TEST(V2XRadioClient, testSocket)
{
    boost::asio::ip::tcp::endpoint remote_endpoint = boost::asio::ip::tcp::endpoint( boost::asio::ip::address_v4::from_string( "192.168.88.40" ), 5398 );
    ASSERT_EQ("192.168.88.40",remote_endpoint.address().to_string());
}

TEST(V2XRadioClient,testConnection)
{
    V2XDriverApplication::UdpRadioClient v2x_radio_client_;
    boost::system::error_code ec;

    ASSERT_FALSE(v2x_radio_client_.connected());
    auto result = v2x_radio_client_.connect("192.168.88.40", 1516, 5398, ec);
    ASSERT_TRUE(result);
    ASSERT_EQ(0,ec.value());
    ASSERT_EQ("Success",ec.message());

    ASSERT_TRUE(v2x_radio_client_.connected());
    result = v2x_radio_client_.connect("192.168.88.40", 1516, 5398);
    ASSERT_FALSE(result);

    v2x_radio_client_.close();
    ASSERT_FALSE(v2x_radio_client_.connected());
    result = v2x_radio_client_.connect("192.168.88.40", 1516, 5398);
    ASSERT_TRUE(result);
    
}

TEST(V2XRadioClient,testConnectionDnsResolution)
{
    V2XDriverApplication::UdpRadioClient v2x_radio_client_;
    boost::system::error_code ec;

    EXPECT_FALSE(v2x_radio_client_.connected());
    auto result = v2x_radio_client_.connect("localhost", 1516, 5398, ec);
    EXPECT_TRUE(result);
    EXPECT_EQ(0,ec.value());
    EXPECT_EQ("Success",ec.message());
    
}

TEST(V2XRadioClient,testConnectionDnsResolutionInvalid)
{
    V2XDriverApplication::UdpRadioClient v2x_radio_client_;
    boost::system::error_code ec;

    EXPECT_FALSE(v2x_radio_client_.connected());
    EXPECT_THROW(v2x_radio_client_.connect("invalid", 1516, 5398, ec), boost::system::system_error);
    
}

TEST(V2XRadioClient,testValidateMsgId)
{
    V2XDriverApplication::UdpRadioClient v2x_radio_client_;
    //No valid msg_id loaded yet
    uint16_t msg_id = 20;
    ASSERT_FALSE(v2x_radio_client_.IsValidMsgID(std::to_string(msg_id)));


    std::string package_share_directory = ament_index_cpp::get_package_share_directory("v2x_ros_driver");

    //load wrong wave config file
    v2x_radio_client_.set_wave_file_path(package_share_directory + "/etc/wave_invalid.json");
    ASSERT_FALSE(v2x_radio_client_.IsValidMsgID(std::to_string(msg_id)));

    //read list of valid msg_id from correct wave.json file
    v2x_radio_client_.set_wave_file_path(package_share_directory + "/etc/wave.json");
    ASSERT_TRUE(v2x_radio_client_.IsValidMsgID(std::to_string(msg_id)));
    msg_id = 31;
    ASSERT_TRUE(v2x_radio_client_.IsValidMsgID(std::to_string(msg_id)));
    msg_id = 18;
    ASSERT_TRUE(v2x_radio_client_.IsValidMsgID(std::to_string(msg_id)));
    msg_id = 19;
    ASSERT_TRUE(v2x_radio_client_.IsValidMsgID(std::to_string(msg_id)));
    msg_id = 240;
    ASSERT_TRUE(v2x_radio_client_.IsValidMsgID(std::to_string(msg_id)));
    msg_id = 241;
    ASSERT_TRUE(v2x_radio_client_.IsValidMsgID(std::to_string(msg_id)));
    msg_id = 242;
    ASSERT_TRUE(v2x_radio_client_.IsValidMsgID(std::to_string(msg_id)));
    msg_id = 243;
    ASSERT_TRUE(v2x_radio_client_.IsValidMsgID(std::to_string(msg_id)));
    msg_id = 896; //invalid msg_id
    ASSERT_FALSE(v2x_radio_client_.IsValidMsgID(std::to_string(msg_id)));
}
TEST(V2XRadioClient,testsendV2xMessage)
{
    V2XDriverApplication::UdpRadioClient v2x_radio_client_;
    std::shared_ptr<std::vector<uint8_t>> messagePtr;
    std::vector<uint8_t> message =
     {0, 243, 124, 29, 89, 212, 226, 212, 58, 179, 169, 197, 168,
     193, 131, 6, 12, 24, 48, 96, 193, 131, 6, 12, 24, 48, 96, 181, 131, 6, 12, 22, 176, 96, 193, 130, 214, 12,
     24, 48, 90, 193, 131, 6, 12, 24, 48, 96, 193, 131, 6, 12, 24, 48, 96, 193, 131, 6, 12, 24, 48, 96, 193, 131,
     6, 12, 24, 48, 54, 167, 46, 184, 245, 201, 221, 207, 134, 92, 125, 52, 239, 220, 24, 118, 211, 186, 254, 238,
     187, 113, 101, 228, 233, 140, 106, 178, 163, 200, 137, 89, 204, 57, 144, 168, 56, 112, 205, 154, 204, 120, 121,
     114, 211, 151, 149, 253, 216, 118, 229, 117, 26, 108, 58, 112, 106, 101, 198};
    messagePtr= std::make_shared<std::vector<uint8_t> >(std::move(message));
    ASSERT_FALSE(v2x_radio_client_.sendV2xMessage(messagePtr));
    auto result = v2x_radio_client_.connect("192.168.88.40", 1516, 5398);
    ASSERT_TRUE(result);
    ASSERT_TRUE(v2x_radio_client_.connected());
    ASSERT_TRUE(v2x_radio_client_.sendV2xMessage(messagePtr));
}
TEST(V2XRadioClient,testIsValidMsgSize)
{
    V2XDriverApplication::UdpRadioClient v2x_radio_client_;
    size_t start_index = 0;

    // Case 1: Message vector too short (msg_vec.size() < 3)
    {
        std::vector<uint8_t> msg_vec = {0, 1};  // Only 2 bytes
        std::vector<uint8_t> entry = {0, 1};
        ASSERT_FALSE(v2x_radio_client_.isValidMsgSize(msg_vec, start_index, entry));
    }

    // Case 2: Short frame (msg_vec.size() < 128)
    // In this case, msg_vec[2] holds the payload length
    {
        std::vector<uint8_t> msg_vec = 
        {0, 32, 59, 3, 128, 56, 0, 20, 53, 84, 110, 235, 28, 175, 212, 133, 230, 190, 195, 243, 28, 
            231, 55, 89, 11, 207, 4, 132, 128, 0, 112, 0, 128, 0, 253, 125, 55, 208, 0, 127, 255, 0, 
            0, 101, 144, 160, 0, 56, 192, 0, 255, 117, 63, 233, 47, 1, 255, 250, 255, 254, 200, 0};

        std::vector<uint8_t> valid_entry = 
        {0, 32, 59, 3, 128, 56, 0, 20, 53, 84, 110, 235, 28, 175, 212, 133, 230, 190, 195, 243, 28, 
            231, 55, 89, 11, 207, 4, 132, 128, 0, 112, 0, 128, 0, 253, 125, 55, 208, 0, 127, 255, 0, 
            0, 101, 144, 160, 0, 56, 192, 0, 255, 117, 63, 233, 47, 1, 255, 250, 255, 254, 200, 0};
        ASSERT_TRUE(v2x_radio_client_.isValidMsgSize(msg_vec, start_index, valid_entry));

        std::vector<uint8_t> invalid_entry = 
        {1, 2, 32, 59, 3, 128, 56, 0, 20, 53, 84, 110, 235, 28, 175, 212, 133, 230, 190, 195, 243, 28, 
            231, 55, 89, 11, 207, 4, 132, 128, 0, 112, 0, 128, 0, 253, 125, 55, 208, 0, 127, 255, 0, 
            0, 101, 144, 160, 0, 56, 192, 0, 255, 117, 63, 233, 47, 1, 255, 250, 255, 254, 200, 0};
        ASSERT_FALSE(v2x_radio_client_.isValidMsgSize(msg_vec, start_index, invalid_entry));
    }

    // Case 3: Long frame (msg_vec.size() > 127)
    // In this case, msg_vec[2] and msg_vec[3] hold the payload length
    {
        std::vector<uint8_t> msg_vec =
        {0, 18, 129, 145, 56, 0, 48, 0, 33, 136, 122, 1, 70, 103, 163, 112, 58, 131, 73, 205, 17, 244, 2, 220, 40, 32, 8, 36, 0, 0, 0, 0, 175, 147, 193, 130, 
            196, 187, 200, 128, 24, 144, 0, 0, 0, 2, 173, 110, 197, 11, 12, 237, 162, 0, 66, 64, 0, 0, 0, 10, 115, 185, 112, 43, 227, 205, 136, 0, 137, 0, 0, 0, 
            0, 40, 146, 224, 112, 175, 239, 42, 72, 43, 8, 0, 0, 0, 0, 145, 227, 50, 136, 210, 214, 115, 192, 176, 100, 0, 5, 15, 36, 20, 132, 0, 0, 0, 0, 70, 
            99, 142, 4, 103, 127, 90, 224, 88, 100, 0, 1, 7, 18, 9, 194, 0, 0, 0, 0, 34, 19, 192, 98, 51, 131, 174, 184, 44, 54, 0, 0, 131, 73, 4, 161, 0, 0, 0,
            0, 16, 114, 93, 25, 25, 199, 215, 172, 86, 29, 0, 0, 65, 107, 17, 32, 0, 32, 194, 64, 82, 64, 0, 0, 0, 7, 139, 186, 40, 15, 197, 20, 31, 21, 136, 
            192, 0, 32, 10, 193, 8, 0, 16, 8, 64, 24, 200, 0, 0, 0, 1, 226, 238, 88, 195, 249, 125, 24, 68, 1, 204, 128, 0, 0, 0, 30, 53, 100, 16, 63, 168, 81, 
            0, 144, 33, 16, 0, 0, 0, 1, 117, 250, 202, 14, 101, 101, 23, 21, 130, 64, 0, 48, 26, 193, 200, 0, 24, 16, 144, 37, 16, 0, 0, 0, 1, 108, 234, 159, 
            142, 98, 37, 10, 5, 129, 192, 0, 48, 41, 32, 82, 32, 0, 0, 0, 2, 199, 196, 214, 28, 192, 202, 20, 11, 2, 128, 0, 96, 98, 64, 180, 64, 0, 0, 0, 3, 
            222, 195, 208, 230, 20, 80, 160, 88, 138, 0, 0, 131, 136, 6, 41, 0, 0, 0, 0, 37, 143, 54, 0, 205, 242, 26, 32, 26, 164, 0, 0, 0, 0, 144, 228, 199, 
            195, 59, 8, 104, 128, 114, 144, 0, 0, 0, 2, 53, 34, 195, 12, 223, 36, 36, 130, 12, 128, 0, 0, 0, 12, 205, 205, 240, 16, 207, 49, 224, 44, 26, 0, 2, 
            2, 137, 3, 217, 0, 0, 0, 0, 25, 172, 25, 132, 33, 165, 101, 209, 88, 60, 0, 4, 4, 44, 56, 128, 2, 2, 68, 4, 92, 128, 0, 0, 0, 25, 154, 30, 32, 33, 
            79, 98, 32};

        std::vector<uint8_t> valid_entry =
        {0, 18, 129, 145, 56, 0, 48, 0, 33, 136, 122, 1, 70, 103, 163, 112, 58, 131, 73, 205, 17, 244, 2, 220, 40, 32, 8, 36, 0, 0, 0, 0, 175, 147, 193, 130, 
            196, 187, 200, 128, 24, 144, 0, 0, 0, 2, 173, 110, 197, 11, 12, 237, 162, 0, 66, 64, 0, 0, 0, 10, 115, 185, 112, 43, 227, 205, 136, 0, 137, 0, 0, 0, 
            0, 40, 146, 224, 112, 175, 239, 42, 72, 43, 8, 0, 0, 0, 0, 145, 227, 50, 136, 210, 214, 115, 192, 176, 100, 0, 5, 15, 36, 20, 132, 0, 0, 0, 0, 70, 
            99, 142, 4, 103, 127, 90, 224, 88, 100, 0, 1, 7, 18, 9, 194, 0, 0, 0, 0, 34, 19, 192, 98, 51, 131, 174, 184, 44, 54, 0, 0, 131, 73, 4, 161, 0, 0, 0,
            0, 16, 114, 93, 25, 25, 199, 215, 172, 86, 29, 0, 0, 65, 107, 17, 32, 0, 32, 194, 64, 82, 64, 0, 0, 0, 7, 139, 186, 40, 15, 197, 20, 31, 21, 136, 
            192, 0, 32, 10, 193, 8, 0, 16, 8, 64, 24, 200, 0, 0, 0, 1, 226, 238, 88, 195, 249, 125, 24, 68, 1, 204, 128, 0, 0, 0, 30, 53, 100, 16, 63, 168, 81, 
            0, 144, 33, 16, 0, 0, 0, 1, 117, 250, 202, 14, 101, 101, 23, 21, 130, 64, 0, 48, 26, 193, 200, 0, 24, 16, 144, 37, 16, 0, 0, 0, 1, 108, 234, 159, 
            142, 98, 37, 10, 5, 129, 192, 0, 48, 41, 32, 82, 32, 0, 0, 0, 2, 199, 196, 214, 28, 192, 202, 20, 11, 2, 128, 0, 96, 98, 64, 180, 64, 0, 0, 0, 3, 
            222, 195, 208, 230, 20, 80, 160, 88, 138, 0, 0, 131, 136, 6, 41, 0, 0, 0, 0, 37, 143, 54, 0, 205, 242, 26, 32, 26, 164, 0, 0, 0, 0, 144, 228, 199, 
            195, 59, 8, 104, 128, 114, 144, 0, 0, 0, 2, 53, 34, 195, 12, 223, 36, 36, 130, 12, 128, 0, 0, 0, 12, 205, 205, 240, 16, 207, 49, 224, 44, 26, 0, 2, 
            2, 137, 3, 217, 0, 0, 0, 0, 25, 172, 25, 132, 33, 165, 101, 209, 88, 60, 0, 4, 4, 44, 56, 128, 2, 2, 68, 4, 92, 128, 0, 0, 0, 25, 154, 30, 32, 33, 
            79, 98, 32};
        ASSERT_TRUE(v2x_radio_client_.isValidMsgSize(msg_vec, start_index, valid_entry));

        std::vector<uint8_t> invalid_entry =
        {1, 2, 18, 129, 145, 56, 0, 48, 0, 33, 136, 122, 1, 70, 103, 163, 112, 58, 131, 73, 205, 17, 244, 2, 220, 40, 32, 8, 36, 0, 0, 0, 0, 175, 147, 193, 130, 
            196, 187, 200, 128, 24, 144, 0, 0, 0, 2, 173, 110, 197, 11, 12, 237, 162, 0, 66, 64, 0, 0, 0, 10, 115, 185, 112, 43, 227, 205, 136, 0, 137, 0, 0, 0, 
            0, 40, 146, 224, 112, 175, 239, 42, 72, 43, 8, 0, 0, 0, 0, 145, 227, 50, 136, 210, 214, 115, 192, 176, 100, 0, 5, 15, 36, 20, 132, 0, 0, 0, 0, 70, 
            99, 142, 4, 103, 127, 90, 224, 88, 100, 0, 1, 7, 18, 9, 194, 0, 0, 0, 0, 34, 19, 192, 98, 51, 131, 174, 184, 44, 54, 0, 0, 131, 73, 4, 161, 0, 0, 0,
            0, 16, 114, 93, 25, 25, 199, 215, 172, 86, 29, 0, 0, 65, 107, 17, 32, 0, 32, 194, 64, 82, 64, 0, 0, 0, 7, 139, 186, 40, 15, 197, 20, 31, 21, 136, 
            192, 0, 32, 10, 193, 8, 0, 16, 8, 64, 24, 200, 0, 0, 0, 1, 226, 238, 88, 195, 249, 125, 24, 68, 1, 204, 128, 0, 0, 0, 30, 53, 100, 16, 63, 168, 81, 
            0, 144, 33, 16, 0, 0, 0, 1, 117, 250, 202, 14, 101, 101, 23, 21, 130, 64, 0, 48, 26, 193, 200, 0, 24, 16, 144, 37, 16, 0, 0, 0, 1, 108, 234, 159, 
            142, 98, 37, 10, 5, 129, 192, 0, 48, 41, 32, 82, 32, 0, 0, 0, 2, 199, 196, 214, 28, 192, 202, 20, 11, 2, 128, 0, 96, 98, 64, 180, 64, 0, 0, 0, 3, 
            222, 195, 208, 230, 20, 80, 160, 88, 138, 0, 0, 131, 136, 6, 41, 0, 0, 0, 0, 37, 143, 54, 0, 205, 242, 26, 32, 26, 164, 0, 0, 0, 0, 144, 228, 199, 
            195, 59, 8, 104, 128, 114, 144, 0, 0, 0, 2, 53, 34, 195, 12, 223, 36, 36, 130, 12, 128, 0, 0, 0, 12, 205, 205, 240, 16, 207, 49, 224, 44, 26, 0, 2, 
            2, 137, 3, 217, 0, 0, 0, 0, 25, 172, 25, 132, 33, 165, 101, 209, 88, 60, 0, 4, 4, 44, 56, 128, 2, 2, 68, 4, 92, 128, 0, 0, 0, 25, 154, 30, 32, 33, 
            79, 98, 32};
        ASSERT_FALSE(v2x_radio_client_.isValidMsgSize(msg_vec, start_index, invalid_entry));
    }
}

// MqttRadioClient tests

TEST(MqttRadioClient, testInitialState)
{
    V2XDriverApplication::MqttRadioClient client;
    ASSERT_FALSE(client.connected());
}

TEST(MqttRadioClient, testSetConfig)
{
    V2XDriverApplication::MqttRadioClient client;
    EXPECT_NO_THROW(client.setMqttConfig("test-client-id", 1, 10, "Ettifos/V2X"));
    EXPECT_NO_THROW(client.setMqttConfig("", 0, 5, "CustomOEM/V2X"));
}

TEST(MqttRadioClient, testCloseWithoutConnect)
{
    V2XDriverApplication::MqttRadioClient client;
    EXPECT_NO_THROW(client.close());
    EXPECT_NO_THROW(client.close()); // double close
    ASSERT_FALSE(client.connected());
}

TEST(MqttRadioClient, testSendWithoutConnect)
{
    // Mirrors the first part of V2XRadioClient::testsendV2xMessage — send before connect
    V2XDriverApplication::MqttRadioClient client;
    std::vector<uint8_t> message =
     {0, 243, 124, 29, 89, 212, 226, 212, 58, 179, 169, 197, 168,
     193, 131, 6, 12, 24, 48, 96, 193, 131, 6, 12, 24, 48, 96, 181, 131, 6, 12, 22, 176, 96, 193, 130, 214, 12,
     24, 48, 90, 193, 131, 6, 12, 24, 48, 96, 193, 131, 6, 12, 24, 48, 96, 193, 131, 6, 12, 24, 48, 96, 193, 131,
     6, 12, 24, 48, 54, 167, 46, 184, 245, 201, 221, 207, 134, 92, 125, 52, 239, 220, 24, 118, 211, 186, 254, 238,
     187, 113, 101, 228, 233, 140, 106, 178, 163, 200, 137, 89, 204, 57, 144, 168, 56, 112, 205, 154, 204, 120, 121,
     114, 211, 151, 149, 253, 216, 118, 229, 117, 26, 108, 58, 112, 106, 101, 198};
    auto messagePtr = std::make_shared<std::vector<uint8_t>>(std::move(message));
    ASSERT_FALSE(client.sendV2xMessage(messagePtr));
}

TEST(MqttRadioClient, testSendNullMessage)
{
    V2XDriverApplication::MqttRadioClient client;
    std::shared_ptr<std::vector<uint8_t>> msg = nullptr;
    ASSERT_FALSE(client.sendV2xMessage(msg));
}

TEST(MqttRadioClient, testSendEmptyMessage)
{
    V2XDriverApplication::MqttRadioClient client;
    auto empty = std::make_shared<std::vector<uint8_t>>(std::vector<uint8_t>{});
    ASSERT_FALSE(client.sendV2xMessage(empty));
}

TEST(MqttRadioClient, testSendSingleByteMessage)
{
    V2XDriverApplication::MqttRadioClient client;
    auto msg = std::make_shared<std::vector<uint8_t>>(std::vector<uint8_t>{0x14});
    ASSERT_FALSE(client.sendV2xMessage(msg));
}

TEST(MqttRadioClient, testConnectWithoutBroker)
{
    V2XDriverApplication::MqttRadioClient client;
    client.setMqttConfig("", 0, 5, "Ettifos/V2X");

    boost::system::error_code ec;
    client.connect("127.0.0.1", 19999, 0, ec);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ASSERT_FALSE(client.connected());
    client.close();
}

TEST(MqttRadioClient, testMultipleInstances)
{
    {
        V2XDriverApplication::MqttRadioClient c1;
        V2XDriverApplication::MqttRadioClient c2;
        V2XDriverApplication::MqttRadioClient c3;
        ASSERT_FALSE(c1.connected());
        ASSERT_FALSE(c2.connected());
        ASSERT_FALSE(c3.connected());
    }
    V2XDriverApplication::MqttRadioClient c4;
    ASSERT_FALSE(c4.connected());
}

TEST(MqttRadioClient, testPolymorphism)
{
    std::unique_ptr<V2XDriverApplication::BaseRadioClient> client =
        std::make_unique<V2XDriverApplication::MqttRadioClient>();
    ASSERT_FALSE(client->connected());
    ASSERT_NE(dynamic_cast<V2XDriverApplication::MqttRadioClient*>(client.get()), nullptr);
}

TEST(MqttRadioClient, testValidateMsgId)
{
    V2XDriverApplication::MqttRadioClient client;

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("v2x_ros_driver");
    client.set_wave_file_path(package_share_directory + "/etc/wave.json");

    ASSERT_TRUE(client.IsValidMsgID("20"));   // BSM
    ASSERT_TRUE(client.IsValidMsgID("18"));   // MAP
    ASSERT_TRUE(client.IsValidMsgID("19"));   // SPaT
    ASSERT_TRUE(client.IsValidMsgID("31"));   // TIM
    ASSERT_TRUE(client.IsValidMsgID("243"));  // MobilityOperation
    ASSERT_FALSE(client.IsValidMsgID("896")); // invalid
}

TEST(MqttRadioClient, testPsidLookup)
{
    V2XDriverApplication::MqttRadioClient client;

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("v2x_ros_driver");
    client.loadWaveConfigIds(package_share_directory + "/etc/wave.json");

    ASSERT_EQ(client.psidForDsrcMsgId("20"), "0020");   // BSM
    ASSERT_EQ(client.psidForDsrcMsgId("243"), "BFEE");  // MobilityOperation
    ASSERT_EQ(client.psidForDsrcMsgId("245"), "8003");  // TrafficControlMessage
    ASSERT_EQ(client.psidForDsrcMsgId("999"), "");       // unknown
}

// Config tests 

TEST(V2XRosDriverConfig, testDefaultProtocol)
{
    V2XDriverApplication::Config cfg;
    ASSERT_EQ(cfg.protocol, "udp");
    ASSERT_EQ(cfg.broker_address, "localhost");
    ASSERT_EQ(cfg.broker_port, 1883);
    ASSERT_EQ(cfg.client_id, "");
    ASSERT_EQ(cfg.qos_default, 0);
    ASSERT_EQ(cfg.reconnect_interval_sec, 5);
    ASSERT_EQ(cfg.mqtt_topic_prefix, "Ettifos/V2X");
}

TEST(V2XRosDriverConfig, testStreamOutput)
{
    V2XDriverApplication::Config cfg;
    cfg.protocol = "mqtt";
    cfg.broker_address = "192.168.88.50";
    cfg.broker_port = 8883;

    std::ostringstream oss;
    oss << cfg;
    std::string output = oss.str();

    ASSERT_NE(output.find("protocol: mqtt"), std::string::npos);
    ASSERT_NE(output.find("broker_address: 192.168.88.50"), std::string::npos);
    ASSERT_NE(output.find("broker_port: 8883"), std::string::npos);
}

// Protocol switching test

TEST(ProtocolFactory, testCreateRadioClient)
{
    auto createClient = [](const std::string &protocol) -> std::unique_ptr<V2XDriverApplication::BaseRadioClient>
    {
        if (protocol == "mqtt")
        {
            auto mqtt = std::make_unique<V2XDriverApplication::MqttRadioClient>();
            mqtt->setMqttConfig("", 0, 5, "Ettifos/V2X");
            return mqtt;
        }
        else
        {
            return std::make_unique<V2XDriverApplication::UdpRadioClient>();
        }
    };

    auto udp_client = createClient("udp");
    ASSERT_NE(dynamic_cast<V2XDriverApplication::UdpRadioClient*>(udp_client.get()), nullptr);
    ASSERT_EQ(dynamic_cast<V2XDriverApplication::MqttRadioClient*>(udp_client.get()), nullptr);

    auto mqtt_client = createClient("mqtt");
    ASSERT_EQ(dynamic_cast<V2XDriverApplication::UdpRadioClient*>(mqtt_client.get()), nullptr);
    ASSERT_NE(dynamic_cast<V2XDriverApplication::MqttRadioClient*>(mqtt_client.get()), nullptr);

    auto default_client = createClient("unknown");
    ASSERT_NE(dynamic_cast<V2XDriverApplication::UdpRadioClient*>(default_client.get()), nullptr);
}

// Test connecting/sending 

class MqttIntegrationTest : public ::testing::Test
{
protected:
    static bool broker_available_;
    static pid_t broker_pid_;

    static void SetUpTestSuite()
    {
        broker_pid_ = fork();
        if (broker_pid_ == 0)
        {
            execlp("mosquitto", "mosquitto", "-p", "11883", nullptr);
            _exit(1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        int status;
        pid_t result = waitpid(broker_pid_, &status, WNOHANG);
        broker_available_ = (result == 0);
    }

    static void TearDownTestSuite()
    {
        if (broker_pid_ > 0)
        {
            kill(broker_pid_, SIGTERM);
            waitpid(broker_pid_, nullptr, 0);
        }
    }
};

bool MqttIntegrationTest::broker_available_ = false;
pid_t MqttIntegrationTest::broker_pid_ = -1;

TEST_F(MqttIntegrationTest, testConnect)
{
    if (!broker_available_) GTEST_SKIP() << "mosquitto not available";

    V2XDriverApplication::MqttRadioClient client;
    client.setMqttConfig("test-connect", 0, 5, "Ettifos/V2X");

    boost::system::error_code ec;
    bool result = client.connect("127.0.0.1", 11883, 0, ec);
    ASSERT_TRUE(result);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ASSERT_TRUE(client.connected());

    client.close();
    ASSERT_FALSE(client.connected());
}

TEST_F(MqttIntegrationTest, testsendV2xMessage)
{
    if (!broker_available_) GTEST_SKIP() << "mosquitto not available";

    // Same message payload as V2XRadioClient::testsendV2xMessage but over MQTT
    V2XDriverApplication::MqttRadioClient client;
    client.setMqttConfig("test-send-v2x", 0, 5, "Ettifos/V2X");

    std::string pkg_dir = ament_index_cpp::get_package_share_directory("v2x_ros_driver");
    client.loadWaveConfigIds(pkg_dir + "/etc/wave.json");

    std::shared_ptr<std::vector<uint8_t>> messagePtr;
    std::vector<uint8_t> message =
     {0, 243, 124, 29, 89, 212, 226, 212, 58, 179, 169, 197, 168,
     193, 131, 6, 12, 24, 48, 96, 193, 131, 6, 12, 24, 48, 96, 181, 131, 6, 12, 22, 176, 96, 193, 130, 214, 12,
     24, 48, 90, 193, 131, 6, 12, 24, 48, 96, 193, 131, 6, 12, 24, 48, 96, 193, 131, 6, 12, 24, 48, 96, 193, 131,
     6, 12, 24, 48, 54, 167, 46, 184, 245, 201, 221, 207, 134, 92, 125, 52, 239, 220, 24, 118, 211, 186, 254, 238,
     187, 113, 101, 228, 233, 140, 106, 178, 163, 200, 137, 89, 204, 57, 144, 168, 56, 112, 205, 154, 204, 120, 121,
     114, 211, 151, 149, 253, 216, 118, 229, 117, 26, 108, 58, 112, 106, 101, 198};
    messagePtr = std::make_shared<std::vector<uint8_t>>(std::move(message));

    // Send before connect should fail
    ASSERT_FALSE(client.sendV2xMessage(messagePtr));

    // Connect to local broker
    boost::system::error_code ec;
    auto result = client.connect("127.0.0.1", 11883, 0, ec);
    ASSERT_TRUE(result);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ASSERT_TRUE(client.connected());

    // Send after connect should succeed
    // Re-create messagePtr since std::move emptied the original
    std::vector<uint8_t> message2 =
     {0, 243, 124, 29, 89, 212, 226, 212, 58, 179, 169, 197, 168,
     193, 131, 6, 12, 24, 48, 96, 193, 131, 6, 12, 24, 48, 96, 181, 131, 6, 12, 22, 176, 96, 193, 130, 214, 12,
     24, 48, 90, 193, 131, 6, 12, 24, 48, 96, 193, 131, 6, 12, 24, 48, 96, 193, 131, 6, 12, 24, 48, 96, 193, 131,
     6, 12, 24, 48, 54, 167, 46, 184, 245, 201, 221, 207, 134, 92, 125, 52, 239, 220, 24, 118, 211, 186, 254, 238,
     187, 113, 101, 228, 233, 140, 106, 178, 163, 200, 137, 89, 204, 57, 144, 168, 56, 112, 205, 154, 204, 120, 121,
     114, 211, 151, 149, 253, 216, 118, 229, 117, 26, 108, 58, 112, 106, 101, 198};
    messagePtr = std::make_shared<std::vector<uint8_t>>(std::move(message2));
    ASSERT_TRUE(client.sendV2xMessage(messagePtr));

    client.close();
}

TEST_F(MqttIntegrationTest, testSendMultipleMessageTypes)
{
    if (!broker_available_) GTEST_SKIP() << "mosquitto not available";

    V2XDriverApplication::MqttRadioClient client;
    client.setMqttConfig("test-multi-send", 0, 5, "Ettifos/V2X");

    std::string pkg_dir = ament_index_cpp::get_package_share_directory("v2x_ros_driver");
    client.loadWaveConfigIds(pkg_dir + "/etc/wave.json");

    boost::system::error_code ec;
    client.connect("127.0.0.1", 11883, 0, ec);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ASSERT_TRUE(client.connected());

    // BSM (DSRCmsgID 20)
    auto bsm = std::make_shared<std::vector<uint8_t>>(
        std::vector<uint8_t>{0x00, 0x14, 0x03, 0xAA, 0xBB, 0xCC});
    ASSERT_TRUE(client.sendV2xMessage(bsm));

    // MAP (DSRCmsgID 18)
    auto map_msg = std::make_shared<std::vector<uint8_t>>(
        std::vector<uint8_t>{0x00, 0x12, 0x03, 0xDD, 0xEE, 0xFF});
    ASSERT_TRUE(client.sendV2xMessage(map_msg));

    // MobilityOperation (DSRCmsgID 243)
    auto mob_op = std::make_shared<std::vector<uint8_t>>(
        std::vector<uint8_t>{0x00, 0xF3, 0x03, 0x77, 0x88, 0x99});
    ASSERT_TRUE(client.sendV2xMessage(mob_op));

    // TrafficControlMessage (DSRCmsgID 245)
    auto tcm = std::make_shared<std::vector<uint8_t>>(
        std::vector<uint8_t>{0x00, 0xF5, 0x03, 0x44, 0x55, 0x66});
    ASSERT_TRUE(client.sendV2xMessage(tcm));

    client.close();
}

TEST_F(MqttIntegrationTest, testInboundMessage)
{
    if (!broker_available_) GTEST_SKIP() << "mosquitto not available";

    V2XDriverApplication::MqttRadioClient client;
    client.setMqttConfig("test-inbound", 0, 5, "Ettifos/V2X");

    std::string pkg_dir = ament_index_cpp::get_package_share_directory("v2x_ros_driver");
    client.set_wave_file_path(pkg_dir + "/etc/wave.json");

    std::vector<std::pair<std::vector<uint8_t>, uint16_t>> received;
    std::mutex mtx;
    client.onMessageReceived.connect(
        [&received, &mtx](const std::vector<uint8_t> &msg, uint16_t id) {
            std::lock_guard<std::mutex> lock(mtx);
            received.push_back({msg, id});
        });

    boost::system::error_code ec;
    client.connect("127.0.0.1", 11883, 0, ec);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ASSERT_TRUE(client.connected());

    struct mosquitto *pub = mosquitto_new("test-publisher", true, nullptr);
    ASSERT_NE(pub, nullptr);
    ASSERT_EQ(mosquitto_connect(pub, "127.0.0.1", 11883, 60), MOSQ_ERR_SUCCESS);

    std::vector<uint8_t> bsm = {0x00, 0x14, 0x03, 0xAA, 0xBB, 0xCC};
    mosquitto_publish(pub, nullptr, "Ettifos/V2X/ind/J2735/20",
                      static_cast<int>(bsm.size()), bsm.data(), 0, false);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    mosquitto_disconnect(pub);
    mosquitto_destroy(pub);

    {
        std::lock_guard<std::mutex> lock(mtx);
        ASSERT_EQ(received.size(), 1u);
        ASSERT_EQ(received[0].second, 20u);
    }

    client.close();
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    bool success = RUN_ALL_TESTS();

    return success;
}