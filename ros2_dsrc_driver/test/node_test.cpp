
/*------------------------------------------------------------------------------
* Copyright (C) 2020-2021 LEIDOS.
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
#include "../src/dsrc_client.cpp"

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/schema.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

TEST(DSRCClientTest, testSocket)
{
    boost::asio::ip::tcp::endpoint remote_endpoint = boost::asio::ip::tcp::endpoint( boost::asio::ip::address_v4::from_string( "192.168.88.40" ), 5398 );
    ASSERT_EQ("192.168.88.40",remote_endpoint.address().to_string());    
}

TEST(DSRCClientTest,testConnection)
{
    DSRCApplication::DSRCOBUClient dsrc_client_;
    boost::system::error_code ec;
    
    ASSERT_FALSE(dsrc_client_.connected());
    auto result = dsrc_client_.connect("192.168.88.40", 1516, 5398, ec);  
    ASSERT_TRUE(result);
    ASSERT_EQ(0,ec.value());
    ASSERT_EQ("Success",ec.message());
    
    ASSERT_TRUE(dsrc_client_.connected());
    result = dsrc_client_.connect("192.168.88.40", 1516, 5398); 
    ASSERT_FALSE(result);
    
    dsrc_client_.close();
    ASSERT_FALSE(dsrc_client_.connected());
    result = dsrc_client_.connect("192.168.88.40", 1516, 5398); 
    ASSERT_TRUE(result);
}

TEST(DSRCClientTest,testValidateMsgId)
{
    DSRCApplication::DSRCOBUClient dsrc_client_;
    //No valid msg_id loaded yet
    uint16_t msg_id = 20; 
    ASSERT_FALSE(dsrc_client_.IsValidMsgID(std::to_string(msg_id)));

    //load wrong wave config file
    dsrc_client_.set_wave_file_path("etc/wave_invalid.json"); 
    ASSERT_FALSE(dsrc_client_.IsValidMsgID(std::to_string(msg_id)));

    //read list of valid msg_id from correct wave.json file
    dsrc_client_.set_wave_file_path("etc/wave.json"); 
    ASSERT_TRUE(dsrc_client_.IsValidMsgID(std::to_string(msg_id)));
    msg_id = 31;           
    ASSERT_TRUE(dsrc_client_.IsValidMsgID(std::to_string(msg_id)));
    msg_id = 18;    
    ASSERT_TRUE(dsrc_client_.IsValidMsgID(std::to_string(msg_id)));
    msg_id = 19;    
    ASSERT_TRUE(dsrc_client_.IsValidMsgID(std::to_string(msg_id)));
    msg_id = 240;    
    ASSERT_TRUE(dsrc_client_.IsValidMsgID(std::to_string(msg_id)));
    msg_id = 241;    
    ASSERT_TRUE(dsrc_client_.IsValidMsgID(std::to_string(msg_id)));
    msg_id = 242;    
    ASSERT_TRUE(dsrc_client_.IsValidMsgID(std::to_string(msg_id)));
    msg_id = 243;    
    ASSERT_TRUE(dsrc_client_.IsValidMsgID(std::to_string(msg_id)));
    msg_id = 896; //invalid msg_id
    ASSERT_FALSE(dsrc_client_.IsValidMsgID(std::to_string(msg_id)));
}
TEST(DSRCClientTest,testSendDsrcMessage)
{
    DSRCApplication::DSRCOBUClient dsrc_client_;
    std::shared_ptr<std::vector<uint8_t>> messagePtr;
    std::vector<uint8_t> message =
     {0, 243, 124, 29, 89, 212, 226, 212, 58, 179, 169, 197, 168, 
     193, 131, 6, 12, 24, 48, 96, 193, 131, 6, 12, 24, 48, 96, 181, 131, 6, 12, 22, 176, 96, 193, 130, 214, 12,
     24, 48, 90, 193, 131, 6, 12, 24, 48, 96, 193, 131, 6, 12, 24, 48, 96, 193, 131, 6, 12, 24, 48, 96, 193, 131,
     6, 12, 24, 48, 54, 167, 46, 184, 245, 201, 221, 207, 134, 92, 125, 52, 239, 220, 24, 118, 211, 186, 254, 238,
     187, 113, 101, 228, 233, 140, 106, 178, 163, 200, 137, 89, 204, 57, 144, 168, 56, 112, 205, 154, 204, 120, 121,
     114, 211, 151, 149, 253, 216, 118, 229, 117, 26, 108, 58, 112, 106, 101, 198};
    messagePtr= std::make_shared<std::vector<uint8_t> >(std::move(message));
    ASSERT_FALSE(dsrc_client_.sendDsrcMessage(messagePtr));   
    auto result = dsrc_client_.connect("192.168.88.40", 1516, 5398); 
    ASSERT_TRUE(result);
    ASSERT_TRUE(dsrc_client_.connected());
    ASSERT_TRUE(dsrc_client_.sendDsrcMessage(messagePtr));
}


int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    bool success = RUN_ALL_TESTS();

    return success;
} 
