#include <iostream>
#include <functional>
#include "../src/dsrc_client.cpp"

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/schema.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(DSRCClientTest, testSocket)
{
    boost::asio::ip::tcp::endpoint remote_endpoint = boost::asio::ip::tcp::endpoint( boost::asio::ip::address_v4::from_string( "192.168.88.40" ), 5398 );
    ASSERT_EQ("192.168.88.40",remote_endpoint.address().to_string());    
}

TEST(DSRCClientTest,testConnection)
{
    DSRCOBUClient dsrc_client_;
    boost::system::error_code ec;
    ASSERT_FALSE(dsrc_client_.connected());
    auto result = dsrc_client_.connect("192.168.88.40", 1516, 5398, ec);  
    ASSERT_TRUE(result);
    ASSERT_TRUE(dsrc_client_.connected());
    ASSERT_EQ(0,ec.value());
    ASSERT_EQ("Success",ec.message());
}

TEST(DSRCClientTest,testValidateMsgId)
{
    DSRCOBUClient dsrc_client_;
    dsrc_client_.set_wave_file_path("../etc/wave.json"); 
    uint16_t msg_id = 20;    
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

// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}