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

#include "v2x_ros_driver/udp_radio_client.h"

#include <iostream>
#include <iomanip>
#include <functional>

namespace V2XDriverApplication
{

UdpRadioClient::UdpRadioClient()
    : running_(false)
{
    logger_ = rclcpp::get_logger("udp_radio_client");
}

UdpRadioClient::~UdpRadioClient()
{
    try { close(); } catch (...) {}
}

bool UdpRadioClient::connect(const std::string &remote_address,
                             unsigned short remote_port,
                             unsigned short local_port,
                             boost::system::error_code &ec)
{
    if (running_) return false;

    try
    {
        remote_udp_ep_ = boost::asio::ip::udp::endpoint(
            boost::asio::ip::address::from_string(remote_address), remote_port);
    }
    catch (const boost::system::system_error &e) {
        RCLCPP_WARN_STREAM(logger_, "Was not able to read address " << remote_address << " as IPv4 address. Error : " << e.what() );
        RCLCPP_WARN_STREAM(logger_, "Attempting DNS name resolution ... ");
        try {
             boost::asio::io_context io_context;

            // 1. Create a resolver to handle DNS lookups
            boost::asio::ip::udp::resolver resolver(io_context);
            boost::asio::ip::udp::resolver::results_type endpoints = resolver.resolve(boost::asio::ip::udp::v4(), remote_address, std::stoi(remote_port));
            remote_udp_ep_ = *endpoint.begin();
            RCLCPP_INFO_STREAM(logger_, "Successfully resolved " << remote_address << ":" << remote_port << " to " << remote_udp_ep_.address());
            
        }
        catch( const boost::system::system_error &er) {
            RCLCPP_ERROR_STREAM(logger_, "DNS resolution failed for address " << remote_address << ":" << remote_port << er.what());
            throw er;
        }

    }
    catch (std::exception e)
    {
        ec = boost::asio::error::invalid_argument;
        throw e;
    }

    ec.clear();

    io_.reset(new boost::asio::io_service());
    output_strand_.reset(new boost::asio::io_service::strand(*io_));

    try
    {
        if (udp_listener_) {
            udp_listener_.reset(nullptr);
        }
        udp_listener_.reset(new cav::UDPListener(*io_, local_port));
    }
    catch (boost::system::system_error e)
    {
        ec = e.code();
        return false;
    }
    catch (std::exception e)
    {
        RCLCPP_ERROR_STREAM(logger_, "UdpRadioClient::connect threw exception : " << e.what());
        return false;
    }

    // Connect receive signal to base class process()
    udp_listener_->onReceive.connect(
        [this](const std::shared_ptr<const std::vector<uint8_t>> &data) {
            process(data);
        });

    udp_out_socket_.reset(new boost::asio::ip::udp::socket(*io_, remote_udp_ep_.protocol()));

    work_.reset(new boost::asio::io_service::work(*io_));
    udp_listener_->start();
    io_thread_.reset(new std::thread([this]()
    {
        boost::system::error_code err;
        io_->run(err);
        if (err) { onError(err); }
    }));

    running_ = true;
    onConnect();
    return true;
}

void UdpRadioClient::close()
{
    if (!running_) return;
    running_ = false;
    work_.reset();
    io_->stop();
    io_thread_->join();
    udp_listener_->stop();
    udp_out_socket_.reset();
    onDisconnect();
}

bool UdpRadioClient::sendV2xMessage(const std::shared_ptr<std::vector<uint8_t>> &message)
{
    if (!running_) return false;
    try {
        output_strand_->post([this, message]()
        {
            try
            {
                udp_out_socket_->send_to(boost::asio::buffer(*message), remote_udp_ep_);
            }
            catch (boost::system::system_error error_code)
            {
                onError(error_code.code());
            }
            catch (...)
            {
                onError(boost::asio::error::fault);
            }
        });
        return true;
    }
    catch (std::exception &e) {
        return false;
    }
}

} // namespace V2XDriverApplication