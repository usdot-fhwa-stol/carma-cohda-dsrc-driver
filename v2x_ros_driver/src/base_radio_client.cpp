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

#include "v2x_ros_driver/base_radio_client.h"

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/schema.h>
#include <fstream>
#include <iostream>
#include <iomanip>

namespace V2XDriverApplication
{

namespace
{

// Decode a COER length determinant at index i.
// Short form (b < 0x80): the byte itself is the length.
// Long form  (b & 0x7F): low 7 bits give the number of trailing length octets.
// Returns false on overrun / unsupported width.
bool coerLength(const std::vector<uint8_t> &buf, size_t i,
                size_t &value, size_t &n_bytes)
{
    if (i >= buf.size()) { return false; }
    uint8_t b = buf[i];
    if (b < 0x80)
    {
        value = b;
        n_bytes = 1;
        return true;
    }
    size_t n = b & 0x7F;
    if (n == 0 || n > sizeof(size_t) || i + 1 + n > buf.size()) { return false; }
    size_t v = 0;
    for (size_t k = 0; k < n; ++k) { v = (v << 8) | buf[i + 1 + k]; }
    value = v;
    n_bytes = 1 + n;
    return true;
}

// Interpret buf[s:] as an IEEE 1609.2 Ieee1609Dot2Data and, if it ultimately
// carries a J2735 MessageFrame as its unsecuredData, return that MessageFrame.
//
//   03 80 <len> <MessageFrame>             content = unsecuredData  -> MessageFrame
//   03 81 <hashId> <preamble> <data> ...   content = signedData     -> recurse into
//                                                                       tbsData.payload.data
//
// All multi-byte fields are COER-encoded. Returns false for anything that does
// not parse as the above, so non-1609.2 data is left untouched by the caller.
bool extractIeee1609Dot2Payload(const std::vector<uint8_t> &buf, size_t s,
                                std::vector<uint8_t> &out)
{
    if (s + 2 > buf.size() || buf[s] != 0x03) { return false; } // protocolVersion == 3
    uint8_t content = buf[s + 1];

    if (content == 0x80) // unsecuredData (Opaque) -> this is the MessageFrame
    {
        size_t len = 0, n = 0;
        if (!coerLength(buf, s + 2, len, n)) { return false; }
        size_t start = s + 2 + n;
        if (len == 0 || start + len > buf.size()) { return false; }
        out.assign(buf.begin() + start, buf.begin() + start + len);
        return true;
    }

    if (content == 0x81) // signedData -> recurse into the signed payload's data field
    {
        // SignedData ::= SEQUENCE { hashId(1), tbsData, signer, signature }
        // tbsData    ::= SEQUENCE { payload SignedDataPayload, headerInfo }
        // SignedDataPayload preamble: the 'data' OPTIONAL is present when bit 0x40 is set.
        size_t i = s + 2;
        i += 1;                       // skip hashId enumeration (1 byte)
        if (i >= buf.size()) { return false; }
        uint8_t preamble = buf[i++];  // SignedDataPayload preamble byte
        if (!(preamble & 0x40)) { return false; } // 'data' field absent -> no MessageFrame here
        return extractIeee1609Dot2Payload(buf, i, out);
    }

    return false;
}

} // namespace

BaseRadioClient::BaseRadioClient() {}

BaseRadioClient::~BaseRadioClient() {}

bool BaseRadioClient::connect(const std::string &remote_address,
                              unsigned short remote_port,
                              unsigned short local_port)
{
    boost::system::error_code ignored_ec;
    return connect(remote_address, remote_port, local_port, ignored_ec);
}

//  IEEE 1609.2 security-header stripping

bool BaseRadioClient::stripIeee1609Dot2Header(const std::vector<uint8_t> &in,
                                              std::vector<uint8_t> &out)
{
    // The IEEE 1609.2 envelope may sit behind a transport/capture header whose
    // length varies by source: the raw radio feed prepends only a few bytes of
    // framing, but the Commsignia "capture" feed prepends ~48 bytes of metadata
    // (record type, sequence, timestamps, interface info) before the message.
    // So scan the whole datagram for the start of the SPDU rather than assuming
    // a fixed offset or a small window. A successful strip requires both a
    // well-formed envelope AND that the recovered MessageFrame begins with a
    // configured DSRCmsgID; that guard keeps capture-metadata records, bare
    // MessageFrames, and certificate/signature bytes from ever being
    // misidentified and altered.
    for (size_t s = 0; s + 3 < in.size(); ++s)
    {
        std::vector<uint8_t> frame;
        if (!extractIeee1609Dot2Payload(in, s, frame)) { continue; }
        if (frame.size() < 3) { continue; }

        auto msg_id = (static_cast<uint16_t>(frame[0]) << 8) | static_cast<uint16_t>(frame[1]);
        if (IsValidMsgID(std::to_string(msg_id)))
        {
            RCLCPP_DEBUG_STREAM(logger_, "BaseRadioClient::process() : IEEE 1609.2 security "
                                         "header detected; stripped to inner MessageFrame ("
                                         << frame.size() << " bytes).");
            out = std::move(frame);
            return true;
        }
    }
    return false;
}

//  Shared message processing

void BaseRadioClient::process(const std::shared_ptr<const std::vector<uint8_t>> &data)
{
    if (!data || data->size() < 3)
    {
        RCLCPP_WARN_STREAM(logger_, "Received empty or insufficient data, nothing to process.");
        return;
    }

    // J3224 SDSM and other messages may arrive inside an IEEE 1609.2 security
    // envelope (signed: header + certificate + signature). The validation below
    // expects a bare J2735 MessageFrame, so a wrapped message would fail
    // isValidMsgSize() -- the trailing certificate/signature bytes inflate the
    // frame -- and be discarded. If a security header is present, strip it down
    // to the inner MessageFrame first; if it is absent, 'entry' stays bound to
    // the original buffer and the logic below is unchanged, so un-wrapped
    // traffic is processed exactly as if it had arrived without a security envelope.
    std::vector<uint8_t> stripped;
    {
        auto t0 = std::chrono::steady_clock::now();
        bool did_strip = stripIeee1609Dot2Header(*data, stripped);
        auto elapsed_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - t0).count());

        ++strip_call_count_;
        strip_total_ns_ += elapsed_ns;
        if (elapsed_ns > strip_max_ns_) strip_max_ns_ = elapsed_ns;

        static rclcpp::Clock throttle_clock{RCL_SYSTEM_TIME};
        RCLCPP_DEBUG_STREAM_THROTTLE(
            logger_, throttle_clock, 5000,
            "stripIeee1609Dot2Header stats — calls: " << strip_call_count_
            << "  avg: " << (strip_total_ns_ / strip_call_count_) << " ns"
            << "  max: " << strip_max_ns_ << " ns");

        if (!did_strip) stripped.clear();
    }
    const std::vector<uint8_t> &entry = stripped.empty() ? *data : stripped;

    for (size_t i = 0; i < entry.size() - 3; i++)
    {
        auto msg_id = (static_cast<uint16_t>(entry[i]) << 8) | static_cast<uint16_t>(entry[i + 1]);
        if (!IsValidMsgID(std::to_string(msg_id))) { continue; }

        if ((i + short_frame_) >= entry.size()) {
            break;
        }

        auto start_index = i;
        std::vector<uint8_t> msg_vec(entry.begin() + start_index, entry.end());

        if (msg_vec.size() > 16383) { // lengths greater than 16383 (0x3FFF) are encoded by splitting up the message into discrete chunks, each with its own length marker
            RCLCPP_WARN_STREAM(logger_, "BaseRadioClient::process() : discarding received message with length field longer than 16383.");
            break;
        }

        if (!isValidMsgSize(msg_vec, start_index, entry)) {
            RCLCPP_WARN_STREAM(logger_, "Size in possible MessageFrame does not match actual data size. Checking rest of data.");
            continue;
        }

        bool shouldProcess = !isPossiblePSID(std::to_string(msg_id)) ||
                             !isValidMsgAssumingBSMPSID(start_index, entry);

        if (shouldProcess) {
            onMessageReceived(msg_vec, msg_id);
            break;
        } else {
            RCLCPP_WARN_STREAM(logger_, "PSID found, parsing rest of data for MessageID.");
            continue;
        }
    }
}

//  Message-size validation

bool BaseRadioClient::isValidMsgSize(const std::vector<uint8_t> &msg_vec,
                                     size_t start_index,
                                     const std::vector<uint8_t> &entry)
{
    // If message vector is 128 bytes or larger, length field will be 2 bytes
    if (msg_vec.size() > 127)
    {
        auto tmp_start_index = start_index + long_frame_;
        std::vector<uint8_t> long_vec(entry.begin() + tmp_start_index, entry.end());
        // Allows creation of msg_size from two bytes. e.g., [0 20 129 9 ...], we want to combine bytes 2 and 3 from this vector to create msg_size = 0x0109.
        // msg_vec[2] is hex value of 129 (0x81) and msg_vec[3] is hex value of 9 (0x09).
        // Per IEEE 1609.3, we will always have a value of "8" as the most significant value in the message size, if payload size > 128 bytes.
        // So, we bitwise AND (&) the value of msg_vec[2] with 0x7f (01111111) to drop the most significant bit, turning 0x81 (10000001) to 0x01 (00000001).
        // Because we need to merge two bytes, we cast the values to an unsigned 16-bit integer.
        // First byte is shifted to the left 8 bits (0x0001 is now 0x0100), followed by a bitwise OR (|) with msg_vec[3], combining 0x0100 and 0x0009 to create 0x0109.
        auto msg_size = (static_cast<uint16_t>(msg_vec[2] & 0x7F) << 8) | msg_vec[3];
        return (msg_size == long_vec.size());
    }
    // If message vector is smaller than 128 bytes, length field will be 1 byte. Additional check to make sure msg_vec[2] exists.
    else if (msg_vec.size() < 128 && msg_vec.size() >= 3)
    {
        auto tmp_start_index = start_index + short_frame_;
        std::vector<uint8_t> short_vec(entry.begin() + tmp_start_index, entry.end());
        auto msg_size = msg_vec[2];
        return (msg_size == short_vec.size());
    }
    return false;
}

//  PSID / DSRCmsgID checks

bool BaseRadioClient::isPossiblePSID(const std::string &msg_id)
{
    if (wave_cfg_psids_.empty())
    {
        if (wave_file_path.empty()) return false;
        loadWaveConfigIds(wave_file_path);
    }

    for (const auto &psid : wave_cfg_psids_)
    {
        auto psid_value = std::stoi(psid, nullptr, 16);
        auto psidInt = std::to_string(psid_value);
        if (msg_id == psidInt) return true;
    }
    return false;
}

bool BaseRadioClient::isValidMsgAssumingBSMPSID(size_t start_index,
                                                 const std::vector<uint8_t> &entry)
{
    for (auto i = start_index; i < start_index + 6; i++)
    {
        auto element_id = (static_cast<uint16_t>(entry[i]) << 8) | static_cast<uint16_t>(entry[i+1]);
        if (element_id == 896) // 0x0380
        {
            auto element_id_index = i;
            for (auto j = element_id_index; j < element_id_index + 6; j++)
            {
                auto possible_msg_id = (static_cast<uint16_t>(entry[j]) << 8) | static_cast<uint16_t>(entry[j+1]);
                if (possible_msg_id == 20) return true; // BSM DSRCmsgID
            }
        }
    }
    return false;
}

bool BaseRadioClient::IsValidMsgID(const std::string &msg_id)
{
    if (wave_cfg_dsrc_msg_ids_.empty())
    {
        if (wave_file_path.empty()) return false;
        loadWaveConfigIds(wave_file_path);
    }

    for (const auto &dsrc_msg_id : wave_cfg_dsrc_msg_ids_)
    {
        if (msg_id == dsrc_msg_id) return true;
    }
    return false;
}

//  Wave config loading

void BaseRadioClient::set_wave_file_path(const std::string &path)
{
    wave_file_path = path;
}

void BaseRadioClient::loadWaveConfigIds(const std::string &fileName)
{
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
        "      \"dsrc_msg_id\": {\n"
        "        \"description\": \"J2735 DSRCmsgID assigned to message type in decimal\",\n"
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
        "    \"required\":[\"name\",\"psid\",\"dsrc_msg_id\",\"channel\",\"priority\"]"
        "  }\n"
        "}\n";

    std::ifstream file;
    try
    {
        file.open(fileName);
    }
    catch (const std::ifstream::failure &e)
    {
        RCLCPP_ERROR_STREAM(logger_, "Unable to open file : " << fileName << ", exception: " << e.what());
        return;
    }

    rapidjson::Document sd;
    if (sd.Parse(schema).HasParseError())
    {
        RCLCPP_ERROR_STREAM(logger_, "Invalid Wave Config Schema");
        return;
    }

    rapidjson::SchemaDocument schemaDocument(sd);
    rapidjson::Document doc;
    rapidjson::IStreamWrapper isw(file);
    if (doc.ParseStream(isw).HasParseError())
    {
        RCLCPP_ERROR_STREAM(logger_, "Error Parsing Wave Config");
        return;
    }

    rapidjson::SchemaValidator validator(schemaDocument);
    if (!doc.Accept(validator))
    {
        RCLCPP_ERROR_STREAM(logger_, "Wave Config improperly formatted");
        return;
    }

    for (auto &it : doc.GetArray())
    {
        auto entry = it.GetObject();
        wave_cfg_dsrc_msg_ids_.emplace_back(entry["dsrc_msg_id"].GetString());
        wave_cfg_psids_.emplace_back(entry["psid"].GetString());
    }
}

//  PSID lookup helper

std::string BaseRadioClient::psidForDsrcMsgId(const std::string &dsrc_msg_id) const
{
    for (size_t i = 0; i < wave_cfg_dsrc_msg_ids_.size(); ++i)
    {
        if (wave_cfg_dsrc_msg_ids_[i] == dsrc_msg_id)
        {
            return wave_cfg_psids_[i];
        }
    }
    return "";
}

}
