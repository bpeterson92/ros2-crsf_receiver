#ifndef CRSF_PARSER_HPP
#define CRSF_PARSER_HPP

#include <cstring>
#include <chrono>
#include <vector>
#include <algorithm>

#include <CppLinuxSerial/SerialPort.hpp>

#include "utils.h"
#include "crc8.h"
#include "crsf_protocol.h"
#include "crsf_structs.h"

using namespace mn::CppLinuxSerial;
using namespace std;


class CrsfParser
{
public:
    static const unsigned int CRSF_RC_CHANNELS_TIMEOUT_MS = 50;
    static const unsigned int CRSF_LINK_STATISTICS_TIMEOUT_MS = 1000;
    static const unsigned int CRSF_FAILSAFE_STAGE1_MS = 200;

    CrsfParser();
    void parse_incoming_byte(uint8_t b);
    int get_channel_value(unsigned int ch) const { return _channels[ch - 1]; }
    int* get_channels_values() const { return (int*)_channels; };
    crsfLinkStatistics_t get_link_info() const { return link_statistics_packet; };

    bool is_channels_actual();
    bool is_link_statistics_actual();
    bool is_link_up();

private:
    Crc8 _crc;
    
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    // std::vector<uint8_t> rx_buffer;

    crsfLinkStatistics_t link_statistics_packet;
    uint8_t _rx_buf[CRSF_FRAME_SIZE_MAX];
    uint8_t _rx_buf_pos;

    uint32_t _last_receive_time;
    uint32_t _last_channels_time;
    uint32_t _last_link_statistics_time;

    int _channels[CRSF_NUM_CHANNELS];

    void handle_byte_received();
    void shift_rx_buffer(uint8_t cnt);
    void process_packet_in();

    void compile_channels_packet(const crsf_header_t *p);
    void compile_link_statistics_packet(const crsf_header_t *p);
};


#endif 