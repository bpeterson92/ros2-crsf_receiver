#include "crsf_parser.h"


CrsfParser::CrsfParser() : _crc(0xd5)
{
    _linkIsUp = false;
    _lastChannelsPacket = 0;
    _last_receive_time = 0;
    start_time = std::chrono::high_resolution_clock::now();
}


void CrsfParser::parse_incoming_byte(uint8_t b)
{
    _rx_buf[_rx_buf_pos++] = b;
    
    bool reprocess;
    do
    {
        reprocess = false;
        if (_rx_buf_pos > 1)
        {
            uint8_t len = _rx_buf[1];
            if (len < 3 || len > (CRSF_MAX_PAYLOAD_LEN + 2))
            {
                shift_rx_buffer(1);
                reprocess = true;
            }

            else if (_rx_buf_pos >= (len + 2))
            {
                uint8_t in_crc = _rx_buf[2 + len - 1];
                uint8_t crc = _crc.calc(&_rx_buf[2], len - 1);
                if (crc == in_crc)
                {
                    process_packet_in(len);
                    shift_rx_buffer(len + 2);
                    reprocess = true;
                }
                else
                {
                    shift_rx_buffer(1);
                    reprocess = true;
                }
            } 
        }
    } while (reprocess);


    if (_rx_buf_pos == (sizeof(_rx_buf) / sizeof(_rx_buf[0])))
    {
        _rx_buf_pos = 0;
    }

    check_packet_timeout();
    check_link_down();
}


void CrsfParser::process_packet_in(uint8_t len)
{   
    _last_receive_time = millis(this->start_time);
    const crsf_header_t *hdr = (crsf_header_t *)_rx_buf;
    if (hdr->device_addr == CRSF_ADDRESS_FLIGHT_CONTROLLER)
    {
        switch (hdr->type)
        {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            compile_channels_packet(hdr);
            break;
        case CRSF_FRAMETYPE_LINK_STATISTICS:
            compile_link_statistics_packet(hdr);
            break;
        }
    }
}

void CrsfParser::check_packet_timeout()
{
    if (_rx_buf_pos > 0 && millis(this->start_time) - _last_receive_time > CRSF_PACKET_TIMEOUT_MS)
    {
        while (_rx_buf_pos)
        {
            shift_rx_buffer(1);
        }
    }
}

void CrsfParser::check_link_down()
{
    if (_linkIsUp && millis(this->start_time) - _lastChannelsPacket > CRSF_FAILSAFE_STAGE1_MS)
    {
        _linkIsUp = false;
    }
}

void CrsfParser::shift_rx_buffer(uint8_t cnt)
{
    if (cnt >= _rx_buf_pos)
    {
        _rx_buf_pos = 0;
        return;
    }

    // Otherwise do the slow shift down
    uint8_t *src = &_rx_buf[cnt];
    uint8_t *dst = &_rx_buf[0];
    _rx_buf_pos -= cnt;
    uint8_t left = _rx_buf_pos;
    while (left--)
        *dst++ = *src++;
}

void CrsfParser::compile_channels_packet(const crsf_header_t *p)
{
    crsf_channels_t *ch = (crsf_channels_t *)&p->data;
    _channels[0] = ch->ch0;
    _channels[1] = ch->ch1;
    _channels[2] = ch->ch2;
    _channels[3] = ch->ch3;
    _channels[4] = ch->ch4;
    _channels[5] = ch->ch5;
    _channels[6] = ch->ch6;
    _channels[7] = ch->ch7;
    _channels[8] = ch->ch8;
    _channels[9] = ch->ch9;
    _channels[10] = ch->ch10;
    _channels[11] = ch->ch11;
    _channels[12] = ch->ch12;
    _channels[13] = ch->ch13;
    _channels[14] = ch->ch14;
    _channels[15] = ch->ch15;

    for (unsigned int i = 0; i < CRSF_NUM_CHANNELS; ++i)
    {
        int val = _channels[i] - CRSF_CHANNEL_VALUE_MID;
        _channels[i] = convert_range(val, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, -100, 100);
    }

    _linkIsUp = true;
    _lastChannelsPacket = millis(this->start_time);
}

void CrsfParser::compile_link_statistics_packet(const crsf_header_t *p)
{
    const crsfLinkStatistics_t *link = (crsfLinkStatistics_t *)p->data;
    memcpy(&link_statistics_packet, link, sizeof(link_statistics_packet));
}
