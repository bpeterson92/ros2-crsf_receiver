#include "utils.h"


clock_t millis(std::chrono::time_point<std::chrono::high_resolution_clock> start_time)
{
    const auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> fs = end - start_time;
    std::chrono::milliseconds d = std::chrono::duration_cast<std::chrono::milliseconds>(fs);
    return d.count();
}


long convert_range(long x, double in_min, double in_max, double out_min, double out_max)
{
    long v = (x - in_min) * (out_max - out_min) / (in_max - 1 - in_min) + out_min;
    return v = std::max(std::min(v, (long)out_max), (long)out_min);
}


CRSFChannels16 convert_to_channels_message(int* channels) {
    CRSFChannels16 message;

    message.ch1 = channels[0];
    message.ch2 = channels[1];
    message.ch3 = channels[2];
    message.ch4 = channels[3];

    message.ch5 = channels[4];
    message.ch6 = channels[5];
    message.ch7 = channels[6];
    message.ch8 = channels[7];

    message.ch9 = channels[8];
    message.ch10 = channels[9];
    message.ch11 = channels[10];
    message.ch12 = channels[11];

    message.ch13 = channels[12];
    message.ch14 = channels[13];
    message.ch15 = channels[14];
    message.ch16 = channels[15];

    return message;
}

CRSFLinkInfo convert_to_link_info(crsfLinkStatistics_t link_info) {
    CRSFLinkInfo message;
    message.rssi_1.data = link_info.uplink_RSSI_1;
    message.rssi_2.data = link_info.uplink_RSSI_2;
    message.link_quality.data = link_info.uplink_Link_quality;
    message.uplink_snr.data = link_info.uplink_SNR;
    message.active_antenna.data = link_info.active_antenna;
    message.rf_mode.data = link_info.rf_Mode;
    message.uplink_tx_power.data = link_info.uplink_TX_Power;
    message.downlink_rssi.data = link_info.downlink_RSSI;
    message.downlink_link_quality.data = link_info.downlink_Link_quality;
    message.downlink_snr.data = link_info.downlink_SNR;
    return message;
}
