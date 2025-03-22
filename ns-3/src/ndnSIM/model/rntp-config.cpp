/*
 * This work is licensed under CC BY-NC-SA 4.0
 * (https://creativecommons.org/licenses/by-nc-sa/4.0/).
 * Copyright (c) 2024 Boyang Zhou
 *
 * This file is a part of "Resilient Subpath-Based NDN Transport Protocol (RNTP) for Ad-Hoc Stationary Wireless Sensor Networks"
 * (https://github.com/zhouby-zjl/drtp/).
 * Written by Boyang Zhou (zhouby@zhejianglab.com / zby_zju@163.com)
 *
 */

#include "rntp-config.hpp"
#include <fstream>
#include <regex>
#include <stdlib.h>

using namespace std;

string	 RntpConfig::LOG_DIR = "/tmp/";
string   RntpConfig::STANDARD = "802.11a";
string   RntpConfig::DATA_MODE = "OfdmRate54Mbps";
double	 RntpConfig::TX_POWER_START_IN_DBM = 20;
double	 RntpConfig::TX_POWER_END_IN_DBM = 20;
double	 RntpConfig::RX_GAIN_IN_DBM = 0.0;
uint32_t RntpConfig::N_NODES = 64;
uint32_t RntpConfig::GRID_WIDTH_IN_NODES = 8;
double	 RntpConfig::GRID_DELTA_X = 10.0;
double	 RntpConfig::GRID_DELTA_Y = 10.0;
uint32_t RntpConfig::CONSUMER_NODE_ID = 0;
uint32_t RntpConfig::PRODUCER_NODE_ID = 63;
bool	 RntpConfig::NOISE = true;
string	 RntpConfig::NODE_IDS_UNDER_NOISES = "4,7,9";
double   RntpConfig::NOISE_START_SEC = 5.0;
double   RntpConfig::NOISE_STOP_SEC = 15.0;
double 	 RntpConfig::NOISE_MEAN = 10.0;
double	 RntpConfig::NOISE_VAR = 5.0;
double   RntpConfig::SIM_TIME_IN_SECS = 20;
double	 RntpConfig::EXTENSION_TIME_IN_SECS = 10;
double	 RntpConfig::CAPSULE_PER_HOP_TIMEOUT = 1.0;
uint32_t RntpConfig::CAPSULE_RETRYING_TIMES = 3;
uint32_t RntpConfig::CONGESTION_CONTROL_THRESHOLD = 16;
uint32_t RntpConfig::CONGESTION_CONTROL_INIT_WIN = 1;
uint32_t RntpConfig::THROUGHPUT_QUEUE_SIZE_IN_SECS = 2;
double   RntpConfig::PIAT_ESTIMATION_CONFIDENT_RATIO = 0.9999;
uint32_t RntpConfig::INTEREST_SEND_TIMES = 3;
double	 RntpConfig::ECHO_PERIOD_IN_SECS = 1.0;
double	 RntpConfig::MSG_TIMEOUT_IN_SECS = 3.5;
double	 RntpConfig::INTEREST_CONTENTION_TIME_IN_SECS = 0.005;
double   RntpConfig::CONSUMER_MAX_WAIT_TIME_IN_SECS = 5.0;
double	 RntpConfig::QUALITY_ALPHA = 1.0 / 8.0;
uint32_t RntpConfig::PRODUCER_FREQ = 10;
bool	 RntpConfig::CONSUMER_NEED_TO_TERMINATE_TRANSPORT = false;
double	 RntpConfig::CONSUMER_TERMINATE_TRANSPORT_DELAY_IN_SECS = 100;
double   RntpConfig::ENEGERY_BATTERY_CAPACITY_IN_MAH = 3000;
double   RntpConfig::ENEGERY_BATTERY_VOLTAGE_IN_V = 1.5;
bool	 RntpConfig::TRACE_BATTERY = false;

std::vector<std::pair<std::string, std::string>> RntpConfig::standardModes = {
		{"802.11a", "OfdmRate6Mbps"},
		{"802.11a", "OfdmRate9Mbps"},
		{"802.11a", "OfdmRate12Mbps"},
		{"802.11a", "OfdmRate18Mbps"},
		{"802.11a", "OfdmRate24Mbps"},
		{"802.11a", "OfdmRate36Mbps"},
		{"802.11a", "OfdmRate48Mbps"},
		{"802.11a", "OfdmRate54Mbps"},
		{"802.11b", "DsssRate1Mbps"},
		{"802.11b", "DsssRate2Mbps"},
		{"802.11b", "DsssRate5_5Mbps"},
		{"802.11b", "DsssRate11Mbps"},
		{"802.11g", "ErpOfdmRate6Mbps"},
		{"802.11g", "ErpOfdmRate9Mbps"},
		{"802.11g", "ErpOfdmRate12Mbps"},
		{"802.11g", "ErpOfdmRate18Mbps"},
		{"802.11g", "ErpOfdmRate24Mbps"},
		{"802.11g", "ErpOfdmRate36Mbps"},
		{"802.11g", "ErpOfdmRate48Mbps"},
		{"802.11g", "ErpOfdmRate54Mbps"},
		{"802.11n", "HtMcs0"},
		{"802.11n", "HtMcs1"},
		{"802.11n", "HtMcs2"},
		{"802.11n", "HtMcs3"},
		{"802.11n", "HtMcs4"},
		{"802.11n", "HtMcs5"},
		{"802.11n", "HtMcs6"},
		{"802.11n", "HtMcs7"},
		{"802.11ac", "VhtMcs0"},
		{"802.11ac", "VhtMcs1"},
		{"802.11ac", "VhtMcs2"},
		{"802.11ac", "VhtMcs3"},
		{"802.11ac", "VhtMcs4"},
		{"802.11ac", "VhtMcs5"},
		{"802.11ac", "VhtMcs6"},
		{"802.11ac", "VhtMcs7"},
		{"802.11ac", "VhtMcs8"},
		{"802.11ac", "VhtMcs9"},
		{"802.11ax", "HeMcs0"},
		{"802.11ax", "HeMcs1"},
		{"802.11ax", "HeMcs2"},
		{"802.11ax", "HeMcs3"},
		{"802.11ax", "HeMcs4"},
		{"802.11ax", "HeMcs5"},
		{"802.11ax", "HeMcs6"},
		{"802.11ax", "HeMcs7"},
		{"802.11ax", "HeMcs8"},
		{"802.11ax", "HeMcs9"},
		{"802.11ax", "HeMcs10"},
		{"802.11ax", "HeMcs11"}
};

double RntpConfig::getFreq() {
    double frequency;
    if (STANDARD == "802.11a") {
        frequency = 5.0e9; // 5 GHz
    } else if (STANDARD == "802.11b" || STANDARD == "802.11g" || STANDARD == "802.11n" || STANDARD == "802.11ax") {
        frequency = 2.4e9; // 2.4 GHz
    } else if (STANDARD == "802.11ac") {
        frequency = 5.0e9; // 5 GHz
    } else {
        return -1;
    }
    return frequency;
}

WifiPhyStandard RntpConfig::getStandard() {
    if (STANDARD == "802.11a") {
    	return WIFI_PHY_STANDARD_80211a;
    } else if (STANDARD == "802.11b") {
        return WIFI_PHY_STANDARD_80211b;
    } else if (STANDARD == "802.11g") {
        return WIFI_PHY_STANDARD_80211g;
    } else if (STANDARD == "802.11n") {
        return WIFI_PHY_STANDARD_80211n_2_4GHZ;
    } else if (STANDARD == "802.11ac") {
        return WIFI_PHY_STANDARD_80211ac;
    } else if (STANDARD == "802.11ax") {
        return WIFI_PHY_STANDARD_80211ax_2_4GHZ;
    }
    return WIFI_PHY_STANDARD_UNSPECIFIED;
}

bool RntpConfig::loadConfigFile(string filePath) {
	ifstream f;
	f.open(filePath, ios::in);
	if (!f.is_open()) return false;

	string line;
	regex setPattern("\\s*([A-Za-z0-9_]+)\\s*=\\s*(\\S*)\\s*");
	smatch m;
	while (getline(f, line)) {
		if (regex_match(line, m, setPattern)) {
			string name = m[1].str();
			string value = m[2].str();

			if (name.compare("LOG_DIR") == 0) {
				LOG_DIR = value.c_str();
			} else if (name.compare("STANDARD") == 0) {
				STANDARD = value.c_str();
			} else if (name.compare("DATA_MODE") == 0) {
				DATA_MODE = value.c_str();
			} else if (name.compare("TX_POWER_START_IN_DBM") == 0) {
				TX_POWER_START_IN_DBM = atof(value.c_str());
			} else if (name.compare("TX_POWER_END_IN_DBM") == 0) {
				TX_POWER_END_IN_DBM = atof(value.c_str());
			} else if (name.compare("RX_GAIN_IN_DBM") == 0) {
				RX_GAIN_IN_DBM = atof(value.c_str());
			} else if (name.compare("N_NODES") == 0) {
				N_NODES = atoi(value.c_str());
			} else if (name.compare("GRID_WIDTH_IN_NODES") == 0) {
				GRID_WIDTH_IN_NODES = atoi(value.c_str());
			} else if (name.compare("GRID_DELTA_X") == 0) {
				GRID_DELTA_X = atof(value.c_str());
			} else if (name.compare("GRID_DELTA_Y") == 0) {
				GRID_DELTA_Y = atof(value.c_str());
			} else if (name.compare("CONSUMER_NODE_ID") == 0) {
				CONSUMER_NODE_ID = atoi(value.c_str());
			} else if (name.compare("PRODUCER_NODE_ID") == 0) {
				PRODUCER_NODE_ID = atoi(value.c_str());
			} else if (name.compare("NOISE") == 0) {
				NOISE = (value.compare("true") == 0);
			} else if (name.compare("NODE_IDS_UNDER_NOISES") == 0) {
				NODE_IDS_UNDER_NOISES = value.c_str();
			} else if (name.compare("NOISE_START_SEC") == 0) {
				NOISE_START_SEC = atof(value.c_str());
			} else if (name.compare("NOISE_STOP_SEC") == 0) {
				NOISE_STOP_SEC = atof(value.c_str());
			} else if (name.compare("THROUGHPUT_QUEUE_SIZE_IN_SECS") == 0) {
				THROUGHPUT_QUEUE_SIZE_IN_SECS = atoi(value.c_str());
			} else if (name.compare("PIAT_ESTIMATION_CONFIDENT_RATIO") == 0) {
				PIAT_ESTIMATION_CONFIDENT_RATIO = atof(value.c_str());
			} else if (name.compare("NOISE_MEAN") == 0) {
				NOISE_MEAN = atof(value.c_str());
			} else if (name.compare("NOISE_VAR") == 0) {
				NOISE_VAR = atof(value.c_str());
			} else if (name.compare("SIM_TIME_IN_SECS") == 0) {
				SIM_TIME_IN_SECS = atof(value.c_str());
			} else if (name.compare("EXTENSION_TIME_IN_SECS") == 0) {
				EXTENSION_TIME_IN_SECS = atof(value.c_str());
			} else if (name.compare("CAPSULE_PER_HOP_TIMEOUT") == 0) {
				CAPSULE_PER_HOP_TIMEOUT = atof(value.c_str());
			} else if (name.compare("CAPSULE_RETRYING_TIMES") == 0) {
				CAPSULE_RETRYING_TIMES = atoi(value.c_str());
			} else if (name.compare("CONGESTION_CONTROL_THRESHOLD") == 0) {
				CONGESTION_CONTROL_THRESHOLD = atoi(value.c_str());
			} else if (name.compare("INTEREST_SEND_TIMES") == 0) {
				INTEREST_SEND_TIMES = atoi(value.c_str());
			} else if (name.compare("CONGESTION_CONTROL_INIT_WIN") == 0) {
				CONGESTION_CONTROL_INIT_WIN = atoi(value.c_str());
			} else if (name.compare("ECHO_PERIOD_IN_SECS") == 0) {
				ECHO_PERIOD_IN_SECS = atof(value.c_str());
			} else if (name.compare("MSG_TIMEOUT_IN_SECS") == 0) {
				MSG_TIMEOUT_IN_SECS = atof(value.c_str());
			} else if (name.compare("INTEREST_CONTENTION_TIME_IN_SECS") == 0) {
				INTEREST_CONTENTION_TIME_IN_SECS = atof(value.c_str());
			} else if (name.compare("QUALITY_ALPHA") == 0) {
				QUALITY_ALPHA = atof(value.c_str());
			} else if (name.compare("CONSUMER_MAX_WAIT_TIME_IN_SECS") == 0) {
				CONSUMER_MAX_WAIT_TIME_IN_SECS = atof(value.c_str());
			} else if (name.compare("PRODUCER_FREQ") == 0) {
				PRODUCER_FREQ = atoi(value.c_str());
			} else if (name.compare("ENEGERY_BATTERY_CAPACITY_IN_MAH") == 0) {
				ENEGERY_BATTERY_CAPACITY_IN_MAH = atof(value.c_str());
			} else if (name.compare("ENEGERY_BATTERY_VOLTAGE_IN_V") == 0) {
				ENEGERY_BATTERY_VOLTAGE_IN_V = atof(value.c_str());
			} else if (name.compare("TRACE_BATTERY") == 0) {
				TRACE_BATTERY = (value.compare("true") == 0);
			} else if (name.compare("CONSUMER_NEED_TO_TERMINATE_TRANSPORT") == 0) {
				CONSUMER_NEED_TO_TERMINATE_TRANSPORT = (value.compare("true") == 0);
			} else if (name.compare("CONSUMER_TERMINATE_TRANSPORT_DELAY_IN_SECS") == 0) {
				CONSUMER_TERMINATE_TRANSPORT_DELAY_IN_SECS = atof(value.c_str());
			}
		}
	}

	return true;
}
