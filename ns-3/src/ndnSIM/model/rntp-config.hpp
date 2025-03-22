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

#ifndef SRC_NDNSIM_MODEL_RNTP_CONFIG_HPP_
#define SRC_NDNSIM_MODEL_RNTP_CONFIG_HPP_


#include <string>
#include <vector>
#include "ns3/wifi-phy-standard.h"

using namespace std;
using namespace ns3;

class RntpConfig {
public:
	static bool loadConfigFile(string filePath);
	static double getFreq();
	static WifiPhyStandard getStandard();

	static string	LOG_DIR;

	static string   STANDARD;
	static string   DATA_MODE;
	static double	TX_POWER_START_IN_DBM;
	static double	TX_POWER_END_IN_DBM;
	static double	RX_GAIN_IN_DBM;

	static uint32_t N_NODES;
	static uint32_t GRID_WIDTH_IN_NODES;
	static double	GRID_DELTA_X;
	static double	GRID_DELTA_Y;
	static uint32_t CONSUMER_NODE_ID;
	static uint32_t PRODUCER_NODE_ID;

	static bool		NOISE;
	static string	NODE_IDS_UNDER_NOISES;
	static double	NOISE_START_SEC;
	static double	NOISE_STOP_SEC;
	static double 	NOISE_MEAN;
	static double	NOISE_VAR;

	static double	SIM_TIME_IN_SECS;
	static double	EXTENSION_TIME_IN_SECS;

	static double	CAPSULE_PER_HOP_TIMEOUT;
	static uint32_t	CAPSULE_RETRYING_TIMES;
	static uint32_t CONGESTION_CONTROL_THRESHOLD;
	static uint32_t CONGESTION_CONTROL_INIT_WIN;

	static uint32_t THROUGHPUT_QUEUE_SIZE_IN_SECS;
	static double   PIAT_ESTIMATION_CONFIDENT_RATIO;

	static uint32_t INTEREST_SEND_TIMES;

	static double	ECHO_PERIOD_IN_SECS;

	static double	MSG_TIMEOUT_IN_SECS;
	static double	INTEREST_CONTENTION_TIME_IN_SECS;
	static double	QUALITY_ALPHA;

	static double   CONSUMER_MAX_WAIT_TIME_IN_SECS;

	static uint32_t CACHE_MAX_SIZE_IN_PACKETS;
	static uint32_t PRODUCER_FREQ;
	static bool		CONSUMER_NEED_TO_TERMINATE_TRANSPORT;
	static double	CONSUMER_TERMINATE_TRANSPORT_DELAY_IN_SECS;

	static double  ENEGERY_BATTERY_CAPACITY_IN_MAH;
	static double  ENEGERY_BATTERY_VOLTAGE_IN_V;

	static bool	   TRACE_BATTERY;

	static std::vector<std::pair<std::string, std::string>> standardModes;
};

#endif /* SRC_NDNSIM_MODEL_RNTP_CONFIG_HPP_ */
