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

#ifndef SRC_NDNSIM_MODEL_RNTP_UTILS_HPP_
#define SRC_NDNSIM_MODEL_RNTP_UTILS_HPP_


#include "NFD/daemon/face/face-common.hpp"

#include "ns3/nstime.h"

#include <string>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <fstream>

using namespace std;
using namespace ::nfd;

class RntpUtils {
public:
	static void setLogDirPath(const char* logDirPath);
	static void openLogs();
	static void closeLogs();
	static void cleanLogDir();
	static ofstream* getLogMsgInterest();
	static ofstream* getLogMsgInterestBroadcast();
	static ofstream* getLogMsgCapsule();
	static ofstream* getLogMsgCapAck();
	static ofstream* getLogMsgCQUpdate();
	static ofstream* getLogMsgEcho();
	static ofstream* getLogConsumer();
	static ofstream* getLogConsumerQueueSize();
	static ofstream* getLogConsumerReseq();
	static ofstream* getLogProducer();
	static ofstream* getLogRoutes();
	static ofstream* getLogCongestionControl();
	static ofstream* getLogBuffer();
	static ofstream* getLogEnergy();
	static ofstream* getLogOthers();

private:
	static string*   logDirPath;
	static ofstream* logMsgInterest;
	static ofstream* logMsgInterestBroadcast;
	static ofstream* logMsgCapsule;
	static ofstream* logMsgCapAck;
	static ofstream* logMsgCQUpdate;
	static ofstream* logMsgEcho;
	static ofstream* logConsumer;
	static ofstream* logConsumerReseq;
	static ofstream* logConsumerQueueSize;
	static ofstream* logProducer;
	static ofstream* logRoutes;
	static ofstream* logCongestionControl;
	static ofstream* logBuffer;
	static ofstream* logEnergy;
	static ofstream* logOthers;
};

#endif /* SRC_NDNSIM_MODEL_RNTP_UTILS_HPP_ */
