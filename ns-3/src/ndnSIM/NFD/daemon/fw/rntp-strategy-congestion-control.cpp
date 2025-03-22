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

#include "ns3/nstime.h"
#include "ns3/ndnSIM/ndn-cxx/lp/tags.hpp"
#include "ns3/ndnSIM/helper/ndn-stack-helper.hpp"
#include "ns3/ndnSIM/ndn-cxx/encoding/block-helpers.hpp"
#include "ns3/ndnSIM/ndn-cxx/name.hpp"
#include "ns3/rng-seed-manager.h"

#include "NFD/daemon/fw/forwarder.hpp"
#include "NFD/daemon/face/face-common.hpp"
#include "model/ndn-net-device-transport.hpp"
#include "NFD/daemon/table/pit-entry.hpp"
#include "NFD/daemon/table/pit-in-record.hpp"
#include "NFD/daemon/table/cs-policy-priority-fifo.hpp"

#include "ns3/ndnSIM/model/generic-log.hpp"
#include "ns3/ndnSIM/ndn-cxx/lp/tags.hpp"
#include "ns3/wifi-tx-vector.h"

#include "ns3/wifi-utils.h"
#include "ns3/double.h"
#include <float.h>
#include "rntp-strategy.hpp"

#include <algorithm>

#include "ns3/ndnSIM/model/rntp-utils.hpp"

using namespace std;
using namespace ns3;

namespace nfd {
namespace fw {

void RntpCongestionControl::init(TransportStates* ts,  NodeInfo* nodeInfo) {
	ts->window = nodeInfo->congestionControlInitWin;
	ts->slowStartThres = nodeInfo->congestionControlSlowStartThres;
	log("Init", ts, nodeInfo);
}

void RntpCongestionControl::onCapAckReceived(TransportStates* ts,  NodeInfo* nodeInfo) {
	if (ts->window >= 1 && ts->window < ts->slowStartThres) {
		ts->window *= 2;
	} else {
		++ts->window;
		++ts->slowStartThres;
	}
	log("AckReceived", ts, nodeInfo);
}
void RntpCongestionControl::onCapAckTimeout(TransportStates* ts,  NodeInfo* nodeInfo, int nextHopID) {
	if (nextHopID == -1) {
		ts->window = 0;
		log("NoRoute", ts, nodeInfo);
		return;
	}
	if (ts->window > 0) {
		ts->window /= 2;
	}
	if (ts->window == 0) {
		ts->window = 1;
	}
	ts->slowStartThres /= 2;
	if (ts->slowStartThres == 0) {
		ts->slowStartThres = 1;
	}
	log("AckTimeout", ts, nodeInfo);
}

void RntpCongestionControl::onChannelWaken(TransportStates* ts,  NodeInfo* nodeInfo, int fromNodeID) {
	if (ts->window == 0) {
		ts->window = nodeInfo->congestionControlInitWin;
		ts->slowStartThres = nodeInfo->congestionControlSlowStartThres;
	}
	log("Waken", ts, nodeInfo);
}

void RntpCongestionControl::log(string reason, TransportStates* ts,  NodeInfo* nodeInfo) {
	ofstream* log = RntpUtils::getLogCongestionControl();
	*log << nodeInfo->nodeID << "," << Simulator::Now() << "," << reason << "," << ts->window
			<< "," << ts->slowStartThres << ","
			<< ts->capSendQueue.countElements() << endl;
}

}
}
