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

void RntpStrategy::logMsgInterest(bool isRecv, InterestInfo& info, PhyInfo* phyInfo) {
	auto log = RntpUtils::getLogMsgInterest();
	*log << this->nodeInfo->nodeID << "," << Simulator::Now() << "," << (phyInfo != NULL ? phyInfo->snr : -1) << ",";
	if (isRecv) {
		*log << "r,";
	} else {
		*log << "s,";
	}
	*log << info.consumerNodeID << "," << info.nextHopNodeID << "," << info.prefix << endl;
}

void RntpStrategy::logMsgInterestBroadcast(bool isRecv, InterestBroadcastInfo& info, PhyInfo* phyInfo) {
	auto log = RntpUtils::getLogMsgInterestBroadcast();
	*log << this->nodeInfo->nodeID << "," << Simulator::Now() << "," << (phyInfo != NULL ? phyInfo->snr : -1) << ",";
	if (isRecv) {
		*log << "r,";
	} else if (!info.end) {
		*log << "s,";
	} else {
		*log << "t,";
	}
	*log << info.consumerNodeID << "," << info.transHopNodeID << "," << info.producerPrefix << "," <<
			info.hopCount << "," <<	info.nonce << ",";
	bool begin = true;
	for (auto iter = info.visitedNodeIDs.begin(); iter != info.visitedNodeIDs.end(); ++iter) {
		if (begin) {
			*log << *iter;
			begin = false;
		} else {
			*log << "|" << *iter;
		}
	}
	*log << ",";
	begin = true;
	for (auto iter = info.channelQualities.begin(); iter != info.channelQualities.end(); ++iter) {
		if (begin) {
			*log << *iter;
			begin = false;
		} else {
			*log << "|" << *iter;
		}
	}
	*log << endl;
}

void RntpStrategy::logMsgCapsule(bool isRecv, CapsuleInfo& info, PhyInfo* phyInfo) {
	auto log = RntpUtils::getLogMsgCapsule();
	*log << this->nodeInfo->nodeID << "," << Simulator::Now() << "," << (phyInfo != NULL ? phyInfo->snr : -1) << ",";
	if (isRecv) {
		*log << "r,";
	} else {
		*log << "s,";
	}

	*log << info.transHopNodeID << "," << info.prefix << "," << info.dataID << ",";
	bool begin = true;
	for (auto iter = info.nodeIDs.begin(); iter != info.nodeIDs.end(); ++iter) {
		if (begin) {
			*log << *iter;
			begin = false;
		} else {
			*log << "|" << *iter;
		}
	}
	*log << "," << info.nHops;
	*log << endl;
}

void RntpStrategy::logMsgCapAck(bool isRecv, CapsuleACKInfo& info, PhyInfo* phyInfo) {
	auto log = RntpUtils::getLogMsgCapAck();
	*log << this->nodeInfo->nodeID << "," << Simulator::Now() << "," << (phyInfo != NULL ? phyInfo->snr : -1) << ",";
	if (isRecv) {
		*log << "r,";
	} else {
		*log << "s,";
	}
	*log << info.consumerNodeID << ",";
	bool begin = true;
	for (uint32_t upstreamNodeID : info.upstreamNodeIDs) {
		*log << (begin ? "" : "-") << upstreamNodeID;
		begin = false;
	}
	*log << "," << info.downstreamNodeID << "," <<
			info.prefix << ",";
	begin = true;
	for (auto iter = info.dataIDsReceived.begin(); iter != info.dataIDsReceived.end(); ++iter) {
		if (begin) {
			*log << *iter;
			begin = false;
		} else {
			*log << "|" << *iter;
		}
	}
	*log << endl;
}

void RntpStrategy::logMsgEcho(bool isRecv, EchoInfo& info, PhyInfo* phyInfo) {
	auto log = RntpUtils::getLogMsgEcho();
	*log << this->nodeInfo->nodeID << "," << Simulator::Now() << "," << (phyInfo != NULL ? phyInfo->snr : -1) << ",";
	if (isRecv) {
		*log << "r,";
	} else {
		*log << "s,";
	}
	*log << info.sourceNodeID << "," << info.seqNum << endl;
}


void RntpStrategy::logRoutes() {
	auto log = RntpUtils::getLogRoutes();
	*log << this->nodeInfo->nodeID << ",";
	bool begin_routes, begin_route;
	for (auto iter = this->routes_all.begin(); iter != this->routes_all.end(); ++iter) {
		RoutesPerPair* routesPerPair = *iter;
		*log << this->nodeInfo->nodeID << "," << Simulator::Now() << "," << routesPerPair->consumerNodeID << ","
				<< routesPerPair->routes.size() << ",";
		begin_routes = true;
		for (auto iter2 = routesPerPair->routes.begin(); iter2 != routesPerPair->routes.end(); ++iter2) {
			if (!begin_routes) {
				*log << "|";
			} else {
				begin_routes = false;
			}
			Route* route = *iter2;
			*log << route->id << "#" << route->n_hops << "#" << route->updateTime << "#" << route->metric << "#";

			begin_route = true;
			for (auto iter3 = route->nodeIDs.begin(); iter3 != route->nodeIDs.end(); ++iter3) {
				if (begin_route) {
					*log << *iter3;
					begin_route = false;
				} else {
					*log << "-" << *iter3;
				}
			}
			*log << "#";
			begin_route = true;
			for (auto iter3 = route->channelQualities.begin(); iter3 != route->channelQualities.end(); ++iter3) {
				if (begin_route) {
					*log << *iter3;
					begin_route = false;
				} else {
					*log << "-" << *iter3;
				}
			}
		}

		*log << endl;

	}
}


void RntpStrategy::dumpRoute(uint32_t consumerNodeID, string producerPrefix) {
	cout << "---------------- DUMP ROUTE (curNodeID: " << this->nodeInfo->nodeID << ", consumerNodeID: " << consumerNodeID <<
			", producerPrefix: " << producerPrefix << ") -----------------" << endl;
	for (route_table::iterator iter = routes_all.begin(); iter != routes_all.end(); ++iter) {
		RoutesPerPair* routesPerPair = *iter;
		if (routesPerPair->consumerNodeID == consumerNodeID && routesPerPair->producerPrefix == producerPrefix) {
			for (vector<Route*>::iterator iter2 = routesPerPair->routes.begin(); iter2 != routesPerPair->routes.end(); ++iter2) {
				Route* route = *iter2;
				cout << route->id << ", " << route->metric << ", " << route->n_hops << ", " << route->updateTime << ", route: (";
				for (list<uint32_t>::iterator iter3 = route->nodeIDs.begin(); iter3 != route->nodeIDs.end(); ++iter3) {
					cout << *iter3 << " ";
				}
				cout << "), qualities: (";
				for (list<double>::iterator iter3 = route->channelQualities.begin();
										iter3 != route->channelQualities.end(); ++iter3) {
					cout << *iter3 << " ";
				}
				cout << ")" << endl;
			}
			break;
		}
	}
	cout << "----------------------------------------------------------------" << endl;
}

}
}
