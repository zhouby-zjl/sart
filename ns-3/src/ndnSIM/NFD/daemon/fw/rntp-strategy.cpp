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

using namespace ::nfd::cs;
using namespace ::nfd::cs::priority_fifo;
using namespace std;
using namespace ns3;
using namespace ns3::ndn;
using namespace ::ndn;

#define CHANNEL_QUALITY_MAX 100000000000000.0

namespace nfd {
namespace fw {

NFD_LOG_INIT(RntpStrategy);
NFD_REGISTER_STRATEGY(RntpStrategy);

unordered_map<int, unordered_map<string, int>*> RntpStrategy::performance_res;
Ptr<ns3::UniformRandomVariable> RntpStrategy::rand;
bool RntpStrategy::enableLog_msgs;

void initializeRntpStrategyRand() {
	RntpStrategy::rand = ns3::CreateObject<ns3::UniformRandomVariable>();
	RntpStrategy::rand->SetAttribute ("Min", ns3::DoubleValue (1.0));
	RntpStrategy::rand->SetAttribute ("Max", ns3::DoubleValue ((double) UINT_MAX));
}

RntpStrategy::RntpStrategy(Forwarder& forwarder, const Name& name) : Strategy(forwarder) {
	this->setInstanceName(makeInstanceName(name, getStrategyName()));
	nodeInfo = NodeInfoManager::getRoutesByForwarder(&forwarder);
	forwarder.rntpStrategy = this;
	RntpStrategy::performance_res[nodeInfo->nodeID] = new unordered_map<string, int>();

	is_face_found = false;
	channelQualityUpdate_seqno_cur = 0;
	echo_seqnum = 0;
	route_id_cur = 0;

	Time delayTime_echo = Seconds(rand->GetValue(0.000001, this->nodeInfo->echoPeriod.GetSeconds()));
	Time delayTime_CQUpdate = Seconds(rand->GetValue(0.000001, this->nodeInfo->channelQualityUpdatePeriod.GetSeconds()));

	sendEchoEvent = Simulator::Schedule(delayTime_echo, &RntpStrategy::sendEchoInPeriodical, this);
	underPreparationToSendCQUpdate = false;
	congestionControl = new RntpCongestionControl();
}

RntpStrategy::~RntpStrategy() {
	logRoutes();
}

const Name& RntpStrategy::getStrategyName() {
	static Name strategyName("ndn:/localhost/nfd/strategy/rntp/%FD%01");
	return strategyName;
}

void RntpStrategy::initializeFaces() {
	if (is_face_found) return;

	const ::nfd::FaceTable& table = this->getFaceTable();
	for (::nfd::FaceTable::const_iterator iter = table.begin(); iter != table.end(); ++iter) {
		::nfd::face::Face* face = table.get(iter->getId());
		const string& scheme = face->getLocalUri().getScheme();
		if (scheme == "netdev") {
			this->face_netdev = face;
		} else if (scheme == "appFace") {
			this->face_app = face;
		}
	}
	is_face_found = true;
}

string RntpStrategy::hashPrefixAndConsumerID(string prefix, uint32_t consumerID) {
	stringstream ss;
	ss << prefix << "|" << consumerID;
	return ss.str();
}

void RntpStrategy::afterReceiveInterest(const FaceEndpoint& ingress, const Interest& interest,
		const shared_ptr<pit::Entry>& pitEntry) {
	initializeFaces();
	InterestInfo info;
	this->extractInterest(interest, &info);

	PhyInfo phyInfo;
	this->extractPhyInfo(interest, phyInfo);

	this->logMsgInterest(true, info, &phyInfo);

	if (enableLog_msgs) {
		cout << "[Node " << nodeInfo->nodeID << ", " << Simulator::Now().GetNanoSeconds() << " ns] receives an Interest with prefix: " << info.prefix << ", consumerNodeID: " <<
			info.consumerNodeID << ", nextHopNodeID: " << info.nextHopNodeID << ", SNR: " << RatioToDb(phyInfo.snr) << ", RSSI: " << WToDbm(phyInfo.rssi) << endl;
	}

	if (hasPendingOutRecords(*pitEntry)) return;

	if (this->findPrefix(info.prefix) >= 0 && face_app != NULL) {
		this->sendInterest(pitEntry, FaceEndpoint(*face_app, 0), interest);

	} else if (info.consumerNodeID == 0xffffffff) {
		string h = hashPrefixAndConsumerID(info.prefix, this->nodeInfo->nodeID);
		if (transportStates_all.find(h) != transportStates_all.end()) {
			return;
		} else {
			TransportStates* tranStates = new TransportStates;
			tranStates->prefix = info.prefix;
			InterestBroadcastStates* rreqStates = new InterestBroadcastStates;
			rreqStates->recvTime = Simulator::Now();
			tranStates->receivedInterestBroadcasts = rreqStates;
			tranStates->capSendQueue.setNodeID(this->nodeInfo->nodeID);
			congestionControl->init(tranStates, nodeInfo);
			transportStates_all[h] = tranStates;

			list<uint32_t> visitedNodeIDs;
			visitedNodeIDs.push_back(this->nodeInfo->nodeID);
			list<double> channelQualities;

			propagateInterests(rreqStates, info.prefix, visitedNodeIDs, channelQualities,
								this->nodeInfo->nodeID, 0, false);
		}
	}

}


void RntpStrategy::afterReceiveData(const shared_ptr<pit::Entry>& pitEntry,
		const FaceEndpoint& ingress, const Data& data) {
	initializeFaces();

	Name dataName = data.getName();
	//cout << "%%%%%%%>> " << dataName << " @ NodeID: " << nodeInfo->nodeID << ", time: " << Simulator::Now().GetSeconds() << endl;
	int nDataNameComponents = pitEntry->getName().size();
	string operationStr = dataName.get(nDataNameComponents).toUri(name::UriFormat::DEFAULT);
	string pitPrefixStr = pitEntry->getName().toUri(name::UriFormat::DEFAULT);

	if (operationStr.compare("InterestBroadcast") == 0) {
		onReceiveInterestBroadcast(ingress, data);
	} else if (operationStr.compare("Capsule") == 0) {
		onReceiveCapsule(ingress, data);
	} else if (operationStr.compare("CapsuleAck") == 0) {
		onReceiveCapsuleAck(ingress, data);
	} else if (operationStr.compare("Echo") == 0) {
		onReceiveEcho(ingress, data);
	} else {
		cout << "===> unknown operationStr: " << operationStr << endl;
	}
}


void RntpStrategy::afterReceiveNonPitData(const FaceEndpoint& ingress, const Data& data) {
	initializeFaces();

	Name dataName = data.getName();
	string operationStr = dataName.get(2).toUri(name::UriFormat::DEFAULT);
	//cout << "NonPitData >> " << dataName << " @ NodeID: " << nodeInfo->nodeID << ", time: " << Simulator::Now().GetSeconds() << endl;

	if (operationStr.compare("InterestBroadcast") == 0) {
		onReceiveInterestBroadcast(ingress, data);
	} else if (operationStr.compare("Capsule") == 0) {
		onReceiveCapsule(ingress, data);
	} else if (operationStr.compare("CapsuleAck") == 0) {
		onReceiveCapsuleAck(ingress, data);
	} else if (operationStr.compare("Echo") == 0) {
		onReceiveEcho(ingress, data);
	} else {
		cout << "===> unknown operationStr: " << operationStr << endl;
	}
}

void RntpStrategy::onReceiveInterestBroadcast(const FaceEndpoint& ingress, const Data& data) {
	InterestBroadcastInfo info;
	this->extractInterestBroadcastInfo(data, &info);
	PhyInfo phyInfo;
	this->extractPhyInfo(data, phyInfo);

	this->logMsgInterestBroadcast(true, info, &phyInfo);

	double curQuality = phyInfo.snr;

	updateChannelQuality(info.transHopNodeID, curQuality);

	if (info.end == true) { // for terminating transport states
		if (std::find(info.visitedNodeIDs.begin(), info.visitedNodeIDs.end(), this->nodeInfo->nodeID) != info.visitedNodeIDs.end()) {
			return;
		}

		string h = this->hashPrefixAndConsumerID(info.producerPrefix, info.consumerNodeID);
		auto iter = transportStates_all.find(h);
		if (iter != transportStates_all.end()) {
			transportStates_all.erase(h);

			cout << "[Node " << this->nodeInfo->nodeID << ", " << Simulator::Now().GetMicroSeconds() << " us] terminate transport states with"
					" producerPrefix: "  <<  info.producerPrefix <<
					", consumerNodeID: " << info.consumerNodeID << endl;

			if (info.visitedNodeIDs.size() > 0) {
				info.channelQualities.push_back(curQuality);
			}
			info.visitedNodeIDs.push_back(this->nodeInfo->nodeID);

			this->propagateInterests(NULL, info.producerPrefix, info.visitedNodeIDs, info.channelQualities,
									info.consumerNodeID, 0, true);
		}
		return;
	}

	if (info.consumerNodeID == this->nodeInfo->nodeID) {
		return;
	}

	uint32_t hopCount = info.hopCount + 1;


	if (enableLog_msgs) {
		cout << "[Node " << nodeInfo->nodeID << ", " << Simulator::Now().GetNanoSeconds() << " ns] receives an InterestBroadcast with"
				" hopCount: "  << info.hopCount <<
				", producerPrefix: "  <<  info.producerPrefix <<
				", consumerNodeID: " << info.consumerNodeID <<
				", transHopNodeID: " << info.transHopNodeID <<
				", SNR: " << RatioToDb(phyInfo.snr) << ", RSSI: " << WToDbm(phyInfo.rssi) <<
				", curQuality: " << curQuality << ", visitedNodeIDs: ";


		for (auto iter = info.visitedNodeIDs.begin(); iter != info.visitedNodeIDs.end(); ++iter) {
			cout << *iter << " ";
		}
		cout << ", end: " << info.end << endl;
	}

	int32_t prefixIdx = findPrefix(info.producerPrefix);

	if (prefixIdx >= 0) {
		info.visitedNodeIDs.push_back(this->nodeInfo->nodeID);
		info.channelQualities.push_back(curQuality);

		this->addRoute(info.producerPrefix, info.consumerNodeID, hopCount,
						info.visitedNodeIDs, info.channelQualities);

		string h = this->hashPrefixAndConsumerID(info.producerPrefix, info.consumerNodeID);
		auto iter = transportStates_all.find(h);
		if (iter == transportStates_all.end()) {
			TransportStates* tranStates = new TransportStates;
			tranStates->prefix = info.producerPrefix;
			InterestBroadcastStates* rreqStates = new InterestBroadcastStates;
			rreqStates->recvTime = Simulator::Now();
			tranStates->receivedInterestBroadcasts = rreqStates;
			tranStates->capSendQueue.setNodeID(this->nodeInfo->nodeID);
			congestionControl->init(tranStates, nodeInfo);
			transportStates_all[h] = tranStates;
		} else {
			return;
		}

		if (face_app != NULL) {
			shared_ptr<Interest> interestOut = this->constructInterest(info.producerPrefix, info.consumerNodeID, -1);
			FaceEndpoint egress(*this->face_app, 0);
			egress.face.sendInterest(*interestOut, egress.endpoint);
			this->logMsgInterestBroadcast(false, info, NULL);
		}

	} else {
		if (std::find(info.visitedNodeIDs.begin(), info.visitedNodeIDs.end(), this->nodeInfo->nodeID) != info.visitedNodeIDs.end()) {
			return;
		}

		info.visitedNodeIDs.push_back(this->nodeInfo->nodeID);
		info.channelQualities.push_back(curQuality);

		this->addRoute(info.producerPrefix, info.consumerNodeID, hopCount,
						info.visitedNodeIDs, info.channelQualities);

		string h = this->hashPrefixAndConsumerID(info.producerPrefix, info.consumerNodeID);

		auto iter = transportStates_all.find(h);
		if (iter == transportStates_all.end()) {
			InterestBroadcastStates* rreqStates = NULL;

			TransportStates* tranStates = new TransportStates;
			tranStates->prefix = info.producerPrefix;
			rreqStates = new InterestBroadcastStates;
			rreqStates->recvTime = Simulator::Now();
			tranStates->receivedInterestBroadcasts = rreqStates;
			tranStates->capSendQueue.setNodeID(this->nodeInfo->nodeID);
			congestionControl->init(tranStates, nodeInfo);
			transportStates_all[h] = tranStates;

			this->propagateInterests(rreqStates, info.producerPrefix, info.visitedNodeIDs, info.channelQualities,
									info.consumerNodeID, hopCount, false);
		}

	}
}

bool RntpStrategy::checkIfCapsuleArrivedInDownstream(CapsuleInfo& capInfo) {
	int nodeIdx_cur = -1, nodeIdx_trans = -1;
	size_t idx = 0;
	for (auto iter = capInfo.nodeIDs.begin(); iter != capInfo.nodeIDs.end(); ++iter) {
		if (*iter == this->nodeInfo->nodeID && nodeIdx_cur == -1) {
			nodeIdx_cur = idx;
		}
		if (*iter == capInfo.transHopNodeID && nodeIdx_trans == -1) {
			nodeIdx_trans = idx;
		}
		if (nodeIdx_cur != -1 && nodeIdx_trans != -1) break;
		++idx;
	}

	if (nodeIdx_cur == -1 || nodeIdx_cur == 0 || nodeIdx_trans == -1 || nodeIdx_cur >= nodeIdx_trans) {
		return false;
	}
	return true;
}

int RntpStrategy::checkCapsuleArrivalDirection(CapsuleInfo& capInfo) {
	int nodeIdx_cur = -1, nodeIdx_trans = -1;
	size_t idx = 0;
	for (auto iter = capInfo.nodeIDs.begin(); iter != capInfo.nodeIDs.end(); ++iter) {
		if (*iter == this->nodeInfo->nodeID && nodeIdx_cur == -1) {
			nodeIdx_cur = idx;
		}
		if (*iter == capInfo.transHopNodeID && nodeIdx_trans == -1) {
			nodeIdx_trans = idx;
		}
		if (nodeIdx_cur != -1 && nodeIdx_trans != -1) break;
		++idx;
	}

	if (nodeIdx_cur != -1 && nodeIdx_trans != -1 && nodeIdx_cur > 0 && nodeIdx_cur < nodeIdx_trans) {
		return CAP_ARRIVAL_FROM_UPSTREAM;

	} else if (nodeIdx_cur != -1 && nodeIdx_trans != -1 && nodeIdx_cur > nodeIdx_trans) {
		return CAP_ARRIVAL_FROM_DOWNSTREAM;

	} else if (nodeIdx_cur == -1 && nodeIdx_trans != -1) {
		for (route_table::iterator iter = routes_all.begin(); iter != routes_all.end(); ++iter) {
			RoutesPerPair* routesPerPair = *iter;
			if (routesPerPair->consumerNodeID == capInfo.consumerNodeID && routesPerPair->producerPrefix == capInfo.prefix) {
				for (vector<Route*>::iterator iter2 = routesPerPair->routes.begin(); iter2 != routesPerPair->routes.end(); ++iter2) {
					Route* route = *iter2;
					for (auto iter3 = route->nodeIDs.begin(); iter3 != route->nodeIDs.end(); ++iter3) {
						if (*iter3 == capInfo.transHopNodeID) {
							return CAP_ARRIVAL_FROM_DOWNSTREAM;
						}
					}
				}
			}
		}

		return CAP_ARRIVAL_FROM_OTHERS;
	} else {
		return CAP_ARRIVAL_FROM_OTHERS;
	}

}

uint64_t RntpStrategy::hashDataIDAndNextHopID(uint32_t dataID, uint32_t nextHopID) {
	return  (uint64_t) nextHopID << 32 | (uint64_t) dataID ;
}

list<uint32_t> RntpStrategy::getUpstreamNodeIDsFromCapsule(list<uint32_t> nodeIDs, uint32_t transHopNodeID) {
	list<uint32_t> nodeIDs_s;
	auto iter = std::find(nodeIDs.begin(), nodeIDs.end(), this->nodeInfo->nodeID);
	if (iter != nodeIDs.end()) {
		++iter;
		for (; iter != nodeIDs.end(); ++iter) {
			nodeIDs_s.push_back(*iter);
			if (*iter == transHopNodeID) {
				return nodeIDs_s;
			}
		}
	}
	return list<uint32_t>();
}

int RntpStrategy::getNextHop(list<uint32_t>& nodeIDs) {
	int nextHopID = -1;
	if (nodeIDs.size() != 0) {
		for (auto iter = nodeIDs.begin(); iter != nodeIDs.end(); ++iter) {
			if (*iter == this->nodeInfo->nodeID) {
				break;
			}
			nextHopID = *iter;
		}
	}
	return nextHopID;
}

void RntpStrategy::onReceiveCapsule(const FaceEndpoint& ingress, const Data& data) {
	CapsuleInfo capInfo;
	this->extractCapsuleInfo(data, &capInfo);

	PhyInfo phyInfo;
	this->extractPhyInfo(data, phyInfo);

	this->logMsgCapsule(true, capInfo, &phyInfo);

	if (capInfo.nodeIDs.size() > 0) {
		updateChannelQuality(capInfo.transHopNodeID, phyInfo.snr);
	}

	string h = this->hashPrefixAndConsumerID(capInfo.prefix, capInfo.consumerNodeID);

	auto iter = this->transportStates_all.find(h);
	if (iter == this->transportStates_all.end()) {
		return;
	}
	TransportStates* ts = this->transportStates_all[h];

	if (capInfo.consumerNodeID == 0xffffffff) {
		return;
	}

	int direction = checkCapsuleArrivalDirection(capInfo);
	if (direction == CAP_ARRIVAL_FROM_DOWNSTREAM && capInfo.consumerNodeID != this->nodeInfo->nodeID) {
		dealWithAck(ts, capInfo.dataID, capInfo.transHopNodeID, this->nodeInfo->nodeID, 1);
		return;
	}

	if (capInfo.consumerNodeID == this->nodeInfo->nodeID) {
		int nextHopID = getNextHop(capInfo.nodeIDs);
		uint64_t dn = hashDataIDAndNextHopID(capInfo.dataID, nextHopID);

		if (ts->sentDataIDAndNextHops.find(dn) != ts->sentDataIDAndNextHops.end()) {
			list<uint32_t> dataIDsReceived;
			dataIDsReceived.push_back(capInfo.dataID);
			list<uint32_t> upstreamNodeIDs = this->getUpstreamNodeIDsFromCapsule(capInfo.nodeIDs, capInfo.transHopNodeID);
			this->sendCapsuleACK(capInfo.prefix, dataIDsReceived, this->nodeInfo->nodeID,
								upstreamNodeIDs, capInfo.transHopNodeID, capInfo.consumerNodeID);

			return;
		}

		if (enableLog_msgs) {
			cout << "[Node " << nodeInfo->nodeID << ", " << Simulator::Now().GetNanoSeconds() << " ns] receives a Capsule ($1) with prefix: " <<  capInfo.prefix << ", dataID: " << capInfo.dataID <<
					", consumerNodeID: " << capInfo.consumerNodeID << ", SNR: " << RatioToDb(phyInfo.snr) << ", RSSI: " << WToDbm(phyInfo.rssi)
					<< ", nodeIDs: ";

			for (auto iter = capInfo.nodeIDs.begin(); iter != capInfo.nodeIDs.end(); ++iter) {
				cout << *iter << " ";
			}
			cout << endl;
		}

		if (face_app != NULL) {
			shared_ptr<Data> d = this->constructCapsule(&capInfo, capInfo.nodeIDs, data, capInfo.nHops + 1);

			FaceEndpoint egress(*this->face_app, 0);
			egress.face.sendData(*d, egress.endpoint);
			ts->sentDataIDAndNextHops.insert(dn);

			this->logMsgCapsule(false, capInfo, NULL);
		}

		list<uint32_t> dataIDsReceived;
		dataIDsReceived.push_back(capInfo.dataID);
		list<uint32_t> upstreamNodeIDs = this->getUpstreamNodeIDsFromCapsule(capInfo.nodeIDs, capInfo.transHopNodeID);
		this->sendCapsuleACK(capInfo.prefix, dataIDsReceived, this->nodeInfo->nodeID,
							upstreamNodeIDs, capInfo.transHopNodeID, capInfo.consumerNodeID);

	} else if (capInfo.nodeIDs.size() == 0) {
		if (enableLog_msgs) {
			cout << "[Node " << nodeInfo->nodeID << ", " << Simulator::Now().GetNanoSeconds() << " ns] receives a Capsule ($2) with prefix: " <<  capInfo.prefix << ", dataID: " << capInfo.dataID <<
					", consumerNodeID: " << capInfo.consumerNodeID << ", SNR: " << RatioToDb(phyInfo.snr) << ", RSSI: " << WToDbm(phyInfo.rssi)
					<< ", nodeIDs: ";

			for (auto iter = capInfo.nodeIDs.begin(); iter != capInfo.nodeIDs.end(); ++iter) {
				cout << *iter << " ";
			}
			cout << endl;
		}

		sendCapsuleViaQueue(ts, capInfo, data, SEND_CAP_FROM_PRODUCER);

	} else if (direction == CAP_ARRIVAL_FROM_UPSTREAM) {
		if (enableLog_msgs) {
			cout << "[Node " << nodeInfo->nodeID << ", " << Simulator::Now().GetNanoSeconds() << " ns] receives a Capsule ($3) with prefix: " <<  capInfo.prefix << ", dataID: " << capInfo.dataID <<
					", consumerNodeID: " << capInfo.consumerNodeID << ", transNodeID: " << capInfo.transHopNodeID << ", SNR: " << RatioToDb(phyInfo.snr) << ", RSSI: " << WToDbm(phyInfo.rssi)
					<< ", nodeIDs: ";

			for (auto iter = capInfo.nodeIDs.begin(); iter != capInfo.nodeIDs.end(); ++iter) {
				cout << *iter << " ";
			}
			cout << endl;
		}

		sendCapsuleViaQueue(ts, capInfo, data, SEND_CAP_FROM_PERVIOUS_HOP);
	}
}

int RntpStrategy::dealWithAck(TransportStates* ts, uint32_t dataID, uint32_t downstreamNodeID, uint32_t transNodeID, uint32_t reason) {
	auto iter2 = ts->sendCapStates.find(dataID);
	if (iter2 == ts->sendCapStates.end()) {
		return DEAL_ACK_NO_NEED;
	}

	if (iter2->second == NULL || iter2->second->nodeIDs_down.find(downstreamNodeID) == iter2->second->nodeIDs_down.end()) {
		return DEAL_ACK_NO_NEED;
	}

	uint64_t dn = this->hashDataIDAndNextHopID(dataID, downstreamNodeID);
	if (ts->sentDataIDAndNextHops.find(dn) == ts->sentDataIDAndNextHops.end()) {
		congestionControl->onCapAckReceived(ts, nodeInfo);
	}

	ts->sentDataIDAndNextHops.insert(dn);

	Simulator::Remove(iter2->second->sendEventID);
	ts->sendCapStates.erase(dataID);
	ts->capSendQueue.removeElement(dataID);

	this->sendCapsulesInQueue(ts);
	return DEAL_ACK_DONE;
}

void RntpStrategy::onReceiveCapsuleAck(const FaceEndpoint& ingress, const Data& data) {
	CapsuleACKInfo ackInfo;
	this->extractCapsuleACK(data, ackInfo);
	PhyInfo phyInfo;
	this->extractPhyInfo(data, phyInfo);

	this->logMsgCapAck(true, ackInfo, &phyInfo);

	updateChannelQuality(ackInfo.downstreamNodeID, phyInfo.snr);

	auto iter_un = std::find(ackInfo.upstreamNodeIDs.begin(), ackInfo.upstreamNodeIDs.end(), this->nodeInfo->nodeID);

	if (iter_un == ackInfo.upstreamNodeIDs.end()) {
		return;
	}
	if (enableLog_msgs) {
		cout << "[Node " << nodeInfo->nodeID << ", " << Simulator::Now().GetMicroSeconds() <<
				" us] receives an CapsuleAck with " <<
				"downstreamNodeID: " << ackInfo.downstreamNodeID <<
				", dataID: ";
		for (auto iter = ackInfo.dataIDsReceived.begin(); iter != ackInfo.dataIDsReceived.end(); ++iter) {
			cout << *iter << " ";
		}
		cout << "." << endl;
	}

	string h = this->hashPrefixAndConsumerID(ackInfo.prefix, ackInfo.consumerNodeID);
	auto iter = this->transportStates_all.find(h);
	if (iter == this->transportStates_all.end()) {
		return;
	}

	TransportStates* ts = transportStates_all[h];
	uint32_t dataID = *ackInfo.dataIDsReceived.begin();

	dealWithAck(ts, dataID, ackInfo.downstreamNodeID, ackInfo.transHopNodeID, 2);
}


void RntpStrategy::onReceiveEcho(const FaceEndpoint& ingress, const Data& data) {
	EchoInfo info;
	this->extractEcho(data, &info);
	PhyInfo phyInfo;
	this->extractPhyInfo(data, phyInfo);

	this->logMsgEcho(true, info, &phyInfo);

	if (enableLog_msgs) {
		cout << "[Node " << nodeInfo->nodeID << ", " << Simulator::Now().GetNanoSeconds() << " ns] receives a Echo with sourceNodeID: "
				<< info.sourceNodeID <<
				", seqNum: "  << info.seqNum << endl;
	}

	updateChannelQuality(info.sourceNodeID, phyInfo.snr);
}

void RntpStrategy::propagateInterests(InterestBroadcastStates* rreqStates, string producerPrefix,
										list<uint32_t>& visitedNodeIDs, list<double>& channelQualities,
										uint32_t consumerNodeID, uint32_t initialHopCount, bool end) {
	InterestBroadcastInfo* info = new InterestBroadcastInfo;
	info->hopCount = initialHopCount;
	info->producerPrefix = producerPrefix;
	info->consumerNodeID = consumerNodeID;
	info->transHopNodeID = this->nodeInfo->nodeID;
	info->nonce = 0;
	info->visitedNodeIDs = visitedNodeIDs;
	info->channelQualities = channelQualities;

	info->nonce = this->rand->GetInteger();
	info->end = end;
	if (rreqStates != NULL) {
		rreqStates->nonce = info->nonce;
	}
	shared_ptr<Data> data = this->constructInterestBroadcast(info);

	double waitTime = rand->GetValue(0, this->nodeInfo->interestContentionTimeInSecs);

	Simulator::Schedule(ns3::Seconds(waitTime), &RntpStrategy::propagateInterestsAsync, this, info, data, this->nodeInfo->interestSendTimes);
}

void RntpStrategy::propagateInterestsAsync(InterestBroadcastInfo* info, shared_ptr<Data> data, uint32_t times) {
	this->sendLltcNonPitData(*data);

	this->logMsgInterestBroadcast(false, *info, NULL);
	if (enableLog_msgs) {
		cout << "[Node " << this->nodeInfo->nodeID << ", " << Simulator::Now().GetMicroSeconds() << " us] sends a InterestBroadcast with"
				" hopCount: "  << info->hopCount <<
				", producerPrefix: "  <<  info->producerPrefix <<
				", consumerNodeID: " << info->consumerNodeID <<
				", transHopNodeID: " << info->transHopNodeID << ", nonce: " << info->nonce << ", end: " << info->end << endl;
	}
	if (times >= 2) {
		double waitTime = rand->GetValue(0, this->nodeInfo->interestContentionTimeInSecs);
		Simulator::Schedule(ns3::Seconds(waitTime), &RntpStrategy::propagateInterestsAsync, this, info, data, times - 1);
	}
}

uint64_t RntpStrategy::getHash(CapsuleInfo& capInfo) {
	int nextHopID = -1;
	if (capInfo.nodeIDs.size() != 0) {
		for (auto iter = capInfo.nodeIDs.begin(); iter != capInfo.nodeIDs.end(); ++iter) {
			if (*iter == this->nodeInfo->nodeID) {
				break;
			}
			nextHopID = *iter;
		}
	}

	uint64_t dn = hashDataIDAndNextHopID(capInfo.dataID, nextHopID);
	return dn;
}

void RntpStrategy::sendCapsuleViaQueue(TransportStates* ts, CapsuleInfo& capInfo, const Data& data,
								int code) {

	//auto iter = ts->sendCapStates.find(capInfo.dataID);
	if (/*iter != ts->sendCapStates.end() || */ ts->capSendQueue.isDataIDInBuffer(capInfo.dataID)) {
		list<uint32_t> upstreamNodeIDs = this->getUpstreamNodeIDsFromCapsule(capInfo.nodeIDs, capInfo.transHopNodeID);
		list<uint32_t> dataIDsReceived;
		dataIDsReceived.push_back(capInfo.dataID);
		this->sendCapsuleACK(capInfo.prefix, dataIDsReceived, this->nodeInfo->nodeID,
							upstreamNodeIDs, capInfo.transHopNodeID, capInfo.consumerNodeID);

		return;
	}

	CapsuleToSend cs;
	cs.capInfo = capInfo;
	cs.data = data;
	cs.code = code;
	cs.nTimesRetried = 0;
	ts->capSendQueue.pushElement(cs);

	unordered_set<uint32_t> dataIDsSent = sendCapsulesInQueue(ts);

	if (dataIDsSent.find(capInfo.dataID) == dataIDsSent.end()) {
		list<uint32_t> upstreamNodeIDs = this->getUpstreamNodeIDsFromCapsule(capInfo.nodeIDs, capInfo.transHopNodeID);
		if (upstreamNodeIDs.size() > 0) {
			list<uint32_t> dataIDsReceived;
			dataIDsReceived.push_back(capInfo.dataID);
			this->sendCapsuleACK(capInfo.prefix, dataIDsReceived, this->nodeInfo->nodeID,
								upstreamNodeIDs, capInfo.transHopNodeID, capInfo.consumerNodeID);
		}
	}
}

unordered_set<uint32_t> RntpStrategy::sendCapsulesInQueue(TransportStates* ts) {
	unordered_set<uint32_t> dataIDsSent;
	for (int i = 0; i < ts->window; ++i) {
		if (ts->capSendQueue.countElements() > 0) {
			CapsuleToSend* cs_send = ts->capSendQueue.getFront();
			ts->capSendQueue.transientlyPopElement();

			uint32_t transHopNodeID_prev = cs_send->capInfo.transHopNodeID;
			cs_send->capInfo.transHopNodeID = this->nodeInfo->nodeID;
			int r = sendCapsule(ts, cs_send->capInfo, transHopNodeID_prev, cs_send->data, cs_send->nTimesRetried,
								cs_send->code);

			if (r == SEND_CAP_NORM) {
				dataIDsSent.insert(cs_send->capInfo.dataID);
			}

		} else {
			break;
		}
	}
	return dataIDsSent;
}

int RntpStrategy::sendCapsule(TransportStates* ts, CapsuleInfo& capInfo, uint32_t transHopNodeID_prev,
								const Data& data, uint32_t nTimesRetried, int code) {
	SendCapState* tranState = new SendCapState();
	tranState->sendTimes = 0;
	ts->sendCapStates[capInfo.dataID] = tranState;

	SendCapsuleIterParam params;
	params.nTimesRetried = nTimesRetried;
	params.transHopNodeID_prev = transHopNodeID_prev;
	params.code = code;

	return sendCapsuleIterative(ts, capInfo, data, params);
}


int RntpStrategy::sendCapsuleIterative(TransportStates* ts, CapsuleInfo& capInfo,
										const Data& data, SendCapsuleIterParam params) {
	SendCapState* tranState = ts->sendCapStates[capInfo.dataID];
	if (tranState == NULL) {
		ts->capSendQueue.removeElement(capInfo.dataID);
		return SEND_CAP_CANCELED;
	}
	string h = this->hashPrefixAndConsumerID(capInfo.prefix, capInfo.consumerNodeID);
	auto iter = this->transportStates_all.find(h);
	if (iter == this->transportStates_all.end()) {
		ts->sendCapStates.erase(capInfo.dataID);
		ts->capSendQueue.removeElement(capInfo.dataID);
		return SEND_CAP_DUPLICATED;
	}
	TransportStates* ts_serv = this->transportStates_all[h];

	Route* route_to_choose = this->matchRoute(capInfo.consumerNodeID, capInfo.prefix, capInfo.nodeIDs, &capInfo.nodeIDs);

	if (route_to_choose == NULL) {
		uint32_t rank = 0;
		if (params.code == SEND_CAP_FOR_RETRYING) {
			rank = params.nTimesRetried + 1;
		}
		route_to_choose = this->lookupRoute(capInfo.consumerNodeID, capInfo.prefix,
											&capInfo.nodeIDs, rank);
	}

	if (route_to_choose == NULL || tranState->sendTimes == this->nodeInfo->capsuleRetryingMaxTimes) {
		ts_serv->sendCapStates.erase(capInfo.dataID);


		CapsuleToSend* cts = ts_serv->capSendQueue.restoreElement(capInfo.dataID);

		if (cts->code == SEND_CAP_FOR_RETRYING) {
			++cts->nTimesRetried;
		} else {
			cts->nTimesRetried = 0;
		}
		cts->code = SEND_CAP_FOR_RETRYING;

		int nextHopID = -1;
		if (route_to_choose != NULL) {
			nextHopID = this->getNextHop(route_to_choose->nodeIDs);
		}

		congestionControl->onCapAckTimeout(ts_serv, nodeInfo, nextHopID);

		if (route_to_choose == NULL) {
			return SEND_CAP_NO_ROUTE;
		}

		if (tranState->sendTimes == this->nodeInfo->capsuleRetryingMaxTimes) {
			return SEND_CAP_EXCEED_MAX_RETRYING_TIME;
		}

	} else {
		int nextHopID = getNextHop(route_to_choose->nodeIDs);
		uint64_t dn = hashDataIDAndNextHopID(capInfo.dataID, nextHopID);

		if (ts->sentDataIDAndNextHops.find(dn) != ts->sentDataIDAndNextHops.end()) {
			list<uint32_t> upstreamNodeIDs = this->getUpstreamNodeIDsFromCapsule(capInfo.nodeIDs, params.transHopNodeID_prev);
			if (upstreamNodeIDs.size() > 0) {
				list<uint32_t> dataIDsReceived;
				dataIDsReceived.push_back(capInfo.dataID);
				this->sendCapsuleACK(capInfo.prefix, dataIDsReceived, this->nodeInfo->nodeID,
									upstreamNodeIDs, capInfo.transHopNodeID, capInfo.consumerNodeID);
			}

			ts->sendCapStates.erase(capInfo.dataID);
			ts->capSendQueue.removeElement(capInfo.dataID);

			return SEND_CAP_ALREADY_SENT;
		}
	}

	shared_ptr<Data> capOut = NULL;
	if (route_to_choose != NULL) { // route needs to be changed
		list<uint32_t> nodeIDs_new = route_to_choose->nodeIDs;

		if (nodeIDs_new.back() == this->nodeInfo->nodeID) {
			int n = capInfo.nodeIDs.size();
			int pos = n - 1;
			auto iter = capInfo.nodeIDs.rbegin();
			for (; iter != capInfo.nodeIDs.rend(); ++iter) {
				if (*iter == this->nodeInfo->nodeID) {
					break;
				}
				--pos;
			}
			if (pos >= 0 && pos < n - 1) {
				--iter;
				++pos;
				for (; pos < n; ++pos, --iter) {
					nodeIDs_new.push_back(*iter);
				}
			}
		}

		capInfo.nodeIDs = nodeIDs_new;
		capOut = this->constructCapsule(&capInfo, nodeIDs_new, data, capInfo.nHops + 1);
		addDownStreamNodes(&tranState->nodeIDs_down, nodeIDs_new);

	} else {
		capOut = this->constructCapsule(&capInfo, capInfo.nodeIDs, data, capInfo.nHops + 1);
		addDownStreamNodes(&tranState->nodeIDs_down, capInfo.nodeIDs);
	}

	FaceEndpoint egress(*this->face_netdev, 0);
	egress.face.sendData(*capOut, egress.endpoint);
	++tranState->sendTimes;

	this->logMsgCapsule(false, capInfo, NULL);

	if (enableLog_msgs) {
		cout << "[Node " << nodeInfo->nodeID << ", " << Simulator::Now().GetNanoSeconds() << " ns] sends a Capsule with prefix: " <<  capInfo.prefix << ", dataID: " << capInfo.dataID <<
			", consumerNodeID: " << capInfo.consumerNodeID << ", transNodeID: " << capInfo.transHopNodeID << ", times: " << tranState->sendTimes << ", nodeIDs: ";

		for (auto iter = capInfo.nodeIDs.begin(); iter != capInfo.nodeIDs.end(); ++iter) {
			cout << *iter << " ";
		}
		cout << ", code: " << params.code << endl;
	}

	if (tranState->sendTimes <= this->nodeInfo->capsuleRetryingMaxTimes) {
		ns3::Time timeout = nodeInfo->capsulePerHopTimeout;
		tranState->sendEventID = Simulator::Schedule(timeout, &RntpStrategy::sendCapsuleIterative, this,
										ts, capInfo, data, params);
	}

	return SEND_CAP_NORM;
}

void RntpStrategy::addDownStreamNodes(unordered_set<uint32_t>* A, list<uint32_t>& B) {
	for (auto iter = B.begin(); iter != B.end(); ++iter) {
		if (*iter == this->nodeInfo->nodeID) {
			break;
		}
		A->insert(*iter);
	}
}

void RntpStrategy::updateChannelQuality(uint32_t fromNodeID, double quality) {
	double smoothed_quality = 0;
	auto iter = channelQualities.find(fromNodeID);
	ns3::Time msgTimeout_est = this->nodeInfo->msgTimeout;

	if (iter == channelQualities.end()) {
		ChannelQualityStates cqs;
		cqs.quality_smooth = quality;
		cqs.fromNodeID = fromNodeID;
		cqs.thQueue = new ThroughputQueue(this->nodeInfo->thQueueSize);
		cqs.thQueue->recordPacketArrival(Simulator::Now());

		channelQualities[fromNodeID] = cqs;
		smoothed_quality = quality;

	} else {
		iter->second.quality_smooth = (1 - this->nodeInfo->quality_alpha) * iter->second.quality_smooth +
						this->nodeInfo->quality_alpha * quality;

		iter->second.thQueue->recordPacketArrival(Simulator::Now());
		msgTimeout_est = iter->second.thQueue->estimateLongestPIAT(this->nodeInfo->longestPIATEstCondifentRatio,
																	this->nodeInfo->msgTimeout);

		smoothed_quality = iter->second.quality_smooth;
		Simulator::Cancel(iter->second.waitMsgEvent);
	}

	this->updateRoutesWithQuality(fromNodeID, this->nodeInfo->nodeID, smoothed_quality);

	channelQualities[fromNodeID].waitMsgEvent = Simulator::Schedule(msgTimeout_est,
											&RntpStrategy::markChannelBroken, this, fromNodeID, "message timeout");


	if (iter != channelQualities.end()) {
		for (auto iter2 = transportStates_all.begin(); iter2 != transportStates_all.end(); ++iter2) {
			set<uint32_t> neighbors = this->getNeighboredIDsInRoutes(iter2->second->consumerID, iter2->second->prefix);
			if (neighbors.find(fromNodeID) != neighbors.end()) {
				this->congestionControl->onChannelWaken(iter2->second, nodeInfo, fromNodeID);
				sendCapsulesInQueue(iter2->second);
			}
		}

	}
}

void RntpStrategy::sendCapsuleACK(string prefix, list<uint32_t>& dataIDsReceived,
								uint32_t downstreamNodeID, list<uint32_t> upstreamNodeIDs,
								uint32_t transHopNodeID, uint32_t consumerNodeID) {
	CapsuleACKInfo info;
	info.prefix = prefix;

	info.upstreamNodeIDs = upstreamNodeIDs;
	info.downstreamNodeID = downstreamNodeID;
	info.dataIDsReceived = dataIDsReceived;
	info.transHopNodeID = transHopNodeID;
	info.consumerNodeID = consumerNodeID;

	shared_ptr<Data> data = this->constructCapsuleACK(info);
	FaceEndpoint egress(*this->face_netdev, 0);
	egress.face.sendData(*data, egress.endpoint);

	this->logMsgCapAck(false, info, NULL);

	if (enableLog_msgs) {
		cout << "[Node " << nodeInfo->nodeID << ", " << Simulator::Now().GetMicroSeconds() <<
				" us] sends an CapsuleAck with " <<
				"downstreamNodeID: " << info.downstreamNodeID <<
				", upstreamNodeID: ";
		bool begin = true;
		for (uint32_t upstreamNodeID : info.upstreamNodeIDs) {
			cout << (begin ? "" : "-") << upstreamNodeID;
			begin = false;
		}

		cout << ", consumerNodeID: " << consumerNodeID << ", dataIDsReceived: ";
		for (auto iter = info.dataIDsReceived.begin(); iter != info.dataIDsReceived.end(); ++iter) {
				cout << *iter << " ";
		}
		cout << endl;
	}

}


void RntpStrategy::sendEchoInPeriodical() {
	initializeFaces();

	EchoInfo info;
	info.sourceNodeID = this->nodeInfo->nodeID;
	info.seqNum = echo_seqnum++;
	shared_ptr<Data> data = this->constructEcho(&info);
	FaceEndpoint egress(*this->face_netdev, 0);
	egress.face.sendData(*data, egress.endpoint);

	this->logMsgEcho(false, info, NULL);

	if (enableLog_msgs) {
		cout << "[Node " << nodeInfo->nodeID << ", " << Simulator::Now().GetNanoSeconds() << " ns] sends a Echo with sourceNodeID: "
				<< info.sourceNodeID <<
				", seqNum: "  << info.seqNum << endl;
	}
	sendEchoEvent = Simulator::Schedule(this->nodeInfo->echoPeriod, &RntpStrategy::sendEchoInPeriodical, this);
}

void RntpStrategy::sendLltcNonPitData(Data& data) {
	data.setTag<lp::LltcTransientTag>(make_shared<lp::LltcTransientTag>(1));
	this->sendNonPitData(data, FaceEndpoint(*face_netdev, 0));
}

void RntpStrategy::addRoute(string producerPrefix, uint32_t consumerNodeID,
							uint32_t n_hops, list<uint32_t>& nodeIDs, list<double>& channelQualities) {
	if (enableLog_msgs) {
		cout << "[Node " << nodeInfo->nodeID << ", " << Simulator::Now().GetNanoSeconds() << " ns] " <<
				"add route with prefix: " << producerPrefix << ", consumerNodeID: " << consumerNodeID <<
				", n_hops: " << n_hops << ", nodeIDs: ";

		for (list<uint32_t>::iterator iter = nodeIDs.begin(); iter != nodeIDs.end(); ++iter) {
			cout << *iter << " ";
		}

		cout << ", channelQualities: ";
		for (list<double>::iterator iter = channelQualities.begin(); iter != channelQualities.end(); ++iter) {
			cout << *iter << " ";
		}
		cout << endl;
	}

	bool hasRoutesPerPair = false;
	RoutesPerPair* routesPerPair = NULL;
	for (vector<RoutesPerPair*>::iterator iter = this->routes_all.begin(); iter != this->routes_all.end(); ++iter) {
		routesPerPair = *iter;
		if (routesPerPair->producerPrefix == producerPrefix && routesPerPair->consumerNodeID == consumerNodeID) {
			hasRoutesPerPair = true;
			break;
		}
	}

	if (hasRoutesPerPair) {
		for (auto route : routesPerPair->routes) {
			if (route->nodeIDs.size() == nodeIDs.size()) {
				bool theSame = true;
				auto iter1 = route->nodeIDs.begin();
				auto iter2 = nodeIDs.begin();
				while (iter1 != route->nodeIDs.end()) {
					if (*iter1 != *iter2) {
						theSame = false;
						break;
					}
					++iter1;
					++iter2;
				}

				if (theSame) {
					cout << "duplicated route" << endl;
					return;
				}
			}
		}

		struct Route* route = new Route;
		route->id = route_id_cur++;
		route->n_hops = n_hops;
		route->nodeIDs = nodeIDs;
		route->channelQualities = channelQualities;
		route->updateTime = Simulator::Now();
		routesPerPair->routes.push_back(route);

	} else {
		routesPerPair = new RoutesPerPair;
		routesPerPair->producerPrefix = producerPrefix;
		routesPerPair->consumerNodeID = consumerNodeID;
		routesPerPair->lastHitTime = Simulator::Now();

		struct Route* route = new Route;
		route->id = route_id_cur++;
		route->n_hops = n_hops;
		route->nodeIDs = nodeIDs;
		route->channelQualities = channelQualities;
		route->updateTime = Simulator::Now();
		routesPerPair->routes.push_back(route);

		routes_all.push_back(routesPerPair);
	}
}

Route* RntpStrategy::matchRoute(uint32_t consumerNodeID, string producerPrefix, list<uint32_t> nodeIDs,
								list<uint32_t>* nodeIDs_prev) {
	unordered_set<uint32_t> prevNodes;
	if (nodeIDs_prev != NULL) {
		for (auto iter = nodeIDs_prev->rbegin(); iter != nodeIDs_prev->rend(); ++iter) {
			if (*iter == this->nodeInfo->nodeID) {
				break;
			}
			prevNodes.insert(*iter);
		}
	}

	for (route_table::iterator iter = routes_all.begin(); iter != routes_all.end(); ++iter) {
		RoutesPerPair* routesPerPair = *iter;
		if (routesPerPair->consumerNodeID == consumerNodeID && routesPerPair->producerPrefix == producerPrefix) {
			for (vector<Route*>::iterator iter2 = routesPerPair->routes.begin(); iter2 != routesPerPair->routes.end(); ++iter2) {
				Route* route = *iter2;

				auto iter_nodeIDs = nodeIDs.begin();
				auto iter_route = route->nodeIDs.begin();
				int k = 0;
				for (; iter_nodeIDs != nodeIDs.end() && iter_route != route->nodeIDs.end(); ++iter_nodeIDs, ++iter_route) {
					if (*iter_nodeIDs != *iter_route) break;
					++k;
				}
				if (k == route->nodeIDs.size()) {
					for (auto iter_quality = route->channelQualities.begin();
							iter_quality != route->channelQualities.end(); ++iter_quality) {
						if (*iter_quality == QUALITY_BROKEN) {
							return NULL;
						}
					}

					return route;
				}

			}
		}
	}
	return NULL;
}


Route* RntpStrategy::lookupRoute(uint32_t consumerNodeID, string producerPrefix,
								list<uint32_t>* nodeIDs_prev, uint32_t rank) {
	refreshRouteMetric(consumerNodeID, producerPrefix);
	dumpRoute(consumerNodeID, producerPrefix);

	unordered_set<uint32_t> prevNodes;
	if (nodeIDs_prev != NULL) {
		for (auto iter = nodeIDs_prev->rbegin(); iter != nodeIDs_prev->rend(); ++iter) {
			if (*iter == this->nodeInfo->nodeID) {
				break;
			}
			prevNodes.insert(*iter);
		}
	}

	for (route_table::iterator iter = routes_all.begin(); iter != routes_all.end(); ++iter) {
		RoutesPerPair* routesPerPair = *iter;
		if (routesPerPair->consumerNodeID == consumerNodeID && routesPerPair->producerPrefix == producerPrefix) {
			vector<Route*> routes_ranked;
			unordered_map<uint32_t, double> routeIDToMetricMap;
			for (vector<Route*>::iterator iter2 = routesPerPair->routes.begin(); iter2 != routesPerPair->routes.end(); ++iter2) {
				Route* route = *iter2;

				bool isLoop = false;
				for (auto iter = route->nodeIDs.begin(); iter != route->nodeIDs.end(); ++iter) {
					if (prevNodes.find(*iter) != prevNodes.end()) {
						isLoop = true;
						break;
					}
				}
				if (isLoop) {
					continue;
				}

				routeIDToMetricMap[route->id] = route->metric;
				auto iter_ranked = routes_ranked.begin();
				for (; iter_ranked != routes_ranked.end(); ++iter_ranked) {
					if ((*iter_ranked)->metric < route->metric) {
						break;
					}
				}
				if (iter_ranked != routes_ranked.end()) {
					routes_ranked.insert(iter_ranked, route);
				} else {
					routes_ranked.push_back(route);
				}
			}

			uint32_t n = routes_ranked.size();
			return n == 0 ? NULL : (n > rank ? routes_ranked[rank] : routes_ranked[rank % n]);
		}
	}
	return NULL;
}

void RntpStrategy::refreshRouteMetric(uint32_t consumerNodeID, string producerPrefix) {
	for (route_table::iterator iter = routes_all.begin(); iter != routes_all.end(); ++iter) {
		RoutesPerPair* routesPerPair = *iter;
		if (routesPerPair->consumerNodeID == consumerNodeID && routesPerPair->producerPrefix == producerPrefix) {
			double quality_higest = -10000000000000.0;
			double quality_lowest = DBL_MAX;
			for (vector<Route*>::iterator iter2 = routesPerPair->routes.begin();
					iter2 != routesPerPair->routes.end(); ++iter2) {
				Route* route = *iter2;

				for (list<double>::iterator iter3 = route->channelQualities.begin();
						iter3 != route->channelQualities.end(); ++iter3) {
					double quality = *iter3;
					if (quality <= QUALITY_BROKEN) {
						(*iter2)->metric = -1.0;
						continue;
					}
					if (quality > quality_higest) {
						quality_higest = quality;
					}

					if (quality < quality_lowest) {
						quality_lowest = quality;
					}
				}
			}

			if (quality_higest <= QUALITY_BROKEN) return;

			Route* route_best = NULL;

			for (vector<Route*>::iterator iter2 = routesPerPair->routes.begin(); iter2 != routesPerPair->routes.end(); ++iter2) {
				Route* route = *iter2;
				double metric = 1.0;
				int k = 0;
				for (list<double>::iterator iter3 = route->channelQualities.begin();
						iter3 != route->channelQualities.end(); ++iter3) {
					double quality = *iter3;
					if (quality <= QUALITY_BROKEN) {
						metric = -1.0;
						break;
					}
					metric *= quality;
					++k;
				}
				if (metric >= 0)
					(*iter2)->metric = pow(metric, 1.0 / (2 * k));
				else
					(*iter2)->metric = -1.0;
			}

			return;
		}
	}
}

int RntpStrategy::updateRoutesWithQuality(uint32_t fromNodeID, uint32_t toNodeID, double channelQuality) {
	int nRadicalChanges = 0;
	for (route_table::iterator iter = routes_all.begin(); iter != routes_all.end(); ++iter) {
		RoutesPerPair* routesPerPair = *iter;
		for (vector<Route*>::iterator iter2 = routesPerPair->routes.begin(); iter2 != routesPerPair->routes.end(); ++iter2) {
			Route* route = *iter2;
			if (route->nodeIDs.size() <= 1) continue;

			list<uint32_t>::iterator iter_nodeIDs = route->nodeIDs.begin();
			uint32_t nodeID_prev = *iter_nodeIDs;
			++iter_nodeIDs;
			list<double>::iterator iter_qualities = route->channelQualities.begin();
			bool radicalChange = false;
			for (; iter_nodeIDs != route->nodeIDs.end(); ++iter_nodeIDs) {
				uint32_t nodeID_cur = *iter_nodeIDs;
				if (nodeID_prev == fromNodeID && nodeID_cur == toNodeID) {
					if ((*iter_qualities == QUALITY_BROKEN && channelQuality > QUALITY_BROKEN) ||
						(*iter_qualities > QUALITY_BROKEN && channelQuality == QUALITY_BROKEN)) {
						radicalChange = true;
					}
					*iter_qualities = channelQuality;
				}
				nodeID_prev = nodeID_cur;
				++iter_qualities;
			}
			if (radicalChange) ++nRadicalChanges;
		}
	}
	return nRadicalChanges;
}


void RntpStrategy::markChannelBroken(uint32_t fromNodeID, string reason) {
	auto iter = channelQualities.find(fromNodeID);

	ns3::Time now = Simulator::Now();
	ChannelQualityStates& cqs = iter->second;
	this->updateRoutesWithQuality(fromNodeID, this->nodeInfo->nodeID, QUALITY_BROKEN);

	if (enableLog_msgs) {
		cout << "[Node " << nodeInfo->nodeID << ", " << Simulator::Now().GetNanoSeconds() << " ns] << update a link between "
			<< fromNodeID << ", " <<  this->nodeInfo->nodeID << " with quality: BROKEN, reason:" << reason << endl;
	}
}

double RntpStrategy::findEquivalentQualityOfBestRoute(uint32_t upstreamNeighboredNodeID) {
	vector<double> equiQualities;
	for (route_table::iterator iter = routes_all.begin(); iter != routes_all.end(); ++iter) {
		RoutesPerPair* routesPerPair = *iter;
		refreshRouteMetric(routesPerPair->consumerNodeID, routesPerPair->producerPrefix);
		Route* route_best = NULL;
		Route* route_related = NULL;
		double metric_highest = -1.0;
		bool viable = false;
		for (vector<Route*>::iterator iter2 = routesPerPair->routes.begin();
				iter2 != routesPerPair->routes.end(); ++iter2) {
			Route* route = *iter2;
			auto iter3 = route->nodeIDs.end();
			--iter3;
			--iter3;
			if (*iter3 == upstreamNeighboredNodeID) {
				route_related = route;
			}

			if ((*iter2)->metric > metric_highest) {
				metric_highest = (*iter2)->metric;
				route_best = route;
			}
		}
		if (route_best == route_related || route_best == NULL || route_best->metric == -1.0) {
			equiQualities.push_back(QUALITY_BROKEN);
			continue;
		}
		auto iter_best = route_best->nodeIDs.begin();
		int k = -1;
		if (route_related == NULL) {
			k = route_best->nodeIDs.size();
		} else {
			auto iter_related = route_related->nodeIDs.begin();

			for (; iter_best != route_best->nodeIDs.end() && iter_related != route_related->nodeIDs.end();
					++iter_best, ++iter_related) {
				if (*iter_best != *iter_related) {
					break;
				}
				++k;
			}
		}

		int i = 0;
		double best_quality_product = 1.0;
		for (auto iter_quality_best = route_best->channelQualities.begin();
				iter_quality_best != route_best->channelQualities.end();
				++iter_quality_best) {
			if (i < k) {
				++i;
				continue;
			}
			best_quality_product *= *iter_quality_best;
			++i;
		}

		double related_quality_product = 1.0;
		if (route_related != NULL) {
			int n = route_related->channelQualities.size();
			i = 0;
			for (auto iter_quality_related = route_related->channelQualities.begin();
					iter_quality_related != route_related->channelQualities.end();
					++iter_quality_related) {
				if (i < k) {
					++i;
					continue;
				}
				if (i == n - 1) break;
				related_quality_product *= *iter_quality_related;
				++i;
			}
		}

		equiQualities.push_back(best_quality_product / related_quality_product);
	}

	double equiQualities_total = 0.0;
	for (int i = 0; i < equiQualities.size(); ++i) {
		if (equiQualities[i] == QUALITY_BROKEN)
			return QUALITY_BROKEN;
		equiQualities_total += equiQualities[i];
	}
	return equiQualities_total / (double) equiQualities.size();
}

set<uint32_t> RntpStrategy::getNeighboredIDsInRoutes() {
	set<uint32_t> neighbors;
	for (route_table::iterator iter = routes_all.begin(); iter != routes_all.end(); ++iter) {
		RoutesPerPair* routesPerPair = *iter;
		for (vector<Route*>::iterator iter2 = routesPerPair->routes.begin(); iter2 != routesPerPair->routes.end(); ++iter2) {
			Route* route = *iter2;
			if (route->nodeIDs.size() <= 1) continue;

			list<uint32_t>::iterator iter_nodeIDs = route->nodeIDs.end();
			--iter_nodeIDs;
			--iter_nodeIDs;
			uint32_t neighborID = *iter_nodeIDs;
			neighbors.insert(neighborID);
		}
	}
	return neighbors;
}

set<uint32_t> RntpStrategy::getNeighboredIDsInRoutes(uint32_t consumerID, string prefix) {
	set<uint32_t> neighbors;
	for (route_table::iterator iter = routes_all.begin(); iter != routes_all.end(); ++iter) {
		RoutesPerPair* routesPerPair = *iter;
		if (routesPerPair->consumerNodeID != consumerID && routesPerPair->producerPrefix != prefix) {
			continue;
		}
		for (vector<Route*>::iterator iter2 = routesPerPair->routes.begin(); iter2 != routesPerPair->routes.end(); ++iter2) {
			Route* route = *iter2;
			if (route->nodeIDs.size() <= 1) continue;

			list<uint32_t>::iterator iter_nodeIDs = route->nodeIDs.end();
			--iter_nodeIDs;
			--iter_nodeIDs;
			uint32_t neighborID = *iter_nodeIDs;
			neighbors.insert(neighborID);
		}
	}
	return neighbors;
}

double RntpStrategy::getWorstChannelQuality(list<double>& channelQualities) {
	double worstQuality = DBL_MAX;
	for (auto iter = channelQualities.begin(); iter != channelQualities.end(); ++iter) {
		double q = *iter;
		if (q < worstQuality) {
			worstQuality = q;
		}
	}
	return worstQuality;
}

double RntpStrategy::getMeanChannelQuality(list<double>& channelQualities) {
	double total = 0;
	int n = 0;
	for (auto iter = channelQualities.begin(); iter != channelQualities.end(); ++iter) {
		double q = *iter;
		if (q <= QUALITY_BROKEN) return QUALITY_BROKEN;
		total += q;
		++n;
	}
	return total / (double)n;
}

double RntpStrategy::estimateBER(double snr) {
	  double EbN0 = snr * 22000000.0 / 1000000.0;
	  double ber = 0.5 * std::exp (-EbN0);
	  return ber;
}

int32_t RntpStrategy::findPrefix(string prefix) {
	int32_t i;
	for (vector<string>::iterator iter = nodeInfo->prefixes.begin(); iter != nodeInfo->prefixes.end(); ++iter) {
		if (*iter == prefix) {
			return i;
		}
		++i;
	}
	return -1;
}

}
}
