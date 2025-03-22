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


using namespace ::nfd::cs;
using namespace ::nfd::cs::priority_fifo;
using namespace std;
using namespace ns3;
using namespace ns3::ndn;
using namespace ::ndn;

#define CHANNEL_QUALITY_MAX 100000000000000.0

namespace nfd {
namespace fw {

shared_ptr<Interest> RntpStrategy::constructInterest(string pitPrefixStr, uint32_t consumerNodeID, uint32_t nextHopNodeID) {
	stringstream ss;
	ss << pitPrefixStr;

	auto interest = std::make_shared<Interest>(string(ss.str()));
	RouteTagInfo routeTagInfo;
	routeTagInfo.consumerNodeID = consumerNodeID;
	routeTagInfo.nextHopNodeID = nextHopNodeID;

	this->writeRouteTag(routeTagInfo, *interest);
	return interest;
}

shared_ptr<Data> RntpStrategy::constructCapsule(CapsuleInfo* info, list<uint32_t>& nodeIDs, const Data& origData, uint32_t nHops) {
	stringstream ss;
	if (nodeIDs.size() == 0) {
		ss << info->prefix << "/Capsule/" << info->dataID;
	} else {
		ss << info->prefix << "/Capsule/" << info->dataID << "/" << info->nonce << "/" << info->transHopNodeID;
		bool isFirst = true;
		for (auto iter = nodeIDs.begin(); iter != nodeIDs.end(); ++iter) {
			ss << (isFirst ? "/" : "-") << *iter;
			isFirst = false;
		}
		ss << "/" << nHops;
	}

	auto data = std::make_shared<Data>(string(ss.str()));
	data->setFreshnessPeriod(origData.getFreshnessPeriod());
	data->setContent(origData.getContent());
	StackHelper::getKeyChain().sign(*data);

	RouteTagInfo routeTagInfo;
	routeTagInfo.consumerNodeID = info->consumerNodeID;
	routeTagInfo.nextHopNodeID = 0;
	this->writeRouteTag(routeTagInfo, *data);
	return data;
}

shared_ptr<Data> RntpStrategy::constructCapsuleACK(CapsuleACKInfo& info) {
	stringstream ss;
	ss << info.prefix << "/CapsuleAck/" << info.downstreamNodeID << "/";

	bool begin = true;
	for (uint32_t upstreamNodeID : info.upstreamNodeIDs) {
		ss << (begin ? "" : "-") << upstreamNodeID;
		begin = false;
	}
	ss << "/" << info.transHopNodeID << "/" << info.consumerNodeID;

	size_t n = info.dataIDsReceived.size();
	size_t nBufBytes = sizeof(uint32_t) * n + sizeof(size_t);
	uint8_t* bufBytes = new uint8_t[nBufBytes];
	size_t* nDataIdRegion = (size_t*) bufBytes;
	nDataIdRegion[0] = n;
	uint32_t* dataIdRegion = (uint32_t*) (bufBytes + sizeof(size_t));
	int i = 0;
	for (list<uint32_t>::iterator iter = info.dataIDsReceived.begin(); iter != info.dataIDsReceived.end(); ++iter) {
		dataIdRegion[i] = *iter;
		++i;
	}

	auto data = std::make_shared<Data>(ss.str());
	data->setFreshnessPeriod(time::milliseconds(1000));
	shared_ptr<::ndn::Buffer> buf = std::make_shared<::ndn::Buffer>(nBufBytes);
	data->setContent(buf);
	for (size_t i = 0; i < nBufBytes; ++i) {
		(*buf)[i] = bufBytes[i];
	}
	StackHelper::getKeyChain().sign(*data);

	return data;
}

shared_ptr<Data> RntpStrategy::constructInterestBroadcast(InterestBroadcastInfo* info) {
	stringstream ss;
	ss << info->producerPrefix << "/InterestBroadcast/" << info->hopCount <<
			"/" << info->consumerNodeID <<
			"/" << info->transHopNodeID <<
			"/" << info->nonce << "/" << (info->end ? "true" : "false");

	string name = ss.str();

	size_t n = info->visitedNodeIDs.size();
	size_t nNodeIdsBytes = sizeof(size_t) + sizeof(uint32_t) * n;
	size_t nchannelQualitiesBytes = sizeof(size_t) +  sizeof(double) * (n - 1);
	size_t allBufBytes = nNodeIdsBytes + nchannelQualitiesBytes;
	uint8_t* bufBytes = new uint8_t[allBufBytes];
	size_t* nNodeIdRegion = (size_t*) bufBytes;
	nNodeIdRegion[0] = n;
	uint32_t* nodeIdRegion = (uint32_t*) (bufBytes + sizeof(size_t));
	int i = 0;
	for (list<uint32_t>::iterator iter = info->visitedNodeIDs.begin(); iter != info->visitedNodeIDs.end(); ++iter) {
		nodeIdRegion[i] = *iter;
		++i;
	}

	size_t* nchannelQualities = (size_t*) (bufBytes + nNodeIdsBytes);
	nchannelQualities[0] = n - 1;
	double* channelQualityRegion = (double*) (bufBytes + nNodeIdsBytes + sizeof(size_t));
	i = 0;
	for (list<double>::iterator iter = info->channelQualities.begin(); iter != info->channelQualities.end(); ++iter) {
		channelQualityRegion[i] = *iter;
		++i;
	}

	auto data = std::make_shared<Data>(ss.str());
	data->setFreshnessPeriod(time::milliseconds(1000));
	shared_ptr<::ndn::Buffer> buf = std::make_shared<::ndn::Buffer>(allBufBytes);
	data->setContent(buf);
	for (size_t i = 0; i < allBufBytes; ++i) {
		(*buf)[i] = bufBytes[i];
	}
	StackHelper::getKeyChain().sign(*data);

	return data;
}

shared_ptr<Data> RntpStrategy::constructEcho(EchoInfo* info) {
	stringstream ss;
	ss << nodeInfo->ndnNameSpace << "/ALL/Echo/" << info->sourceNodeID << "/" << info->seqNum;
	string name = ss.str();
	auto data = std::make_shared<Data>(string(ss.str()));
	data->setFreshnessPeriod(time::seconds(10));
	StackHelper::getKeyChain().sign(*data);
	return data;
}

void RntpStrategy::extractEcho(const Data& data, EchoInfo* info) {
	Name dataName = data.getName();
	info->sourceNodeID = stoull(dataName.get(3).toUri(name::UriFormat::DEFAULT));
	info->seqNum = stoull(dataName.get(4).toUri(name::UriFormat::DEFAULT));
}

void RntpStrategy::extractInterest(const Interest& interest, InterestInfo* info) {
	info->prefix = interest.getName().toUri(name::UriFormat::DEFAULT);
	struct RouteTagInfo routeTagInfo;
	this->extractRouteTag(interest, &routeTagInfo);
	info->consumerNodeID = routeTagInfo.consumerNodeID;
	info->nextHopNodeID = routeTagInfo.nextHopNodeID;
}

void RntpStrategy::extractCapsuleInfo(const Data& data, CapsuleInfo* info) {
	Name dataName = data.getName();
	stringstream ss;
	ss << "/" << dataName.get(0).toUri(name::UriFormat::DEFAULT) << "/" << dataName.get(1).toUri(name::UriFormat::DEFAULT);
	info->prefix = ss.str();
	info->dataID = stoull(dataName.get(3).toUri(name::UriFormat::DEFAULT));
	if (dataName.size() >= 7) {
		info->nonce = stoull(dataName.get(4).toUri(name::UriFormat::DEFAULT));
		info->transHopNodeID = stoull(dataName.get(5).toUri(name::UriFormat::DEFAULT));
		info->nodeIDs.clear();
		string nodeIDsStr = dataName.get(6).toUri(name::UriFormat::DEFAULT);
		char* str = const_cast<char*>(nodeIDsStr.c_str());
		std::string s;
		std::vector<std::string> elems;
		char* splitted = strtok(str, "-");
		while (splitted != NULL) {
			info->nodeIDs.push_back(stoull(splitted));
			splitted = strtok(NULL, "-");
		}

		info->nHops = stoull(dataName.get(7).toUri(name::UriFormat::DEFAULT));

	} else {
		info->nonce = 0;
		info->nodeIDs.clear();
		info->transHopNodeID = 0xffffffff;
		info->nHops = 0;
	}

	struct RouteTagInfo routeTagInfo;
	this->extractRouteTag(data, &routeTagInfo);
	info->consumerNodeID = routeTagInfo.consumerNodeID;
}

void RntpStrategy::extractInterestBroadcastInfo(const Data& data, InterestBroadcastInfo* info) {
	Name dataName = data.getName();
	stringstream ss;
	ss << "/" << dataName.get(0).toUri(name::UriFormat::DEFAULT) << "/" << dataName.get(1).toUri(name::UriFormat::DEFAULT);
	info->producerPrefix = ss.str();
	info->hopCount = stoull(dataName.get(3).toUri(name::UriFormat::DEFAULT));
	info->consumerNodeID = stoull(dataName.get(4).toUri(name::UriFormat::DEFAULT));
	info->transHopNodeID = stoull(dataName.get(5).toUri(name::UriFormat::DEFAULT));
	info->nonce = stoull(dataName.get(6).toUri(name::UriFormat::DEFAULT));
	info->end = dataName.get(7).toUri(name::UriFormat::DEFAULT).compare("true") == 0;

	info->visitedNodeIDs.clear();
	info->channelQualities.clear();

	const Block& payload = data.getContent();
	const uint8_t* buf = payload.value();

	size_t* nNodeIdRegion = (size_t*) buf;
	uint32_t* nodeIdRegion = (uint32_t*) (buf + sizeof(size_t));
	size_t n = nNodeIdRegion[0];

	for (size_t i = 0; i < n; ++i) {
		uint32_t nodeId = nodeIdRegion[i];
		info->visitedNodeIDs.push_back(nodeId);
	}

	size_t* nchannelQualityRegion = (size_t*) (buf + sizeof(size_t) + n * sizeof(uint32_t));
	double* channelQualityRegion = (double*) (buf + sizeof(size_t) * 2 + n * sizeof(uint32_t));
	size_t nchannelQualities = nchannelQualityRegion[0];

	for (size_t i = 0; i < nchannelQualities; ++i) {
		double channelQuality = channelQualityRegion[i];
		info->channelQualities.push_back(channelQuality);
	}
}

void RntpStrategy::extractRouteTag(const Data& data, RouteTagInfo* info) {
	shared_ptr<lp::LltcConsumerIdTag> consumerIdTag = data.getTag<lp::LltcConsumerIdTag>();
	if (consumerIdTag != NULL) {
		uint64_t tagValue = consumerIdTag->get();
		info->consumerNodeID = (uint32_t) (tagValue >> 32);
		info->nextHopNodeID = (uint32_t) tagValue;
	} else {
		info->consumerNodeID = this->nodeInfo->nodeID;
		info->nextHopNodeID = this->nodeInfo->nodeID;
	}
}

void RntpStrategy::extractRouteTag(const Interest& interest, RouteTagInfo* info) {
	shared_ptr<lp::LltcConsumerIdTag> consumerIdTag = interest.getTag<lp::LltcConsumerIdTag>();
	if (consumerIdTag != NULL) {
		uint64_t tagValue = consumerIdTag->get();
		info->consumerNodeID = (uint32_t) (tagValue >> 32);
		info->nextHopNodeID = (uint32_t) tagValue;
	} else {
		info->consumerNodeID = 0xffffffff;
		info->nextHopNodeID = 0xffffffff;
	}
}

void RntpStrategy::extractPhyInfo(const Interest& interest, PhyInfo& info) {
	shared_ptr<lp::LltcSnrTag> snrTag = interest.getTag<lp::LltcSnrTag>();
	if (snrTag != NULL) {
		info.snr = RatioToDb(snrTag->getSnr());
		info.rssi = snrTag->getRssi();
	} else {
		info.snr = -1;
		info.rssi = -1;
	}
}

void RntpStrategy::extractPhyInfo(const Data& data, PhyInfo& info) {
	shared_ptr<lp::LltcSnrTag> snrTag = data.getTag<lp::LltcSnrTag>();
	if (snrTag != NULL) {
		info.snr = snrTag->getSnr();
		info.rssi = snrTag->getRssi();
	} else {
		info.snr = -1;
		info.rssi = -1;
	}
}

void RntpStrategy::extractCapsuleACK(const Data& data, CapsuleACKInfo& info) {
	Name dataName = data.getName();
	stringstream ss;
	ss << "/" << dataName.get(0).toUri(name::UriFormat::DEFAULT) << "/" << dataName.get(1).toUri(name::UriFormat::DEFAULT);
	info.prefix = ss.str();
	info.downstreamNodeID = stoull(dataName.get(3).toUri(name::UriFormat::DEFAULT));
	string upstreamNodeIDs_str = dataName.get(4).toUri(name::UriFormat::DEFAULT);

	char* str = const_cast<char*>(upstreamNodeIDs_str.c_str());
	std::string s;
	std::vector<std::string> elems;
	char* splitted = strtok(str, "-");
	while (splitted != NULL) {
		info.upstreamNodeIDs.push_back(stoull(splitted));
		splitted = strtok(NULL, "-");
	}
	info.transHopNodeID = stoull(dataName.get(5).toUri(name::UriFormat::DEFAULT));
	info.consumerNodeID = stoull(dataName.get(6).toUri(name::UriFormat::DEFAULT));
	info.dataIDsReceived.clear();

	const Block& payload = data.getContent();
	const uint8_t* buf = payload.value();

	size_t* nDataIdRegion = (size_t*) buf;
	uint32_t* dataIdRegion = (uint32_t*) (buf + sizeof(size_t));
	size_t n = nDataIdRegion[0];

	for (size_t i = 0; i < n; ++i) {
		uint32_t dataId = dataIdRegion[i];
		info.dataIDsReceived.push_back(dataId);
	}
}


void RntpStrategy::writeRouteTag(const RouteTagInfo& info, Data& data) {
	uint64_t tagValue = ((uint64_t) info.consumerNodeID << 32) | (uint64_t) info.nextHopNodeID;
	data.setTag<lp::LltcConsumerIdTag>(make_shared<lp::LltcConsumerIdTag>(tagValue));
}

void RntpStrategy::writeRouteTag(const RouteTagInfo& info, Interest& interest) {
	uint64_t tagValue = ((uint64_t) info.consumerNodeID << 32) | (uint64_t) info.nextHopNodeID;
	interest.setTag<lp::LltcConsumerIdTag>(make_shared<lp::LltcConsumerIdTag>(tagValue));
}

}
}
