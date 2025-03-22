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

#ifndef SRC_NDNSIM_NFD_DAEMON_FW_R2T_STRATEGY_HPP_
#define SRC_NDNSIM_NFD_DAEMON_FW_R2T_STRATEGY_HPP_

#include "ns3/random-variable-stream.h"

#include "strategy.hpp"
#include "algorithm.hpp"
#include "common/global.hpp"
#include "common/logger.hpp"
#include "ndn-cxx/tag.hpp"
#include "NFD/daemon/face/face-endpoint.hpp"
#include <list>
#include <unordered_map>
#include <queue>

#include "generic-routes-manager.hpp"
#include "ns3/nist-error-rate-model.h"
#include "ns3/double.h"

using namespace ::nfd;
using namespace std;

namespace nfd {
namespace fw {

class RntpStrategy;
typedef void (*route_searched_callback) (RntpStrategy*, unsigned int);  // parameter: rreqId

#define QUALITY_BROKEN			-1000000000.0

struct PhyInfo {
	double snr;
	double rssi;
};

struct RouteTagInfo {
	uint32_t 	consumerNodeID;
	uint32_t 	nextHopNodeID;
};

struct InterestInfo {
	string 			prefix;
	uint32_t 		consumerNodeID;
	uint32_t 		nextHopNodeID;
};

struct CapsuleInfo {
	string 			prefix;
	uint32_t 		dataID;
	uint32_t 		consumerNodeID;
	uint32_t 		transHopNodeID;
	list<uint32_t> 	nodeIDs;
	uint32_t 		nonce;
	uint32_t		nHops;
};

struct CapsuleACKInfo {
	string 			prefix;
	list<uint32_t> 	dataIDsReceived;
	uint32_t 		downstreamNodeID;
	list<uint32_t>	upstreamNodeIDs;
	uint32_t		transHopNodeID;
	uint32_t		consumerNodeID;
};

struct InterestBroadcastInfo {
	uint32_t 		hopCount;
	string 			producerPrefix;
	uint32_t 		consumerNodeID;
	uint32_t		transHopNodeID;
	uint32_t		nonce;
	list<uint32_t> 	visitedNodeIDs;
	list<double>	channelQualities;
	bool			end;
};

struct EchoInfo {
	uint32_t		sourceNodeID;
	uint32_t 		seqNum;
};

struct Route {
	uint32_t 		id;
	uint32_t 		n_hops;
	list<uint32_t> 	nodeIDs;
	list<double> 	channelQualities;
	ns3::Time 		updateTime;
	double			metric;
};

struct RoutesPerPair {
	uint32_t 		consumerNodeID;
	string 			producerPrefix;
	vector<Route*> 	routes;
	ns3::Time 		lastHitTime;
};

typedef vector<RoutesPerPair*> route_table;


struct CapsuleToSend {
	CapsuleInfo capInfo;
	Data data;
	uint32_t nTimesRetried;
	int code;
	bool hidden;
};

class CapsuleQueue {
private:
    std::list<CapsuleToSend> buffer;

public:
    CapsuleQueue();
    void removeElement(int id);
    void pushElement(CapsuleToSend& element);
    void transientlyPopElement();
    CapsuleToSend* restoreElement(int id);
    CapsuleToSend* getFront();
    size_t countElements();
    void setNodeID(uint32_t nodeID);
    bool isDataIDInBuffer(uint32_t dataID);
    void logBuffer();
    uint32_t nHiddenElements;
    uint32_t nodeID;
    unordered_set<uint32_t> dataIDsInBuffer;
};

struct ThroughputElement {
    uint32_t secs;
    uint32_t packet_count;
};

class ThroughputQueue {
public:
    ThroughputQueue(size_t size);
    ~ThroughputQueue();

    void recordPacketArrival(ns3::Time time);
    ns3::Time estimateLongestPIAT(double confidentRatio, ns3::Time maxPIAT);

private:
    double countPacketMeanArrivalRate();
    void enqueue(ThroughputElement item);
    ThroughputElement dequeue();
    ThroughputElement* rear();
    bool isEmpty();
    bool isFull();
    size_t size();

private:
    ThroughputElement* buffer;
    size_t capacity;
    size_t frontIndex;
    size_t rearIndex;
    size_t count;
};


struct InterestBroadcastStates {
	ns3::Time recvTime;
	uint32_t nonce;
};

struct SendCapState {
	ns3::EventId 				sendEventID;
	uint32_t 					sendTimes;
	unordered_set<uint32_t> 	nodeIDs_down;
};

struct TransportStates {
	string									prefix;
	uint32_t 								consumerID;
	InterestBroadcastStates*				receivedInterestBroadcasts;
	unordered_set<uint64_t> 				sentDataIDAndNextHops;
	unordered_map<uint32_t, SendCapState*> 	sendCapStates;
	int										window;
	int 									slowStartThres;
	CapsuleQueue							capSendQueue;
};

struct ChannelQualityStates {
	uint32_t 			fromNodeID;
	double 				quality_smooth;
	ns3::EventId 		waitMsgEvent;
	ThroughputQueue* 	thQueue;
};

class RntpCongestionControl {
public:
	void init(TransportStates* ts,  NodeInfo* nodeInfo);
	void onCapAckReceived(TransportStates* ts,  NodeInfo* nodeInfo);
	void onCapAckTimeout(TransportStates* ts,  NodeInfo* nodeInfo, int nextHopID);
	void onChannelWaken(TransportStates* ts,  NodeInfo* nodeInfo, int fromNodeID);
	void log(string reason, TransportStates* ts,  NodeInfo* nodeInfo);
};

struct SendCapsuleIterParam {
	uint32_t nTimesRetried;
	uint32_t transHopNodeID_prev;
	int code;
};

#define SEND_CAP_NORM			 		   0
#define SEND_CAP_ALREADY_SENT 			  -1
#define SEND_CAP_NO_ROUTE				  -2
#define SEND_CAP_EXCEED_MAX_RETRYING_TIME -3
#define SEND_CAP_CANCELED				  -4
#define SEND_CAP_DUPLICATED				  -5

#define SEND_CAP_FROM_PRODUCER			   0
#define SEND_CAP_FROM_PERVIOUS_HOP   	   1
#define SEND_CAP_FOR_RETRYING			   2

#define CAP_ARRIVAL_FROM_UPSTREAM			0
#define CAP_ARRIVAL_FROM_DOWNSTREAM			1
#define CAP_ARRIVAL_FROM_OTHERS				2

#define DEAL_ACK_NO_NEED					0
#define DEAL_ACK_DONE						1

class RntpStrategy : public Strategy {
public:
	RntpStrategy(Forwarder& forwarder, const Name& name = getStrategyName());
	virtual ~RntpStrategy() override;
    static const Name& getStrategyName();

    void afterReceiveInterest(const FaceEndpoint& ingress, const Interest& interest,
	                       const shared_ptr<pit::Entry>& pitEntry) override;
    void afterReceiveData(const shared_ptr<pit::Entry>& pitEntry,
                     const FaceEndpoint& ingress, const Data& data) override;
    void afterReceiveNonPitData(const FaceEndpoint& ingress, const Data& data);
    void onReceiveCapsule(const FaceEndpoint& ingress, const Data& data);
    void onReceiveInterestBroadcast(const FaceEndpoint& ingress, const Data& data);
    void onReceiveCapsuleAck(const FaceEndpoint& ingress, const Data& data);
    void onReceiveEcho(const FaceEndpoint& ingress, const Data& data);

    void propagateInterests(InterestBroadcastStates* rreqStates, string producerPrefix,
    						list<uint32_t>& visitedNodeIDs, list<double>& channelQualities,
    						uint32_t consumerNodeID, uint32_t initialHopCount, bool end);

    void propagateInterestsAsync(InterestBroadcastInfo* info, shared_ptr<Data> data, uint32_t times);

    int sendCapsule(TransportStates* ts, CapsuleInfo& capInfo, uint32_t transHopNodeID_prev,
    				const Data& data, uint32_t nTimesRetried, int code);
    int sendCapsuleIterative(TransportStates* ts, CapsuleInfo& capInfo, const Data& data, SendCapsuleIterParam params);
    void sendCapsuleACK(string prefix, list<uint32_t>& dataIDsReceived, uint32_t downstreamNodeID,
    					list<uint32_t> upstreamNodeIDs, uint32_t transHopNodeID, uint32_t consumerNodeID);
    bool checkIfCapsuleArrivedInDownstream(CapsuleInfo& capInfo);
    int checkCapsuleArrivalDirection(CapsuleInfo& capInfo);
    unordered_set<uint32_t> sendCapsulesInQueue(TransportStates* ts);
    void sendCapsuleViaQueue(TransportStates* ts, CapsuleInfo& capInfo, const Data& data,
    								int code);
    uint64_t hashDataIDAndNextHopID(uint32_t dataID, uint32_t nextHopID);
    uint64_t getHash(CapsuleInfo& capInfo);
    int getNextHop(list<uint32_t>& nodeIDs);
    int dealWithAck(TransportStates* ts, uint32_t dataID, uint32_t downstreamNodeID, uint32_t transNodeID, uint32_t reason);

    void refreshRouteMetric(uint32_t consumerNodeID, string producerPrefix);
    Route* lookupRoute(uint32_t consumerNodeID, string producerPrefix,
    					list<uint32_t>* nodeIDs_prev, uint32_t rank);
    Route* matchRoute(uint32_t consumerNodeID, string producerPrefix, list<uint32_t> nodeIDs, list<uint32_t>* nodeIDs_prev);
    void dumpRoute(uint32_t consumerNodeID, string producerPrefix);
    void addRoute(string producerPrefix, uint32_t consumerNodeID, uint32_t n_hops,
    					list<uint32_t>& nodeIDs, list<double>& channelQualities);
    double getWorstChannelQuality(list<double>& channelQualities);
    double getMeanChannelQuality(list<double>& channelQualities);
    int updateRoutesWithQuality(uint32_t fromNodeID, uint32_t toNodeID, double channelQuality);
    void updateChannelQuality(uint32_t fromNodeID, double quality);
    list<uint32_t> getUpstreamNodeIDsFromCapsule(list<uint32_t> nodeIDs, uint32_t transHopNodeID);

    void markChannelBroken(uint32_t fromNodeID, string reason);

    void sendEchoInPeriodical();
    double findEquivalentQualityOfBestRoute(uint32_t upstreamNeighboredNodeID);

    set<uint32_t> getNeighboredIDsInRoutes();
    set<uint32_t> getNeighboredIDsInRoutes(uint32_t consumerID, string prefix);
    string hashPrefixAndConsumerID(string prefix, uint32_t consumerID);
    void addDownStreamNodes(unordered_set<uint32_t>* A, list<uint32_t>& B);

    int32_t findPrefix(string prefix);

    void sendLltcNonPitData(Data& data);

    shared_ptr<Interest> constructInterest(string pitPrefixStr, uint32_t consumerNodeID, uint32_t nextHopNodeID);
    shared_ptr<Data> constructCapsule(CapsuleInfo* info, list<uint32_t>& nodeIDs, const Data& origData, uint32_t nHops);
    shared_ptr<Data> constructInterestBroadcast(InterestBroadcastInfo* info);
    shared_ptr<Data> constructCapsuleACK(CapsuleACKInfo& info);
    shared_ptr<Data> constructEcho(EchoInfo* info);

    void extractEcho(const Data& data, EchoInfo* info);
    void extractInterest(const Interest& interest, InterestInfo* info);
    void extractCapsuleInfo(const Data& data, CapsuleInfo* info);
    void extractInterestBroadcastInfo(const Data& data, InterestBroadcastInfo* info);
    void extractCapsuleACK(const Data& data, CapsuleACKInfo& info);
    void extractRouteTag(const Data& data, RouteTagInfo* info);
    void extractRouteTag(const Interest& interest, RouteTagInfo* info);
    void writeRouteTag(const RouteTagInfo& info, Data& data);
    void writeRouteTag(const RouteTagInfo& info, Interest& interest);
    void extractPhyInfo(const Interest& interest, PhyInfo& info);
    void extractPhyInfo(const Data& data, PhyInfo& info);
    double estimateBER(double snr);

    void logMsgInterest(bool isRecv, InterestInfo& info, PhyInfo* phyInfo);
    void logMsgInterestBroadcast(bool isRecv, InterestBroadcastInfo& info, PhyInfo* phyInfo);
    void logMsgCapsule(bool isRecv, CapsuleInfo& info, PhyInfo* phyInfo);
    void logMsgCapAck(bool isRecv, CapsuleACKInfo& info, PhyInfo* phyInfo);
    void logMsgEcho(bool isRecv, EchoInfo& info, PhyInfo* phyInfo);
    void logRoutes();

    // map from node ID to a dict of <Message name, counter>
    static unordered_map<int, unordered_map<string, int>*> 	performance_res;
    static ns3::Ptr<ns3::UniformRandomVariable> 			rand;
    static bool												enableLog_msgs;

private:
    void initializeFaces();

    NodeInfo* 										nodeInfo;

    unordered_map<string, TransportStates*> 		transportStates_all;

    uint32_t 										route_id_cur;
    route_table 									routes_all;

    unordered_map<uint32_t, ChannelQualityStates> 	channelQualities;
    unordered_set<uint64_t> 						receivedChannelQualityUpdates;
    ns3::EventId 									updateChannelQualityEvent;
    bool											underPreparationToSendCQUpdate;
    uint32_t 										channelQualityUpdate_seqno_cur;

    ns3::EventId 									sendEchoEvent;
    uint32_t 										echo_seqnum;

    bool 											is_face_found;
    ::nfd::face::Face* 								face_netdev;
    ::nfd::face::Face* 								face_app;

    RntpCongestionControl*							congestionControl;
};

void initializeRntpStrategyRand();

}
}


#endif /* SRC_NDNSIM_NFD_DAEMON_FW_R2T_STRATEGY_HPP_ */
