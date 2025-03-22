#ifndef GENERIC_CONSUMER_H
#define GENERIC_CONSUMER_H

#include "ns3/ndnSIM/model/ndn-common.hpp"

#include "ndn-consumer.hpp"
#include "ns3/traced-value.h"
#include "ns3/ndnSIM/utils/batches.hpp"
#include <string>
#include <list>
#include <unordered_map>
#include <queue>
#include <fstream>

using namespace std;

namespace ns3 {
namespace ndn {

struct CapsuleInfoC {
	string 			prefix;
	uint32_t 		dataID;
	uint32_t 		transHopNodeID;
	list<uint32_t> 	nodeIDs;
	uint32_t 		nonce;
	uint32_t		nHops;
};

struct InterestBroadcastInfoC {
	uint32_t 		hopCount;
	string 			producerPrefix;
	uint32_t 		consumerNodeID;
	uint32_t		transHopNodeID;
	uint32_t		nonce;
	list<uint32_t> 	visitedNodeIDs;
	list<double>	channelQualities;
	bool			end;
};


struct DataElement {
	CapsuleInfoC du;
	ns3::Time arriveTime;
	shared_ptr<const Data> data;

	friend bool operator < (const DataElement& de1, const DataElement& de2) {
		return de1.du.dataID > de2.du.dataID;
	}
};

class GenericConsumer;
class RntpResequenceQueue;

class GenericConsumer : public Consumer {
public:
	static TypeId
	GetTypeId();
	GenericConsumer();
	void setNodeID(uint32_t nodeID);
	void setMaxWaitTime(ns3::Time maxWaitTime);
	virtual void OnData(shared_ptr<const Data> contentObject);
	int32_t getNRecvCapsules();
	void onDataReseq(CapsuleInfoC du, shared_ptr<const Data> data);
	void extractCapsuleInfo(const Data& data, CapsuleInfoC* info);
	shared_ptr<Data> constructInterestBroadcast(InterestBroadcastInfoC* info);
	void logMsgCapsule(ofstream* log, CapsuleInfoC& info);
	void setToTerminateTransport(ns3::Time delayToTerminate);
	void terminateTransport();

protected:
	virtual void ScheduleNextPacket();

private:
	virtual void StartApplication();
	int32_t 										n_recvCapsules;
	uint32_t 										nodeID;
	RntpResequenceQueue* 							queue;
	ns3::Time 										maxWaitTime;
	ns3::Time 										delayToTerminateTransport;
	bool	  										needToTerminateTransport;
	static ns3::Ptr<ns3::UniformRandomVariable> 	rand;
};


class RntpResequenceQueue {
public:
	RntpResequenceQueue(int queueSize, ns3::Time maxWaitTime);
	void receiveData(CapsuleInfoC du, shared_ptr<const Data> data);
	void releaseQueue(ns3::Time curTime);
	void boundApp(GenericConsumer* app);
	size_t getQueueSize();

private:
	void sendData(CapsuleInfoC du, shared_ptr<const Data> data);
	void autoDequeueTask(RntpResequenceQueue* rq);

	size_t size;
	ns3::Time maxWaitTime;
	int64_t lastDataId;

	priority_queue<DataElement> q_seq;
	queue<DataElement> q_time;

	ns3::EventId curAutoDequeueTaskEvent;
	string prefix;
	GenericConsumer* app;
};

} // namespace ndn
} // namespace ns3

#endif
