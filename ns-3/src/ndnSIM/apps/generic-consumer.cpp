#include "generic-consumer.hpp"
#include "ns3/ptr.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/packet.h"
#include "ns3/callback.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/ndnSIM/helper/ndn-stack-helper.hpp"
#include "ns3/ndnSIM/model/rntp-utils.hpp"

#include "utils/batches.hpp"

NS_LOG_COMPONENT_DEFINE("ndn.GenericConsumer");

using namespace ns3::ndn;

namespace ns3 {
namespace ndn {

NS_OBJECT_ENSURE_REGISTERED(GenericConsumer);

Ptr<ns3::UniformRandomVariable> GenericConsumer::rand = NULL;

TypeId
GenericConsumer::GetTypeId(void)
{
	static TypeId tid =
			TypeId("ns3::ndn::GenericConsumer")
			.SetGroupName("Ndn")
			.SetParent<Consumer>()
			.AddConstructor<GenericConsumer>();

	return tid;
}

GenericConsumer::GenericConsumer()
{
	m_interestLifeTime = Seconds(10);
	n_recvCapsules = 0;
	queue = NULL;
	needToTerminateTransport = false;
}

void GenericConsumer::setNodeID(uint32_t nodeID) {
	this->nodeID = nodeID;
}

void GenericConsumer::setToTerminateTransport(ns3::Time delayToTerminate) {
	this->delayToTerminateTransport = delayToTerminate;
	this->needToTerminateTransport = true;
}

void
GenericConsumer::StartApplication()
{
	App::StartApplication();

	shared_ptr<Interest> interest = make_shared<Interest>();
	interest->setNonce(m_rand->GetValue(0, std::numeric_limits<uint32_t>::max()));
	interest->setName(m_interestName);
	time::milliseconds interestLifeTime(m_interestLifeTime.GetMilliSeconds());
	interest->setInterestLifetime(interestLifeTime);

	std::cout << "==> nonce: " << interest->getNonce() << std::endl;

	m_transmittedInterests(interest, this, m_face);
	m_appLink->onReceiveInterest(*interest);

	queue = new RntpResequenceQueue(2000, maxWaitTime);
	queue->boundApp(this);

	ofstream* log = RntpUtils::getLogConsumer();
	*log << this->nodeID << "," << Simulator::Now() << ",s,Interest," << m_interestName << endl;

	if (this->needToTerminateTransport) {
		Simulator::Schedule(delayToTerminateTransport, &GenericConsumer::terminateTransport, this);
	}
}

void GenericConsumer::terminateTransport() {
	if (GenericConsumer::rand == NULL) {
		GenericConsumer::rand = ns3::CreateObject<ns3::UniformRandomVariable>();
		GenericConsumer::rand->SetAttribute ("Min", ns3::DoubleValue (1.0));
		GenericConsumer::rand->SetAttribute ("Max", ns3::DoubleValue ((double) UINT_MAX));
	}

	InterestBroadcastInfoC info;
	info.producerPrefix = m_interestName.toUri(name::UriFormat::DEFAULT);
	info.consumerNodeID = this->nodeID;
	info.nonce = rand->GetInteger();
	info.transHopNodeID = this->nodeID;
	info.end = true;
	info.hopCount = 0;

	shared_ptr<Data> data = this->constructInterestBroadcast(&info);
	m_transmittedDatas(data, this, m_face);
	m_appLink->onReceiveData(*data);

	cout << "GenericConsumerApp: send terminateTransport with prefix: " << info.producerPrefix
			<< ", consumerNodeID: " << info.consumerNodeID << ", time: " << Simulator::Now() << endl;
}

void
GenericConsumer::ScheduleNextPacket()
{
}

void GenericConsumer::setMaxWaitTime(ns3::Time maxWaitTime) {
	this->maxWaitTime = maxWaitTime;
}

void GenericConsumer::OnData(shared_ptr<const Data> data) {
	CapsuleInfoC capInfo;
	this->extractCapsuleInfo(*data, &capInfo);
	ofstream* log = RntpUtils::getLogConsumer();
	logMsgCapsule(log, capInfo);
	std::cout << "consumer recv Data with prefix " << capInfo.prefix << ", dataID: " << capInfo.dataID << ", time: " << Simulator::Now().GetSeconds() << std::endl;

	queue->receiveData(capInfo, data);
	ofstream* log_q = RntpUtils::getLogConsumerQueueSize();
	*log_q << Simulator::Now() << "," << queue->getQueueSize() << endl;

	n_recvCapsules++;
}

void GenericConsumer::extractCapsuleInfo(const Data& data, CapsuleInfoC* info) {
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
}


shared_ptr<Data> GenericConsumer::constructInterestBroadcast(InterestBroadcastInfoC* info) {
	stringstream ss;
	ss << info->producerPrefix << "/InterestBroadcast/" << info->hopCount <<
			"/" << info->consumerNodeID <<
			"/" << info->transHopNodeID <<
			"/" << info->nonce << "/" << (info->end ? "true" : "false");

	string name = ss.str();

	size_t nNodeIdsBytes = sizeof(size_t);
	size_t nchannelQualitiesBytes = sizeof(size_t);
	size_t allBufBytes = nNodeIdsBytes + nchannelQualitiesBytes;
	uint8_t* bufBytes = new uint8_t[allBufBytes];
	size_t* nNodeIdRegion = (size_t*) bufBytes;
	nNodeIdRegion[0] = 0;

	size_t* nchannelQualities = (size_t*) (bufBytes + nNodeIdsBytes);
	nchannelQualities[0] = 0;

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

void GenericConsumer::logMsgCapsule(ofstream* log, CapsuleInfoC& info) {
	*log << this->nodeID << "," << Simulator::Now() << ",r,Data," << info.transHopNodeID << ","
			<< info.prefix << "," << info.dataID << ",";
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

int32_t GenericConsumer::getNRecvCapsules() {
	return n_recvCapsules;
}

void GenericConsumer::onDataReseq(CapsuleInfoC du, shared_ptr<const Data> data) {
	ofstream* log = RntpUtils::getLogConsumerReseq();
	this->logMsgCapsule(log, du);
}

RntpResequenceQueue::RntpResequenceQueue(int queueSize, ns3::Time maxWaitTime) {
	this->size = queueSize;
	this->maxWaitTime = maxWaitTime;
	this->lastDataId = -1;
	this->app = NULL;
}

void RntpResequenceQueue::boundApp(GenericConsumer* app) {
	this->app = app;
}

void RntpResequenceQueue::receiveData(CapsuleInfoC du, shared_ptr<const Data> data) {
	if (lastDataId == -1) {
		sendData(du, data);
		lastDataId = du.dataID;
		return;
	} else if (du.dataID == lastDataId + 1) {
		sendData(du, data);
		++lastDataId;
		return;
	}

	if (q_seq.size() == size) {
		DataElement _d = q_seq.top();
		q_seq.pop();
		sendData(_d.du, _d.data);
		lastDataId = _d.du.dataID;
	}

	ns3::Time curTime = Simulator::Now();
	DataElement de;
	de.arriveTime = curTime;
	de.data = data;
	de.du = du;
	q_seq.push(de);
	q_time.push(de);

	releaseQueue(curTime);

	if (q_seq.size() > 0) {
		if (curAutoDequeueTaskEvent.IsRunning()) {
			Simulator::Remove(curAutoDequeueTaskEvent);
		}

		DataElement _d = q_time.front();
		Simulator::Schedule(_d.arriveTime + maxWaitTime - curTime, &RntpResequenceQueue::autoDequeueTask, this, this);
	}
}

size_t RntpResequenceQueue::getQueueSize() {
	return q_seq.size();
}

void RntpResequenceQueue::releaseQueue(ns3::Time curTime) {
	while (q_seq.size() > 0) {
		DataElement _d = q_seq.top();
		if (_d.du.dataID == lastDataId + 1) {
			q_seq.pop();
			sendData(_d.du, _d.data);
			++lastDataId;
		} else {
			break;
		}
	}

	if (q_seq.size() == 0) return;

	DataElement d_oldest;
	int64_t dataId_max = -1;
	while (q_time.size() > 0) {
		d_oldest = q_time.front();
		if (d_oldest.arriveTime <= curTime - maxWaitTime) {
			q_time.pop();
			if (dataId_max < d_oldest.du.dataID) {
				dataId_max = d_oldest.du.dataID;
			}
		} else {
			break;
		}
	}
	if (dataId_max == -1) return;

	while (q_seq.size() > 0) {
		DataElement _d = q_seq.top();
		if (_d.du.dataID <= dataId_max) {
			q_seq.pop();
			sendData(_d.du, _d.data);
			lastDataId = _d.du.dataID;
		} else {
			break;
		}
	}
}

void RntpResequenceQueue::sendData(CapsuleInfoC du, shared_ptr<const Data> data) {
	if (this->app != NULL) {
		this->app->onDataReseq(du, data);
	}
}

void RntpResequenceQueue::autoDequeueTask(RntpResequenceQueue* rq) {
	rq->releaseQueue(Simulator::Now());
}

} // namespace ndn
} // namespace ns3
