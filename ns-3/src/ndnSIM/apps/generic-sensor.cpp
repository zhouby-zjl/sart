#include "generic-sensor.hpp"

#include "ns3/ptr.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/packet.h"
#include "ns3/random-variable-stream.h"
#include "ns3/ndnSIM/ndn-cxx/lp/tags.hpp"

#include "ns3/ndnSIM/helper/ndn-stack-helper.hpp"
#include "ns3/ndnSIM/helper/ndn-fib-helper.hpp"
//#include "ns3/ndnSIM/NFD/daemon/fw/lltc-common.hpp"
//#include "ns3/ndnSIM/NFD/daemon/fw/lltc-fs.hpp"
//#include "lltc-utils.hpp"

#include "ns3/ndnSIM/model/generic-log.hpp"
#include "ns3/ndnSIM/ndn-cxx/lp/tags.hpp"
#include "ns3/ndnSIM/model/rntp-utils.hpp"
#include "ns3/double.h"

#include <string>

using namespace ns3;
using namespace ns3::ndn;
using namespace std;
using namespace ::nfd::fw;

namespace generic {

NS_LOG_COMPONENT_DEFINE("generic.GenericSensorApp");

NS_OBJECT_ENSURE_REGISTERED(GenericSensorApp);

GenericSensorApp::GenericSensorApp() {
	n_CapsuleSent = 0;
	dataId = 0;

	rand = ns3::CreateObject<ns3::UniformRandomVariable>();
	rand->SetAttribute ("Min", ns3::DoubleValue (1.0));
	rand->SetAttribute ("Max", ns3::DoubleValue ((double) UINT_MAX));
}

TypeId GenericSensorApp::GetTypeId() {
	static TypeId tid = TypeId("generic::GenericSensorApp")
							.SetGroupName("Generic")
							.SetParent<App>().AddConstructor<GenericSensorApp>();
	return tid;
}


void GenericSensorApp::StartApplication() {
	App::StartApplication();
	FibHelper::AddRoute(GetNode(), sensorName, m_face, 0);
}

void GenericSensorApp::StopApplication() {
}

void GenericSensorApp::DoInitialize() {
	App::DoInitialize();
}

void GenericSensorApp::DoDispose() {
	App::DoDispose();
}

void GenericSensorApp::setSensorName(string sensorName) {
	this->sensorName = sensorName;
}

void GenericSensorApp::setNodeID(uint32_t nodeID) {
	this->nodeID = nodeID;
}

void GenericSensorApp::setFreq(int freq) {
	piat = MicroSeconds(1000000.0 / (double) freq);
}

void GenericSensorApp::OnInterest(std::shared_ptr<const Interest> interest) {
	cout << "GenericSensorApp: receive Interest packet with the nonce of " << interest->getNonce() << endl;
	App::OnInterest(interest);
	string prefix = interest->getName().toUri(::ndn::name::UriFormat::DEFAULT);

	shared_ptr<::ndn::lp::LltcConsumerIdTag> consumerIdTag = interest->getTag<::ndn::lp::LltcConsumerIdTag>();
	uint32_t consumerNodeID = 0;

	if (consumerIdTag != NULL) {
		uint64_t tagValue = consumerIdTag->get();
		consumerNodeID = (uint32_t) (tagValue >> 32);
	} else {
		consumerNodeID = 0xffffffff;
	}

	ofstream* log = RntpUtils::getLogProducer();
	*log << this->nodeID << "," << Simulator::Now() << ",r,Interest," << consumerNodeID << endl;

	sendData(consumerNodeID);

}

void GenericSensorApp::sendData(uint32_t consumerNodeID) {
	stringstream ss;
	ss << sensorName << "/Capsule/" << dataId;
	auto data = std::make_shared<Data>(string(ss.str()));
	data->setFreshnessPeriod(::ndn::time::milliseconds(1000));
	data->setContent(std::make_shared< ::ndn::Buffer>(1024));
	uint64_t tagValue = ((uint64_t) consumerNodeID << 32) | (uint64_t) 0xffffffff;
	data->setTag<::ndn::lp::LltcConsumerIdTag>(make_shared<::ndn::lp::LltcConsumerIdTag>(tagValue));
	data->setTag<::ndn::lp::LltcPathIdTag>(make_shared<::ndn::lp::LltcPathIdTag>(0));

	StackHelper::getKeyChain().sign(*data);
	m_transmittedDatas(data, this, m_face);
	m_appLink->onReceiveData(*data);

	GenericLog::TimeSendingData[dataId] = Simulator::Now();
	GenericLog::dataID_sent.push_back(dataId);

	ofstream* log = RntpUtils::getLogProducer();
	*log << this->nodeID << "," << Simulator::Now() << ",s,Data," << dataId << "," << consumerNodeID << endl;

	cout << "GenericSensorApp: send data " << ss.str() << ", time: " << Simulator::Now() << endl;
	++dataId;
	n_CapsuleSent++;

	if (Simulator::Now() > m_stopTime) {
		return;
	}

	double x = rand->GetValue(-1e3, 1e3);
	ns3::Time interval = piat + ns3::NanoSeconds(x);
	Simulator::Schedule(interval, &GenericSensorApp::sendData, this, consumerNodeID);
}


void GenericSensorApp::OnData(std::shared_ptr<const Data> data) {
	std::cout << "--> Capsule received at sensor with the name of " << data->getName().toUri(::ndn::name::UriFormat::DEFAULT) << " @ NodeID: " << GetNode()->GetId() << std::endl;
}

int32_t GenericSensorApp::getNCapsuleSent() {
	return n_CapsuleSent;
}

}

