#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/ndnSIM-module.h"
#include "ns3/energy-module.h"
#include "ns3/ndnSIM/apps/ndn-consumer-cbr.hpp"
#include "ns3/ndnSIM/apps/generic-sensor.hpp"
#include "ns3/ndnSIM/model/generic-log.hpp"
#include "ns3/ndnSIM/model/ndn-block-header.hpp"
#include "ns3/ndnSIM/apps/ndn-consumer-cbr.hpp"
#include "ns3/ndnSIM/apps/ndn-consumer-batches.hpp"
#include "ns3/ndnSIM/apps/generic-consumer.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/rntp-strategy.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/generic-routes-manager.hpp"
#include "ns3/okumura-hata-propagation-loss-model.h"
#include "ns3/ndnSIM/model/rntp-config.hpp"
#include "ns3/ndnSIM/model/rntp-utils.hpp"
#include "ns3/wifi-radio-energy-model-helper.h"
//#include "ns3/ndnSIM/ndn-cxx/lp/packet.hpp"
#include "time.h"
#include <ctime>
#include <regex>
#include <random>
#include <unordered_map>

using namespace std;
using namespace ns3;
using namespace ::nfd::fw;

set<int32_t> nodeIds_under_noise;
double noise_start_sec;
double noise_stop_sec;

void setNodeIds_under_noise(string s) {
	std::regex rgx(",");
	std::sregex_token_iterator iter(s.begin(), s.end(), rgx, -1);
	std::sregex_token_iterator end;

	while (iter != end) {
		string sub = *iter++;
		if (sub.length() == 0) continue;
		nodeIds_under_noise.insert(atoi(sub.c_str()));
	}
}

namespace ns3 {
class RandomInterferencePropagationLossModel : public PropagationLossModel
{
public:
	static TypeId GetTypeId (void);
	RandomInterferencePropagationLossModel ();
	virtual ~RandomInterferencePropagationLossModel ();

private:
	RandomInterferencePropagationLossModel (const RandomInterferencePropagationLossModel &);
	RandomInterferencePropagationLossModel & operator = (const RandomInterferencePropagationLossModel &);
	virtual double DoCalcRxPower (double txPowerDbm,
			Ptr<MobilityModel> a,
			Ptr<MobilityModel> b) const;
	virtual int64_t DoAssignStreams (int64_t stream);
	int32_t getNodeID(Ptr<ns3::MobilityModel> model) const;
	Ptr<RandomVariableStream> m_variable;
};

NS_OBJECT_ENSURE_REGISTERED (RandomInterferencePropagationLossModel);

TypeId
RandomInterferencePropagationLossModel::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::RandomInterferencePropagationLossModel")
    		.SetParent<PropagationLossModel> ()
			.SetGroupName ("Propagation")
			.AddConstructor<RandomInterferencePropagationLossModel> ()
			.AddAttribute ("Variable", "The random variable used to pick a loss every time CalcRxPower is invoked.",
					StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"),
					MakePointerAccessor (&RandomInterferencePropagationLossModel::m_variable),
					MakePointerChecker<RandomVariableStream> ());
	return tid;
}
RandomInterferencePropagationLossModel::RandomInterferencePropagationLossModel ()
: PropagationLossModel ()
{
}

RandomInterferencePropagationLossModel::~RandomInterferencePropagationLossModel ()
{
}

int32_t RandomInterferencePropagationLossModel::getNodeID(Ptr<ns3::MobilityModel> model) const {
	NodeContainer nodes = NodeContainer::GetGlobal();
	int32_t k = 0;
	for (NodeContainer::Iterator i = nodes.Begin (); i != nodes.End (); ++i) {
		Ptr<Node> node = *i;
		Ptr<Object> object = node;
		Ptr<MobilityModel> m = object->GetObject<MobilityModel> ();
		if (model == m) {
			return k;
		}
		k++;
	}
	return -1;
}

double
RandomInterferencePropagationLossModel::DoCalcRxPower (double txPowerDbm,
		Ptr<MobilityModel> a,
		Ptr<MobilityModel> b) const
{
	int32_t nodeID_a = getNodeID(a);
	int32_t nodeID_b = getNodeID(b);

	double secs = Simulator::Now().GetSeconds();

	if (nodeID_a != -1 && nodeID_b != -1) {
		if ((nodeIds_under_noise.find(nodeID_a) != nodeIds_under_noise.end() ||
				nodeIds_under_noise.find(nodeID_b) != nodeIds_under_noise.end()) &&
				(secs >= noise_start_sec && secs <= noise_stop_sec)) {
			double rxc = -m_variable->GetValue ();
			//cout << "---> add noise " << rxc << " between nodeID_a: " << nodeID_a << ", nodeID_b: " << nodeID_b << " @ secs: " << secs << endl;
			return txPowerDbm + rxc;
		} else {
			return txPowerDbm;
		}
	} else {
		return txPowerDbm;
	}
}

int64_t
RandomInterferencePropagationLossModel::DoAssignStreams (int64_t stream)
{
	m_variable->SetStream (stream);
	return 1;
}
}

void RxDropCallback (std::string context, Ptr<const Packet> p, WifiPhyRxfailureReason reason) {
	Ptr<Packet> packet = p->Copy ();
	WifiMacHeader hdr;
	packet->RemoveHeader (hdr);
	BlockHeader blockHeader;
	packet->RemoveHeader(blockHeader);

}

NS_LOG_COMPONENT_DEFINE("ndn.WifiExample");

vector<int> pos_vec;
int cur_pos = 10;

/*
void advancePos(Ptr<Node> node, int deltaX, int deltaSecs) {
	Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
	Vector pos = mobility->GetPosition ();
	pos.x += deltaX;
	cur_pos = pos.x;
	mobility->SetPosition(pos);

	Simulator::Schedule(Seconds(deltaSecs), &advancePos, node, deltaX, deltaSecs);
}*/

void setNodeInfo(string nameSpace, NodeInfo* ni, uint32_t id) {
	ni->nodeID = id;
	ni->ndnNameSpace = nameSpace;
	ni->interestSendTimes = RntpConfig::INTEREST_SEND_TIMES;
	ni->capsulePerHopTimeout = ns3::Seconds(RntpConfig::CAPSULE_PER_HOP_TIMEOUT);
	ni->capsuleRetryingMaxTimes = RntpConfig::CAPSULE_RETRYING_TIMES;
	ni->congestionControlSlowStartThres = RntpConfig::CONGESTION_CONTROL_THRESHOLD;
	ni->congestionControlInitWin = RntpConfig::CONGESTION_CONTROL_INIT_WIN;
	ni->echoPeriod = ns3::Seconds(RntpConfig::ECHO_PERIOD_IN_SECS);
	ni->msgTimeout = ns3::Seconds(RntpConfig::MSG_TIMEOUT_IN_SECS);
	ni->interestContentionTimeInSecs = RntpConfig::INTEREST_CONTENTION_TIME_IN_SECS;
	ni->quality_alpha = RntpConfig::QUALITY_ALPHA;
	ni->thQueueSize = RntpConfig::THROUGHPUT_QUEUE_SIZE_IN_SECS;
	ni->longestPIATEstCondifentRatio = RntpConfig::PIAT_ESTIMATION_CONFIDENT_RATIO;
}

std::unordered_set<int> generateRandomInterferedNodes(int k, int numNodes, int consumerNodeIdx, int sensorNodeIdx) {
	std::unordered_set<int> result;
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> dist(0, numNodes);

	while (result.size() < k) {
		int num = dist(gen);
		if (num == consumerNodeIdx || num == sensorNodeIdx) {
			continue;
		}
		result.insert(num);
	}

	return result;
}

class DelayedNormalRandomVariable : public NormalRandomVariable
{
public:
	virtual double GetValue (void);
	virtual uint32_t GetInteger (void);
	void setDelayedTime(int64_t delay_microsecs);

private:
	int64_t m_delay_microsecs;
};

void DelayedNormalRandomVariable::setDelayedTime(int64_t delay_microsecs) {
	this->m_delay_microsecs = delay_microsecs;
}

double DelayedNormalRandomVariable::GetValue() {
	int64_t now = Simulator::Now().GetMicroSeconds();
	if (now > m_delay_microsecs)
		return this->GetValue();
	else
		return 0.0;
}

uint32_t DelayedNormalRandomVariable::GetInteger() {
	int64_t now = Simulator::Now().GetMicroSeconds();
	if (now > m_delay_microsecs)
		return this->GetInteger();
	else
		return 0;
}

void RemainingEnergyWrapperCallback (uint32_t nodeID, double oldEnergy, double newEnergy)
{
	auto log = RntpUtils::getLogEnergy();
	*log << nodeID << "," << Simulator::Now() << ",CurRemain," << oldEnergy << "," << newEnergy << endl;
}

void TotalEnergyWrapperCallback (uint32_t nodeID, double oldEnergy, double newEnergy)
{
	auto log = RntpUtils::getLogEnergy();
	*log << nodeID << "," << Simulator::Now() << ",CurTotal," << oldEnergy << "," << newEnergy << endl;
}

double getSourceInitialEnegeryJ(double batteryVoltage, double batteryCapacityInmAh) {
	return batteryVoltage * batteryCapacityInmAh * 0.001 * 3600;
}

void PhyRxDropCallback (std::string context, Ptr<const Packet> packet, WifiPhyRxfailureReason reason)
{
	stringstream ss;
	switch (reason) {
	case UNKNOWN:
		ss << "UNKNOWN";
		break;
	case UNSUPPORTED_SETTINGS:
		ss << "UNSUPPORTED_SETTINGS";
		break;
	case NOT_ALLOWED:
		ss << "NOT_ALLOWED";
		break;
	case ERRONEOUS_FRAME:
		ss << "ERRONEOUS_FRAME";
		break;
	case MPDU_WITHOUT_PHY_HEADER:
		ss << "MPDU_WITHOUT_PHY_HEADER";
		break;
	case PREAMBLE_DETECT_FAILURE:
		ss << "PREAMBLE_DETECT_FAILURE";
		break;
	case L_SIG_FAILURE:
		ss << "L_SIG_FAILURE";
		break;
	case SIG_A_FAILURE:
		ss << "SIG_A_FAILURE";
		break;
	case PREAMBLE_DETECTION_PACKET_SWITCH:
		ss << "PREAMBLE_DETECTION_PACKET_SWITCH";
		break;
	case FRAME_CAPTURE_PACKET_SWITCH:
		ss << "FRAME_CAPTURE_PACKET_SWITCH";
		break;
	case OBSS_PD_CCA_RESET:
		ss << "OBSS_PD_CCA_RESET";
		break;
	}

	Ptr<Packet> nonConstPacket = packet->Copy ();
	BlockHeader header;
	nonConstPacket->RemoveHeader (header);
	Block netPkt = header.getBlock();

	uint32_t type = netPkt.type();
	if (type == tlv::Interest) {
		auto interest = make_shared<Interest>(netPkt);
		string prefix = interest->getName().toUri(::ndn::name::UriFormat::DEFAULT);
		cout << "--> PhyRxDrop at " << Simulator::Now () << ", reason: " << ss.str() << ", context: " << context << ", Interest, prefix: " << prefix;

	} else if (type == tlv::Data) {
		auto data = make_shared<Data>(netPkt);
		string prefix = data->getName().toUri(::ndn::name::UriFormat::DEFAULT);
		cout << "--> PhyRxDrop at " << Simulator::Now () << ", reason: " << ss.str() << ", context: " << context << ", Data, prefix: " << prefix;
	}
}


int
main(int argc, char* argv[])
{
	if (argc != 2) {
		cerr << "Usage: " << argv[0] << " [config-file-path]" << endl;
		return -1;
	}

	const char* configFilePath = argv[1];
	if (!RntpConfig::loadConfigFile(configFilePath)) {
		cerr << "Error to load the configuration file. Please Check." << endl;
		return -1;
	}

	//LogComponentEnable("WifiPhy", LOG_LEVEL_DEBUG);

	RntpUtils::setLogDirPath(RntpConfig::LOG_DIR.c_str());
	RntpUtils::openLogs();

	WifiPhy::enableRntpExtension = true;
	RntpStrategy::enableLog_msgs = true;

	uint32_t numNodes = RntpConfig::N_NODES;
	uint32_t gridWidthInNodes = RntpConfig::GRID_WIDTH_IN_NODES;
	uint32_t consumerNodeIdx = RntpConfig::CONSUMER_NODE_ID;
	uint32_t sensorNodeIdx = RntpConfig::PRODUCER_NODE_ID;
	bool withNoises = RntpConfig::NOISE;
	string nodeIdStrWithNoises = RntpConfig::NODE_IDS_UNDER_NOISES;

	double SIM_TIME_SECS = RntpConfig::SIM_TIME_IN_SECS;
	double EXTENSION_TIME_SECS = RntpConfig::EXTENSION_TIME_IN_SECS;
	noise_start_sec = RntpConfig::NOISE_START_SEC;
	noise_stop_sec = RntpConfig::NOISE_STOP_SEC;
	double noise_mean = RntpConfig::NOISE_MEAN;
	double noise_var = RntpConfig::NOISE_VAR;
	double gridDeltaX =  RntpConfig::GRID_DELTA_X;
	double gridDeltaY =  RntpConfig::GRID_DELTA_Y;

	srand((unsigned) std::time(NULL));
	//RngSeedManager::SetSeed((unsigned) std::time(NULL));
	RngSeedManager::SetSeed(1);
	RngSeedManager::SetRun(7);

	::nfd::fw::initializeRntpStrategyRand();

	string nameSpace = "/rntp";
	stringstream ss;
	ss << nameSpace << "/sensor" << sensorNodeIdx;
	string sensorNodeName = ss.str();

	setNodeIds_under_noise(nodeIdStrWithNoises);

	WifiHelper wifi;
	wifi.SetStandard(RntpConfig::getStandard());

	if (RntpConfig::DATA_MODE == "AarfWifiManager") {
		wifi.SetRemoteStationManager ("ns3::AarfWifiManager");
	} else {
		wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
			                          "DataMode", StringValue (RntpConfig::DATA_MODE),
			                          "ControlMode", StringValue (RntpConfig::DATA_MODE),
									  "RtsCtsThreshold", UintegerValue (0));
	}

	YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();

	if (withNoises) {
		Ptr<NormalRandomVariable> normVar = CreateObjectWithAttributes<NormalRandomVariable> (
				"Mean", DoubleValue (noise_mean),
				"Variance", DoubleValue(noise_var));
		wifiChannel.AddPropagationLoss("ns3::RandomInterferencePropagationLossModel",
				"Variable", PointerValue(normVar));
	}

	YansWifiPhyHelper wifiPhyHelper = YansWifiPhyHelper::Default();
	wifiPhyHelper.SetChannel(wifiChannel.Create());
	wifiPhyHelper.Set("TxPowerStart", DoubleValue(RntpConfig::TX_POWER_START_IN_DBM));
	wifiPhyHelper.Set("TxPowerEnd", DoubleValue(RntpConfig::TX_POWER_END_IN_DBM));
	wifiPhyHelper.Set("RxGain", DoubleValue(RntpConfig::RX_GAIN_IN_DBM));
	//wifiPhyHelper.Set("RxSensitivity", DoubleValue(-200.0));

	WifiMacHelper wifiMacHelper;
	wifiMacHelper.SetType("ns3::AdhocWifiMac");

	NodeContainer nodes;
	nodes.Create(numNodes);
	NetDeviceContainer wifiNetDevices = wifi.Install(wifiPhyHelper, wifiMacHelper, nodes);

	MobilityHelper mobility;
	mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
			"MinX", DoubleValue (0.0),
			"MinY", DoubleValue (0.0),
			"DeltaX", DoubleValue (gridDeltaX),
			"DeltaY", DoubleValue (gridDeltaY),
			"Z", DoubleValue (1.0),
			"GridWidth", UintegerValue (gridWidthInNodes),
			"LayoutType", StringValue ("RowFirst"));
	mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobility.Install(nodes);

	/** Energy Model **/
	/***************************************************************************/
	BasicEnergySourceHelper basicSourceHelper;
	basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ",
							DoubleValue (getSourceInitialEnegeryJ(RntpConfig::ENEGERY_BATTERY_VOLTAGE_IN_V,
									RntpConfig::ENEGERY_BATTERY_CAPACITY_IN_MAH)));
	basicSourceHelper.Set ("BasicEnergySupplyVoltageV", DoubleValue (RntpConfig::ENEGERY_BATTERY_VOLTAGE_IN_V));
	EnergySourceContainer sources = basicSourceHelper.Install (nodes);
	WifiRadioEnergyModelHelper radioEnergyHelper;
	DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (wifiNetDevices, sources);
	/***************************************************************************/



	NS_LOG_INFO("Installing NDN stack");
	StackHelper ndnHelper;
	ndnHelper.setPolicy("nfd::cs::lru");
	ndnHelper.setCsSize(1000);
	ndnHelper.SetDefaultRoutes(true);
	ndnHelper.Install(nodes);

	for (size_t i = 0; i < nodes.GetN(); ++i) {
		Ptr<Node> node = nodes.Get(i);
		Ptr<L3Protocol> proto = node->GetObject<L3Protocol>();
		::nfd::Forwarder* forwarder = proto->getForwarder().get();
		NodeInfo* ni = new NodeInfo;
		setNodeInfo(nameSpace, ni, i);

		NodeInfoManager::boundForwarderWithRoutes(forwarder, i, ni);
	}

	StrategyChoiceHelper::InstallAll<::nfd::fw::RntpStrategy>(nameSpace);
	Forwarder::setExtensionLltcPrefix(new string(nameSpace));
	Forwarder::setExtensionStreamingPitDurationInMilliSecs(100000);

	NS_LOG_INFO("Installing Applications");

	AppHelper consumerHelper("ns3::ndn::GenericConsumer");
	consumerHelper.SetPrefix(sensorNodeName);
	ApplicationContainer ac_consumer = consumerHelper.Install(nodes.Get(consumerNodeIdx));
	Ptr<GenericConsumer> app_consumer = ns3::DynamicCast<GenericConsumer>(ac_consumer.Get(0));
	app_consumer->setNodeID(consumerNodeIdx);
	app_consumer->setMaxWaitTime(ns3::Seconds(RntpConfig::CONSUMER_MAX_WAIT_TIME_IN_SECS));
	app_consumer->SetStartTime(Seconds(0.0));
	app_consumer->SetStopTime(Seconds(SIM_TIME_SECS));

	if (RntpConfig::CONSUMER_NEED_TO_TERMINATE_TRANSPORT) {
		app_consumer->setToTerminateTransport(ns3::Seconds(RntpConfig::CONSUMER_TERMINATE_TRANSPORT_DELAY_IN_SECS));
	}

	Ptr<Node> sensorNode = nodes.Get(sensorNodeIdx);
	AppHelper sensorAppHelper("generic::GenericSensorApp");
	ApplicationContainer sensorAC = sensorAppHelper.Install(sensorNode);
	Ptr<generic::GenericSensorApp> sensorApp = ns3::DynamicCast<generic::GenericSensorApp, Application>(sensorAC.Get(0));
	sensorApp->setSensorName(sensorNodeName);
	sensorApp->setFreq(RntpConfig::PRODUCER_FREQ);
	sensorApp->SetStartTime(Seconds(0.0));
	sensorApp->SetStopTime(Seconds(SIM_TIME_SECS));

	NodeInfoManager::nodeIDnodeInfoMap[sensorNodeIdx]->prefixes.push_back(sensorNodeName);

	//Simulator::Schedule(Seconds(1), advancePos, nodes.Get(1), 10, 1);

	Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyRxDrop", MakeCallback (&RxDropCallback));


	/***************************************************************************/
	if (RntpConfig::TRACE_BATTERY) {
		for (uint32_t i = 0; i < numNodes; ++i) {
			Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource>(sources.Get(i));
			basicSourcePtr->TraceConnectWithoutContext ("RemainingEnergy",
									MakeBoundCallback (&RemainingEnergyWrapperCallback, i));
			Ptr<DeviceEnergyModel> basicRadioModelPtr =
					basicSourcePtr->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get (0);
			basicRadioModelPtr->TraceConnectWithoutContext ("TotalEnergyConsumption",
									MakeBoundCallback (&TotalEnergyWrapperCallback, i));
		}
	}
	/***************************************************************************/

	Config::Connect ("/NodeList/*/DeviceList/*/Phy/PhyRxDrop", MakeCallback (&PhyRxDropCallback));

	Simulator::Stop(Seconds(SIM_TIME_SECS + EXTENSION_TIME_SECS));

	Simulator::Run();
	auto log = RntpUtils::getLogEnergy();
	for (uint32_t i = 0; i < numNodes; ++i) {
		double energyConsumed = deviceModels.Get(i)->GetTotalEnergyConsumption();
		double energyRemained = sources.Get(i)->GetRemainingEnergy();
		double supplyVoltage = sources.Get(i)->GetSupplyVoltage();
		*log << i << "," << Simulator::Now() << ",Final," << energyConsumed << "," << energyRemained << "," << supplyVoltage << endl;
	}

	Simulator::Destroy();

	cout << "# Capsule sent: " << sensorApp->getNCapsuleSent() << ", # Capsule recv: " << app_consumer->getNRecvCapsules() << endl;

	RntpUtils::closeLogs();

	return 0;
}
