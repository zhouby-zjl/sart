#ifndef SRC_NDNSIM_NFD_DAEMON_FW_GENERIC_ROUTES_MANAGER_HPP_
#define SRC_NDNSIM_NFD_DAEMON_FW_GENERIC_ROUTES_MANAGER_HPP_

#include "NFD/daemon/face/face-common.hpp"
#include "ns3/ndnSIM/model/ndn-common.hpp"
#include "ns3/ptr.h"
#include "NFD/daemon/fw/forwarder.hpp"
#include "NFD/daemon/face/face-common.hpp"
//#include "model/lltc/lltc-resilient-routes-subgraph.hpp"

#include <list>
#include <unordered_map>

using namespace ns3::ndn;
using namespace std;
using namespace ::nfd::face;


namespace nfd {
namespace fw {

struct GenericFIBEntry {
	Face* face_nextHop;
	int direction;  // 0 stands for the downstream, and 1 for the upstream
};

struct NodeInfo {
	uint32_t 		nodeID;
	vector<string> 	prefixes;
	vector<string>  consumerReqPrefixes;
	string 			ndnNameSpace;
	ns3::Time		capsulePerHopTimeout;
	double			interestContentionTimeInSecs;
	uint32_t		interestSendTimes;
	uint32_t		capsuleRetryingMaxTimes;
	uint32_t		congestionControlSlowStartThres;
	uint32_t		congestionControlInitWin;
	uint32_t		testTimesToIdentifyLinkAwaken;
	ns3::Time		periodToIdentifyLinkAwaken;
	ns3::Time 		echoPeriod;
	ns3::Time		channelQualityUpdatePeriod;
	ns3::Time		CQUpdateDelayTime;
	ns3::Time		msgTimeout;
	double			quality_alpha;
	size_t			cache_maxSize;
	uint32_t		maxTimesForRevokingToSendCapsule;
	uint32_t		thQueueSize;
	double			longestPIATEstCondifentRatio;
	ns3::Time		ETO;   // for RT-CaCC
	uint32_t		frpSrcNodeID;  // for FRP
	uint32_t		frpDstNodeID;  // for FRP
	double			frpRssiMin;		// for FRP
	double			frpPREQContentionTimeInSecs; // for FRP
	double			frpBREQContentionTimeInSecs; // for FRP
	double			frpBREPContentionTimeInSecs; // for FRP
	double			frpPREPDelayTimeInSecs; // for FRP
	ns3::Time		frpPREPSendingDelay;   // for FRP
	double			frpBREQSendingDelayInSecs;   // for FRP
	double			frpDataSendingDelayInSecs;   // for FRP
	ns3::Time		frpDataSendingStopTime; // for FRP
	ns3::Time		frpDataSendingPIAT; // for FRP
	ns3::Time		frpPIATMaxTime; // for FRP
};

class NodeInfoManager {
public:
	NodeInfoManager();
	static void boundForwarderWithRoutes(Forwarder* forwarder, uint32_t nodeID, NodeInfo* routes);
	static NodeInfo* getRoutesByForwarder(Forwarder* forwarder);

	static unordered_map<Forwarder*, NodeInfo*> nodeInfoMap;
	static unordered_map<uint32_t, NodeInfo*> nodeIDnodeInfoMap;
};

}
}



#endif /* SRC_NDNSIM_NFD_DAEMON_FW_GENERIC_ROUTES_MANAGER_HPP_ */
