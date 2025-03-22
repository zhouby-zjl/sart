#include "generic-routes-manager.hpp"
#include "ns3/ptr.h"

using namespace ns3::ndn;
using namespace std;


namespace nfd {
namespace fw {

unordered_map<Forwarder*, NodeInfo*> NodeInfoManager::nodeInfoMap;
unordered_map<uint32_t, NodeInfo*> NodeInfoManager::nodeIDnodeInfoMap;

NodeInfoManager::NodeInfoManager() {

}

void NodeInfoManager::boundForwarderWithRoutes(Forwarder* forwarder,  uint32_t nodeID, NodeInfo* routes) {
	nodeInfoMap[forwarder] = routes;
	nodeIDnodeInfoMap[nodeID] = routes;
}

NodeInfo* NodeInfoManager::getRoutesByForwarder(Forwarder* forwarder) {
	if (nodeInfoMap.find(forwarder) == nodeInfoMap.end()) return nullptr;
	NodeInfo* routes = nodeInfoMap[forwarder];
	return routes;
}

}
}
