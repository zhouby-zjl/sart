#ifndef SRC_NDNSIM_APPS_GENERIC_SENSOR_HPP_
#define SRC_NDNSIM_APPS_GENERIC_SENSOR_HPP_

#include "ns3/ndnSIM/apps/ndn-app.hpp"
#include "ns3/random-variable-stream.h"
#include <string>
#include <fstream>

namespace generic {

class GenericSensorApp : public ns3::ndn::App {
public:
	static ns3::TypeId GetTypeId();

	GenericSensorApp();
	virtual void DoInitialize();
	virtual void DoDispose();
	virtual void StartApplication();
	virtual void StopApplication();

	virtual void OnInterest(std::shared_ptr<const ns3::ndn::Interest> interest);
	virtual void OnData(std::shared_ptr<const ns3::ndn::Data> contentObject);

	void setFreq(int freq);
	void setSensorName(std::string sensorName);
	void setNodeID(uint32_t nodeID);
	int32_t getNCapsuleSent();

private:
	void sendData(uint32_t consumerNodeID);

	int dataId;
	std::ofstream outLog_pmu;
	ns3::Time piat;
	std::string sensorName;
	int32_t n_CapsuleSent;
	uint32_t nodeID;
	ns3::Ptr<ns3::UniformRandomVariable> rand;
};

}

#endif /* SRC_NDNSIM_APPS_GENERIC_SENSOR_HPP_ */
