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

using namespace std;
using namespace ns3;

namespace nfd {
namespace fw {

CapsuleQueue::CapsuleQueue() {
	nHiddenElements = 0;
}

void CapsuleQueue::setNodeID(uint32_t nodeID) {
	this->nodeID = nodeID;
}

void CapsuleQueue::removeElement(int dataID) {
    auto it = buffer.begin();
    while (it != buffer.end()) {
        if (it->capInfo.dataID == dataID) {
        	if (it->hidden) {
        		--nHiddenElements;
        	}
            it = buffer.erase(it);
            dataIDsInBuffer.erase(dataID);
        } else {
            ++it;
        }
    }
    logBuffer();
}

bool CapsuleQueue::isDataIDInBuffer(uint32_t dataID) {
	return dataIDsInBuffer.find(dataID) != dataIDsInBuffer.end();
}

void CapsuleQueue::pushElement(CapsuleToSend& element) {
	element.hidden = false;
	buffer.push_back(element);
	dataIDsInBuffer.insert(element.capInfo.dataID);
	logBuffer();
}

void CapsuleQueue::transientlyPopElement() {
	for (auto iter = buffer.begin(); iter != buffer.end(); ++iter) {
		if (!iter->hidden) {
			iter->hidden = true;
			++nHiddenElements;
			logBuffer();
			return;
		}
	}
}

CapsuleToSend* CapsuleQueue::getFront() {
	if (!buffer.empty()) {
		for (auto iter = buffer.begin(); iter != buffer.end(); ++iter) {
			if (!iter->hidden) {
				return &(*iter);
			}
		}
	}
	return NULL;
}

CapsuleToSend* CapsuleQueue::restoreElement(int id) {
	for (auto iter = buffer.begin(); iter != buffer.end(); ++iter) {
		if (iter->capInfo.dataID == id) {
			if (iter->hidden) {
				iter->hidden = false;
				--nHiddenElements;
				logBuffer();
			}
			return &(*iter);
		}
	}
	return NULL;
}

size_t CapsuleQueue::countElements() {
    return buffer.size() - nHiddenElements;
}

void CapsuleQueue::logBuffer() {
	auto log = RntpUtils::getLogBuffer();
	*log << this->nodeID << "," << Simulator::Now() << "," << buffer.size() << "," << nHiddenElements << endl;
}

}
}
