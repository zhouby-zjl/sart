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

ThroughputQueue::ThroughputQueue(size_t size)
    : capacity(size), frontIndex(0), rearIndex(0), count(0) {
    buffer = new ThroughputElement[capacity];
}

ThroughputQueue::~ThroughputQueue() {
    delete[] buffer;
}

void ThroughputQueue::recordPacketArrival(ns3::Time time) {
	uint32_t secs = (uint32_t) floor(time.GetSeconds());
	if (count == 0) {
		ThroughputElement el;
		el.secs = secs;
		el.packet_count = 1;
		enqueue(el);
	} else {
		ThroughputElement* el = rear();
		if (el->secs == secs) {
			++el->packet_count;
		} else if (secs > el->secs) {
			ThroughputElement el;
			el.secs = secs;
			el.packet_count = 1;
			enqueue(el);
		} else {
			cout << "unexpected" << endl;
		}
	}
}

double ThroughputQueue::countPacketMeanArrivalRate() {
    if (isEmpty() || count == 1) {
        return -1.0;
    }

    uint32_t totalPacketCount = 0;
    size_t elementsCount = 0;

    for (int i = frontIndex; elementsCount < count - 1; i = (i + 1) % capacity) {
    	totalPacketCount += buffer[i].packet_count;
    	elementsCount++;
    }

    return (double) totalPacketCount / (double) elementsCount;
}

ns3::Time ThroughputQueue::estimateLongestPIAT(double confidentRatio, ns3::Time maxPIAT) {
	double meanRate = countPacketMeanArrivalRate();
	if (meanRate < 0) {
		return maxPIAT;
	}
	double est = - log(1 - confidentRatio) / meanRate;
	return est > maxPIAT.GetSeconds() ? maxPIAT : ns3::Seconds(est);
}

void ThroughputQueue::enqueue(ThroughputElement item) {
    if (isFull()) {
        dequeue();
    }
    buffer[rearIndex] = item;
    rearIndex = (rearIndex + 1) % capacity;
    count++;
}

ThroughputElement* ThroughputQueue::rear() {
    if (isEmpty()) {
        return NULL;
    }
    return &buffer[rearIndex == 0 ? capacity - 1: rearIndex - 1];
}

ThroughputElement ThroughputQueue::dequeue() {
    if (isEmpty()) {
        throw std::underflow_error("Queue is empty");
    }
    ThroughputElement item = buffer[frontIndex];
    frontIndex = (frontIndex + 1) % capacity;
    count--;
    return item;
}

bool ThroughputQueue::isEmpty() {
    return count == 0;
}

bool ThroughputQueue::isFull() {
    return count == capacity;
}

size_t ThroughputQueue::size() {
    return count;
}

}
}
