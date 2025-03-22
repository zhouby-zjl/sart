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

#include "rntp-utils.hpp"

#include <string>

string* RntpUtils::logDirPath = NULL;

ofstream* RntpUtils::logMsgInterest = NULL;
ofstream* RntpUtils::logMsgInterestBroadcast = NULL;
ofstream* RntpUtils::logMsgCapsule = NULL;
ofstream* RntpUtils::logMsgCapAck = NULL;
ofstream* RntpUtils::logMsgCQUpdate = NULL;
ofstream* RntpUtils::logMsgEcho = NULL;
ofstream* RntpUtils::logConsumer = NULL;
ofstream* RntpUtils::logConsumerQueueSize = NULL;
ofstream* RntpUtils::logConsumerReseq = NULL;
ofstream* RntpUtils::logProducer = NULL;
ofstream* RntpUtils::logRoutes = NULL;
ofstream* RntpUtils::logCongestionControl = NULL;
ofstream* RntpUtils::logBuffer = NULL;
ofstream* RntpUtils::logEnergy = NULL;
ofstream* RntpUtils::logOthers = NULL;

void RntpUtils::setLogDirPath(const char* logDirPath) {
	RntpUtils::logDirPath = new string(logDirPath);
}

void RntpUtils::openLogs() {
	stringstream ss;
	ss.str("");
	ss << *logDirPath << "logMsgInterest";
	logMsgInterest = new ofstream;
	logMsgInterest->open(ss.str(), ios::trunc);
	ss.str("");
	ss << *logDirPath << "logMsgInterestBroadcast";
	logMsgInterestBroadcast = new ofstream;
	logMsgInterestBroadcast->open(ss.str(), ios::trunc);
	ss.str("");
	ss << *logDirPath << "logMsgCapsule";
	logMsgCapsule = new ofstream;
	logMsgCapsule->open(ss.str(), ios::trunc);
	ss.str("");
	ss << *logDirPath << "logMsgCapAck";
	logMsgCapAck = new ofstream;
	logMsgCapAck->open(ss.str(), ios::trunc);
	ss.str("");
	ss << *logDirPath << "logMsgCQUpdate";
	logMsgCQUpdate = new ofstream;
	logMsgCQUpdate->open(ss.str(), ios::trunc);
	ss.str("");
	ss << *logDirPath << "logMsgEcho";
	logMsgEcho = new ofstream;
	logMsgEcho->open(ss.str(), ios::trunc);
	ss.str("");
	ss << *logDirPath << "logConsumer";
	logConsumer = new ofstream;
	logConsumer->open(ss.str(), ios::trunc);
	ss.str("");
	ss << *logDirPath << "logConsumerQueueSize";
	logConsumerQueueSize = new ofstream;
	logConsumerQueueSize->open(ss.str(), ios::trunc);
	ss.str("");
	ss << *logDirPath << "logConsumerReseq";
	logConsumerReseq = new ofstream;
	logConsumerReseq->open(ss.str(), ios::trunc);
	ss.str("");
	ss << *logDirPath << "logProducer";
	logProducer = new ofstream;
	logProducer->open(ss.str(), ios::trunc);
	ss.str("");
	ss << *logDirPath << "logRoutes";
	logRoutes = new ofstream;
	logRoutes->open(ss.str(), ios::trunc);
	ss.str("");
	ss << *logDirPath << "logCongestionControl";
	logCongestionControl = new ofstream;
	logCongestionControl->open(ss.str(), ios::trunc);
	ss.str("");
	ss << *logDirPath << "logBuffer";
	logBuffer = new ofstream;
	logBuffer->open(ss.str(), ios::trunc);
	ss.str("");
	ss << *logDirPath << "logEnergy";
	logEnergy = new ofstream;
	logEnergy->open(ss.str(), ios::trunc);
	ss.str("");
	ss << *logDirPath << "logOthers";
	logOthers = new ofstream;
	logOthers->open(ss.str(), ios::trunc);
}

void RntpUtils::closeLogs() {
	logMsgInterestBroadcast->close();
	logMsgCapsule->close();
	logMsgCapAck->close();
	logMsgCQUpdate->close();
	logMsgEcho->close();
	logConsumer->close();
	logConsumerQueueSize->close();
	logConsumerReseq->close();
	logProducer->close();
	logRoutes->close();
	logCongestionControl->close();
	logBuffer->close();
	logEnergy->close();
	logOthers->close();
}

void RntpUtils::cleanLogDir() {
	stringstream ss;
	ss << "rm -rf " << *logDirPath << "log*";
	system(ss.str().c_str());
}

ofstream* RntpUtils::getLogMsgInterest() {
	return logMsgInterest;
}

ofstream* RntpUtils::getLogMsgInterestBroadcast() {
	return logMsgInterestBroadcast;
}

ofstream* RntpUtils::getLogMsgCapsule() {
	return logMsgCapsule;
}

ofstream* RntpUtils::getLogMsgCapAck() {
	return logMsgCapAck;
}

ofstream* RntpUtils::getLogMsgCQUpdate() {
	return logMsgCQUpdate;
}

ofstream* RntpUtils::getLogMsgEcho() {
	return logMsgEcho;
}

ofstream* RntpUtils::getLogProducer() {
	return logProducer;
}

ofstream* RntpUtils::getLogConsumer() {
	return logConsumer;
}

ofstream* RntpUtils::getLogConsumerQueueSize() {
	return logConsumerQueueSize;
}

ofstream* RntpUtils::getLogConsumerReseq() {
	return logConsumerReseq;
}

ofstream* RntpUtils::getLogRoutes() {
	return logRoutes;
}

ofstream* RntpUtils::getLogCongestionControl() {
	return logCongestionControl;
}

ofstream* RntpUtils::getLogBuffer() {
	return logBuffer;
}

ofstream* RntpUtils::getLogEnergy() {
	return logEnergy;
}

ofstream* RntpUtils::getLogOthers() {
	return logOthers;
}
