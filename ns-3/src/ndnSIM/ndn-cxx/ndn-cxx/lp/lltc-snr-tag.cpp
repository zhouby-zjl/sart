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

#include "ndn-cxx/lp/lltc-snr-tag.hpp"
#include "ndn-cxx/lp/tlv.hpp"

namespace ndn {
namespace lp {

LltcSnrTag::LltcSnrTag(const Block& block)
{
  wireDecode(block);
}

template<encoding::Tag TAG>
size_t
LltcSnrTag::wireEncode(EncodingImpl<TAG>& encoder) const
{
  size_t length = 0;
  length += prependDoubleBlock(encoder, tlv::LltcSnr, std::get<1>(m_snr_rssi));
  length += prependDoubleBlock(encoder, tlv::LltcSnr, std::get<0>(m_snr_rssi));
  length += encoder.prependVarNumber(length);
  length += encoder.prependVarNumber(tlv::LltcSnr);
  return length;
}

template size_t
LltcSnrTag::wireEncode<encoding::EncoderTag>(EncodingImpl<encoding::EncoderTag>& encoder) const;

template size_t
LltcSnrTag::wireEncode<encoding::EstimatorTag>(EncodingImpl<encoding::EstimatorTag>& encoder) const;

const Block&
LltcSnrTag::wireEncode() const
{
  if (m_wire.hasWire()) {
    return m_wire;
  }

  EncodingEstimator estimator;
  size_t estimatedSize = wireEncode(estimator);

  EncodingBuffer buffer(estimatedSize, 0);
  wireEncode(buffer);

  m_wire = buffer.block();

  return m_wire;
}

void
LltcSnrTag::wireDecode(const Block& wire)
{
  if (wire.type() != tlv::LltcSnr) {
    NDN_THROW(ndn::tlv::Error("expecting LltcSnrTag block"));
  }

  m_wire = wire;
  m_wire.parse();

  if (m_wire.elements().size() < 2 ||
      m_wire.elements()[0].type() != tlv::LltcSnr ||
      m_wire.elements()[1].type() != tlv::LltcSnr) {
    NDN_THROW(ndn::tlv::Error("Unexpected input while decoding LltcSnrTag"));
  }
  m_snr_rssi = {encoding::readDouble(m_wire.elements()[0]),
           	   encoding::readDouble(m_wire.elements()[1])};
}

} // namespace lp
} // namespace ndn
