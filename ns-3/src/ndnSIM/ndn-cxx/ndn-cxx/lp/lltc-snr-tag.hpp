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

#ifndef SRC_NDNSIM_NDN_CXX_NDN_CXX_LP_LLTC_SNR_TAG_HPP_
#define SRC_NDNSIM_NDN_CXX_NDN_CXX_LP_LLTC_SNR_TAG_HPP_

#include "ndn-cxx/encoding/block-helpers.hpp"
#include "ndn-cxx/encoding/encoding-buffer.hpp"
#include "ndn-cxx/tag.hpp"

namespace ndn {
namespace lp {

class LltcSnrTag : public Tag
{
public:
  static constexpr int
  getTypeId() noexcept
  {
    return 0x60000004;
  }

  LltcSnrTag() = default;

  explicit
  LltcSnrTag(std::pair<double, double> snr_rssi)
    : m_snr_rssi(snr_rssi)
  {
  }

  explicit
  LltcSnrTag(const Block& block);

  template<encoding::Tag TAG>
  size_t
  wireEncode(EncodingImpl<TAG>& encoder) const;

  const Block&
  wireEncode() const;

  void
  wireDecode(const Block& wire);

public:
  double
  getSnr() const
  {
	  return m_snr_rssi.first;
  }

  double
  getRssi() const
  {
	  return m_snr_rssi.second;
  }

  LltcSnrTag*
  setSnrRssi(double snr, double rssi)
  {
	  m_snr_rssi.first = snr;
	  m_snr_rssi.second = rssi;
    return this;
  }

private:
  std::pair<double, double> m_snr_rssi = {0.0, 0.0};
  mutable Block m_wire;
};

} // namespace lp
} // namespace ndn


#endif /* SRC_NDNSIM_NDN_CXX_NDN_CXX_LP_LLTC_SNR_TAG_HPP_ */
