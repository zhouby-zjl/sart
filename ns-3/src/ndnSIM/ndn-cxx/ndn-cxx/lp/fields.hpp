/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013-2019 Regents of the University of California.
 *
 * This file is part of ndn-cxx library (NDN C++ library with eXperimental eXtensions).
 *
 * ndn-cxx library is free software: you can redistribute it and/or modify it under the
 * terms of the GNU Lesser General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * ndn-cxx library is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
 *
 * You should have received copies of the GNU General Public License and GNU Lesser
 * General Public License along with ndn-cxx, e.g., in COPYING.md file.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * See AUTHORS.md for complete list of ndn-cxx authors and contributors.
 */

#ifndef NDN_CXX_LP_FIELDS_HPP
#define NDN_CXX_LP_FIELDS_HPP

#include "ndn-cxx/lp/field-decl.hpp"

#include "ndn-cxx/lp/cache-policy.hpp"
#include "ndn-cxx/lp/geo-tag.hpp"
#include "ndn-cxx/lp/lltc-snr-tag.hpp"
#include "ndn-cxx/lp/nack-header.hpp"
#include "ndn-cxx/lp/prefix-announcement-header.hpp"

#include <boost/mpl/set.hpp>

namespace ndn {
namespace lp {

typedef FieldDecl<field_location_tags::Header,
                  Sequence,
                  tlv::Sequence> SequenceField;
BOOST_CONCEPT_ASSERT((Field<SequenceField>));

typedef FieldDecl<field_location_tags::Header,
                  uint64_t,
                  tlv::FragIndex,
                  false,
                  NonNegativeIntegerTag,
                  NonNegativeIntegerTag> FragIndexField;
BOOST_CONCEPT_ASSERT((Field<FragIndexField>));

typedef FieldDecl<field_location_tags::Header,
                  uint64_t,
                  tlv::FragCount,
                  false,
                  NonNegativeIntegerTag,
                  NonNegativeIntegerTag> FragCountField;
BOOST_CONCEPT_ASSERT((Field<FragCountField>));

typedef FieldDecl<field_location_tags::Header,
                  std::pair<Buffer::const_iterator, Buffer::const_iterator>,
                  tlv::PitToken> PitTokenField;
BOOST_CONCEPT_ASSERT((Field<PitTokenField>));

typedef FieldDecl<field_location_tags::Header,
                  NackHeader,
                  tlv::Nack> NackField;
BOOST_CONCEPT_ASSERT((Field<NackField>));

typedef FieldDecl<field_location_tags::Header,
                  uint64_t,
                  tlv::NextHopFaceId,
                  false,
                  NonNegativeIntegerTag,
                  NonNegativeIntegerTag> NextHopFaceIdField;
BOOST_CONCEPT_ASSERT((Field<NextHopFaceIdField>));

typedef FieldDecl<field_location_tags::Header,
                  CachePolicy,
                  tlv::CachePolicy> CachePolicyField;
BOOST_CONCEPT_ASSERT((Field<CachePolicyField>));

typedef FieldDecl<field_location_tags::Header,
                  uint64_t,
                  tlv::IncomingFaceId,
                  false,
                  NonNegativeIntegerTag,
                  NonNegativeIntegerTag> IncomingFaceIdField;
BOOST_CONCEPT_ASSERT((Field<IncomingFaceIdField>));

typedef FieldDecl<field_location_tags::Header,
                  uint64_t,
                  tlv::CongestionMark,
                  false,
                  NonNegativeIntegerTag,
                  NonNegativeIntegerTag> CongestionMarkField;
BOOST_CONCEPT_ASSERT((Field<CongestionMarkField>));

typedef FieldDecl<field_location_tags::Header,
                  Sequence,
                  tlv::Ack,
                  true> AckField;
BOOST_CONCEPT_ASSERT((Field<AckField>));

typedef FieldDecl<field_location_tags::Header,
                  Sequence,
                  tlv::TxSequence> TxSequenceField;
BOOST_CONCEPT_ASSERT((Field<TxSequenceField>));

typedef FieldDecl<field_location_tags::Header,
                  EmptyValue,
                  tlv::NonDiscovery> NonDiscoveryField;
BOOST_CONCEPT_ASSERT((Field<NonDiscoveryField>));

typedef FieldDecl<field_location_tags::Header,
                  PrefixAnnouncementHeader,
                  tlv::PrefixAnnouncement> PrefixAnnouncementField;
BOOST_CONCEPT_ASSERT((Field<PrefixAnnouncementField>));

typedef FieldDecl<field_location_tags::Header,
                  uint16_t,
                  tlv::HopCountTag,
                  false,
                  NonNegativeIntegerTag,
                  NonNegativeIntegerTag> HopCountTagField;
BOOST_CONCEPT_ASSERT((Field<HopCountTagField>));

typedef FieldDecl<field_location_tags::Header,
                  GeoTag,
                  tlv::GeoTag> GeoTagField;
BOOST_CONCEPT_ASSERT((Field<GeoTagField>));

/** \brief Declare the Fragment field.
 *
 *  The fragment (i.e. payload) is the bytes between two provided iterators. During encoding,
 *  these bytes are copied from the Buffer into the LpPacket.
 */
typedef FieldDecl<field_location_tags::Fragment,
                  std::pair<Buffer::const_iterator, Buffer::const_iterator>,
                  tlv::Fragment> FragmentField;
BOOST_CONCEPT_ASSERT((Field<FragmentField>));


typedef FieldDecl<field_location_tags::Header,
        uint64_t, tlv::LltcPathId> LltcPathIdField;
BOOST_CONCEPT_ASSERT((Field<LltcPathIdField>));

typedef FieldDecl<field_location_tags::Header,
        uint64_t, tlv::LltcTransient> LltcTransientField;
BOOST_CONCEPT_ASSERT((Field<LltcTransientField>));

typedef FieldDecl<field_location_tags::Header,
        uint64_t, tlv::LltcConsumerId> LltcConsumerIdField;
BOOST_CONCEPT_ASSERT((Field<LltcConsumerIdField>));

typedef FieldDecl<field_location_tags::Header,
		LltcSnrTag, tlv::LltcSnr> LltcSnrField;
BOOST_CONCEPT_ASSERT((Field<LltcSnrField>));

//typedef FieldDecl<field_location_tags::Header,
//        uint64_t, tlv::LltcHopCount> LltcHopCountField;
//BOOST_CONCEPT_ASSERT((Field<LltcHopCountField>));
//
//typedef FieldDecl<field_location_tags::Header,
//        uint64_t, tlv::LltcRreqID> LltcRreqIDField;
//BOOST_CONCEPT_ASSERT((Field<LltcRreqIDField>));
//
//typedef FieldDecl<field_location_tags::Header,
//        uint64_t, tlv::LltcDestNodeID> LltcDestNodeIDField;
//BOOST_CONCEPT_ASSERT((Field<LltcDestNodeIDField>));
//
//typedef FieldDecl<field_location_tags::Header,
//        uint64_t, tlv::LltcDestSeqNo> LltcDestSeqNoField;
//BOOST_CONCEPT_ASSERT((Field<LltcDestSeqNoField>));
//
//typedef FieldDecl<field_location_tags::Header,
//        uint64_t, tlv::LltcOrigNodeID> LltcOrigNodeIDField;
//BOOST_CONCEPT_ASSERT((Field<LltcOrigNodeIDField>));
//
//typedef FieldDecl<field_location_tags::Header,
//        uint64_t, tlv::LltcOrigSeqNo> LltcOrigSeqNoField;
//BOOST_CONCEPT_ASSERT((Field<LltcOrigSeqNoField>));

/** \brief Set of all field declarations.
 */
typedef boost::mpl::set<
  FragmentField,
  SequenceField,
  FragIndexField,
  FragCountField,
  PitTokenField,
  NackField,
  NextHopFaceIdField,
  IncomingFaceIdField,
  CachePolicyField,
  CongestionMarkField,
  AckField,
  TxSequenceField,
  NonDiscoveryField,
  PrefixAnnouncementField,
  HopCountTagField,
  GeoTagField,
  LltcPathIdField,
  LltcTransientField,
  LltcConsumerIdField,
  LltcSnrField
//  LltcHopCountField,
//  LltcRreqIDField,
//  LltcDestNodeIDField,
//  LltcDestSeqNoField,
//  LltcOrigNodeIDField,
//  LltcOrigSeqNoField
  > FieldSet;

} // namespace lp
} // namespace ndn

#endif // NDN_CXX_LP_FIELDS_HPP
