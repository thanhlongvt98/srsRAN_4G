/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2017 Software Radio Systems Limited
 *
 * \section LICENSE
 *
 * This file is part of srsLTE.
 *
 * srsUE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsUE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#ifndef INTERFACE_COMMON_H
#define INTERFACE_COMMON_H

#include "srslte/common/timers.h"
#include "srslte/common/security.h"
#include "srslte/asn1/liblte_rrc.h"


namespace srslte {

class srslte_nas_config_t
{
public:
  srslte_nas_config_t(uint32_t lcid_ = 0)
    :lcid(lcid_)
    {}

  uint32_t lcid;
};


class srslte_pdcp_config_t
{
public:
  srslte_pdcp_config_t(bool is_control_ = false, bool is_data_ = false, uint8_t direction_ = SECURITY_DIRECTION_UPLINK)
    :direction(direction_)
    ,is_control(is_control_)
    ,is_data(is_data_)
    ,do_security(false)
    ,sn_len(12) {}

  uint8_t             direction;
  bool                is_control;
  bool                is_data;
  bool                do_security;
  uint8_t             sn_len;

  // TODO: Support the following configurations
  // bool do_rohc;
};

class mac_interface_timers
{
public: 
  /* Timer services with ms resolution. 
   * timer_id must be lower than MAC_NOF_UPPER_TIMERS
   */
  virtual timers::timer* get(uint32_t timer_id) = 0;
  virtual uint32_t               get_unique_id() = 0;
};

class read_pdu_interface
{
public:
  virtual int read_pdu(uint32_t lcid, uint8_t *payload, uint32_t requested_bytes) = 0; 
};

}

#endif
