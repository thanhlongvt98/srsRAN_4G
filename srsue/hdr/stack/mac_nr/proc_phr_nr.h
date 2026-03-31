#ifndef SRSUE_PROC_PHR_NR_H
#define SRSUE_PROC_PHR_NR_H

#include "srsran/common/task_scheduler.h"
#include "srsran/interfaces/mac_interface_types.h"
#include "srsran/interfaces/ue_nr_interfaces.h"
#include "srsran/srslog/srslog.h"
#include <mutex>

namespace srsue {

class proc_phr_nr : public srsran::timer_callback
{
public:
  struct se_phr_ce_t {
    uint8_t phr    = 0;
    uint8_t pcmax  = 0;
  };

  explicit proc_phr_nr(srslog::basic_logger& logger_);

  void init(phy_interface_mac_nr* phy_h_, srsran::ext_task_sched_handle* task_sched_);
  void reset();
  void step();
  void set_config(const srsran::phr_cfg_nr_t& cfg_);
  bool generate_phr_on_ul_grant(uint32_t grant_size, bool is_msg3_pending, se_phr_ce_t* phr_ce);
  void timer_expired(uint32_t timer_id) override;

private:
  bool pathloss_changed();
  bool read_se_phr_report(phy_interface_mac_nr::se_phr_report_t& report) const;

  static uint8_t encode_ph_field(float phr_db);
  static uint8_t encode_pcmax_field(float pcmax_dbm);

  srslog::basic_logger&          logger;
  phy_interface_mac_nr*          phy_h      = nullptr;
  srsran::ext_task_sched_handle* task_sched = nullptr;
  srsran::phr_cfg_nr_t           phr_cfg    = {};
  bool                           initiated  = false;
  float                          last_pathloss_db = 0.0f;
  bool                           pathloss_valid   = false;
  bool                           phr_is_triggered = false;

  srsran::timer_handler::unique_timer timer_periodic;
  srsran::timer_handler::unique_timer timer_prohibit;

  mutable std::mutex mutex;
};

} // namespace srsue

#endif // SRSUE_PROC_PHR_NR_H
