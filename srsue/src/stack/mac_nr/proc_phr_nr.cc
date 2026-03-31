#include "srsue/hdr/stack/mac_nr/proc_phr_nr.h"
#include <cmath>

namespace srsue {

proc_phr_nr::proc_phr_nr(srslog::basic_logger& logger_) : logger(logger_) {}

void proc_phr_nr::init(phy_interface_mac_nr* phy_h_, srsran::ext_task_sched_handle* task_sched_)
{
  phy_h      = phy_h_;
  task_sched = task_sched_;
  initiated  = true;

  timer_periodic = task_sched->get_unique_timer();
  timer_prohibit = task_sched->get_unique_timer();

  reset();
}

void proc_phr_nr::reset()
{
  std::lock_guard<std::mutex> lock(mutex);
  timer_periodic.stop();
  timer_prohibit.stop();
  phr_cfg.reset();
  phr_is_triggered = false;
  pathloss_valid   = false;
  last_pathloss_db = 0.0f;
}

void proc_phr_nr::set_config(const srsran::phr_cfg_nr_t& cfg_)
{
  std::lock_guard<std::mutex> lock(mutex);

  phr_cfg = cfg_;

  timer_periodic.stop();
  timer_prohibit.stop();
  phr_is_triggered = false;

  if (!phr_cfg.enabled) {
    logger.info("PHR-NR: Released PHR configuration");
    pathloss_valid = false;
    return;
  }

  if (phr_cfg.multiple_phr || phr_cfg.phr_type2_other_cell ||
      phr_cfg.phr_mode_other_cg != srsran::phr_cfg_nr_t::phr_mode_other_cg_t::none) {
    logger.warning("PHR-NR: Only single-entry serving-cell Type1 PHR is implemented; extra config is stored only");
  }
  if (phr_cfg.dummy) {
    logger.warning("PHR-NR: dummy flag configured; reporting real PHY-derived SE-PHR");
  }

  if (phr_cfg.periodic_timer > 0) {
    timer_periodic.set(phr_cfg.periodic_timer, [this](uint32_t tid) { timer_expired(tid); });
    timer_periodic.run();
  }
  if (phr_cfg.prohibit_timer > 0) {
    timer_prohibit.set(phr_cfg.prohibit_timer, [this](uint32_t tid) { timer_expired(tid); });
  }

  phy_interface_mac_nr::se_phr_report_t report = {};
  if (read_se_phr_report(report) && std::isfinite(report.pathloss_db)) {
    last_pathloss_db = report.pathloss_db;
    pathloss_valid   = true;
  } else {
    pathloss_valid = false;
  }

  phr_is_triggered = true;
  logger.info("PHR-NR: Configured periodic=%d prohibit=%d tx_power_change=%d extended=%d",
              phr_cfg.periodic_timer,
              phr_cfg.prohibit_timer,
              phr_cfg.tx_pwr_factor_change,
              phr_cfg.extended);
}

bool proc_phr_nr::read_se_phr_report(phy_interface_mac_nr::se_phr_report_t& report) const
{
  return initiated && phy_h != nullptr && phy_h->get_se_phr_report(report);
}

bool proc_phr_nr::pathloss_changed()
{
  if (!phr_cfg.enabled || phr_cfg.tx_pwr_factor_change <= 0) {
    return false;
  }

  phy_interface_mac_nr::se_phr_report_t report = {};
  if (!read_se_phr_report(report) || !std::isfinite(report.pathloss_db)) {
    return false;
  }

  if (!pathloss_valid) {
    last_pathloss_db = report.pathloss_db;
    pathloss_valid   = true;
    return false;
  }

  if (std::fabs(report.pathloss_db - last_pathloss_db) > phr_cfg.tx_pwr_factor_change) {
    last_pathloss_db = report.pathloss_db;
    return true;
  }

  return false;
}

void proc_phr_nr::step()
{
  std::lock_guard<std::mutex> lock(mutex);

  if (!phr_cfg.enabled || !initiated) {
    return;
  }

  if (pathloss_changed() && (phr_cfg.prohibit_timer == 0 || timer_prohibit.is_expired())) {
    phr_is_triggered = true;
    logger.info("PHR-NR: Triggered by serving-cell pathloss change");
  }
}

bool proc_phr_nr::generate_phr_on_ul_grant(uint32_t grant_size, bool is_msg3_pending, se_phr_ce_t* phr_ce)
{
  std::lock_guard<std::mutex> lock(mutex);

  if (!phr_cfg.enabled || !phr_is_triggered || phr_ce == nullptr) {
    return false;
  }

  const uint32_t phr_ce_total_size = 3;
  if (grant_size < phr_ce_total_size || is_msg3_pending) {
    return false;
  }

  phy_interface_mac_nr::se_phr_report_t report = {};
  if (!read_se_phr_report(report) || !std::isfinite(report.phr_db) || !std::isfinite(report.p_cmax_dbm)) {
    return false;
  }

  phr_ce->phr   = encode_ph_field(report.phr_db);
  phr_ce->pcmax = encode_pcmax_field(report.p_cmax_dbm);

  if (phr_cfg.periodic_timer > 0) {
    timer_periodic.run();
  }
  if (phr_cfg.prohibit_timer > 0) {
    timer_prohibit.run();
  }

  phr_is_triggered = false;

  logger.info("PHR-NR: Generated SE-PHR ph_db=%.2f tx_power_dbm=%.2f pathloss_db=%.2f p_cmax_dbm=%.2f field=(%u,%u)",
              report.phr_db,
              report.tx_power_dbm,
              report.pathloss_db,
              report.p_cmax_dbm,
              phr_ce->phr,
              phr_ce->pcmax);
  return true;
}

void proc_phr_nr::timer_expired(uint32_t timer_id)
{
  std::lock_guard<std::mutex> lock(mutex);

  if (!phr_cfg.enabled) {
    return;
  }

  if (timer_id == timer_periodic.id()) {
    timer_periodic.run();
    phr_is_triggered = true;
    logger.debug("PHR-NR: Triggered by periodic timer");
    return;
  }

  if (timer_id == timer_prohibit.id() && pathloss_changed()) {
    phr_is_triggered = true;
    logger.debug("PHR-NR: Triggered by pathloss change after prohibit timer");
  }
}

uint8_t proc_phr_nr::encode_ph_field(float phr_db)
{
  if (phr_db <= -32.0f) {
    return 0;
  }
  if (phr_db >= 38.0f) {
    return 63;
  }
  if (phr_db < 23.5f) {
    return static_cast<uint8_t>(std::min<long>(63, std::lround(phr_db + 32.5f)));
  }
  return static_cast<uint8_t>(std::min<long>(63, std::lround(phr_db + 32.0f)));
}

uint8_t proc_phr_nr::encode_pcmax_field(float pcmax_dbm)
{
  if (pcmax_dbm <= -29.0f) {
    return 0;
  }
  if (pcmax_dbm >= 33.0f) {
    return 63;
  }
  return static_cast<uint8_t>(std::lround(pcmax_dbm + 29.5f));
}

} // namespace srsue
