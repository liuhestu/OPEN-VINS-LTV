#pragma once
#include "LtvLandmarkManager.h"
#include <cmath>
#include <iomanip>
#include <ostream>
#include <stdexcept>

namespace ltv {
namespace bounded_diagnostics {
inline void string(std::ostream &out, const std::string &value) {
  static const char digits[] = "0123456789abcdef";
  out << '"';
  for (unsigned char c : value) {
    if (c == '"' || c == '\\')
      out << '\\' << static_cast<char>(c);
    else if (c < 32)
      out << "\\u00" << digits[c >> 4] << digits[c & 15];
    else
      out << static_cast<char>(c);
  }
  out << '"';
}
inline void validate(const ManagedLandmark &record) {
  if (!std::isfinite(record.first_seen) || !std::isfinite(record.entered) || !std::isfinite(record.seeded) ||
      !std::isfinite(record.last_seen))
    throw std::runtime_error("Nonfinite bounded landmark diagnostic time");
}
inline const char *kind(LandmarkRetirementKind value) {
  switch (value) {
  case LandmarkRetirementKind::ActiveRetired:
    return "ACTIVE_RETIRED";
  case LandmarkRetirementKind::NeverAdmittedCandidateTtl:
    return "CANDIDATE_TTL";
  }
  throw std::runtime_error("Unknown bounded retirement event kind");
}
inline void ids(std::ostream &out, const std::vector<size_t> &values) {
  out << '[';
  for (size_t j = 0; j < values.size(); ++j) {
    if (j)
      out << ',';
    out << values[j];
  }
  out << ']';
}
} // namespace bounded_diagnostics

// Complete legacy-compatible track record, reusable for one-time retirements.
inline void writeBoundedLandmarkRecord(std::ostream &out, size_t id, const ManagedLandmark &record) {
  bounded_diagnostics::validate(record);
  const auto precision = out.precision();
  out << std::setprecision(17) << "{\"id\":" << id << ",\"phase\":";
  bounded_diagnostics::string(out, toString(record.phase));
  out << ",\"reason\":";
  bounded_diagnostics::string(out, record.reason);
  out << ",\"first_seen\":" << record.first_seen << ",\"entered\":" << record.entered << ",\"seeded\":" << record.seeded
      << ",\"last_seen\":" << record.last_seen << ",\"missed_frames\":" << record.missed_frames
      << ",\"ever_opportunity\":" << record.ever_opportunity << ",\"seed_written\":" << record.seed_written << '}';
  out.precision(precision);
  if (!out)
    throw std::runtime_error("Bounded landmark diagnostics stream failed");
}

// Added only when enabled: OFF preserves the previous serialized interface.
inline void writeActiveConsistency(std::ostream &out, const LandmarkManagerFrame &frame) {
  if (!frame.active_consistency_enabled)
    return;
  auto number = [&](double x) {
    if (std::isfinite(x))
      out << x;
    else
      out << "null";
  };
  const auto precision = out.precision();
  out << std::setprecision(17)
      << ",\"active_consistency_schema\":\"ACTIVE_CONSISTENCY_V1\",\"consistency_active_count\":" << frame.consistency_active_count
      << ",\"consistency_evaluable_count\":" << frame.consistency_evaluable_count
      << ",\"consistency_pass_count\":" << frame.consistency_pass_count << ",\"consistency_skip_count\":" << frame.consistency_skip_count
      << ",\"consistency_retire_count\":" << frame.consistency_retire_count << ",\"observations_sent_to_ltv\":" << frame.observations.size()
      << ",\"active_consistency\":[";
  for (size_t i = 0; i < frame.active_consistency.size(); ++i) {
    if (i)
      out << ',';
    const auto &d = frame.active_consistency[i];
    const auto &r = d.result;
    out << "{\"timestamp\":" << frame.time << ",\"feature_id\":" << r.feature_id << ",\"evaluable\":" << r.evaluable
        << ",\"history_count\":" << r.history_count << ",\"missing_pose_count\":" << r.missing_pose_count
        << ",\"history_span_s\":" << r.history_span_s << ",\"history_max_residual_rad\":";
    number(r.history_max_residual_rad);
    out << ",\"holdout_residual_rad\":";
    number(r.holdout_residual_rad);
    out << ",\"consistency_pass\":" << r.pass << ",\"consistency_fail_count\":" << d.fail_count
        << ",\"consistency_reject_count\":" << d.reject_count << ",\"action\":";
    bounded_diagnostics::string(out, d.action);
    out << ",\"reason\":";
    bounded_diagnostics::string(out, toString(r.reason));
    out << ",\"fit_reason\":";
    bounded_diagnostics::string(out, r.fit_reason);
    out << '}';
  }
  out << ']';
  out.precision(precision);
  if (!out)
    throw std::runtime_error("Active consistency diagnostics stream failed");
}

// JSON object suffix including its leading comma, but not the enclosing '}'.
// Pure serialization: never mutates the manager or substitutes zeros for NaNs.
inline void writeBoundedManagement(std::ostream &out, const LandmarkManagerFrame &frame) {
  if (!std::isfinite(frame.time))
    throw std::runtime_error("Nonfinite bounded management diagnostic time");
  for (const auto &event : frame.retirement_events) {
    if (!std::isfinite(event.time))
      throw std::runtime_error("Nonfinite bounded retirement event time");
    bounded_diagnostics::validate(event.record);
    bounded_diagnostics::kind(event.kind);
  }
  const auto precision = out.precision();
  out << std::setprecision(17) << ",\"bounded_management_schema\":\"BOUNDED_V1\",\"retirement_events\":[";
  for (size_t j = 0; j < frame.retirement_events.size(); ++j) {
    if (j)
      out << ',';
    const auto &event = frame.retirement_events[j];
    out << "{\"id\":" << event.feature_id << ",\"epoch\":" << frame.epoch << ",\"kind\":\"" << bounded_diagnostics::kind(event.kind)
        << "\",\"time\":" << event.time << ",\"record\":";
    writeBoundedLandmarkRecord(out, event.feature_id, event.record);
    out << '}';
  }
  out << "],\"active_retired_total\":" << frame.active_retired_total << ",\"candidate_ttl_total\":" << frame.candidate_ttl_total
      << ",\"identity_guard_rejections_total\":" << frame.identity_guard_rejections_total
      << ",\"identity_guard_insertions\":" << frame.identity_guard_insertions
      << ",\"identity_guard_set_bits\":" << frame.identity_guard_set_bits << ",\"exact_records\":" << frame.exact_records
      << ",\"identity_guard_rejected_ids\":";
  bounded_diagnostics::ids(out, frame.identity_guard_rejected_ids);
  out << ",\"capacity_rejected_ids\":";
  bounded_diagnostics::ids(out, frame.capacity_rejected_ids);
  writeActiveConsistency(out, frame);
  out.precision(precision);
  if (!out)
    throw std::runtime_error("Bounded management diagnostics stream failed");
}
} // namespace ltv
