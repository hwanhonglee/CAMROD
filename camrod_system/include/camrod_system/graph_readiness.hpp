#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace camrod_system
{
namespace graph_readiness
{

// HH_260730 - Keep graph availability separate from runtime health.  A graph
// can be present (READY) while one of its live diagnostics is degraded/WARN.
enum class Severity : std::uint8_t
{
  kOk = 0,
  kWarn = 1,
  kError = 2,
};

struct Decision
{
  Severity severity;
  std::string_view operating_state;
};

// A missing required graph is expected only during the bounded launch grace
// period.  Once that grace expires it is an autonomy-blocking fault.
constexpr Decision requiredGraph(bool complete, bool in_startup_grace)
{
  if (complete) {
    return {Severity::kOk, "READY"};
  }
  if (in_startup_grace) {
    return {Severity::kWarn, "STARTING"};
  }
  return {Severity::kError, "FAULT"};
}

// Exactly one alternative implementation must own a mutually-exclusive
// capability.  Zero implementations may still be starting during launch;
// multiple implementations are an immediate authority conflict.
constexpr Decision alternativeGroup(
  std::size_t healthy_alternatives,
  bool in_startup_grace)
{
  if (healthy_alternatives == 1U) {
    return {Severity::kOk, "READY"};
  }
  if (healthy_alternatives == 0U && in_startup_grace) {
    return {Severity::kWarn, "STARTING"};
  }
  return {Severity::kError, "FAULT"};
}

}  // namespace graph_readiness
}  // namespace camrod_system
