#include <cassert>
#include <string_view>

#include "camrod_system/graph_readiness.hpp"

namespace
{

using camrod_system::graph_readiness::Severity;

void expect(
  const camrod_system::graph_readiness::Decision & decision,
  Severity severity,
  std::string_view operating_state)
{
  assert(decision.severity == severity);
  assert(decision.operating_state == operating_state);
}

}  // namespace

int main()
{
  using camrod_system::graph_readiness::alternativeGroup;
  using camrod_system::graph_readiness::requiredGraph;

  expect(requiredGraph(true, true), Severity::kOk, "READY");
  expect(requiredGraph(true, false), Severity::kOk, "READY");
  expect(requiredGraph(false, true), Severity::kWarn, "STARTING");
  expect(requiredGraph(false, false), Severity::kError, "FAULT");

  expect(alternativeGroup(1U, true), Severity::kOk, "READY");
  expect(alternativeGroup(1U, false), Severity::kOk, "READY");
  expect(alternativeGroup(0U, true), Severity::kWarn, "STARTING");
  expect(alternativeGroup(0U, false), Severity::kError, "FAULT");
  expect(alternativeGroup(2U, true), Severity::kError, "FAULT");
  expect(alternativeGroup(2U, false), Severity::kError, "FAULT");

  return 0;
}
