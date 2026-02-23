#pragma once

#include <lanelet2_core/primitives/RegulatoryElement.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>

namespace camping_cart::map
{

// HH_251217 Minimal stub so that Lanelet parser accepts speed_bump regulatory elements.
class SpeedBumpRegulatoryElement : public lanelet::RegulatoryElement
{
public:
  static constexpr char RuleName[] = "speed_bump";

  explicit SpeedBumpRegulatoryElement(const lanelet::RegulatoryElementDataPtr & data)
  : lanelet::RegulatoryElement(data)
  {
  }

  using Ptr = std::shared_ptr<SpeedBumpRegulatoryElement>;
  using ConstPtr = std::shared_ptr<const SpeedBumpRegulatoryElement>;

  static Ptr make(const lanelet::RegulatoryElementDataPtr & data)
  {
    return Ptr(new SpeedBumpRegulatoryElement(data));
  }
};

}  // namespace camping_cart::map

// HH_251231: Register globally so Lanelet2 loader can construct "speed_bump".
using SpeedBumpElement = camping_cart::map::SpeedBumpRegulatoryElement;
inline const lanelet::RegisterRegulatoryElement<SpeedBumpElement> reg_speed_bump{};
