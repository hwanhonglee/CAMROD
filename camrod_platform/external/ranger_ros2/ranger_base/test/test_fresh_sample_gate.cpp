// Copyright 2026 hwanhonglee
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the copyright holder nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// HH_260819 - Prove cached SDK BMS state cannot be counted repeatedly by the
// platform charging debounce.
#include <chrono>

#include "gtest/gtest.h"
#include "ranger_base/fresh_sample_gate.hpp"

namespace westonrobot
{

TEST(FreshSampleGate, RejectsUninitializedAndCachedTimestamps)
{
  using Clock = std::chrono::steady_clock;
  using Stamp = Clock::time_point;
  FreshSampleGate<Stamp> gate;

  const Stamp empty;
  const Stamp first(std::chrono::nanoseconds(100));
  const Stamp second(std::chrono::nanoseconds(200));

  EXPECT_FALSE(gate.accept(empty));
  EXPECT_TRUE(gate.accept(first));
  EXPECT_FALSE(gate.accept(first));
  EXPECT_FALSE(gate.accept(first));
  EXPECT_TRUE(gate.accept(second));
  EXPECT_FALSE(gate.accept(second));
}

TEST(FreshSampleGate, ResetRequiresANewNonzeroSample)
{
  using Stamp = std::chrono::steady_clock::time_point;
  FreshSampleGate<Stamp> gate;
  const Stamp sample(std::chrono::nanoseconds(100));

  ASSERT_TRUE(gate.accept(sample));
  gate.reset();
  EXPECT_FALSE(gate.accept(Stamp{}));
  EXPECT_TRUE(gate.accept(sample));
}

}  // namespace westonrobot
