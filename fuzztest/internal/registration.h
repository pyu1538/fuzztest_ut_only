// Copyright 2022 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FUZZTEST_FUZZTEST_INTERNAL_REGISTRATION_H_
#define FUZZTEST_FUZZTEST_INTERNAL_REGISTRATION_H_

#include <cstdio>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/functional/any_invocable.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "./fuzztest/domain_core.h"
#include "./fuzztest/internal/domains/aggregate_of_impl.h"
#include "./fuzztest/internal/domains/domain.h"
#include "./fuzztest/internal/meta.h"
#include "./fuzztest/internal/printer.h"
#include "./fuzztest/internal/type_support.h"

namespace fuzztest {
namespace internal {

struct BasicTestInfo {
  std::string suite_name;
  std::string test_name;
  std::string file;
  int line = 0;
  bool uses_fixture = false;
};

// Use base classes to progressively add members/behavior to the registerer
// object. This way we can statically assert that certain functions are called
// in the right order.

struct NoFixture {};

// Initial base class. No custom domain, no seeds.
template <typename... Args>
struct DefaultRegistrationBase {
  static constexpr bool kHasDomain = false;
  static constexpr bool kHasSeeds = false;
  static constexpr bool kHasSeedProvider = false;
  static constexpr size_t kNumArgs = sizeof...(Args);

  static_assert((std::is_same_v<Args, std::decay_t<Args>> && ...));

  Domain<std::tuple<Args...>> GetDomains() const {
    return TupleOf(Arbitrary<Args>()...);
  }

  using SeedT = std::tuple<Args...>;
};

template <typename... Args>
DefaultRegistrationBase<std::decay_t<Args>...> DefaultRegistrationBaseImpl(
    NoFixture*, void (*)(Args...));

template <typename Fixture, typename TargetFunction>
using DefaultRegistrationBaseT = decltype(DefaultRegistrationBaseImpl(
    static_cast<Fixture*>(nullptr), static_cast<TargetFunction>(nullptr)));

// A custom domain was specified.
template <typename... Args>
struct RegistrationWithDomainsBase {
  static constexpr bool kHasDomain = true;
  static constexpr bool kHasSeeds = false;
  static constexpr bool kHasSeedProvider = false;
  static constexpr size_t kNumArgs = sizeof...(Args);

  Domain<std::tuple<Args...>> domains_;

  const auto& GetDomains() const { return domains_; }

  using SeedT = decltype(domains_.GetValue({}));
};

// Seeds were specified. It derived from the existing base to augment it.
template <typename Base>
struct RegistrationWithSeedsBase : Base {
  static constexpr bool kHasSeeds = true;

  explicit RegistrationWithSeedsBase(Base base) : Base(std::move(base)) {}

  std::vector<
      corpus_type_t<std::decay_t<decltype(std::declval<Base>().GetDomains())>>>
      seeds_;
};

template <typename Base, typename SeedProvider>
struct RegistrationWithSeedProviderBase : Base {
  static constexpr bool kHasSeedProvider = true;

  explicit RegistrationWithSeedProviderBase(Base base,
                                            SeedProvider seed_provider)
      : Base(std::move(base)), seed_provider_(std::move(seed_provider)) {}

  SeedProvider seed_provider_;
};

class PerIterationFixture;
struct RegistrationToken;

template <typename Fixture, typename TargetFunction,
          typename Base = DefaultRegistrationBaseT<Fixture, TargetFunction>,
          typename SeedProvider = void*>
class Registration : private Base {
  using SeedT = typename Base::SeedT;

 public:
  explicit Registration(BasicTestInfo info, TargetFunction target_function)
      : test_info_(std::move(info)), target_function_(target_function) {}

 private:
  std::vector<GenericDomainCorpusType> seeds() const {
    if constexpr (Base::kHasSeeds) {
      return this->seeds_;
    } else {
      return {};
    }
  }

  SeedProvider seed_provider() {
    if constexpr (Base::kHasSeedProvider) {
      return std::move(this->seed_provider_);
    } else {
      return {};
    }
  }

  template <typename, typename, typename, typename>
  friend class Registration;
  friend struct RegistrationToken;

  explicit Registration(BasicTestInfo info, TargetFunction target_function,
                        Base base)
      : Base(std::move(base)),
        test_info_(std::move(info)),
        target_function_(target_function) {}

  BasicTestInfo test_info_;
  TargetFunction target_function_;
};

}  // namespace internal

// Returns a registration for a fuzz test based on `target_function`, which
// should be a function pointer.
//
// This is an advanced API; in almost all cases you should prefer registring
// your fuzz tests with the FUZZ_TEST macro. Unlike the macro, this function
// allows customizing `suite_name`, `test_name`, `file`, and `line`. For
// example, it is suitable when you want to register fuzz tests with dynamically
// generated test names.
template <typename TargetFunction>
auto GetRegistration(std::string suite_name, std::string test_name,
                     std::string file, int line,
                     TargetFunction target_function) {
  return ::fuzztest::internal::Registration<::fuzztest::internal::NoFixture,
                                            TargetFunction>(
      ::fuzztest::internal::BasicTestInfo{std::move(suite_name),
                                          std::move(test_name), std::move(file),
                                          line, false},
      target_function);
}

}  // namespace fuzztest

#endif  // FUZZTEST_FUZZTEST_INTERNAL_REGISTRATION_H_
