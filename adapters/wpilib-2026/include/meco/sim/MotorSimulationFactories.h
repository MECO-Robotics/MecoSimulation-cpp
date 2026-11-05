#pragma once

#include "meco/sim/LoadedMotorIO.h"
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>
#include <set>
#include <string>
#include <tuple>

namespace meco::sim {
// These callbacks bind configuration without constructing a simulator. Supply
// them to the Base mode-selection factory, which calls them only in simulation
// after replay selection. No simulation objects or hardware are created on the
// roboRIO. Callback captures own their values; a load callback must keep its
// captured state alive for as long as the resulting MotorIO uses it.
using MotorSimulationFactory = std::function<std::unique_ptr<base::MotorIO>()>;

inline MotorSimulationFactory LoadedMotorFactory(base::MotorConfig motor,
                                                 LoadedMotorModel model,
                                                 std::function<double()> loadTorqueNm) {
#ifndef __FRC_ROBORIO__
  return [motor = std::move(motor), model, load = std::move(loadTorqueNm)] {
    return base::MakeMotorIO(motor, model, load);
  };
#else
  (void)motor; (void)model; (void)loadTorqueNm;
  return {};
#endif
}

inline MotorSimulationFactory InertialMotorFactory(base::MotorConfig motor,
                                                   base::Vendor vendor,
                                                   double outputInertia) {
#ifndef __FRC_ROBORIO__
  return [motor = std::move(motor), vendor, outputInertia] {
    return base::MakeMotorIO(base::Mode::Sim, vendor, motor, outputInertia);
  };
#else
  (void)motor; (void)vendor; (void)outputInertia;
  return {};
#endif
}
// One robot-owned entry per simulated motor identity. Secondary follower IDs
// remain in MotorConfig and are passed through to the selected plant.
struct MotorSimulationEntry {
  base::Vendor vendor;
  std::string canBus;
  int primaryId;
  double outputInertia{0.025};
  std::optional<LoadedMotorModel> loadedModel;
  std::function<double()> loadTorqueNm;
};
using MotorSimulationProvider = std::function<std::unique_ptr<base::MotorIO>(base::Vendor, const base::MotorConfig&)>;

// Defer configuration and model construction until the first simulated motor
// request. Copies share the same initialization; exceptions are propagated and
// a later request retries. The builder and provider retain their captures.
inline MotorSimulationProvider MakeLazyMotorSimulationProvider(
    std::function<MotorSimulationProvider()> build) {
#ifndef __FRC_ROBORIO__
  if (!build) throw std::invalid_argument("Missing motor simulation provider builder");
  struct State {
    explicit State(std::function<MotorSimulationProvider()> factory) : build(std::move(factory)) {}
    std::function<MotorSimulationProvider()> build;
    std::once_flag initialized;
    MotorSimulationProvider provider;
  };
  auto state = std::make_shared<State>(std::move(build));
  return [state](base::Vendor vendor, const base::MotorConfig& config) {
    std::call_once(state->initialized, [&] {
      auto provider = state->build();
      if (!provider) throw std::runtime_error("Motor simulation provider builder returned null");
      state->provider = std::move(provider);
    });
    return state->provider(vendor, config);
  };
#else
  (void)build;
  return {};
#endif
}

// Build one provider for the robot. Missing or ambiguous identities are errors,
// so a motor cannot silently acquire another mechanism's physics. Entries and
// load callbacks are retained by value; callbacks must own captured state.
inline MotorSimulationProvider MakeMotorSimulationProvider(std::vector<MotorSimulationEntry> entries) {
#ifndef __FRC_ROBORIO__
  std::set<std::tuple<base::Vendor, std::string, int>> keys;
  for (const auto& entry : entries) {
    if (entry.primaryId < 0 || entry.primaryId > 62 ||
        !keys.emplace(entry.vendor, entry.canBus, entry.primaryId).second ||
        !std::isfinite(entry.outputInertia) || entry.outputInertia <= 0 ||
        (entry.loadedModel && !entry.loadTorqueNm))
      throw std::invalid_argument("Invalid or duplicate motor simulation entry");
  }
  return [entries = std::move(entries)](base::Vendor vendor, const base::MotorConfig& config) {
    if (config.ids.empty()) throw std::invalid_argument("Missing motor simulation identity");
    for (const auto& entry : entries) {
      if (entry.vendor != vendor || entry.canBus != config.canBus || entry.primaryId != config.ids.front()) continue;
      if (entry.loadedModel) {
        auto model = *entry.loadedModel;
        model.outputInertiaKgM2 = entry.outputInertia;
        return base::MakeMotorIO(config, model, entry.loadTorqueNm);
      }
      return base::MakeMotorIO(base::Mode::Sim, vendor, config, entry.outputInertia);
    }
    throw std::invalid_argument("No motor simulation entry for configured motor");
  };
#else
  (void)entries;
  return {};
#endif
}

} // namespace meco::sim
