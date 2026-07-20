// Copyright 2026 realisticCar project
// Offline PSO tuner for FuzzyPidCore or PidCore base gains.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#ifdef TUNE_PLAIN_PID
#include "car_controller/pid_core.hpp"
#else
#include "car_controller/fuzzy_pid_core.hpp"
#endif
#include "car_controller/longitudinal_limits.hpp"
#include "car_controller/longitudinal_vehicle_model.hpp"

namespace
{

#ifndef DEFAULT_FUZZY_PID_YAML
#define DEFAULT_FUZZY_PID_YAML "src/car_controller/config/fuzzy_pid_controller.yaml"
#endif
#ifndef DEFAULT_PID_YAML
#define DEFAULT_PID_YAML "src/car_controller/config/pid_controller.yaml"
#endif
#ifndef DEFAULT_VEHICLE_MODEL_YAML
#define DEFAULT_VEHICLE_MODEL_YAML "src/car_controller/config/vehicle_simulator.yaml"
#endif

using Gains = std::array<double, 3>;

#ifdef TUNE_PLAIN_PID
using ControllerCore = car_controller::PidCore;
constexpr const char * kControllerLabel = "plain PID";
constexpr const char * kProgramName = "pso_plain_pid_tuner";
constexpr const char * kDefaultControllerYaml = DEFAULT_PID_YAML;
#else
using ControllerCore = car_controller::FuzzyPidCore;
constexpr const char * kControllerLabel = "fuzzy-PID";
constexpr const char * kProgramName = "pso_pid_tuner";
constexpr const char * kDefaultControllerYaml = DEFAULT_FUZZY_PID_YAML;
#endif

struct Options
{
  int particles{30};
  int iterations{60};
  uint64_t seed{42};
  Gains lower{0.1, 0.001, 0.0};
  Gains upper{4.0, 1.0, 2.0};
  std::string controller_yaml{kDefaultControllerYaml};
  std::string vehicle_yaml{DEFAULT_VEHICLE_MODEL_YAML};
  std::string output_yaml{kDefaultControllerYaml};
};

struct Metrics
{
  double cost{0.0};
  double iae{0.0};
  double ise{0.0};
  double control_energy{0.0};
  double jerk_energy{0.0};
  double overshoot{0.0};
};

struct TuningConfig
{
  ControllerCore::Params controller;
  car_controller::LongitudinalLimits::Params limits;
  car_controller::LongitudinalVehicleModel::Params plant;
};

struct Particle
{
  Gains position{};
  Gains velocity{};
  Gains best_position{};
  double best_cost{std::numeric_limits<double>::infinity()};
};

void referenceAt(double time, double & velocity, double & acceleration)
{
  // Same defaults as car_planning/config/trapezoid_velocity_profile.yaml.
  constexpr std::array<double, 6> points{0.0, 12.0, 7.0, 17.0, 8.0, 0.0};
  constexpr double ramp_duration = 6.0;
  constexpr double accel_ramp_duration = 1.0;
  constexpr double hold_duration = 3.0;
  constexpr double segment_duration = ramp_duration + hold_duration;
  constexpr double profile_duration = segment_duration * (points.size() - 1);

  if (time >= profile_duration) {
    velocity = points.back();
    acceleration = 0.0;
    return;
  }
  const auto index = static_cast<std::size_t>(time / segment_duration);
  const double segment_time = time - static_cast<double>(index) * segment_duration;
  const double start = points[index];
  const double delta = points[index + 1] - start;
  if (segment_time >= ramp_duration) {
    velocity = points[index + 1];
    acceleration = 0.0;
    return;
  }

  const double peak_accel = delta / (ramp_duration - accel_ramp_duration);
  const double jerk = peak_accel / accel_ramp_duration;
  const double ramp_down_start = ramp_duration - accel_ramp_duration;
  if (segment_time < accel_ramp_duration) {
    acceleration = jerk * segment_time;
    velocity = start + 0.5 * jerk * segment_time * segment_time;
  } else if (segment_time < ramp_down_start) {
    acceleration = peak_accel;
    velocity = start + 0.5 * peak_accel * accel_ramp_duration +
      peak_accel * (segment_time - accel_ramp_duration);
  } else {
    const double t = segment_time - ramp_down_start;
    acceleration = peak_accel - jerk * t;
    const double v0 = start + peak_accel * (ramp_duration - 1.5 * accel_ramp_duration);
    velocity = v0 + peak_accel * t - 0.5 * jerk * t * t;
  }
}

Metrics evaluateScenario(
  const Gains & gains, const TuningConfig & config,
  double grade, double noise_sigma, uint64_t seed)
{
  constexpr double dt = 0.02;
  constexpr double duration = 48.0;

  auto controller_params = config.controller;
  controller_params.kp = gains[0];
  controller_params.ki = gains[1];
  controller_params.kd = gains[2];
  ControllerCore controller(controller_params);

  car_controller::LongitudinalLimits limiter(config.limits);

  auto plant_params = config.plant;
  plant_params.grade_percent = grade;
  plant_params.process_noise_sigma = noise_sigma;
  plant_params.random_seed = seed;
  car_controller::LongitudinalVehicleModel plant(plant_params);

  std::mt19937_64 rng(seed + 9973U);
  std::normal_distribution<double> sensor_noise(0.0, noise_sigma);
  Metrics result;
  double previous_accel = 0.0;

  for (int step = 0; step < static_cast<int>(duration / dt); ++step) {
    const double time = step * dt;
    double ref_velocity = 0.0;
    double ref_accel = 0.0;
    referenceAt(time, ref_velocity, ref_accel);
    const double measured = plant.true_velocity() + sensor_noise(rng);
    const double error = ref_velocity - measured;
    const double correction = controller.update(error, dt);
    double correction_out, unlimited, target_accel, jerk;
    limiter.apply(
      ref_accel, correction, dt, correction_out, unlimited, target_accel, jerk);
    plant.step(target_accel, dt, grade);

    result.iae += std::abs(error) * dt;
    result.ise += error * error * dt;
    result.control_energy += correction_out * correction_out * dt;
    const double actual_jerk = (target_accel - previous_accel) / dt;
    result.jerk_energy += actual_jerk * actual_jerk * dt;
    if (ref_velocity > 0.5) {
      result.overshoot += std::max(0.0, plant.true_velocity() - ref_velocity) * dt;
    }
    previous_accel = target_accel;
  }

  result.cost = result.iae + 0.12 * result.ise + 0.02 * result.control_energy +
    0.003 * result.jerk_energy + 2.0 * result.overshoot;
  return result;
}

Metrics evaluate(const Gains & gains, const TuningConfig & config)
{
  // Robust gains: nominal, uphill and noisy operation. Each has equal weight.
  constexpr std::array<double, 3> grades{0.0, 5.0, 0.0};
  constexpr std::array<double, 3> noises{0.0, 0.0, 0.05};
  Metrics total;
  for (std::size_t i = 0; i < grades.size(); ++i) {
    const Metrics m = evaluateScenario(gains, config, grades[i], noises[i], 1001U + i);
    total.cost += m.cost / grades.size();
    total.iae += m.iae / grades.size();
    total.ise += m.ise / grades.size();
    total.control_energy += m.control_energy / grades.size();
    total.jerk_energy += m.jerk_energy / grades.size();
    total.overshoot += m.overshoot / grades.size();
  }
  return total;
}

double parseDouble(const char * value, const std::string & name)
{
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    throw std::invalid_argument("invalid value for " + name + ": " + value);
  }
}

std::unordered_map<std::string, double> readYamlNumbers(const std::string & yaml_path)
{
  std::ifstream input(yaml_path);
  if (!input) {
    throw std::runtime_error("cannot open YAML: " + yaml_path);
  }
  const std::regex number_line(
    R"(^\s*([A-Za-z_][A-Za-z0-9_]*)\s*:\s*([-+]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][-+]?[0-9]+)?)\s*(?:#.*)?$)");
  std::unordered_map<std::string, double> values;
  std::string line;
  while (std::getline(input, line)) {
    std::smatch match;
    if (std::regex_match(line, match, number_line)) {
      values[match[1].str()] = std::stod(match[2].str());
    }
  }
  return values;
}

double yamlValue(
  const std::unordered_map<std::string, double> & values,
  const std::string & key, const std::string & yaml_path)
{
  const auto value = values.find(key);
  if (value == values.end()) {
    throw std::runtime_error("missing numeric key '" + key + "' in YAML: " + yaml_path);
  }
  return value->second;
}

TuningConfig loadTuningConfig(const Options & options)
{
  const auto controller = readYamlNumbers(options.controller_yaml);
  const auto vehicle = readYamlNumbers(options.vehicle_yaml);
  const auto controller_value = [&](const std::string & key) {
      return yamlValue(controller, key, options.controller_yaml);
    };
  const auto vehicle_value = [&](const std::string & key) {
      return yamlValue(vehicle, key, options.vehicle_yaml);
    };

  TuningConfig config;
#ifndef TUNE_PLAIN_PID
  config.controller.fuzzy_error_gain = controller_value("fuzzy_error_gain");
  config.controller.fuzzy_error_derivative_gain =
    controller_value("fuzzy_error_derivative_gain");
  config.controller.fuzzy_kp_min_ratio = controller_value("fuzzy_kp_min_ratio");
  config.controller.fuzzy_kd_min_ratio = controller_value("fuzzy_kd_min_ratio");
  config.controller.gain_rate_limit = controller_value("gain_rate_limit");
#endif
  config.controller.max_integral_error = controller_value("max_integral_error");
  config.controller.max_output = controller_value("max_acceleration_correction");
  config.controller.derivative_filter_alpha = controller_value("derivative_filter_alpha");

  config.limits.min_target_acceleration = controller_value("min_target_acceleration");
  config.limits.max_target_acceleration = controller_value("max_target_acceleration");
  config.limits.min_jerk = controller_value("min_jerk");
  config.limits.max_jerk = controller_value("max_jerk");
  config.limits.max_acceleration_correction = controller_value("max_acceleration_correction");

  config.plant.actuator_delay_s = vehicle_value("actuator_delay_s");
  config.plant.actuator_lag_tau_s = vehicle_value("actuator_lag_tau_s");
  config.plant.max_accel = vehicle_value("max_accel");
  config.plant.min_accel = vehicle_value("min_accel");
  config.plant.max_jerk = vehicle_value("max_jerk");
  config.plant.min_jerk = vehicle_value("min_jerk");
  config.plant.grade_percent = vehicle_value("grade_percent");
  config.plant.rolling_resistance_coeff = vehicle_value("rolling_resistance_coeff");
  config.plant.drag_coeff = vehicle_value("drag_coeff");
  config.plant.mass_kg = vehicle_value("mass_kg");
  config.plant.process_noise_sigma = vehicle_value("process_noise_sigma");
  config.plant.random_seed = static_cast<uint64_t>(vehicle_value("random_seed"));
  config.plant.gravity = vehicle_value("gravity");
  return config;
}

Options parseOptions(int argc, char ** argv)
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--help") {
      std::cout << "Usage: " << kProgramName <<
        " [--particles N] [--iterations N] [--seed N]\n"
        "                     [--kp-min X] [--kp-max X] [--ki-min X]\n"
        "                     [--ki-max X] [--kd-min X] [--kd-max X]\n"
        "                     [--controller-yaml PATH] [--vehicle-yaml PATH]\n"
        "                     [--output-yaml PATH]\n"
                << "Default controller/output YAML: " << kDefaultControllerYaml << "\n"
                << "Default vehicle YAML: " << DEFAULT_VEHICLE_MODEL_YAML << "\n";
      std::exit(0);
    }
    if (i + 1 >= argc) {
      throw std::invalid_argument("missing value after " + arg);
    }
    if (arg == "--controller-yaml" || arg == "--vehicle-yaml" || arg == "--output-yaml") {
      const std::string path = argv[++i];
      if (arg == "--controller-yaml") {options.controller_yaml = path;}
      if (arg == "--vehicle-yaml") {options.vehicle_yaml = path;}
      if (arg == "--output-yaml") {options.output_yaml = path;}
      continue;
    }
    const double value = parseDouble(argv[++i], arg);
    if (arg == "--particles") {
      options.particles = static_cast<int>(value);
    } else if (arg == "--iterations") {
      options.iterations = static_cast<int>(value);
    } else if (arg == "--seed") {
      options.seed = static_cast<uint64_t>(value);
    } else if (arg == "--kp-min") {options.lower[0] = value;} else if (arg == "--kp-max") {
      options.upper[0] = value;
    } else if (arg == "--ki-min") {options.lower[1] = value;} else if (arg == "--ki-max") {
      options.upper[1] = value;
    } else if (arg == "--kd-min") {options.lower[2] = value;} else if (arg == "--kd-max") {
      options.upper[2] = value;
    } else {throw std::invalid_argument("unknown option: " + arg);}
  }
  if (options.particles < 3 || options.iterations < 1) {
    throw std::invalid_argument("particles must be >= 3 and iterations must be >= 1");
  }
  for (std::size_t d = 0; d < 3; ++d) {
    if (options.lower[d] < 0.0 || options.upper[d] <= options.lower[d]) {
      throw std::invalid_argument("gain bounds must satisfy 0 <= min < max");
    }
  }
  return options;
}

void writeGainsToYaml(const std::string & yaml_path, const Gains & gains)
{
  namespace fs = std::filesystem;
  const fs::path path(yaml_path);
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open output YAML: " + path.string());
  }

  const std::regex gain_line(R"(^(\s*)(kp|ki|kd)(\s*:\s*)([^#\r\n]*)(.*)$)");
  std::vector<std::string> lines;
  std::array<int, 3> replacements{0, 0, 0};
  std::string line;
  while (std::getline(input, line)) {
    std::smatch match;
    if (std::regex_match(line, match, gain_line)) {
      std::size_t index = 0;
      const std::string key = match[2].str();
      if (key == "ki") {index = 1;}
      if (key == "kd") {index = 2;}
      std::ostringstream value;
      value << std::setprecision(10) << gains[index];
      line = match[1].str() + key + match[3].str() + value.str();
      if (!match[5].str().empty()) {
        line += match[5].str();
      }
      ++replacements[index];
    }
    lines.push_back(line);
  }
  input.close();

  for (std::size_t index = 0; index < replacements.size(); ++index) {
    if (replacements[index] != 1) {
      throw std::runtime_error(
              "output YAML must contain exactly one kp, ki and kd key: " + path.string());
    }
  }

  fs::path temporary = path;
  temporary += ".pso.tmp";
  std::ofstream output(temporary, std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot create temporary YAML: " + temporary.string());
  }
  for (const auto & output_line : lines) {
    output << output_line << '\n';
  }
  output.close();
  if (!output) {
    fs::remove(temporary);
    throw std::runtime_error("failed while writing temporary YAML: " + temporary.string());
  }

  std::error_code error;
  fs::rename(temporary, path, error);
  if (error) {
    fs::remove(temporary);
    throw std::runtime_error("cannot replace output YAML: " + error.message());
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parseOptions(argc, argv);
    const TuningConfig config = loadTuningConfig(options);
    std::cout << "Loaded controller YAML: " << options.controller_yaml << "\n"
              << "Loaded vehicle YAML: " << options.vehicle_yaml << "\n"
              << "Plant mass=" << config.plant.mass_kg
              << " kg, acceleration=[" << config.plant.min_accel << ", "
              << config.plant.max_accel << "] m/s^2\n";
    std::mt19937_64 rng(options.seed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::vector<Particle> swarm(static_cast<std::size_t>(options.particles));
    Gains global_best{};
    double global_cost = std::numeric_limits<double>::infinity();

    for (auto & particle : swarm) {
      for (std::size_t d = 0; d < 3; ++d) {
        const double span = options.upper[d] - options.lower[d];
        particle.position[d] = options.lower[d] + unit(rng) * span;
        particle.velocity[d] = (2.0 * unit(rng) - 1.0) * 0.1 * span;
      }
      particle.best_position = particle.position;
    }

    for (int iteration = 0; iteration < options.iterations; ++iteration) {
      for (auto & particle : swarm) {
        const double cost = evaluate(particle.position, config).cost;
        if (cost < particle.best_cost) {
          particle.best_cost = cost;
          particle.best_position = particle.position;
        }
        if (cost < global_cost) {
          global_cost = cost;
          global_best = particle.position;
        }
      }
      std::cout << "iteration " << std::setw(3) << iteration + 1 << "/"
                << options.iterations << "  cost=" << std::fixed << std::setprecision(5)
                << global_cost << "  gains=[" << global_best[0] << ", "
                << global_best[1] << ", " << global_best[2] << "]\n";

      const double progress = static_cast<double>(iteration) / options.iterations;
      const double inertia = 0.9 - 0.5 * progress;
      for (auto & particle : swarm) {
        for (std::size_t d = 0; d < 3; ++d) {
          const double span = options.upper[d] - options.lower[d];
          particle.velocity[d] = inertia * particle.velocity[d] +
            1.7 * unit(rng) * (particle.best_position[d] - particle.position[d]) +
            1.7 * unit(rng) * (global_best[d] - particle.position[d]);
          particle.velocity[d] = std::clamp(particle.velocity[d], -0.2 * span, 0.2 * span);
          particle.position[d] = std::clamp(
            particle.position[d] + particle.velocity[d], options.lower[d], options.upper[d]);
        }
      }
    }

    const Metrics best = evaluate(global_best, config);
    std::cout << "\nBest robust " << kControllerLabel << " base gains:\n"
              << "    kp: " << std::setprecision(8) << global_best[0] << "\n"
              << "    ki: " << global_best[1] << "\n"
              << "    kd: " << global_best[2] << "\n"
              << "cost: " << best.cost << "  IAE: " << best.iae
              << "  overshoot_area: " << best.overshoot << "\n";
    writeGainsToYaml(options.output_yaml, global_best);
    std::cout << "Updated YAML: " << options.output_yaml << "\n";
  } catch (const std::exception & error) {
    std::cerr << kProgramName << ": " << error.what() << '\n';
    return 1;
  }
  return 0;
}
