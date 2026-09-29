#include "modules/control/balance_manager.hpp"
#include <math.h>

BalanceManager::BalanceManager()
  : config_(DefaultConfig()), output_{0, 0, 0.0f, 0.0f, 0.0f, BalanceAlgorithm::Disabled,
    BALANCE_FAULT_NOT_ENABLED, 0U, 0U}, integral_(0.0f), enabled_(0U), tilt_latched_(0U) {}

BalanceConfig BalanceManager::DefaultConfig()
{
  BalanceConfig c{};
  c.algorithm = BalanceAlgorithm::PID;
  c.target_angle_deg = 0.0f;
  c.max_output_percent = 55.0f;
  c.boost_start_deg = 8.0f;
  c.boost_max_output_percent = 70.0f;
  c.tilt_cutoff_deg = 35.0f;
  c.tilt_recover_deg = 10.0f;
  c.stale_timeout_ms = 20U;
  c.invert_output = 0U;
  c.pid = {4.0f, 0.15f, 0.08f, 20.0f};
  c.state_feedback = {5.0f, 0.12f, 0.0f, 0.0f};
  return c;
}

void BalanceManager::init(const BalanceConfig &config) { config_ = config; reset(); }

void BalanceManager::reset()
{
  integral_ = 0.0f;
  enabled_ = 0U;
  tilt_latched_ = 0U;
  output_ = {0, 0, 0.0f, 0.0f, 0.0f, BalanceAlgorithm::Disabled,
             BALANCE_FAULT_NOT_ENABLED, 0U, 0U};
}

void BalanceManager::setConfig(const BalanceConfig &config)
{
  config_ = config;
  integral_ = 0.0f;
  tilt_latched_ = 0U;
}
const BalanceConfig &BalanceManager::config() const { return config_; }
void BalanceManager::setAlgorithm(BalanceAlgorithm algorithm)
{
  config_.algorithm = algorithm;
  integral_ = 0.0f;
  tilt_latched_ = 0U;
}
BalanceAlgorithm BalanceManager::algorithm() const { return config_.algorithm; }

void BalanceManager::setEnabled(uint8_t enabled)
{
  enabled_ = (enabled != 0U) ? 1U : 0U;
  if (enabled_ == 0U) { integral_ = 0.0f; tilt_latched_ = 0U; forceSafeOutput(BALANCE_FAULT_NOT_ENABLED); }
}

uint8_t BalanceManager::enabled() const { return enabled_; }

float BalanceManager::clamp(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

float BalanceManager::outputLimitForAngle(float angle_error_deg) const
{
  const float magnitude = fabsf(angle_error_deg);
  if (magnitude <= config_.boost_start_deg ||
      config_.boost_max_output_percent <= config_.max_output_percent)
  {
    return config_.max_output_percent;
  }

  const float span = config_.tilt_cutoff_deg - config_.boost_start_deg;
  if (span <= 0.0f)
  {
    return config_.max_output_percent;
  }

  const float ratio = clamp((magnitude - config_.boost_start_deg) / span, 0.0f, 1.0f);
  return config_.max_output_percent +
         ratio * (config_.boost_max_output_percent - config_.max_output_percent);
}

float BalanceManager::computePid(const BalanceInput &input, float error)
{
  const float dt = clamp(input.dt_s, 0.0005f, 0.02f);
  const float output_limit = outputLimitForAngle(error);
  const float previous_integral = integral_;
  integral_ = clamp(integral_ + error * dt,
                    -config_.pid.integral_limit, config_.pid.integral_limit);
  float correction = (config_.pid.kp * error) +
                     (config_.pid.ki * integral_) -
                     (config_.pid.kd * input.angular_rate_dps);
  correction = clamp(correction, -output_limit, output_limit);
  if ((correction >= output_limit && error > 0.0f) ||
      (correction <= -output_limit && error < 0.0f)) integral_ = previous_integral;
  return correction;
}

float BalanceManager::computeStateFeedback(const BalanceInput &input)
{
  const float speed = 0.5f * (input.left_speed + input.right_speed);
  const float position = 0.5f * (input.left_position + input.right_position);
  const float angle_error = config_.target_angle_deg - input.angle_deg;
  return -((config_.state_feedback.angle_gain * angle_error) +
           (config_.state_feedback.angular_rate_gain * input.angular_rate_dps) +
           (config_.state_feedback.speed_gain * speed) +
           (config_.state_feedback.position_gain * position));
}

void BalanceManager::forceSafeOutput(uint32_t faults)
{
  if ((faults & (BALANCE_FAULT_IMU_INVALID | BALANCE_FAULT_IMU_STALE | BALANCE_FAULT_TILT_LIMIT)) != 0U)
  {
    integral_ = 0.0f;
  }
  output_.left_motor_percent = 0;
  output_.right_motor_percent = 0;
  output_.correction_percent = 0.0f;
  output_.integral = integral_;
  output_.algorithm = config_.algorithm;
  output_.faults = faults;
  output_.saturated = 0U;
  output_.active = 0U;
}

BalanceOutput BalanceManager::update(const BalanceInput &input)
{
  output_.algorithm = config_.algorithm;
  output_.angle_error_deg = config_.target_angle_deg - input.angle_deg;
  output_.faults = BALANCE_FAULT_NONE;
  output_.saturated = 0U;
  output_.active = 0U;

  if ((enabled_ == 0U) || (config_.algorithm == BalanceAlgorithm::Disabled))
  { forceSafeOutput(BALANCE_FAULT_NOT_ENABLED); return output_; }
  if (input.imu_valid == 0U)
  { forceSafeOutput(BALANCE_FAULT_IMU_INVALID); return output_; }
  if ((input.timestamp_ms == 0U) || (input.dt_s <= 0.0f) ||
      (input.dt_s > ((float)config_.stale_timeout_ms / 1000.0f)))
  { forceSafeOutput(BALANCE_FAULT_IMU_STALE); return output_; }

  if (fabsf(input.angle_deg) >= config_.tilt_cutoff_deg) tilt_latched_ = 1U;
  else if ((tilt_latched_ != 0U) && (fabsf(input.angle_deg) <= config_.tilt_recover_deg)) tilt_latched_ = 0U;
  if (tilt_latched_ != 0U)
  { forceSafeOutput(BALANCE_FAULT_TILT_LIMIT); return output_; }

  float correction = 0.0f;
  switch (config_.algorithm)
  {
    case BalanceAlgorithm::PID: correction = computePid(input, output_.angle_error_deg); break;
    case BalanceAlgorithm::PD:
      correction = (config_.pid.kp * output_.angle_error_deg) -
                   (config_.pid.kd * input.angular_rate_dps);
      correction = clamp(correction, -outputLimitForAngle(output_.angle_error_deg),
                         outputLimitForAngle(output_.angle_error_deg));
      break;
    case BalanceAlgorithm::StateFeedback:
      correction = clamp(computeStateFeedback(input), -outputLimitForAngle(output_.angle_error_deg),
                         outputLimitForAngle(output_.angle_error_deg));
      break;
    case BalanceAlgorithm::Disabled:
    default: forceSafeOutput(BALANCE_FAULT_NOT_ENABLED); return output_;
  }
  if (config_.invert_output != 0U) correction = -correction;
  output_.correction_percent = correction;
  output_.left_motor_percent = (int16_t)clamp(correction, -100.0f, 100.0f);
  output_.right_motor_percent = (int16_t)clamp(correction, -100.0f, 100.0f);
  output_.saturated = (uint8_t)(fabsf(correction) >= outputLimitForAngle(output_.angle_error_deg));
  output_.active = 1U;
  output_.integral = integral_;
  return output_;
}

const BalanceOutput &BalanceManager::output() const { return output_; }

const char *BalanceManager::AlgorithmName(BalanceAlgorithm algorithm)
{
  switch (algorithm)
  {
    case BalanceAlgorithm::PID: return "PID";
    case BalanceAlgorithm::PD: return "PD";
    case BalanceAlgorithm::StateFeedback: return "STATE_FB";
    case BalanceAlgorithm::Disabled:
    default: return "DISABLED";
  }
}
