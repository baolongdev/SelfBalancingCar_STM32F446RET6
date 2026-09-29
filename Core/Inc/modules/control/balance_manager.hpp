#ifndef BALANCE_MANAGER_HPP
#define BALANCE_MANAGER_HPP

#include <stdint.h>

enum class BalanceAlgorithm : uint8_t { Disabled = 0U, PID, PD, StateFeedback };

enum BalanceFault : uint32_t
{
  BALANCE_FAULT_NONE = 0U,
  BALANCE_FAULT_IMU_INVALID = (1UL << 0),
  BALANCE_FAULT_IMU_STALE = (1UL << 1),
  BALANCE_FAULT_TILT_LIMIT = (1UL << 2),
  BALANCE_FAULT_NOT_ENABLED = (1UL << 3)
};

struct BalanceInput
{
  uint32_t timestamp_ms;
  float dt_s;
  uint8_t imu_valid;
  float angle_deg;
  float angular_rate_dps;
  float left_speed;
  float right_speed;
  float left_position;
  float right_position;
  float battery_voltage;
};

struct BalanceOutput
{
  int16_t left_motor_percent;
  int16_t right_motor_percent;
  float angle_error_deg;
  float correction_percent;
  float integral;
  BalanceAlgorithm algorithm;
  uint32_t faults;
  uint8_t saturated;
  uint8_t active;
};

struct BalancePidConfig { float kp; float ki; float kd; float integral_limit; };
struct BalanceStateFeedbackConfig
{
  float angle_gain;
  float angular_rate_gain;
  float speed_gain;
  float position_gain;
};

struct BalanceConfig
{
  BalanceAlgorithm algorithm;
  float target_angle_deg;
  float max_output_percent;
  float boost_start_deg;
  float boost_max_output_percent;
  float tilt_cutoff_deg;
  float tilt_recover_deg;
  uint32_t stale_timeout_ms;
  uint8_t invert_output;
  BalancePidConfig pid;
  BalanceStateFeedbackConfig state_feedback;
};

class BalanceManager
{
public:
  BalanceManager();
  void init(const BalanceConfig &config);
  void reset();
  void setConfig(const BalanceConfig &config);
  const BalanceConfig &config() const;
  void setAlgorithm(BalanceAlgorithm algorithm);
  BalanceAlgorithm algorithm() const;
  void setEnabled(uint8_t enabled);
  uint8_t enabled() const;
  BalanceOutput update(const BalanceInput &input);
  const BalanceOutput &output() const;
  static BalanceConfig DefaultConfig();
  static const char *AlgorithmName(BalanceAlgorithm algorithm);

private:
  float computePid(const BalanceInput &input, float error);
  float computeStateFeedback(const BalanceInput &input);
  float outputLimitForAngle(float angle_error_deg) const;
  void forceSafeOutput(uint32_t faults);
  static float clamp(float value, float minimum, float maximum);

  BalanceConfig config_;
  BalanceOutput output_;
  float integral_;
  uint8_t enabled_;
  uint8_t tilt_latched_;
};

#endif /* BALANCE_MANAGER_HPP */
