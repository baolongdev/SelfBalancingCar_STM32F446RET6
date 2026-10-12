#include "modules/control/balance_manager.hpp"
#include <math.h>

namespace
{
/* Chi hoc trim khi xe gan can bang va motor chua bao hoa: luc vua dung len,
   bi day manh, bi giu tay hay tua vao vat, PWM lon khong phan anh troi do
   target lech; tich phan luc do se windup va lam xe lao di khi tha ra. */
const float kTrimLearnMaxErrorDeg = 3.0f;
const float kTrimLearnMaxMotorPercent = 90.0f;
/* Xe "em": khong bi day/lac. Sau moi lan khong em, ngung hoc mot luc de luc
   day (xe chay theo cu day) khong bi hoc nham thanh lech diem can bang. */
/* Lac can bang binh thuong gyro ~40 dps rms; cu day > 150 dps. */
const float kTrimLearnMaxGyroDps = 100.0f;
const float kTrimLearnHoldoffS = 0.5f;
}

BalanceManager::BalanceManager()
  : config_(DefaultConfig()), output_{0, 0, 0.0f, 0.0f, 0.0f, BalanceAlgorithm::Disabled,
    BALANCE_FAULT_NOT_ENABLED, 0U, 0U, 0.0f, 0.0f}, integral_(0.0f),
    drift_estimate_percent_(0.0f), trim_deg_(0.0f), trim_holdoff_s_(0.0f), enabled_(0U),
    tilt_latched_(0U) {}

BalanceConfig BalanceManager::DefaultConfig()
{
  BalanceConfig c{};
  c.algorithm = BalanceAlgorithm::PID;
  /* Diem can bang do tren xe that (log [T], 2026-10-12). */
  /* -1.3 deg trong he goc cu co cong +1 deg offset cung -> -2.3 deg khi
     offset IMU = 0. Sau "BAL IMU ZERO", target duoc dich tuong ung. */
  c.target_angle_deg = -2.3f;
  c.max_output_percent = 100.0f;
  c.boost_start_deg = 1.0f;
  c.boost_max_output_percent = 100.0f;
  c.tilt_cutoff_deg = 70.0f;
  c.tilt_recover_deg = 10.0f;
  c.stale_timeout_ms = 20U;
  c.invert_output = 0U;
  /* PID theo goc + gyro, tune tren xe that bang log [T]: kp >= ~27 hoac
     kd >= ~0.45 gay rung bang-bang 8-16 Hz; kp <= 15 lac cham, gan nga. */
  c.pid.kp = 22.0f;
  c.pid.ki = 0.00f;
  c.pid.kd = 0.20f;
  c.state_feedback = {5.0f, 0.12f, 0.0f, 0.0f};
  /* Khau P theo PWM (drift_gain) da tat: khi xe dang nghieng va chay de do,
     no lam giam luc do -> xe nghieng mai roi nga (do tren xe that). Chi giu
     trim tich phan cham, gioi han +/-1 deg: lon hon thi target lech du de
     motor gan bao hoa, het luc do khi bi day. */
  c.drift_gain_deg_per_percent = 0.0f;
  c.drift_tau_s = 0.5f;
  c.drift_max_deg = 2.0f;
  /* Chi hoc khi xe em (xem kTrimLearn*): 40% PWM troi -> 0.8 deg/s, bu lech
     diem can bang ~1.5 deg trong ~2 s sau khi dung xe. */
  c.drift_trim_rate = 0.02f;
  return c;
}

void BalanceManager::init(const BalanceConfig &config) { config_ = config; reset(); }

void BalanceManager::reset()
{
  integral_ = 0.0f;
  drift_estimate_percent_ = 0.0f;
  trim_deg_ = 0.0f;
  trim_holdoff_s_ = 0.0f;
  enabled_ = 0U;
  tilt_latched_ = 0U;
  output_ = {0, 0, 0.0f, 0.0f, 0.0f, BalanceAlgorithm::Disabled,
             BALANCE_FAULT_NOT_ENABLED, 0U, 0U, 0.0f, 0.0f};
}

/* Cho phep doi config khi dang balance (tune live). Giu nguyen tilt latch:
   neu xe dang nga thi van phai dung len trong tilt_recover_deg moi chay lai. */
void BalanceManager::setConfig(const BalanceConfig &config)
{
  config_ = config;
  integral_ = 0.0f;
}
const BalanceConfig &BalanceManager::config() const { return config_; }
void BalanceManager::setAlgorithm(BalanceAlgorithm algorithm)
{
  config_.algorithm = algorithm;
  integral_ = 0.0f;
}
BalanceAlgorithm BalanceManager::algorithm() const { return config_.algorithm; }

void BalanceManager::setEnabled(uint8_t enabled)
{
  enabled_ = (enabled != 0U) ? 1U : 0U;
  if (enabled_ == 0U)
  {
    integral_ = 0.0f;
    tilt_latched_ = 0U;
    forceSafeOutput(BALANCE_FAULT_NOT_ENABLED);
  }
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

  /* Từ lúc vượt boost_start, cho phép toàn bộ output để thắng quán tính. */
  return config_.boost_max_output_percent;
}

float BalanceManager::computePid(const BalanceInput &input, float error)
{
  const float dt = clamp(input.dt_s, 0.0005f, 0.02f);
  const float output_limit = outputLimitForAngle(error);

  const float previous_integral = integral_;
  integral_ += error * dt;

  /* D lay truc tiep tu gyro (dao ham do luong) de tranh derivative kick
     khi doi target. */
  const float correction = (config_.pid.kp * error) +
                           (config_.pid.ki * integral_) -
                           (config_.pid.kd * input.angular_rate_dps);
  const float limited = clamp(correction, -output_limit, output_limit);

  /* Anti-windup: khi output bao hoa cung chieu error thi khong tich phan them,
     nen integral_ khong can gioi han rieng. */
  if ((limited != correction) && ((correction * error) > 0.0f)) integral_ = previous_integral;
  return limited;
}

float BalanceManager::computeStateFeedback(const BalanceInput &input)
{
  const float speed = 0.5f * (input.left_speed + input.right_speed);
  const float position = 0.5f * (input.left_position + input.right_position);
  const float angle_error = config_.target_angle_deg - input.angle_deg;
  /* Cung quy uoc dau voi PID: +gain * (target - angle) va -gain * gyro, de
     thanh phan goc va toc do goc cung la phan hoi am. */
  return (config_.state_feedback.angle_gain * angle_error) -
         (config_.state_feedback.angular_rate_gain * input.angular_rate_dps) -
         (config_.state_feedback.speed_gain * speed) -
         (config_.state_feedback.position_gain * position);
}

void BalanceManager::forceSafeOutput(uint32_t faults)
{
  if ((faults & (BALANCE_FAULT_IMU_INVALID | BALANCE_FAULT_IMU_STALE | BALANCE_FAULT_TILT_LIMIT)) != 0U)
  {
    integral_ = 0.0f;
  }
  /* Motor dung -> xe khong con chay theo lenh cu. Giu trim (diem can bang da
     hoc) qua cac lan nga. */
  drift_estimate_percent_ = 0.0f;
  trim_holdoff_s_ = kTrimLearnHoldoffS;
  output_.target_shift_deg = 0.0f;
  output_.trim_deg = trim_deg_;
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

  const float tilt_error = fabsf(output_.angle_error_deg);
  if (tilt_error >= config_.tilt_cutoff_deg) tilt_latched_ = 1U;
  else if ((tilt_latched_ != 0U) && (tilt_error <= config_.tilt_recover_deg)) tilt_latched_ = 0U;
  if (tilt_latched_ != 0U)
  { forceSafeOutput(BALANCE_FAULT_TILT_LIMIT); return output_; }

  /* Chong troi: PWM on dinh ~ van toc ~ kp * (target - diem can bang that).
     Dich target NGUOC dau PWM de keo van toc ve 0. Dau nay da kiem chung tren
     xe that (log [T]): dich cung dau PWM lam troi tang dan roi chay mat. PWM
     thuc te duoc doi ve chieu cua bo dieu khien neu invert. Tilt van tinh
     theo target goc. */
  if ((config_.drift_gain_deg_per_percent > 0.0f) || (config_.drift_trim_rate > 0.0f))
  {
    const float dt = clamp(input.dt_s, 0.0005f, 0.02f);
    const float alpha = dt / (config_.drift_tau_s + dt);
    const float motor = (config_.invert_output != 0U) ? -input.motor_command_percent
                                                      : input.motor_command_percent;
    drift_estimate_percent_ += alpha * (motor - drift_estimate_percent_);
    /* output_.angle_error_deg luc nay con la target - angle (chua dich). */
    const uint8_t calm = (uint8_t)((fabsf(output_.angle_error_deg + trim_deg_) < kTrimLearnMaxErrorDeg) &&
                                   (fabsf(motor) < kTrimLearnMaxMotorPercent) &&
                                   (fabsf(input.angular_rate_dps) < kTrimLearnMaxGyroDps));
    if (calm == 0U)
    {
      trim_holdoff_s_ = kTrimLearnHoldoffS;
    }
    else if (trim_holdoff_s_ > 0.0f)
    {
      trim_holdoff_s_ -= dt;
    }
    else
    {
      trim_deg_ -= config_.drift_trim_rate * motor * dt;
    }
    trim_deg_ = clamp(trim_deg_, -config_.drift_max_deg, config_.drift_max_deg);
    output_.target_shift_deg = clamp(trim_deg_ - (config_.drift_gain_deg_per_percent * drift_estimate_percent_),
                                     -config_.drift_max_deg, config_.drift_max_deg);
  }
  else
  {
    drift_estimate_percent_ = 0.0f;
    output_.target_shift_deg = 0.0f;
  }
  output_.trim_deg = trim_deg_;
  output_.angle_error_deg += output_.target_shift_deg;

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
float BalanceManager::trimDeg() const { return trim_deg_; }
void BalanceManager::setTrimDeg(float trim_deg)
{
  trim_deg_ = clamp(trim_deg, -config_.drift_max_deg, config_.drift_max_deg);
}

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
