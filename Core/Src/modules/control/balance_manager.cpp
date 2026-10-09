#include "modules/control/balance_manager.hpp"
#include <math.h>

BalanceManager::BalanceManager()
  : config_(DefaultConfig()), output_{0, 0, 0.0f, 0.0f, 0.0f, BalanceAlgorithm::Disabled,
    BALANCE_FAULT_NOT_ENABLED, 0U, 0U}, integral_(0.0f), enabled_(0U),
    tilt_latched_(0U), rescue_latched_(0U) {}

BalanceConfig BalanceManager::DefaultConfig()
{
  BalanceConfig c{};
  c.algorithm = BalanceAlgorithm::PID;
  c.target_angle_deg = 0.0f;
  c.max_output_percent = 100.0f;
  c.boost_start_deg = 1.0f;
  c.boost_max_output_percent = 100.0f;
  c.tilt_cutoff_deg = 70.0f;
  c.tilt_recover_deg = 10.0f;
  c.stale_timeout_ms = 20U;
  c.invert_output = 0U;
  /* Giảm độ gắt của phản hồi góc và tăng damping để hạn chế dao động
     +/-10..15 deg quan sát được trong log imu_balance_log (11). */
  /* Phản ứng nhanh khi góc đổi dấu qua 0 deg; Kd tăng để hãm quán tính. */
  /* Cấu hình test lực mạnh: ưu tiên bắt lại thăng bằng trước khi tối ưu
     hiệu suất động cơ và giảm tiêu thụ. */
  /* Log (12) cho thấy Kd cao và min_run lớn làm correction bão hòa, gây
     nhảy +/-45% quanh target. Hạ damping/gain để điều chỉnh mịn quanh 0 deg. */
  /* PID thuần góc + gyro; không dùng encoder/speed feedback. */
  /* Giảm rung quanh target 0 deg; vùng góc lớn vẫn được khuếch đại
     phi tuyến ở phía dưới để không làm mất lực cứu xe. */
  /* Log COM22: gyro spike tạo output lớn dù angle_error gần 0 deg.
     Giảm Kd để tránh đảo motor do nhiễu tốc độ góc. */
  /* Log COM22: xe dao động +/-4..6 deg rồi mất thăng bằng. Giảm Kp để
     tránh vượt qua target, tăng nhẹ Kd để hãm tốc độ góc. */
  /* Cấu hình test bắt xe lại: ưu tiên lực phản ứng, chấp nhận giật hơn. */
  c.pid = {8.5f, 0.001f, 0.08f, 3.0f};
  c.state_feedback = {5.0f, 0.12f, 0.0f, 0.0f};
  return c;
}

void BalanceManager::init(const BalanceConfig &config) { config_ = config; reset(); }

void BalanceManager::reset()
{
  integral_ = 0.0f;
  enabled_ = 0U;
  tilt_latched_ = 0U;
  rescue_latched_ = 0U;
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
  if (enabled_ == 0U)
  {
    integral_ = 0.0f;
    tilt_latched_ = 0U;
    rescue_latched_ = 0U;
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
  const float magnitude = fabsf(error);

  const float previous_integral = integral_;
  integral_ = clamp(integral_ + error * dt,
                    -config_.pid.integral_limit, config_.pid.integral_limit);
  /* Thu nhỏ phản ứng quanh điểm 0 deg, nhưng khuếch đại phi tuyến khi
     góc lệch lớn để motor lấy lực sớm thay vì chờ PID tuyến tính bão hòa. */
  float angle_scale = 0.90f;
  if (magnitude > 1.0f)
  {
    angle_scale = 0.75f + 0.55f * clamp((magnitude - 1.0f) / 7.0f, 0.0f, 1.0f);
  }
  float correction = (config_.pid.kp * error * angle_scale) +
                     (config_.pid.ki * integral_) -
                     (config_.pid.kd * input.angular_rate_dps);
  correction = clamp(correction, -output_limit, output_limit);

  /* Test lực mạnh: dồn PWM sớm để bánh kéo trọng tâm quay lại. */
  if (magnitude > 1.0f)
  {
    const float x = clamp((magnitude - 1.0f) / 7.0f, 0.0f, 1.0f);
    const float smoothstep = x * x * (3.0f - 2.0f * x);
    const float smooth = smoothstep;
    const float rescue_output = (error >= 0.0f) ? output_limit : -output_limit;
    correction += (rescue_output - correction) * smooth;
  }

  /* Attack khi xe đang đổ xa target: error và angular rate trái dấu.
     Tăng lực sớm để bánh kịp đảo chiều, nhưng không kích hoạt khi chỉ rung
     nhỏ quanh 0 deg. */
  if ((magnitude > 1.0f) && ((error * input.angular_rate_dps) < 0.0f))
  {
    const float angle_attack = clamp((magnitude - 1.0f) / 4.0f, 0.0f, 1.0f);
    const float rate_attack = clamp((fabsf(input.angular_rate_dps) - 5.0f) / 35.0f,
                                    0.0f, 1.0f);
    const float attack = angle_attack * rate_attack;
    const float rescue_output = (error >= 0.0f) ? output_limit : -output_limit;
    correction += (rescue_output - correction) * attack;
  }

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

  const float tilt_error = fabsf(output_.angle_error_deg);
  /* Khi xe đã đổ rõ, giữ lực cứu tối đa cho tới khi quay lại gần target.
     Nếu thả lực sớm theo PID, xe có thể tiếp tục trôi cùng một phía. */
  if ((rescue_latched_ == 0U) && (tilt_error >= 6.0f))
  {
    rescue_latched_ = 1U;
  }
  else if ((rescue_latched_ != 0U) && (tilt_error <= 1.5f))
  {
    rescue_latched_ = 0U;
  }
  if (tilt_error >= config_.tilt_cutoff_deg) tilt_latched_ = 1U;
  else if ((tilt_latched_ != 0U) && (tilt_error <= config_.tilt_recover_deg)) tilt_latched_ = 0U;
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
  if (rescue_latched_ != 0U)
  {
    correction = (output_.angle_error_deg >= 0.0f) ?
                   outputLimitForAngle(output_.angle_error_deg) :
                   -outputLimitForAngle(output_.angle_error_deg);
    integral_ = 0.0f;
  }
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
