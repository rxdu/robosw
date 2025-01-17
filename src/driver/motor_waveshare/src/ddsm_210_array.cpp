/*
 * @file ddsm_210_array.cpp
 * @date 10/19/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "motor_waveshare/ddsm_210_array.hpp"

#include "async_port/async_serial.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
Ddsm210Array::Ddsm210Array(const std::string &dev_name) {
  serial_ = std::make_shared<AsyncSerial>(dev_name, 115200);
}

Ddsm210Array::~Ddsm210Array() {
  for (auto motor : motors_) {
    motor.second->SetSpeed(0);
  }
}

void Ddsm210Array::RegisterMotor(uint8_t id) {
  ids_.push_back(id);
  motors_[id] = std::make_shared<Ddsm210>(id, serial_);
}

void Ddsm210Array::UnregisterMotor(uint8_t id) {
  if (motors_.find(id) != motors_.end()) motors_.erase(id);
  ids_.erase(std::find(ids_.begin(), ids_.end(), id));
}

bool Ddsm210Array::Connect() {
  if (serial_->Open()) {
    serial_->SetReceiveCallback(
        std::bind(&Ddsm210Array::ProcessFeedback, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3));
    return true;
  }
  return serial_->IsOpened();
}

void Ddsm210Array::Disconnect() { serial_->Close(); }

Ddsm210Array::Mode Ddsm210Array::GetMode(uint8_t id) const {
  if (motors_.find(id) != motors_.end()) return motors_.at(id)->GetMode();
  return Ddsm210Array::Mode::kUnknown;
}

int32_t Ddsm210Array::GetEncoderCount(uint8_t id) const {
  if (motors_.find(id) != motors_.end())
    return motors_.at(id)->GetEncoderCount();
  return 0;
}

void Ddsm210Array::RequestOdometryFeedback(uint8_t id) {
  if (motors_.find(id) != motors_.end()) motors_[id]->RequestOdometryFeedback();
}

void Ddsm210Array::RequestModeFeedback(uint8_t id) {
  if (motors_.find(id) != motors_.end()) motors_[id]->RequestModeFeedback();
}

void Ddsm210Array::SetSpeed(uint8_t id, float rpm) {
  if (motors_.find(id) != motors_.end()) motors_[id]->SetSpeed(rpm);
}

void Ddsm210Array::SetSpeeds(const std::vector<float> &speeds) {
  assert(speeds.size() == motors_.size());
  for (int i = 0; i < ids_.size(); i++) {
    if (motors_.find(ids_[i]) != motors_.end())
      motors_[ids_[i]]->SetSpeed(speeds[i]);
  }
}

float Ddsm210Array::GetSpeed(uint8_t id) {
  if (motors_.find(id) != motors_.end()) return motors_[id]->GetSpeed();
  return 0;
}

void Ddsm210Array::GetSpeeds(std::vector<float> &speeds) {
  speeds.clear();
  for (auto id : ids_) {
    if (motors_.find(id) != motors_.end())
      speeds.push_back(motors_[id]->GetSpeed());
  }
}

void Ddsm210Array::SetPosition(uint8_t id, float position) {
  if (motors_.find(id) != motors_.end()) motors_[id]->SetPosition(position);
}

float Ddsm210Array::GetPosition(uint8_t id) {
  if (motors_.find(id) != motors_.end()) return motors_[id]->GetPosition();
  return 0;
}

void Ddsm210Array::ApplyBrake(uint8_t id, float brake) {
  if (motors_.find(id) != motors_.end()) motors_[id]->ApplyBrake(brake);
}

void Ddsm210Array::ReleaseBrake(uint8_t id) {
  if (motors_.find(id) != motors_.end()) motors_[id]->ReleaseBrake();
}

bool Ddsm210Array::IsNormal(uint8_t id) {
  if (motors_.find(id) != motors_.end()) return motors_[id]->IsNormal();
  return false;
}

void Ddsm210Array::ProcessFeedback(uint8_t *data, const size_t bufsize,
                                   size_t len) {
  for (auto motor : motors_) {
    motor.second->ProcessFeedback(data, bufsize, len);
  }
}
}  // namespace xmotion