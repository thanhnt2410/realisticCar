#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/msgs/double.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace realistic_car_ign
{

class RearWheelEffortSystem : public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate
{
public:
  void Configure(
    const ignition::gazebo::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    ignition::gazebo::EntityComponentManager & ecm,
    ignition::gazebo::EventManager &) override
  {
    ignition::gazebo::Model model(entity);
    left_topic_ = sdf->Get<std::string>(
      "left_topic", "/rear_wheel_effort/left").first;
    right_topic_ = sdf->Get<std::string>(
      "right_topic", "/rear_wheel_effort/right").first;
    timeout_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(sdf->Get<double>("command_timeout", 0.25).first));
    for (const auto & name : {"rear_left_wheel_joint", "rear_right_wheel_joint"}) {
      const auto joint = model.JointByName(ecm, name);
      if (joint != ignition::gazebo::kNullEntity) {
        joints_.push_back(joint);
      }
    }
    command_.assign(joints_.size(), 0.0);
    node_.Subscribe(left_topic_, &RearWheelEffortSystem::OnLeftCommand, this);
    node_.Subscribe(right_topic_, &RearWheelEffortSystem::OnRightCommand, this);
  }

  void PreUpdate(
    const ignition::gazebo::UpdateInfo &,
    ignition::gazebo::EntityComponentManager & ecm) override
  {
    std::vector<double> effort(joints_.size(), 0.0);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (have_command_ && std::chrono::steady_clock::now() - last_command_ <= timeout_ &&
        command_.size() == joints_.size())
      {
        effort = command_;
      }
    }
    for (std::size_t index = 0; index < joints_.size(); ++index) {
      auto component = ecm.Component<ignition::gazebo::components::JointForceCmd>(
        joints_[index]);
      if (component) {
        component->Data() = {effort[index]};
      } else {
        ecm.CreateComponent(
          joints_[index],
          ignition::gazebo::components::JointForceCmd({effort[index]}));
      }
    }
  }

private:
  void OnLeftCommand(const ignition::msgs::Double & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (command_.size() == 2) {command_[0] = msg.data();}
    last_command_ = std::chrono::steady_clock::now();
    have_command_ = true;
  }

  void OnRightCommand(const ignition::msgs::Double & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (command_.size() == 2) {command_[1] = msg.data();}
    last_command_ = std::chrono::steady_clock::now();
    have_command_ = true;
  }

  ignition::transport::Node node_;
  std::string left_topic_;
  std::string right_topic_;
  std::vector<ignition::gazebo::Entity> joints_;
  std::vector<double> command_;
  std::mutex mutex_;
  std::chrono::steady_clock::time_point last_command_;
  std::chrono::steady_clock::duration timeout_{};
  bool have_command_{false};
};

}  // namespace realistic_car_ign

IGNITION_ADD_PLUGIN(
  realistic_car_ign::RearWheelEffortSystem,
  ignition::gazebo::System,
  ignition::gazebo::ISystemConfigure,
  ignition::gazebo::ISystemPreUpdate)
