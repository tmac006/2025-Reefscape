#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"
#include "ctre/phoenix6/SignalLogger.hpp"
#include "ctre/phoenix6/controls/Follower.hpp"
#include "ctre/phoenix6/controls/MotionMagicExpoTorqueCurrentFOC.hpp"
#include "ctre/phoenix6/controls/MotionMagicTorqueCurrentFOC.hpp"
#include "ctre/phoenix6/controls/PositionTorqueCurrentFOC.hpp"
#include "frc/Alert.h"
#include "frc/simulation/ElevatorSim.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/sysid/SysIdRoutine.h"
#include "networktables/BooleanTopic.h"
#include "networktables/DoubleTopic.h"
#include "networktables/NetworkTableInstance.h"
#include "str/GainTypes.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/length.h"
#include "units/velocity.h"
#include "units/voltage.h"


class Elevator : public frc2::SubsystemBase {
 public:
  Elevator();

  void Periodic() override;
  void SimulationPeriodic() override;
  units::meter_t GetHeight();
  void GoToHeight(units::meter_t newHeight);
  void SetVoltage(units::volt_t voltage);
  bool IsAtGoalHeight();
  void SetTorqueCurrent(units::ampere_t amps);
  frc2::CommandPtr GoToHeightCmd(std::function<units::meter_t()> newHeight);
  frc2::CommandPtr SysIdElevatorQuasistaticVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdElevatorDynamicVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdElevatorQuasistaticTorqueCurrent(
      frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdElevatorDynamicTorqueCurrent(
      frc2::sysid::Direction dir);
  frc2::CommandPtr TuneElevatorPID(std::function<bool()> isDone);

 private:
  void ConfigureMotors();
   void UpdateNTEntries();
  void SetElevatorGains(str::gains::radial::RadialGainsHolder newGains,
                        units::ampere_t kg);
  units::meter_t ConvertRadiansToHeight(units::radian_t rots);
  units::radian_t ConvertHeightToRadians(units::meter_t height);
  units::meters_per_second_t ConvertRadianVelToHeightVel(
      units::radians_per_second_t radialVel);
  units::radians_per_second_t ConvertHeightVelToRadianVel(
      units::meters_per_second_t vel);
  void LogElevatorVolts(frc::sysid::SysIdRoutineLog* log);
  void LogElevatorTorqueCurrent(frc::sysid::SysIdRoutineLog* log);
  ctre::phoenix6::hardware::TalonFX leftMotor{
      consts::elevator::can_ids::LEFT_MOTOR};
  ctre::phoenix6::hardware::TalonFX rightMotor{
      consts::elevator::can_ids::RIGHT_MOTOR};

  units::meter_t goalHeight = 0_m;
  units::meter_t currentHeight = 0_m;
  bool isAtGoalHeight = false;
  

  ctre::phoenix6::hardware::TalonFX m_leftMotor{0}; //Call Constants here
  ctre::phoenix6::hardware::TalonFX m_rightMotor{1}; //Call Constants here

  ctre::phoenix6::sim::TalonFXSimState& m_leftMotorSim = m_leftMotor.GetSimState();
  ctre::phoenix6::sim::TalonFXSimState& m_rightMotorSim = m_rightMotor.GetSimState();
  
  ctre::phoenix6::StatusSignal<units::turn_t> leftPositionSignal = m_leftMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> leftVelocitySignal = m_leftMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::ampere_t> leftTorqueCurrentSignal = m_leftMotor.GetTorqueCurrent();
  ctre::phoenix6::StatusSignal<units::volt_t> leftVoltageSignal = m_leftMotor.GetMotorVoltage();

  ctre::phoenix6::StatusSignal<units::turn_t> rightPositionSignal = m_rightMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> rightVelocitySignal = m_rightMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::ampere_t> rightTorqueCurrentSignal = m_rightMotor.GetTorqueCurrent();
  ctre::phoenix6::StatusSignal<units::volt_t> rightVoltageSignal = m_rightMotor.GetMotorVoltage();

 ctre::phoenix6::controls::MotionMagicExpoTorqueCurrentFOC
      elevatorHeightSetter{0_rad};
  ctre::phoenix6::controls::VoltageOut elevatorVoltageSetter{0_V};
  ctre::phoenix6::controls::TorqueCurrentFOC elevatorTorqueCurrentSetter{0_A};
  ctre::phoenix6::controls::Follower followerSetter{
      consts::elevator::can_ids::LEFT_MOTOR, true};

  str::gains::radial::RadialGainsHolder currentGains{
      consts::elevator::gains::ELEVATOR_GAINS};
  units::ampere_t currentKg{consts::elevator::gains::kG};

  //SCREW YOU WPI LIBRARY
      frc::sim::ElevatorSim m_elevatorSim{consts::elevator::physical::MOTOR,
                                    consts::elevator::physical::GEARING,
                                    consts::elevator::physical::CARRIAGE_MASS,
                                    consts::elevator::physical::PULLEY_DIAM / 2,
                                    0_m,
                                    consts::elevator::physical::EXTENDED_HEIGHT,
                                    true,
                                    0_m,
                                    {0.00}};

    frc2::sysid::SysIdRoutine elevatorSysIdVoltage{
      frc2::sysid::Config{
          std::nullopt, 10_V, std::nullopt,
          [](frc::sysid::State state) {
            ctre::phoenix6::SignalLogger().WriteString(
                "SysIdElevator_State",
                frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{
          [this](units::volt_t voltsToSend) { SetVoltage(voltsToSend); },
          [this](frc::sysid::SysIdRoutineLog* log) { LogElevatorVolts(log); },
          this, "elevator"}};

  frc2::sysid::SysIdRoutine elevatorSysIdTorqueCurrent{
      frc2::sysid::Config{
          (10_V / 1_s), 500_V, std::nullopt,
          [](frc::sysid::State state) {
            ctre::phoenix6::SignalLogger().WriteString(
                "SysIdElevator_State",
                frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{[this](units::volt_t ampsToSend) {
                               SetTorqueCurrent(
                                   units::ampere_t{ampsToSend.value()});
                             },
                             [this](frc::sysid::SysIdRoutineLog* log) {
                               LogElevatorTorqueCurrent(log);
                             },
                             this, "elevator"}};

 std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Elevator")};
  nt::DoublePublisher currentHeightPub{
      nt->GetDoubleTopic("CurrentHeight").Publish()};
  nt::DoublePublisher heightSetpointPub{
      nt->GetDoubleTopic("HeightSetpoint").Publish()};
  nt::BooleanPublisher isAtSetpointPub{
      nt->GetBooleanTopic("IsAtRequestedHeight").Publish()};
};
