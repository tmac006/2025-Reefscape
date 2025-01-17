#include "subsystems/Elevator.h"

#include "Constants.h"
#include "ctre/phoenix/StatusCodes.h"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "frc/DataLogManager.h"
#include "frc/RobotBase.h"
#include "frc/RobotController.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "str/Units.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/length.h"
#include "units/voltage.h"
#include "frc/smartdashboard/SmartDashboard.h"

Elevator::Elevator() 
{
  ConfigureMotors();

}

void Elevator::Periodic()
{
    ctre::phoenix::StatusCode status =
        ctre::phoenix6::BaseStatusSignal::WaitForAll(
          2.0 / consts::elevator::BUS_UPDATE_FREQ, leftPositionSignal,
          leftVelocitySignal, leftTorqueCurrentSignal, leftVoltageSignal,
          rightPositionSignal, rightVelocitySignal, rightTorqueCurrentSignal,
          rightVoltageSignal);
    
    if (!status.IsOK())
    {
        frc::DataLogManager::Log(fmt::format("Error Updating Position Error Was: {}" , status.GetName())); // Fix later
    }

    isAtGoalHeight = units::math::abs(goalHeight - currentHeight) <
                     consts::elevator::gains::HEIGHT_TOLERANCE;

  UpdateNTEntries();
}

void Elevator::UpdateNTEntries() {
  currentHeightPub.Set(currentHeight.convert<units::inches>().value());
  heightSetpointPub.Set(goalHeight.convert<units::inches>().value());
  isAtSetpointPub.Set(isAtGoalHeight);
}


void Elevator::SimulationPeriodic() 
{
    m_leftMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
    m_rightMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    m_elevatorSim.SetInputVoltage((m_leftMotorSim.GetMotorVoltage() + m_rightMotorSim.GetMotorVoltage()) / 2);

    m_elevatorSim.Update(20_ms);

  units::turn_t pulleyTurns = ConvertHeightToRadians(m_elevatorSim.GetPosition());
  units::turns_per_second_t pulleyRadialVel =
      ConvertHeightVelToRadianVel(m_elevatorSim.GetVelocity());

  units::turn_t motorPos = pulleyTurns * consts::elevator::physical::GEARING;

  units::turns_per_second_t motorVel =
      pulleyRadialVel * consts::elevator::physical::GEARING;

  m_leftMotorSim.SetRawRotorPosition(motorPos);
  m_leftMotorSim.SetRotorVelocity(motorVel);

  m_rightMotorSim.SetRawRotorPosition(motorPos);
  m_rightMotorSim.SetRotorVelocity(motorVel);
}

units::meter_t Elevator::GetHeight() 
{
units::turn_t latencyCompLeft =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          leftPositionSignal, leftVelocitySignal);
  units::turn_t latencyCompRight =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          rightPositionSignal, rightVelocitySignal);
 
  units::meter_t avgHeight =
      ConvertRadiansToHeight((latencyCompLeft + latencyCompRight) / 2) *
      consts::elevator::physical::NUM_OF_STAGES;

  return avgHeight;
}

bool Elevator::IsAtGoalHeight() {
  return isAtGoalHeight;
}

frc2::CommandPtr Elevator::GoToHeightCmd(std::function<units::meter_t()> newHeight) 
{
  return frc2::cmd::Run([this, newHeight] { GoToHeight(newHeight()); }, {this})
      .Until([this] { return IsAtGoalHeight(); });
}

void Elevator::GoToHeight(units::meter_t newHeight) 
{
  goalHeight = newHeight;
  leftMotor.SetControl(elevatorHeightSetter.WithPosition(
      ConvertHeightToRadians(newHeight) /
      consts::elevator::physical::NUM_OF_STAGES));
}

void Elevator::SetVoltage(units::volt_t volts) {
  leftMotor.SetControl(
      elevatorVoltageSetter.WithEnableFOC(true).WithOutput(volts));
}

void Elevator::SetTorqueCurrent(units::ampere_t amps) {
  leftMotor.SetControl(elevatorTorqueCurrentSetter.WithOutput(amps));
}

frc2::CommandPtr Elevator::SysIdElevatorQuasistaticVoltage(
    frc2::sysid::Direction dir) {
  return elevatorSysIdVoltage.Quasistatic(dir).WithName(
      "Elevator Quasistatic Voltage");
}
frc2::CommandPtr Elevator::SysIdElevatorDynamicVoltage(
    frc2::sysid::Direction dir) {
  return elevatorSysIdVoltage.Dynamic(dir).WithName("Elevator Dynamic Voltage");
}

frc2::CommandPtr Elevator::SysIdElevatorQuasistaticTorqueCurrent(
    frc2::sysid::Direction dir) {
  return elevatorSysIdTorqueCurrent.Quasistatic(dir).WithName(
      "Elevator Quasistatic Torque Current");
}
frc2::CommandPtr Elevator::SysIdElevatorDynamicTorqueCurrent(
    frc2::sysid::Direction dir) {
  return elevatorSysIdTorqueCurrent.Dynamic(dir).WithName(
      "Elevator Dynamic Torque Current");
}

frc2::CommandPtr Elevator::TuneElevatorPID(std::function<bool()> isDone) {
  std::string tablePrefix = "Elevator/gains/";
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
          [tablePrefix, this] {
            frc::SmartDashboard::PutNumber(tablePrefix + "setpoint", 0);
            frc::SmartDashboard::PutNumber(
                tablePrefix + "mmCruiseVel",
                consts::elevator::gains::ELEVATOR_GAINS.motionMagicCruiseVel
                    .value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "mmKA", consts::elevator::gains::ELEVATOR_GAINS
                                          .motionMagicExpoKa.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "mmKV", consts::elevator::gains::ELEVATOR_GAINS
                                          .motionMagicExpoKv.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kA",
                consts::elevator::gains::ELEVATOR_GAINS.kA.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kV",
                consts::elevator::gains::ELEVATOR_GAINS.kV.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kS",
                consts::elevator::gains::ELEVATOR_GAINS.kS.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kP",
                consts::elevator::gains::ELEVATOR_GAINS.kP.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kI",
                consts::elevator::gains::ELEVATOR_GAINS.kI.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kD",
                consts::elevator::gains::ELEVATOR_GAINS.kD.value());
            frc::SmartDashboard::PutNumber(tablePrefix + "kG",
                                           consts::elevator::gains::kG.value());
            GoToHeight(0_m);
          },
          {this}),
      frc2::cmd::Run(
          [this, tablePrefix] {
            str::gains::radial::RadialGainsHolder newGains{
                units::turns_per_second_t{frc::SmartDashboard::GetNumber(
                    tablePrefix + "mmCruiseVel", 0)},
                str::gains::radial::turn_volt_ka_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "mmKA", 0)},
                str::gains::radial::turn_volt_kv_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "mmKV", 0)},
                str::gains::radial::turn_amp_ka_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kA", 0)},
                str::gains::radial::turn_amp_kv_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kV", 0)},
                units::ampere_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kS", 0)},
                str::gains::radial::turn_amp_kp_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kP", 0)},
                str::gains::radial::turn_amp_ki_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kI", 0)},
                str::gains::radial::turn_amp_kd_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kD", 0)}};

            units::ampere_t newKg = units::ampere_t{
                frc::SmartDashboard::GetNumber(tablePrefix + "kG", 0)};

            if (newGains != currentGains ||
                !(units::essentiallyEqual(newKg, currentKg, 1e-6))) {
              SetElevatorGains(newGains, newKg);
            }

            GoToHeight(units::inch_t{
                frc::SmartDashboard::GetNumber(tablePrefix + "setpoint", 0)});
          },
          {this})
          .Until(isDone));
}

void Elevator::LogElevatorVolts(frc::sysid::SysIdRoutineLog* log) {
  log->Motor("elevator")
      .voltage((leftVoltageSignal.GetValue() + rightVoltageSignal.GetValue()) / 2.0)
      .position((leftPositionSignal.GetValue() + rightPositionSignal.GetValue()) /
                2.0)
      .velocity((leftVelocitySignal.GetValue() + rightVelocitySignal.GetValue()) /
                2.0);
}

void Elevator::LogElevatorTorqueCurrent(frc::sysid::SysIdRoutineLog* log) {
  log->Motor("elevator")
      .voltage(units::volt_t{(leftTorqueCurrentSignal.GetValueAsDouble() +
                              rightTorqueCurrentSignal.GetValueAsDouble()) /
                             2.0})
      .position((leftPositionSignal.GetValue() + rightPositionSignal.GetValue()) /
                2.0)
      .velocity((leftVelocitySignal.GetValue() + rightVelocitySignal.GetValue()) /
                2.0);
}

void Elevator::SetElevatorGains(str::gains::radial::RadialGainsHolder newGains,
                                units::ampere_t kg) {
  currentGains = newGains;
  ctre::phoenix6::configs::Slot0Configs slotConfig{};
  slotConfig.kV = currentGains.kV.value();
  slotConfig.kA = currentGains.kA.value();
  slotConfig.kS = currentGains.kS.value();
  slotConfig.kP = currentGains.kP.value();
  slotConfig.kI = currentGains.kI.value();
  slotConfig.kD = currentGains.kD.value();
  slotConfig.GravityType =
      ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
  slotConfig.kG = kg.value();

  ctre::phoenix6::configs::MotionMagicConfigs mmConfig{};

  mmConfig.MotionMagicCruiseVelocity = currentGains.motionMagicCruiseVel;
  mmConfig.MotionMagicExpo_kV = currentGains.motionMagicExpoKv;
  mmConfig.MotionMagicExpo_kA = currentGains.motionMagicExpoKa;

  ctre::phoenix::StatusCode statusGains =
      leftMotor.GetConfigurator().Apply(slotConfig);
  if (!statusGains.IsOK()) {
    frc::DataLogManager::Log(
        fmt::format("Left Elevator Motor was unable to set new gains! "
                    "Error: {}, More Info: {}",
                    statusGains.GetName(), statusGains.GetDescription()));
  }

  ctre::phoenix::StatusCode statusMM =
      leftMotor.GetConfigurator().Apply(mmConfig);
  if (!statusMM.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "Left Elevator Motor was unable to set new motion magic config! "
        "Error: {}, More Info: {}",
        statusMM.GetName(), statusMM.GetDescription()));
  }
}

void Elevator::ConfigureMotors() {
  ctre::phoenix6::configs::TalonFXConfiguration config{};
  ctre::phoenix6::configs::Slot0Configs gains{};

  gains.kA = currentGains.kA.value();
  gains.kV = currentGains.kV.value();
  gains.kS = currentGains.kS.value();
  gains.kP = currentGains.kP.value();
  gains.kI = currentGains.kI.value();
  gains.kD = currentGains.kD.value();
  gains.kG = currentKg.value();
  gains.GravityType =
      ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
  config.Slot0 = gains;

  config.MotionMagic.MotionMagicCruiseVelocity =
      currentGains.motionMagicCruiseVel;
  config.MotionMagic.MotionMagicExpo_kV = currentGains.motionMagicExpoKv;
  config.MotionMagic.MotionMagicExpo_kA = currentGains.motionMagicExpoKa;

  config.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  config.Feedback.SensorToMechanismRatio = consts::elevator::physical::GEARING;
  config.MotorOutput.Inverted =
      consts::elevator::physical::INVERT_LEFT
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  config.TorqueCurrent.PeakForwardTorqueCurrent =
      consts::elevator::current_limits::STATOR_LIMIT;
  config.TorqueCurrent.PeakReverseTorqueCurrent =
      -consts::elevator::current_limits::STATOR_LIMIT;

  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.CurrentLimits.SupplyCurrentLimit =
      consts::elevator::current_limits::SUPPLY_LIMIT;

  if (frc::RobotBase::IsSimulation()) {
    config.MotorOutput.Inverted =
        ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  }

  ctre::phoenix::StatusCode configLeftResult =
      leftMotor.GetConfigurator().Apply(config);

  frc::DataLogManager::Log(
      fmt::format("Configured left motor on elevator. Result was: {}",
                  configLeftResult.GetName()));


  // Empty config because we only want to follow left and report output shaft
  // pos
  ctre::phoenix6::configs::TalonFXConfiguration rightConfig{};
  rightConfig.Feedback.SensorToMechanismRatio =
      consts::elevator::physical::GEARING;
  ctre::phoenix::StatusCode configRightResult =
      rightMotor.GetConfigurator().Apply(rightConfig);

  frc::DataLogManager::Log(
      fmt::format("Configured right motor on elevator. Result was: {}",
                  configRightResult.GetName()));

 
}

units::meter_t Elevator::ConvertRadiansToHeight(units::radian_t rots) {
  return (rots / 1_rad) * (consts::elevator::physical::PULLEY_DIAM / 2);
}

units::radian_t Elevator::ConvertHeightToRadians(units::meter_t height) {
  return 1_rad * (height / (consts::elevator::physical::PULLEY_DIAM / 2));
}

units::meters_per_second_t Elevator::ConvertRadianVelToHeightVel(
    units::radians_per_second_t radialVel) {
  return (radialVel / 1_rad) * (consts::elevator::physical::PULLEY_DIAM / 2);
}

units::radians_per_second_t Elevator::ConvertHeightVelToRadianVel(
    units::meters_per_second_t vel) {
  return 1_rad * (vel / (consts::elevator::physical::PULLEY_DIAM / 2));
}