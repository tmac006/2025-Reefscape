#include "subsystems/Elevator.h"

#include "Constants.h"
#include "ctre/phoenix/StatusCodes.h"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "frc/DataLogManager.h"
#include "frc/RobotBase.h"
#include "frc/RobotController.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/voltage.h"

Elevator::Elevator() 
{
  ConfigureMotors();
  ConfigureControlSignals();

  OptimizeBusSignals();
}

void Elevator::Periodic()
{
    ctre::phoenix::StatusCode status =
        ctre::phoenix6::BaseStatusSignal::WaitForAll(
          2.0 / consts::elevator::BUS_UPDATE_FREQ, leftPostionSignal,
          leftVelocitySignal, leftTorqueCurrentSignal, leftVoltageSignal,
          rightPostionSignal, rightVelocitySignal, rightTorqueCurrentSignal,
          rightVoltageSignal);
    
    if (!status.IsOK())
    {
        frc::DataLogManager::Log(fmt::format("Error Updating Position Error Was: {}" , status.GetName())); // Fix later
    }
}
void Elevator::SimulationPeriodic() 
{
    m_leftMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
    m_rightMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    m_elevatorSim.SetInputVoltage((m_leftMotorSim.GetMotorVoltage() + m_rightMotorSim.GetMotorVoltage()) / 2);

    m_elevatorSim.Update(20_ms);

  double motorPos =
      (m_elevatorSim.GetPosition().value() /
       (consts::elevator::physical::PULLEY_DIAM.value() * std::numbers::pi)) *
      (consts::elevator::physical::GEARING);

  double motorVel =
      (m_elevatorSim.GetVelocity().value() /
       (consts::elevator::physical::PULLEY_DIAM.value() * std::numbers::pi)) *
      (consts::elevator::physical::GEARING);

    m_leftMotorSim.SetRawRotorPosition(units::turn_t{motorPos});
    m_leftMotorSim.SetRotorVelocity(units::turns_per_second_t{motorVel});

    m_rightMotorSim.SetRawRotorPosition(units::turn_t{motorPos});
    m_rightMotorSim.SetRotorVelocity(units::turns_per_second_t{motorVel});
}

units::meter_t Elevator::GetHeight() {
  units::meter_t latencyCompLeftHeight = units::meter_t{
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          leftPositionSignal, leftVelocitySignal)
          .value()};
  units::meter_t latencyCompRightHeight = units::meter_t{
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          rightPositionSignal, rightVelocitySignal)
          .value()};

  units::meter_t avgHeight =
      (latencyCompLeftHeight + latencyCompRightHeight) / 2;

  return avgHeight;
};

void Elevator::GoToHeight() {}

void Elevator::SetVoltage(units::volt_t volts) {
  m_leftMotor.SetControl(elevatorVoltageSetter.WithEnableFOC(true).WithOutput(volts));
  m_rightMotor.SetControl(elevatorVoltageSetter.WithEnableFOC(true).WithOutput(volts));
}
  
