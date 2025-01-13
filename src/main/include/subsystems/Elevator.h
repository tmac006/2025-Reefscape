// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <numbers>

#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"
#include "frc/simulation/ElevatorSim.h"
#include "units/voltage.h"

class Elevator : public frc2::SubsystemBase {
 public:
  Elevator();
  void OptimizeBusSignals();

  void Periodic() override;
  void SimulationPeriodic() override;
  units::meter_t GetHeight();
  void GoToHeight();
  void SetVoltage(units::volt_t voltage);

 private:
  void ConfigureMotors();
  void ConfigureControlSignals();

  ctre::phoenix6::hardware::TalonFX m_leftMotor{0}; //Call Constants here
  ctre::phoenix6::hardware::TalonFX m_rightMotor{1}; //Call Constants here

  ctre::phoenix6::sim::TalonFXSimState& m_leftMotorSim = m_leftMotor.GetSimState();
  ctre::phoenix6::sim::TalonFXSimState& m_rightMotorSim = m_rightMotor.GetSimState();
  
  ctre::phoenix6::StatusSignal<units::turn_t> leftPostionSignal = m_leftMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> leftVelocitySignal = m_leftMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::ampere_t> leftTorqueCurrentSignal = m_leftMotor.GetTorqueCurrent();
  ctre::phoenix6::StatusSignal<units::volt_t> leftVoltageSignal = m_leftMotor.GetMotorVoltage();

  ctre::phoenix6::StatusSignal<units::turn_t> rightPostionSignal = m_rightMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> rightVelocitySignal = m_rightMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::ampere_t> rightTorqueCurrentSignal = m_rightMotor.GetTorqueCurrent();
  ctre::phoenix6::StatusSignal<units::volt_t> rightVoltageSignal = m_rightMotor.GetMotorVoltage();

  ctre::phoenix6::controls::MotionMagicExpoTorqueCurrentFOC elevatorHeightSetter{0_rad};
  ctre::phoenix6::controls::VoltageOut elevatorVoltageSetter{0_V};
  ctre::phoenix6::controls::TorqueCurrentFOC elevatorTorqueSetter{0_A};

  //SCREW YOU WPI LIBRARY
      frc::sim::ElevatorSim m_elevatorSim{consts::elevator::physical::MOTOR,
                                    consts::elevator::physical::GEARING,
                                    consts::elevator::physical::CARRIAGE_MASS,
                                    consts::elevator::physical::PULLEY_DIAM / 2,
                                    0_m,
                                    consts::elevator::physical::EXTENDED_HEIGHT,
                                    true,
                                    0_m,
                                    {0.005}};
};
