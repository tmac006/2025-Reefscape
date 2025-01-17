// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/NetworkButton.h>

#include <functional>
#include <memory>
#include "Constants.h"
#include "subsystems/ExampleSubsystem.h"

#include "subsystems/Elevator.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
 void ConfigureBindings();
  frc2::CommandPtr ElevatorVoltsSysIdCommands(
      std::function<bool()> fwd, std::function<bool()> quasistatic);
  frc2::CommandPtr ElevatorTorqueCurrentSysIdCommands(
      std::function<bool()> fwd, std::function<bool()> quasistatic);

  ExampleSubsystem m_subsystem;
  Elevator m_elevatorSub;
std::shared_ptr<nt::NetworkTable> tuningTable{nt::NetworkTableInstance::GetDefault().GetTable("Tuning")};
    frc2::NetworkButton elevatorTuneBtn{tuningTable, "ElevatorPidTuning"};
    frc2::NetworkButton elevatorSysIdVoltsBtn{tuningTable, "ElevatorSysIdVolts"};
    frc2::NetworkButton elevatorSysIdTorqueCurrentBtn{tuningTable, "ElevatorSysIdTorqueCurrent"};

};
