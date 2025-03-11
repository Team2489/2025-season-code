// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralIntake;

public class ScoreLevelHeight extends SequentialCommandGroup {
  public ScoreLevelHeight(CoralIntake coralIntake, Elevator elevator, DigitalInput limitSwitch, double height, double intakePower) {
    addCommands(
      new SetElevatorPosition(elevator, height),
      new IntakeOut(coralIntake, intakePower),
      new ResetElevatorPosition(elevator, limitSwitch)
    );
  }
}
