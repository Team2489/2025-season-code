// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralIntake;

public class IntakeInDelay extends SequentialCommandGroup {
  public IntakeInDelay(CoralIntake coralIntake, double power) {
    addCommands(
      new IntakeOut(coralIntake, power).withTimeout(0.25)
    );
  }
}
