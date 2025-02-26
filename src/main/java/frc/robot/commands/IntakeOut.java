// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

public class IntakeOut extends Command {
  CoralIntake coralIntake;
  double power = 0.0;

  public IntakeOut(CoralIntake coralIntake, double power) {
    this.coralIntake = coralIntake;
    this.power = power;
    addRequirements(coralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralIntake.intakeRun(-power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralIntake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
