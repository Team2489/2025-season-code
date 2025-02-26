// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;
import edu.wpi.first.wpilibj.DigitalInput;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeIn extends Command {
  CoralIntake coralIntake;
  double power = 0.0;
  DigitalInput digitalInput;


  public IntakeIn(CoralIntake coralIntake, double power, DigitalInput digitalInput) {
    this.coralIntake = coralIntake;
    this.power = power;
    this.digitalInput = digitalInput;
    addRequirements(coralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralIntake.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (digitalInput.get()) {
      coralIntake.stop();
    } else {
      coralIntake.intakeRun(power);
    }
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
