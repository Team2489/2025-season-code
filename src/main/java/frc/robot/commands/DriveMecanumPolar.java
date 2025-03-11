// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MecanumDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveMecanumPolar extends Command {
  MecanumDrivetrain mDrive;
  double magnitude;
  Rotation2d angle;
  double zRotation;

  public DriveMecanumPolar(MecanumDrivetrain mDrive, double magnitude, Rotation2d angle, double zRotation) {
    this.mDrive = mDrive;
    this.magnitude = magnitude;
    this.angle = angle;
    this.zRotation = zRotation;
    addRequirements(mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrive.drivePolar(0, null, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrive.drivePolar(magnitude, angle, zRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrive.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
