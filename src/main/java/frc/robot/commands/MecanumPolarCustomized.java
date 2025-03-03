// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.MecanumDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MecanumPolarCustomized extends Command {
  MecanumDrivetrain mDrive;
  double magnitude;
  double angle;
  double zRotation;
  XboxController xboxController;

  public MecanumPolarCustomized(MecanumDrivetrain mDrive, double magnitude, double angle, double zRotation, XboxController xboxController) {
    this.mDrive = mDrive;
    this.magnitude = magnitude;
    this.angle = angle;
    this.zRotation = zRotation;
    this.xboxController = xboxController;
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
    magnitude = Math.hypot(xboxController.getRawAxis(0), xboxController.getRawAxis(1));
    angle = Math.atan2(xboxController.getRawAxis(1), xboxController.getRawAxis(0));
    zRotation = xboxController.getRawAxis(2);

    drivePolarCustomized(magnitude, angle, zRotation);
  }

  public void drivePolarCustomized(double magnitude, double angle, double rotationPower) {
    // motor power
    double topLbotRPower = magnitude * Math.sqrt(2) * 0.5 * (Math.sin(angle) + Math.cos(angle));
    double topRbotLPower = magnitude * Math.sqrt(2) * 0.5 * (Math.sin(angle) - Math.cos(angle));

    // don't want rotation power to interfere with magnitude power, checking if topLbotRPower and topRbotLPower > 1.0 or < -1.0
    double turningScale = Math.max(Math.abs(topLbotRPower + rotationPower), Math.abs(topLbotRPower - rotationPower));
    turningScale = Math.max(turningScale, Math.max(Math.abs(topRbotLPower + rotationPower), Math.abs(topRbotLPower - rotationPower)));

    if (Math.abs(rotationPower) < 1.0) {
      turningScale = 1.0;
    }

    // set the motors, and divide them by turningScale to make sure none of them go over the top, which would alter the translation angle
    mDrive.setMotors((topLbotRPower - turningScale) / turningScale, (topRbotLPower + turningScale) / turningScale, (topRbotLPower - turningScale) / turningScale, (topLbotRPower + turningScale) / turningScale);
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
