// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.MecanumDriveTrain;

public class DriveMecanumCartesian extends Command {
  private final MecanumDriveTrain mDrive;
  private final XboxController controller;

  public DriveMecanumCartesian(MecanumDriveTrain mDrive, XboxController controller) {
    this.mDrive = mDrive;
    this.controller = controller;
    addRequirements(mDrive);
  }

  @Override
  public void initialize() {
    System.out.println("DriveMecanumCartesian Command Initialized");
    mDrive.stopMotors();
  }

  @Override
  public void execute() {
    double xSpeed = -controller.getLeftY();
    double ySpeed = controller.getLeftX();
    double zRotation = controller.getRightX();  
    if (controller.getRightTriggerAxis() > 0) {
      ySpeed = controller.getRightTriggerAxis();
    }
    if (controller.getLeftTriggerAxis() > 0) {
      ySpeed = -controller.getLeftTriggerAxis();
    }

    mDrive.driveCartesian(linearDeadband(xSpeed, 0.1), ySpeed, linearDeadband(zRotation, 0.1));
  }

  public double linearDeadband(double raw, double deadband) {
    if (Math.abs(raw)<deadband) return 0;

    //return Math.signum(raw)*(Math.abs(raw)-deadband)/(1-deadband);
    return raw;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("DriveMecanumCartesian Command Ended");
    mDrive.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false; // Keeps running until interrupted
  }
}
