// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorReefPositions;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MecanumDriveTrain;

public class DriveMecanumCartesian extends Command {
  private final MecanumDriveTrain mDrive;
  private final XboxController controller;
  private final Elevator elevator;

  public DriveMecanumCartesian(MecanumDriveTrain mDrive, XboxController controller, Elevator elevator) {
    this.mDrive = mDrive;
    this.controller = controller;
    this.elevator = elevator;
    addRequirements(mDrive, elevator);
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
      ySpeed = 0.5 * controller.getRightTriggerAxis();
    }
    if (controller.getLeftTriggerAxis() > 0) {
      ySpeed = 0.5 * -controller.getLeftTriggerAxis();
    }
    
    if (((elevator.getElevatorPosition() - 0.5) > elevator.getElevatorPosition())) {
      elevator.ledLightsGreen();
    } else {
      elevator.ledLightRed();
    }

    mDrive.driveCartesian(linearDeadband(xSpeed, 0.1), linearDeadband(ySpeed, 0.1), linearDeadband(zRotation, 0.1));
  }

  public double linearDeadband(double raw, double deadband) {
    if (Math.abs(raw) < deadband)
      return 0;

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
