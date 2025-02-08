// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class driveMecanum extends Command {
  private final DriveTrain driveTrain;
  private final XboxController controller;

  private boolean limelightHasValidTarget = false;
  private double limelightXSpeed = 0;
  private double limelightYSpeed = 0;
  private double limelightZRot = 0;

  public driveMecanum(DriveTrain driveTrain, XboxController controller) {
      this.driveTrain = driveTrain;
      this.controller = controller;
      addRequirements(driveTrain);
  }

  @Override
  public void execute() {
      updateLimelightTracking();
      double xSpeed = -controller.getLeftY();
      double ySpeed = controller.getLeftX();
      double zRotation = controller.getRightX();  
      boolean getA = false;      

      driveTrain.driveCartesian(xSpeed, ySpeed, zRotation);

      // if (controller.getAButton()) {
      //   getA = true;
      // }

      // if (limelightHasValidTarget) {
      //   while (getA) {
      //       if (controller.getAButtonReleased()) {
      //           getA = false;
      //           break;
      //       }
      //       driveTrain.driveCartesian(limelightXSpeed, limelightYSpeed, limelightZRot);

      //   }
      // }

  }

  public void updateLimelightTracking() {
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  
    if (tv < 1.0) {
      limelightHasValidTarget = false;
      limelightXSpeed = 0;
      limelightYSpeed = 0;
      limelightZRot = 0;
      return;
    }
  
    limelightHasValidTarget = true;
    limelightXSpeed = 0.1;
    limelightYSpeed = 0.0;
    limelightZRot = 0.0;
  }

  @Override
  public void end(boolean interrupted) {
      driveTrain.stopMotors();
  }
}