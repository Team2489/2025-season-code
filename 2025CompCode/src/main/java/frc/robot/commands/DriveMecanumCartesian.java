// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;

// import frc.robot.subsystems.MecanumDriveTrain;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class DriveMecanumCartesian extends Command {

  
//   MecanumDriveTrain mDrive;
//   double xSpeed;
//   double ySpeed;
//   double zRotation;

//   public DriveMecanumCartesian(MecanumDriveTrain mDrive, double xSpeed, double ySpeed, double zRotation) {
//     this.mDrive = mDrive;
//     this.xSpeed = xSpeed;
//     this.ySpeed = ySpeed;
//     this.zRotation = zRotation;
//     addRequirements(mDrive);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     mDrive.driveCartesian(0, 0, 0);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     mDrive.driveCartesian(xSpeed, ySpeed, zRotation);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     mDrive.driveCartesian(0, 0, 0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }



package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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

    mDrive.driveCartesian(linearDeadband(xSpeed, 0.1), linearDeadband(ySpeed, 0.1), linearDeadband(zRotation, 0.1));
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
