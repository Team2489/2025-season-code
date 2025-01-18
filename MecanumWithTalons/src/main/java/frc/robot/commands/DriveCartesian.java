// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCartesian extends Command {
  DriveTrain dDrive;
  double xSpeed;
  double ySpeed;
  double zRotation;
  //XboxController xboxController = new XboxController(0);

  /** Creates a new DriveCartesian. */
  public DriveCartesian(DriveTrain dDrive, double xSpeed, double ySpeed, double zRotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dDrive = dDrive;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zRotation = zRotation;
    addRequirements(dDrive);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dDrive.driveCartesian(xSpeed, ySpeed, zRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
