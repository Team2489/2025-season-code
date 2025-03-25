package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MecanumDriveTrain;

public class A_DriveMec extends Command {
  MecanumDriveTrain driveTrain;
  double xSpeed;
  double ySpeed;
  double zRotation;

  public A_DriveMec(MecanumDriveTrain driveTrain, double xSpeed, double ySpeed, double zRotation) {
    this.driveTrain = driveTrain;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zRotation = zRotation;
    addRequirements(driveTrain); 
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    driveTrain.driveCartesian(xSpeed, ySpeed, zRotation);
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}