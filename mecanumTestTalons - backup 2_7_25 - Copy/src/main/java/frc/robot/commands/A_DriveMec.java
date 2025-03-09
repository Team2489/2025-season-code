package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class A_DriveMec extends Command {
  DriveTrain driveTrain;
  double xSpeed;
  double ySpeed;
  double zRotation;

  public A_DriveMec(DriveTrain driveTrain, double xSpeed, double ySpeed, double zRotation) {
    this.driveTrain = driveTrain;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zRotation = zRotation;
    addRequirements(driveTrain); // Ensure this command owns the drivetrain
  }

  @Override
  public void initialize() {
    System.out.println("A_DriveMec started");
  }

  @Override
  public void execute() {
    System.out.println("Executing A_DriveMec: x=" + xSpeed + ", y=" + ySpeed + ", z=" + zRotation);
    driveTrain.driveCartesian(xSpeed, ySpeed, zRotation);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("A_DriveMec ended, interrupted=" + interrupted);
    driveTrain.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false; // This should only be false if used with a timeout or condition
  }
}