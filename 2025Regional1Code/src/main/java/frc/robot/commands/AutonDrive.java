package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MecanumDriveTrain;

public class AutonDrive extends Command {
  MecanumDriveTrain driveTrain;
  double xSpeed;
  double ySpeed;
  double zRotation;

  public AutonDrive(MecanumDriveTrain driveTrain, double xSpeed, double ySpeed, double zRotation) {
    this.driveTrain = driveTrain;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zRotation = zRotation;
    addRequirements(driveTrain); // Ensure this command owns the drivetrain
  }

  @Override
  public void initialize() {driveTrain.driveCartesian(0, 0, 0);}

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
    return false; // This should only be false if used with a timeout or condition
  }
}