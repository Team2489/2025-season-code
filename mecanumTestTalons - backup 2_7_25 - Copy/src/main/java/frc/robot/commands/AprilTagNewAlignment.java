// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AprilTagNewAlignment extends Command {
  private final DriveTrain driveTrain;
  private final LimeLight limeLight;
  private final Encoder FrontLeftEncoder;
  private final double limelightAngle = Constants.LIMELIGHT_ANGLE;
  private final double limelightHeight = Constants.LIMELIGHT_HEIGHT;
  private final double angleLimelightAT = 0.0; //measure
  private final double heightAT = 0; //measure
  private final double kP = 0.01; // Proportional control for rotation
  private final double kVy = 0.3; // Proportional control for vertical movement (ySpeed)
  private final double targetAreaThreshold = 3;
  private final double finalThreshold = 2.0;
  private final double alignmentThreshold = 6; // Threshold for tx alignment
  private final double verticalAlignmentThreshold = -0.1; // Threshold for ty vertical alignment
  private static final double YAW_THRESHOLD = 2.0;
  private static final double kP_YAW = 0.01;

  public AprilTagNewAlignment(DriveTrain driveTrain, LimeLight limeLight, Encoder FrontLeftEncoder) {
    FrontLeftEncoder = new Encoder(0, 1);

    this.driveTrain = driveTrain;
    this.limeLight = limeLight;
    this.FrontLeftEncoder = FrontLeftEncoder;
    addRequirements(driveTrain, limeLight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  //tan(tx)=ydist/lldisttoat
  public double getDistance(double limelightAngle, double limelightHeight, double angleLimelightAT, double heightAT) {
    return (heightAT-limelightHeight) / Math.tan(limelightAngle+angleLimelightAT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tx = limeLight.getTx();
    double ty = limeLight.getTy(); // Vertical offset
    double ta = limeLight.getTa();
    boolean hasTarget = limeLight.hasValidTarget();
    double xSpeed = 0; 
    double ySpeed = 0; // Verticalx` speed
    double zRotation = 0;
    Rotation2d currentRotation = driveTrain.getRotation();
    double yaw = currentRotation.getDegrees();
    double yawCorrection = 0;

    double distancePerPulse = Math.PI * Constants.WHEEL_DIAMETER / Constants.PULSE_PER_REVOLUTION
				/ Constants.ENCODER_GEAR_RATIO / Constants.GEAR_RATIO * Constants.FUDGE_FACTOR;
	        FrontLeftEncoder.setDistancePerPulse(distancePerPulse);

    if (ta < targetAreaThreshold && tx < 0) {
      ySpeed = kVy * ty; // Adjust the constant kVy as necessary for vertical alignment
    } else if (ta < targetAreaThreshold && tx > 0) {
      ySpeed = -kVy * ty; // Adjust the constant kVy as necessary for vertical alignment
    }

    // if (!hasTarget) {
    //   driveTrain.stopMotors();
    //   return;
    // }

    double encoderDistanceReading = FrontLeftEncoder.getDistance();
		SmartDashboard.putNumber("encoder reading", encoderDistanceReading);

    driveTrain.driveCartesian(xSpeed, ySpeed, zRotation);

    if (encoderDistanceReading > 36) {
			driveTrain.driveCartesian(0, 0, 0);
		}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      double tx = limeLight.getTx();
      double ta = limeLight.getTa();

      // Stop rotating and moving vertically when aligned (tx near zero and ty near zero) and when at the target distance
      boolean isAligned = Math.abs(tx) <= 0.1 && Math.abs(driveTrain.getRotation().getDegrees()) < YAW_THRESHOLD && Math.abs(limeLight.getTy()) <= verticalAlignmentThreshold; // Stop rotating and moving vertically when sufficiently aligned
      boolean isAtTargetDistance = ta >= targetAreaThreshold;

      return isAligned && isAtTargetDistance;
  }
}
