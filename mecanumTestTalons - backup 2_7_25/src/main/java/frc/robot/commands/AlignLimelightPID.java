// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import edu.wpi.first.networktables.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignLimelightPID extends Command {
  private final DriveTrain driveTrain;
  private final LimeLight limeLight;

  public double previousError;
  private MovingAverageValues txAvg = new MovingAverageValues(); // can even use tx, ta values

  // speed values for mecanum drivetrain
  public double xSpeed = 0;
  public double ySpeed = 0;
  public double zRotation = 0;

  // PID Constants
  private double kP = Constants.LIMELIGHT_PID_CONSTANTS[0];
  private double kI = Constants.LIMELIGHT_PID_CONSTANTS[1];
  private double kD = Constants.LIMELIGHT_PID_CONSTANTS[2];

  // Collect average values of tx, ty, ta values taken constantly to find error
  private class MovingAverageValues {
    private int size;
		private double valueSum = 0d;
		private int ind = 0;
		private double values[];

    public RollingAverage(int size) {
			this.size = size;
			values = new double[size];
			for (int i = 0; i < size; i++) {
        values[i] = 0d;
      }
		}

    public void add(double x) {
			valueSum -= values[ind];
			values[ind] = x;
			valueSum += x;
			if (++ind == size) {
        ind = 0;
      }
		}

		public double getMovingAverage() {
			return valueSum / size;
		}
  }

  public AlignLimelightPID(DriveTrain drivetrain, Limelight limeLight, String orientation) {
    this.driveTrain = driveTrain;
    this.limeLight = new Limelight(orientation);
    limelight.set("streamMode", 0);
    addRequirements(driveTrain, limeLight);
  }

  // Not moving towards target yet
  private void alignToTarget() {
    limeLight.update();
    updateValues();
    txAvg.add(limeLight.getTx());
    // error, max is +- 27 degrees, scaled to -1 to 1 for speed
    double error = txAvg.getMovingAverage() / 27;
    if (limeLight.hasValidTarget() && Math.abs(error) < Constants.ALIGN_THRESHOLD) {
      driveTrain.driveCartesian(0, 0, 0);
    } else {
      // Centering
      centeringRobot(error, kP, kI, kD);
    }
  }

  private void centeringRobot(double error, double kP, double kI, double kD) {
    updateValues();
    double errorDiff, sumError, output, rotSpeed;
    errorDiff = error - previousError;
    sumError += error;
    output = (kP * error) + (kI * sumError) + (kD * errorDiff);
    // setting speed based on error with PID weights
    rotSpeed = output;
    // Setting speed thresholds
    if (rotSpeed > 0 && rotSpeed < 0.05) {
			rotSpeed = 0.05;
    }
		else if (rotSpeed < 0 && rotSpeed > -0.05) {
			rotSpeed = -0.05;
    }
		else if (rotSpeed > 0.5) {
      rotSpeed = 0.5;
    } 
		else if (rotSpeed < -0.5) {
      rotSpeed = -0.5;
    }

    // update error
    previousError = error;
    // rotates to align
    zRotation = rotSpeed;
    driveTrain(xSpeed, ySpeed, zRotation);
  } 

  // update Limelight tx, ty, ta values
	public void updateValues() {
		limelight.update();
		tx = limelight.getTx();
		ty = limelight.getTy();
		ta = limelight.getTa();
	}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    alignToTarget();
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
