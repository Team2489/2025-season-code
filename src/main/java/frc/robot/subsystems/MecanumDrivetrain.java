// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;


public class MecanumDrivetrain extends SubsystemBase {
  SparkMax frontLeft = new SparkMax(Constants.kFrontLeftChannel, MotorType.kBrushless);
  SparkMax rearLeft = new SparkMax(Constants.kRearLeftChannel, MotorType.kBrushless);
  SparkMax frontRight = new SparkMax(Constants.kFrontRightChannel, MotorType.kBrushless);
  SparkMax rearRight = new SparkMax(Constants.kRearRightChannel, MotorType.kBrushless);

  Rotation2d poseAngle = new Rotation2d(0);

 // MecanumDrive dDrive;
  MecanumDrive mDrive;
  public MecanumDrivetrain() {
    SparkMaxConfig frontLeftConfig = new SparkMaxConfig();
    SparkMaxConfig frontRightConfig = new SparkMaxConfig();
    SparkMaxConfig rearLeftConfig = new SparkMaxConfig();
    SparkMaxConfig rearRightConfig = new SparkMaxConfig();

    frontLeftConfig // global config
      .smartCurrentLimit(80)
      .idleMode(IdleMode.kBrake);
  
    frontRightConfig // right leader
      .apply(frontLeftConfig)
      .inverted(true);
    
    rearLeftConfig // left follower
      .apply(frontLeftConfig)
      .follow(Constants.kFrontLeftChannel);
    
    rearRightConfig // right follower
      .apply(frontLeftConfig)
      .follow(Constants.kFrontRightChannel);

    frontLeft.configure(frontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontRight.configure(frontRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rearLeft.configure(rearLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rearRight.configure(rearRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    mDrive = new MecanumDrive(frontLeft, frontRight, rearLeft, rearRight);
    mDrive.driveCartesian(0, 0, 0);
    mDrive.drivePolar(0, poseAngle, 0);
  }

  public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
    mDrive.driveCartesian(xSpeed, ySpeed, zRotation);
  }

  public void drivePolar(double magnitude, Rotation2d angle, double zRotation) {
    mDrive.drivePolar(magnitude, angle, zRotation);
  }

  public void stopMotors() {
    frontLeft.set(0);
    frontRight.set(0);
    rearLeft.set(0);
    rearRight.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
