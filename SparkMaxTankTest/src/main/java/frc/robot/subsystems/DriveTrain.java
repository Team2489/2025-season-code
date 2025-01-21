// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;


public class DriveTrain extends SubsystemBase {
  
  SparkMax frontLeft = new SparkMax(Constants.kFrontLeftChannel, MotorType.kBrushless);
  SparkMax rearLeft = new SparkMax(Constants.kRearLeftChannel, MotorType.kBrushless);
  SparkMax frontRight = new SparkMax(Constants.kFrontRightChannel, MotorType.kBrushless);
  SparkMax rearRight = new SparkMax(Constants.kRearRightChannel, MotorType.kBrushless);

 // MecanumDrive dDrive;
  DifferentialDrive dDrive;

  public DriveTrain() {    
    // dDrive = new MecanumDrive(frontLeft::set, rearLeft::set, frontRight::set, rearRight::set);
    // dDrive.driveCartesian(-m_stick.getLeftY(), -m_stick.getLeftX(), -m_stick.getRightX());
    // // Drive at 45 degrees relative to the robot, at the speed given by the Y axis of the joystick, with no rotation.
    // dDrive.drivePolar(-m_stick.getLeftY(), Rotation2d.fromDegrees(45), 0);
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

    globalConfig
    .smartCurrentLimit(80)
    .idleMode(IdleMode.kBrake);
  
  rightLeaderConfig
    .apply(globalConfig)
    .inverted(true);
  
  leftFollowerConfig
    .apply(globalConfig)
    .follow(Constants.kFrontLeftChannel);
  
  rightFollowerConfig
    .apply(globalConfig)
    .follow(Constants.kFrontRightChannel);

    frontLeft.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontRight.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rearLeft.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rearRight.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    dDrive = new DifferentialDrive(frontLeft, frontRight);
    dDrive.arcadeDrive(0, 0);
    arcadeDriveCustomized(0, 0);
  }

  public void arcadeDriveCustomized(double speed, double rotation){

    dDrive.feed();
    dDrive.feedWatchdog();
    frontRight.set(rotation-speed);
    rearRight.set(rotation-speed);
    frontLeft.set(speed+rotation);
    rearLeft.set(speed+rotation);
  }

  public void stopMotors() {
    frontLeft.set(0);
    frontRight.set(0);
    rearLeft.set(0);
    rearRight.set(0);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    dDrive.feed();
    dDrive.feedWatchdog();

    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
