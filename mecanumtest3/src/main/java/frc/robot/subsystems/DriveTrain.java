// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase;



public class DriveTrain extends SubsystemBase {
  
  SparkMax frontLeft = new SparkMax(Constants.kFrontLeftChannel, MotorType.kBrushless);
  SparkMax rearLeft = new SparkMax(Constants.kRearLeftChannel, MotorType.kBrushless);
  SparkMax frontRight = new SparkMax(Constants.kFrontRightChannel, MotorType.kBrushless);
  SparkMax rearRight = new SparkMax(Constants.kRearRightChannel, MotorType.kBrushless);

  MecanumDrive dDrive;
  XboxController m_stick;
  
  SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

  public DriveTrain() {    
    sparkMaxConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    
    frontLeft.configure(sparkMaxConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    rearLeft.configure(sparkMaxConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    frontRight.configure(sparkMaxConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    rearRight.configure(sparkMaxConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    
    dDrive = new MecanumDrive(frontLeft::set, rearLeft::set, frontRight::set, rearRight::set);
    dDrive.driveCartesian(-m_stick.getLeftY(), -m_stick.getLeftX(), -m_stick.getRightX());
    // Drive at 45 degrees relative to the robot, at the speed given by the Y axis of the joystick, with no rotation.
    dDrive.drivePolar(-m_stick.getLeftY(), Rotation2d.fromDegrees(45), 0);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
