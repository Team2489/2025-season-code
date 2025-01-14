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

// Phoenix 5 is in the com.ctre.phoenix.* packages
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix6.hardware.TalonFX;




public class DriveTrain extends SubsystemBase {
  
  TalonFX frontLeft = new TalonFX(Constants.kFrontLeftChannel);
  TalonFX rearLeft = new TalonFX(Constants.kRearLeftChannel);
  TalonFX frontRight = new TalonFX(Constants.kFrontRightChannel);
  TalonFX rearRight = new TalonFX(Constants.kRearRightChannel);

  MecanumDrive dDrive;
  XboxController m_stick;
  
  TalonFXConfiguration m_talonFXConfig = new TalonFXConfiguration();

    

  public DriveTrain() {    
    TalonFXConfiguration frontLeftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rearLeftConfig = new TalonFXConfiguration();
    TalonFXConfiguration frontRightConfig = new TalonFXConfiguration();
    TalonFXConfiguration rearRightConfig = new TalonFXConfiguration();

    // Set inversion using InvertedValue
    frontLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rearLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    frontRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rearRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Apply the configurations to the TalonFX motors
    frontLeft.getConfigurator().apply(frontLeftConfig);
    rearLeft.getConfigurator().apply(rearLeftConfig);
    frontRight.getConfigurator().apply(frontRightConfig);
    rearRight.getConfigurator().apply(rearRightConfig);
    
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
