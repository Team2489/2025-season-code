// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class DriveTrain extends SubsystemBase {
  private final WPI_TalonSRX frontLeftMotor;
  private final WPI_TalonSRX frontRightMotor;
  private final WPI_TalonSRX rearLeftMotor;
  private final WPI_TalonSRX rearRightMotor;

  MecanumDrive dDrive;
  XboxController m_stick = new XboxController(0);


  public DriveTrain() {

    frontLeftMotor = new WPI_TalonSRX(Constants.kFrontLeftChannel);  
    frontRightMotor = new WPI_TalonSRX(Constants.kFrontRightChannel);
    rearLeftMotor = new WPI_TalonSRX(Constants.kRearLeftChannel);
    rearRightMotor = new WPI_TalonSRX(Constants.kRearRightChannel);

    frontLeftMotor.setInverted(false);
    frontRightMotor.setInverted(true);
    rearLeftMotor.setInverted(false);
    rearRightMotor.setInverted(true);

    frontLeftMotor.setNeutralMode(NeutralMode.Brake);
    frontRightMotor.setNeutralMode(NeutralMode.Brake);
    rearLeftMotor.setNeutralMode(NeutralMode.Brake);
    rearRightMotor.setNeutralMode(NeutralMode.Brake);

    dDrive = new MecanumDrive(frontLeftMotor::set, rearLeftMotor::set, frontRightMotor::set, rearRightMotor::set);
    
    dDrive.driveCartesian(0, 0, 0);
  }

public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
  dDrive.driveCartesian(xSpeed, ySpeed, zRotation);
}

public void stopMotors() {
  frontLeftMotor.set(ControlMode.PercentOutput, 0);
  frontRightMotor.set(ControlMode.PercentOutput, 0);
  rearLeftMotor.set(ControlMode.PercentOutput, 0);
  rearRightMotor.set(ControlMode.PercentOutput, 0);
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
