// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class CoralIntake extends SubsystemBase {
  SparkMax coralIntake = new SparkMax(Constants.kCoralIntake, MotorType.kBrushless);
  // Add LED blinkin if needed

  public CoralIntake() {
    SparkMaxConfig coralIntakeConfig = new SparkMaxConfig();

    coralIntakeConfig
      .smartCurrentLimit(80)
      .idleMode(IdleMode.kBrake);

    coralIntake.configure(coralIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intakeRun(double power) {
    coralIntake.set(power);
  }

  public void stop() {
    coralIntake.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
