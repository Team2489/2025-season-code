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
  SparkMax coralIntake1 = new SparkMax(Constants.kCoralIntake1, MotorType.kBrushed);
  SparkMax coralIntake2 = new SparkMax(Constants.kCoralIntake2, MotorType.kBrushed);

  public CoralIntake() {
    SparkMaxConfig coralIntake1Config = new SparkMaxConfig();
    SparkMaxConfig coralIntake2Config = new SparkMaxConfig();

    coralIntake1Config
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake);
    coralIntake2Config
      .apply(coralIntake1Config)
      .inverted(true);

    coralIntake1.configure(coralIntake1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralIntake2.configure(coralIntake2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intakeRun(double power) {
    coralIntake1.set(power);
    coralIntake2.set(-power);
  }

  public void stop() {
    coralIntake1.set(0);
    coralIntake2.set(0);
  }

  @Override
  public void periodic() {
    
  }
}
