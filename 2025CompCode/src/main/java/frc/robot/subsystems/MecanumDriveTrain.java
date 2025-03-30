package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class MecanumDriveTrain extends SubsystemBase {
  SparkMax frontLeft = new SparkMax(Constants.kFrontLeftChannel, MotorType.kBrushless);
  SparkMax rearLeft = new SparkMax(Constants.kRearLeftChannel, MotorType.kBrushless);
  SparkMax frontRight = new SparkMax(Constants.kFrontRightChannel, MotorType.kBrushless);
  SparkMax rearRight = new SparkMax(Constants.kRearRightChannel, MotorType.kBrushless);

  MecanumDrive mDrive;
  public MecanumDriveTrain() {
    SparkMaxConfig frontLeftConfig = new SparkMaxConfig();
    SparkMaxConfig frontRightConfig = new SparkMaxConfig();
    SparkMaxConfig rearLeftConfig = new SparkMaxConfig();
    SparkMaxConfig rearRightConfig = new SparkMaxConfig();

    frontLeftConfig
      .smartCurrentLimit(60)
      .idleMode(IdleMode.kBrake);
  
    frontRightConfig
      .apply(frontLeftConfig)
      .inverted(true);
    
    rearLeftConfig
      .apply(frontLeftConfig);
    
    rearRightConfig
      .apply(frontLeftConfig)
      .inverted(true);

    frontLeft.configure(frontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontRight.configure(frontRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rearLeft.configure(rearLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rearRight.configure(rearRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    mDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
  }

  public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
    mDrive.driveCartesian(xSpeed, ySpeed, zRotation);
  }

  public void stopMotors() {
    frontLeft.set(0);
    frontRight.set(0);
    rearLeft.set(0);
    rearRight.set(0);
  }

  public void oppCornerMotors(double ADMotor, double BCMotor) {
    frontLeft.set(ADMotor);
    frontRight.set(BCMotor);
    rearLeft.set(BCMotor);
    rearRight.set(ADMotor);
  }

  @Override
  public void periodic() {

  }
}

