package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static enum ElevatorReefPositions {
    // all arbitrary heights
    INIT(0.0),
    L1(5.0),
    L2(10.0),
    L3(15.0),
    L4(20.0);

    public final double height;

    private ElevatorReefPositions(double height) {
      this.height = height;
    }

  }

  public static final int kFrontLeftChannel = 3;
  public static final int kRearLeftChannel = 1;
  public static final int kFrontRightChannel = 4;
  public static final int kRearRightChannel = 2;

  public static final int kCoralIntake1 = 21;
  public static final int kCoralIntake2 = 10; // arbitrary

  // Elevator constants
  public static final int kElevatorLeft = 11;
  public static final int kElevatorRight = 12;
  public static final double[] ELEVATOR_PID_CONSTANTS = { 0.10, 0.0, 0.0 }; // needs PID tuning
  public static final double[] ELEVATOR_FF_CONSTANTS = { 0.0, 0.0, 0.0, 0.0 }; // arbitrary // kS, kG, kV, kA
  public static final double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
  
  // kS: Static friction, kG: gravity, kV: Velocity, kA: can be ommitted (set to 0.0)
  public static final double kMaxAcceleration = Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);
  public static final double kElevatorMinHeight = 0.0;
  public static final double kElevatorMaxHeight = 31.0;
  public static final double kElevatorCLRate = 0.3;
  public static final double kElevatorOutputRange = 1.0;
  public static final double kElevatorLength = 1; // need to measure, 31 inches extension from each level

  // Feed forward values
  public static final double kFF = 0.0;
  public static final double kVelocityFF = 0.001;

  public static final int XBOX_CONTROLLER_PORT = 0;
  public static final int XBOX_CONTROLLER2_PORT = 1;

  public static final double[] LIMELIGHT_PID_CONSTANTS = { 0.0, 0.0, 0.0 }; //kP, kI, kD -- Arbitrary constants needs PID tuning
  public static final double ALIGN_THRESHOLD = 2.0; 
  public static final double SPEED_MULTIPLIER = 0.1;

  public static final int LINE_BREAKER_PORT = 0;
  public static final int LIMIT_SWITCH_PORT = 1;
}

