package frc.robot;

public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static enum ElevatorReefPositions {
    // all arbitrary heights, should ideally be at a position between two levels
    INIT(0.0),
    L1(21.0),
    L2(34.875),
    L3(50.625),
    L4(75.0);

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
  
  public static final double kElevatorMaxHeight = 75.0; // inches
  public static final double kElevatorCLRate = 0.3;
  public static final double kElevatorOutputRange = 1.0;
  public static final double kCountsPerRev = 42.0 * (5 / 6) * 20;
  public static final double kCountsPerInch = kCountsPerRev; // countsperrev/linear distance 

  // Feed forward values
  public static final double kFF = 0.0;
  public static final double kVelocityFF = 0.001;

  public static final int XBOX_CONTROLLER_PORT = 0;
  public static final int XBOX_CONTROLLER2_PORT = 1;

  public static final double[] LIMELIGHT_PID_CONSTANTS = { 0.0, 0.0, 0.0 }; //kP, kI, kD -- Arbitrary constants needs PID tuning
  public static final double ALIGN_THRESHOLD = 2.0; 
  public static final double SPEED_MULTIPLIER = 0.1;

  public static final int LINE_BREAKER_PORT = 0;
  public static final int LIMIT_SWITCH_PORT = 9;
}

