// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final int kFrontLeftChannel = 0;
  public static final int kRearLeftChannel = 2;
  public static final int kFrontRightChannel = 1;
  public static final int kRearRightChannel = 3;

  public static final int XBOX_CONTROLLER_PORT = 0;
  public static final int XBOX_CONTROLLER2_PORT = 1;

  public static final double[] LIMELIGHT_PID_CONSTANTS = { 0.0, 0.0, 0.0 }; //kP, kI, kD -- Arbitrary constants needs PID tuning
  public static final double ALIGN_THRESHOLD = 2.0; 
  public static final double SPEED_MULTIPLIER = 0.1;
}
