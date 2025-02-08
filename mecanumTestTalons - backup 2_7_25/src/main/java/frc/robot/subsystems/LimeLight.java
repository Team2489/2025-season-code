// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.NetworkTableListener;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;

public class LimeLight extends SubsystemBase {

  private final NetworkTable limelightTable;
  private final double alignThreshold = 2.0;


  /** Creates a new LimeLight. */
  public LimeLight() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = limelightTable.getEntry("tx");
    NetworkTableEntry ty = limelightTable.getEntry("ty");
    NetworkTableEntry ta = limelightTable.getEntry("ta");
    
  }

//   public void getTx() {
//     NetworkTableEntry tx = limelightTable.getEntry("tx");
//     SmartDashboard.putNumber("LimeLightX", tx.getDouble(0.0));
// }

// // Method to get vertical offset (ty) of the AprilTag
// public void getTy() {
//   NetworkTableEntry ty = limelightTable.getEntry("ty");
//   SmartDashboard.putNumber("LimeLightX", ty.getDouble(0.0));
// }

// // Method to get the area (ta) of the AprilTag
// public void getTa() {
//   NetworkTableEntry ta = limelightTable.getEntry("ta");
//   SmartDashboard.putNumber("LimeLightX", ta.getDouble(0.0));
// }
  public double getTx() {
    return limelightTable.getEntry("tx").getDouble(0.0);
  }

  public double getTa() {
    return limelightTable.getEntry("ta").getDouble(0.0);
  }
  public double getTy() {
    return limelightTable.getEntry("ty").getDouble(0.0);
  }
  public boolean hasValidTarget() {
    double tv = limelightTable.getEntry("tv").getDouble(0.0);
    return tv >= 1.0;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
