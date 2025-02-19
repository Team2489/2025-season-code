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

  private String limStatus = "Limelight-";

  public NetworkTableEntry tx = limelightTable.getEntry("tx");
  public NetworkTableEntry ty = limelightTable.getEntry("ty");
  public NetworkTableEntry ta = limelightTable.getEntry("ta");


  /** Creates a new LimeLight. */
  public LimeLight(String orientation) {
    netTable = NetworkTableInstance.getDefault().getTable("limelight-" + orientation);
    update();
    limStatus += orientation;
  }

  public String toString() {
    return limStatus;
  }

  // ------ Setting up pipeline -------
  public void set(int pipeline) {
    setPipeline(pipeline);
  }

  private void setPipeline(int pipeline) {
    set("pipeline", pipeline);
  }

  // update tx, ty, ta limelight values constantly for network tables
  public void update() {
    getTx();
    getTy();
    getTa();
  }

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
    return tv = 1.0;
  }
  public void set(String input, int input2) {
    table.getEntry(input).setNumber(input2);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
