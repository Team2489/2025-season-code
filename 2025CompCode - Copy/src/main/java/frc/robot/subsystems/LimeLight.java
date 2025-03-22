package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;

public class LimeLight extends SubsystemBase {

  private final NetworkTable limelightTable;
  private final double alignThreshold = 2.0;
  
  public LimeLight() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = limelightTable.getEntry("tx");
    NetworkTableEntry ty = limelightTable.getEntry("ty");
    NetworkTableEntry ta = limelightTable.getEntry("ta");
    
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
    return tv >= 1.0;
  }

  @Override
  public void periodic() {
    
  }
}
