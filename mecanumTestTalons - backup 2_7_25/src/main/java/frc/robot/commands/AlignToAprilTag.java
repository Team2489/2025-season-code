// package frc.robot.commands;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.LimeLight;
// import frc.robot.Constants;
// import frc.robot.LimelightHelpers;

// public class AlignToAprilTag extends Command {
//     private final DriveTrain driveTrain;
//     private final LimeLight limeLight;
//     private final double kP = 0.025; // Proportional control for rotation

//     public AlignToAprilTag(DriveTrain driveTrain, LimeLight limeLight) {
//         this.driveTrain = driveTrain;
//         this.limeLight = limeLight;
//         addRequirements(driveTrain, limeLight);
//     }

//     @Override
//     public void initialize() {
//     }

//     public double getDistance() {
//         double tx = LimelightHelpers.getTX("limelight");
//         Rotation2d angleToGoal = Rotation2d.fromDegrees(tx);
//         double distance = 3.0 / angleToGoal.getTan(); 
//         return distance;
//     }

//     @Override
//     public void execute() {
//         double tx = limeLight.getTx();
//         double ta = limeLight.getTa();
//         boolean hasTarget = limeLight.hasValidTarget();
//         double xSpeed = 0; 
//         double zRotation = 0;
//         double targetAreaThreshold = 5.0;

//         // Move forward only if not at the target distance
//         if (ta < targetAreaThreshold) {
//             xSpeed = Constants.SPEED_MULTIPLIER * (targetAreaThreshold - ta);
//         }

//         if (ta > targetAreaThreshold){
//             if (tx != 0) {
//                 xSpeed = 0;
//                 zRotation = 0.0001 * tx;
//             }
  
//         }

//         // Rotate to center the AprilTag
//         if (Math.abs(tx) > 0.1) { // Only rotate if tx is significant
//             zRotation = kP * tx; // Negative sign ensures proper correction
//         }

//         if (!hasTarget) {
//             driveTrain.stopMotors();
//             return;
//         }

//         driveTrain.driveCartesian(xSpeed, 0, zRotation);
//     }

//     @Override
//     public boolean isFinished() {
//         double tx = limeLight.getTx();
//         double ta = limeLight.getTa();
//         double targetAreaThreshold = 5.0;
        
//         boolean isAligned = Math.abs(tx) <= 1.0; // Stop rotating when straight
//         boolean isAtTargetDistance = ta >= targetAreaThreshold;

//         return isAligned && isAtTargetDistance;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         driveTrain.stopMotors();
//     }
// }


// package frc.robot.commands;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.LimeLight;
// import frc.robot.Constants;
// import frc.robot.LimelightHelpers;

// public class AlignToAprilTag extends Command {
//     private final DriveTrain driveTrain;
//     private final LimeLight limeLight;
//     private final double kP = 0.025; // Proportional control for rotation

//     public AlignToAprilTag(DriveTrain driveTrain, LimeLight limeLight) {
//         this.driveTrain = driveTrain;
//         this.limeLight = limeLight;
//         addRequirements(driveTrain, limeLight);
//     }

//     @Override
//     public void initialize() {
//     }

//     public double getDistance() {
//         double tx = LimelightHelpers.getTX("limelight");
//         Rotation2d angleToGoal = Rotation2d.fromDegrees(tx);
//         double distance = 3.0 / angleToGoal.getTan(); 
//         return distance;
//     }

//     @Override
//     public void execute() {
//         double tx = limeLight.getTx();
//         double ta = limeLight.getTa();
//         boolean hasTarget = limeLight.hasValidTarget();
//         double xSpeed = 0; 
//         double zRotation = 0;
//         double targetAreaThreshold = 4.0;  // You can adjust this threshold

//         // Move forward if the robot is not at the target distance
//         if (ta < targetAreaThreshold) {
//             xSpeed = Constants.SPEED_MULTIPLIER * (targetAreaThreshold - ta); // Move forward if the target area is too small
//         }

//         // Apply rotation correction based on tx value
//         if (Math.abs(tx) > 0.1) {  // Only rotate if tx is sufficiently off-center
//             zRotation = kP * tx; // Adjust the constant kP as necessary
//         }

//         // If no valid target is found, stop the motors
//         if (!hasTarget) {
//             driveTrain.stopMotors();
//             return;
//         }

//         // Apply Cartesian drive to align the robot to the AprilTag
//         driveTrain.driveCartesian(xSpeed, 0, zRotation);
//     }

//     @Override
//     public boolean isFinished() {
//         double tx = limeLight.getTx();
//         double ta = limeLight.getTa();
//         double targetAreaThreshold = 4.0;

//         // Check if the robot is aligned and at the correct distance
//         boolean isAligned = Math.abs(tx) <= 0.1; // Stop rotating when tx is close enough to 0
//         boolean isAtTargetDistance = ta >= targetAreaThreshold;

//         return isAligned && isAtTargetDistance;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         driveTrain.stopMotors();
//     }
// }






package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class AlignToAprilTag extends Command {
    private final DriveTrain driveTrain;
    private final LimeLight limeLight;
    private final double kP = 0.01; // Proportional control for rotation
    private final double kVy = 0.3; // Proportional control for vertical movement (ySpeed)
    private final double targetAreaThreshold = 3;
    private final double finalThreshold = 2.0;
    private final double alignmentThreshold = 6; // Threshold for tx alignment
    private final double verticalAlignmentThreshold = -0.1; // Threshold for ty vertical alignment
    private static final double YAW_THRESHOLD = 2.0;
    private static final double kP_YAW = 0.01;

    public AlignToAprilTag(DriveTrain driveTrain, LimeLight limeLight) {
        this.driveTrain = driveTrain;
        this.limeLight = limeLight;
        addRequirements(driveTrain, limeLight);
    }

    @Override
    public void initialize() {
    }

    public double getDistance() {
        double tx = LimelightHelpers.getTX("limelight");
        Rotation2d angleToGoal = Rotation2d.fromDegrees(tx);
        double distance = 3.0 / angleToGoal.getTan(); 
        return distance;
    }

    @Override
    public void execute() {
        double tx = limeLight.getTx();
        double ty = limeLight.getTy(); // Vertical offset
        double ta = limeLight.getTa();
        boolean hasTarget = limeLight.hasValidTarget();
        double xSpeed = 0; 
        double ySpeed = 0; // Vertical speed
        double zRotation = 0;

        Rotation2d currentRotation = driveTrain.getRotation();
        double yaw = currentRotation.getDegrees();
        
        double yawCorrection = 0;
        

        // Move forward if the robot is not at the target distance
        if (ta < targetAreaThreshold) {
            xSpeed = Constants.SPEED_MULTIPLIER * (targetAreaThreshold - ta); // Move forward if the target area is too small
        }

        // Apply rotation correction based on tx value

        

        // Apply vertical movement correction based on ty value (adjust kVy as necessary)
        
        if (ta < targetAreaThreshold && tx < 0) {
            ySpeed = kVy * ty; // Adjust the constant kVy as necessary for vertical alignment
        } else if (ta < targetAreaThreshold && tx > 0) {
            ySpeed = -kVy * ty; // Adjust the constant kVy as necessary for vertical alignment
        }

        if (ta > targetAreaThreshold - 1) {
            xSpeed = Constants.SPEED_MULTIPLIER * (targetAreaThreshold - ta);
            if (Math.abs(tx) > 0.01) {  // Only rotate if tx is sufficiently off-center
                zRotation = 0.001 * tx; // Adjust the constant kP as necessary
                yawCorrection = (kP_YAW * yaw)+1;
            }

        }

        // If no valid target is found, stop the motors
        if (!hasTarget) {
            driveTrain.stopMotors();
            return;
        }


        // Apply Cartesian drive to align the robot to the AprilTag
        driveTrain.driveCartesian(xSpeed, ySpeed, zRotation);
    }



    @Override
    public boolean isFinished() {
        double tx = limeLight.getTx();
        double ta = limeLight.getTa();

        // Stop rotating and moving vertically when aligned (tx near zero and ty near zero) and when at the target distance
        boolean isAligned = Math.abs(tx) <= 0.1 && Math.abs(driveTrain.getRotation().getDegrees()) < YAW_THRESHOLD && Math.abs(limeLight.getTy()) <= verticalAlignmentThreshold; // Stop rotating and moving vertically when sufficiently aligned
        boolean isAtTargetDistance = ta >= targetAreaThreshold;

        return isAligned && isAtTargetDistance;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stopMotors();
    }
}





// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.LimeLight;
// import edu.wpi.first.math.geometry.Rotation2d;

// public class AlignToAprilTag extends Command {
//     private final DriveTrain driveTrain;
//     private final LimeLight limelight;
//     private static final double kP = 0.02; // Proportional gain for alignment
//     private static final double kP_YAW = 0.01; // Proportional gain for yaw correction
//     private static final double TX_THRESHOLD = 1.0;
//     private static final double YAW_THRESHOLD = 2.0;

//     public AlignToAprilTag(DriveTrain driveTrain, LimeLight limelight) {
//         this.driveTrain = driveTrain;
//         this.limelight = limelight;
//         addRequirements(driveTrain, limelight);
//     }

//     @Override
//     public void initialize() {
//         // Ensure the Limelight is in tracking mode
//         // limelight.getCurrentPipelineIndex(0);
//     }

//     @Override
//     public void execute() {
//         if (!limelight.hasValidTarget()) {
//             driveTrain.stopMotors();
//             return;
//         }

//         double tx = limelight.getTx(); // Horizontal offset from crosshair to target
//         Rotation2d currentRotation = driveTrain.getRotation();
//         double yaw = currentRotation.getDegrees(); // Get current yaw from NavX

//         double rotationSpeed = kP * tx;
//         double yawCorrection = kP_YAW * yaw;

//         // Apply rotation and yaw correction
//         driveTrain.driveCartesian(0, 0, -rotationSpeed - yawCorrection);
//     }

//     @Override
//     public boolean isFinished() {
//         return Math.abs(limelight.getTx()) < TX_THRESHOLD && Math.abs(driveTrain.getRotation().getDegrees()) < YAW_THRESHOLD;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         driveTrain.stopMotors();
//     }
// }
