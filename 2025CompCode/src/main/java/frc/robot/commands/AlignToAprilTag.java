package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MecanumDriveTrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class AlignToAprilTag extends Command {
    private final MecanumDriveTrain mDrive;
    private final LimeLight limeLight;
    private final double kVy = 0.3; 
    private final double targetAreaThreshold = Constants.ALIGN_THRESHOLD;
    private final double verticalAlignmentThreshold = -0.1; 
    private static final double YAW_THRESHOLD = 2.0;
    private static final double kP_YAW = 0.01;

    public AlignToAprilTag(MecanumDriveTrain mDrive, LimeLight limeLight) {
        this.mDrive = mDrive;
        this.limeLight = limeLight;
        addRequirements(mDrive, limeLight);
    }

    @Override
    public void initialize() {
        mDrive.driveCartesian(0, 0, 0);
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
        double ty = limeLight.getTy(); 
        double ta = limeLight.getTa();
        boolean hasTarget = limeLight.hasValidTarget();
        double xSpeed = 0; 
        double ySpeed = 0; 
        double zRotation = 0;
        //Rotation2d currentRotation = driveTrain.getRotation();
        //double yaw = currentRotation.getDegrees();
        double yawCorrection = 0;
        
        if (ta < targetAreaThreshold) {
            xSpeed = Constants.SPEED_MULTIPLIER * (targetAreaThreshold - ta + 0.2); 
        }    
        
        if (ta < targetAreaThreshold && tx < 0) {
            ySpeed = kVy * ty; 
        } else if (ta < targetAreaThreshold && tx > 0) {
            ySpeed = -kVy * ty; 
        }

        // if (ta > targetAreaThreshold - 1) {
        //     xSpeed = Constants.SPEED_MULTIPLIER * (targetAreaThreshold - ta);
        //     if (Math.abs(tx) > 0.01) {  
        //         zRotation = 0.001 * tx; 
        //         //yawCorrection = (kP_YAW * yaw)+1;
        //         yawCorrection = (kP_YAW * 0.01)+1;
        //     }

        // }

        if (!hasTarget) {
            mDrive.stopMotors();
            return;
        }

        mDrive.driveCartesian(xSpeed, ySpeed, zRotation);
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.stopMotors();
    }
}