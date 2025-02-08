// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// // import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.drive.MecanumDrive;
// import frc.robot.Constants;


// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


// public class DriveTrain extends SubsystemBase {
//   private final WPI_TalonSRX frontLeftMotor;
//   private final WPI_TalonSRX frontRightMotor;
//   private final WPI_TalonSRX rearLeftMotor;
//   private final WPI_TalonSRX rearRightMotor;

//   MecanumDrive dDrive;
//   XboxController m_stick = new XboxController(0);

//   public DriveTrain() {

//     frontLeftMotor = new WPI_TalonSRX(Constants.kFrontLeftChannel);  
//     frontRightMotor = new WPI_TalonSRX(Constants.kFrontRightChannel);
//     rearLeftMotor = new WPI_TalonSRX(Constants.kRearLeftChannel);
//     rearRightMotor = new WPI_TalonSRX(Constants.kRearRightChannel);

//     frontLeftMotor.setInverted(false);
//     frontRightMotor.setInverted(true);
//     rearLeftMotor.setInverted(false);
//     rearRightMotor.setInverted(true);

//     frontLeftMotor.setNeutralMode(NeutralMode.Brake);
//     frontRightMotor.setNeutralMode(NeutralMode.Brake);
//     rearLeftMotor.setNeutralMode(NeutralMode.Brake);
//     rearRightMotor.setNeutralMode(NeutralMode.Brake);

//     dDrive = new MecanumDrive(frontLeftMotor::set, rearLeftMotor::set, frontRightMotor::set, rearRightMotor::set);

//     dDrive.driveCartesian(0, 0, 0);
//   }

// public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
//   dDrive.driveCartesian(xSpeed, ySpeed, zRotation);
// }

// public void stopMotors() {
//   frontLeftMotor.set(ControlMode.PercentOutput, 0);
//   frontRightMotor.set(ControlMode.PercentOutput, 0);
//   rearLeftMotor.set(ControlMode.PercentOutput, 0);
//   rearRightMotor.set(ControlMode.PercentOutput, 0);
// }
//   /**
//    * Example command factory method.
//    *
//    * @return a command
//    */
//   public Command exampleMethodCommand() {
//     // Inline construction of command goes here.
//     // Subsystem::RunOnce implicitly requires this subsystem.
//     return runOnce(
//         () -> {
//           /* one-time action goes here */
//         });
//   }


//   public boolean exampleCondition() {
//     // Query some boolean state, such as a digital sensor.
//     return false;
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//   }
// }

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;

public class DriveTrain extends SubsystemBase {
    private final WPI_TalonSRX frontLeftMotor;
    private final WPI_TalonSRX frontRightMotor;
    private final WPI_TalonSRX rearLeftMotor;
    private final WPI_TalonSRX rearRightMotor;

    private final Encoder m_frontLeftEncoder;
    private final Encoder m_frontRightEncoder;
    private final Encoder m_backLeftEncoder;
    private final Encoder m_backRightEncoder;

    MecanumDrive dDrive;
    private final AHRS navxGyro = new AHRS(AHRS.NavXComType.kMXP_SPI);  // Connects to the MXP port on the RoboRIO
    private Pose2d pose; // Current robot pose (position + orientation)

    Translation2d m_frontLeftLocation = new Translation2d(-0.295, 0.241);
    Translation2d m_frontRightLocation = new Translation2d(0.295, 0.241);
    Translation2d m_backLeftLocation = new Translation2d(-0.295, -0.241);
    Translation2d m_backRightLocation = new Translation2d(0.295, -0.241);
    
    private final MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

    private final MecanumDriveOdometry m_odometry;

    
    
    public DriveTrain() {
        m_frontLeftEncoder = new Encoder(0, 1);  // Channels 0 and 1
        m_frontRightEncoder = new Encoder(2, 3); // Channels 2 and 3
        m_backLeftEncoder = new Encoder(4, 5);   // Channels 4 and 5
        m_backRightEncoder = new Encoder(6, 7);  // Channels 6 and 7

        
        m_odometry = new MecanumDriveOdometry(
        m_kinematics,
        navxGyro.getRotation2d(),
        new MecanumDriveWheelPositions(
            m_frontLeftEncoder.getDistance(), m_frontRightEncoder.getDistance(),
            m_backLeftEncoder.getDistance(), m_backRightEncoder.getDistance()
        ),
        new Pose2d(0, 0, new Rotation2d())
    );
        frontLeftMotor = new WPI_TalonSRX(Constants.kFrontLeftChannel);  
        frontRightMotor = new WPI_TalonSRX(Constants.kFrontRightChannel);
        rearLeftMotor = new WPI_TalonSRX(Constants.kRearLeftChannel);
        rearRightMotor = new WPI_TalonSRX(Constants.kRearRightChannel);

        // Correctly initialize encoders with proper channel assignments


        frontLeftMotor.setInverted(false);
        frontRightMotor.setInverted(true);
        rearLeftMotor.setInverted(false);
        rearRightMotor.setInverted(true);

        frontLeftMotor.setNeutralMode(NeutralMode.Brake);
        frontRightMotor.setNeutralMode(NeutralMode.Brake);
        rearLeftMotor.setNeutralMode(NeutralMode.Brake);
        rearRightMotor.setNeutralMode(NeutralMode.Brake);

        navxGyro.reset();

        dDrive = new MecanumDrive(frontLeftMotor::set, rearLeftMotor::set, frontRightMotor::set, rearRightMotor::set);
    }

    public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
        dDrive.driveCartesian(xSpeed, ySpeed, zRotation);
    }

    public void stopMotors() {
        frontLeftMotor.set(ControlMode.PercentOutput, 0);
        frontRightMotor.set(ControlMode.PercentOutput, 0);
        rearLeftMotor.set(ControlMode.PercentOutput, 0);
        rearRightMotor.set(ControlMode.PercentOutput, 0);
    }

    public Rotation2d getRotation() {
        // Get the current rotation from the NavX (AHRS)
        return Rotation2d.fromDegrees(navxGyro.getYaw());  // Get Yaw angle from the NavX
    }

    public MecanumDriveWheelSpeeds getCurrentState() {
        return new MecanumDriveWheelSpeeds(
            m_frontLeftEncoder.getRate(),
            m_frontRightEncoder.getRate(),
            m_backLeftEncoder.getRate(),
            m_backRightEncoder.getRate()
        );
    }

    public MecanumDriveWheelPositions getCurrentDistances() {
        return new MecanumDriveWheelPositions(
            m_frontLeftEncoder.getDistance(),
            m_frontRightEncoder.getDistance(),
            m_backLeftEncoder.getDistance(),
            m_backRightEncoder.getDistance()
        );
    }

    public void updateOdometry() {
        // Update the pose
        pose = m_odometry.update(getRotation(), getCurrentDistances());
    }

    public Pose2d getPose() {
        // Return the current pose of the robot
        return pose;
    }

    @Override
    public void periodic() {
        // Update odometry every cycle
        updateOdometry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}