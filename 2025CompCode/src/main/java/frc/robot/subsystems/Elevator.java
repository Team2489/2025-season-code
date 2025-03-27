package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorReefPositions;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;


public class Elevator extends SubsystemBase {
    private final SparkMax elevatorLeftMotor = new SparkMax(Constants.kElevatorLeft, MotorType.kBrushless);
    private final SparkMax elevatorRightMotor = new SparkMax(Constants.kElevatorRight, MotorType.kBrushless);

    private final RelativeEncoder eLRelativeEncoder;
    private final RelativeEncoder eRRelativeEncoder;

    private final SparkClosedLoopController leftClosedLoopController;
    private final SparkClosedLoopController rightClosedLoopController;

    private final PIDController ePidController;

    private double currentPosition = 0.0;
    private double goalPosition = 0.0;
    private double countRotations = 0.0;
    private double countsPerInch = 42.0; // arbitrary
    private double countsPerRev = 42.0;
    private boolean atBottom = false;

    public Elevator() {
        // Initialize motor controllers
        SparkMaxConfig elevatorLeftConfig = new SparkMaxConfig();
        SparkMaxConfig elevatorRightConfig = new SparkMaxConfig();

        leftClosedLoopController = elevatorLeftMotor.getClosedLoopController();
        rightClosedLoopController = elevatorRightMotor.getClosedLoopController();

        // Corrected encoder assignment
        eLRelativeEncoder = elevatorLeftMotor.getEncoder();
        eRRelativeEncoder = elevatorRightMotor.getEncoder();

        // Initialize limit switch

        // PID Controller setup
        ePidController = new PIDController(
            Constants.ELEVATOR_PID_CONSTANTS[0],  // kP
            Constants.ELEVATOR_PID_CONSTANTS[1],  // kI
            Constants.ELEVATOR_PID_CONSTANTS[2]   // kD
        );
        ePidController.setTolerance(0.5);

        // Configure right motor (leader)
        elevatorRightConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(Constants.ELEVATOR_PID_CONSTANTS[0], Constants.ELEVATOR_PID_CONSTANTS[1], Constants.ELEVATOR_PID_CONSTANTS[2], 0)
            .outputRange(-Constants.kElevatorOutputRange, Constants.kElevatorOutputRange);

        elevatorRightConfig
            .smartCurrentLimit(40)
            .voltageCompensation(12.0)
            .idleMode(IdleMode.kBrake)
            .closedLoopRampRate(Constants.kElevatorCLRate)
            .encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        // Left motor follows right motor
        elevatorLeftConfig.follow(elevatorRightMotor, true);

        // Apply configurations
        elevatorLeftMotor.configure(elevatorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorRightMotor.configure(elevatorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void countRotations() {
        System.out.println("Position in rotations: " + getElevatorPosition());
    }

    public void setElevatorPosition(double posInRot) {
        currentPosition = getElevatorPosition();
        goalPosition = posInRot;
        double error = 0.5 * (goalPosition - currentPosition);

        if (getElevatorPosition() > Constants.kElevatorMaxHeightInRot) {
            stop();
        } else {
            elevatorRightMotor.set(error);
            elevatorLeftMotor.set(-error);
        }
    }

    public void handleLimitSwitch() {
        stop();
        eRRelativeEncoder.setPosition(ElevatorReefPositions.INIT.height);
        eLRelativeEncoder.setPosition(ElevatorReefPositions.INIT.height);
        atBottom = true;
        currentPosition = 0.0;
        goalPosition = 0.0;
        countRotations = 0.0;
    }

    // returns double value in rotations/revolutions
    public double getElevatorPosition() {
        System.out.println("R: " + eRRelativeEncoder.getPosition() + "L: " + eLRelativeEncoder.getPosition());
        return (Math.abs(eRRelativeEncoder.getPosition()) + Math.abs(eLRelativeEncoder.getPosition())) / 2; //  shud we avg left right encoder position values
    }

    public void setMotors(double leftPower, double rightPower) {
        elevatorLeftMotor.set(leftPower);
        elevatorRightMotor.set(rightPower);
    }

    public void stop() {
        elevatorLeftMotor.set(0);
        elevatorRightMotor.set(0);
//        ePidController.reset();
    }
}