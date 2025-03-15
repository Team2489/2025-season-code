// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Constants.ElevatorReefPositions;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // need it temporarily for PID tuning

// public class Elevator extends SubsystemBase {
//   SparkMax elevatorLeftMotor = new SparkMax(Constants.kElevatorLeft, MotorType.kBrushless);
//   SparkMax elevatorRightMotor = new SparkMax(Constants.kElevatorRight, MotorType.kBrushless);

//   RelativeEncoder eLRelativeEncoder, eRRelativeEncoder;
//   SparkMaxConfig elevatorLeftConfig, elevatorRightConfig;
//   SparkClosedLoopController leftClosedLoopController, rightClosedLoopControler;

//   double elevator_kP = Constants.ELEVATOR_PID_CONSTANTS[0];
//   double elevator_kI = Constants.ELEVATOR_PID_CONSTANTS[1];
//   double elevator_kD = Constants.ELEVATOR_PID_CONSTANTS[2];

//   PIDController e_PidController;
//   TrapezoidProfile.Constraints profConstraints;
//   TrapezoidProfile.State currentState;
//   TrapezoidProfile.State goalState;
//   TrapezoidProfile profile;

//   boolean atBottom = false;
//   double currentPosition = 0.0;
//   double goalPosition = 0.0;

//   public Elevator() {
//     elevatorLeftConfig = new SparkMaxConfig();
//     elevatorRightConfig = new SparkMaxConfig();

//     leftClosedLoopController = elevatorLeftMotor.getClosedLoopController();
//     rightClosedLoopControler = elevatorRightMotor.getClosedLoopController();

//     eLRelativeEncoder = elevatorLeftMotor.getEncoder();
//     eRRelativeEncoder = elevatorRightMotor.getEncoder();

//   //  limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_PORT);

//     // profConstraints = new TrapezoidProfile.Constraints(Constants.kMaxVelocity, Constants.kMaxAcceleration);
//     // e_PidController = new PIDController(elevator_kP, elevator_kI, elevator_kD);
//     // e_PidController.setTolerance(0.5);

//     elevatorRightConfig
//       .closedLoop
//       .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//       // .pidf(elevator_kP, elevator_kI, elevator_kD, elevator_kD)
//       // .velocityFF(elevator_kD)
//       .outputRange(-Constants.kElevatorOutputRange, Constants.kElevatorOutputRange);

//     elevatorRightConfig
//       .smartCurrentLimit(40)
//       .voltageCompensation(12.0)
//       .idleMode(IdleMode.kBrake)
//       .closedLoopRampRate(Constants.kElevatorCLRate).encoder
//       .positionConversionFactor(1)
//       .velocityConversionFactor(1);
    
//     elevatorLeftConfig.follow(elevatorRightMotor, true);

//     //currentState = new TrapezoidProfile.State(0, 0);
//     //goalState = new TrapezoidProfile.State(0, 0);
//     //profile = new TrapezoidProfile(profConstraints);
    
//     elevatorLeftMotor.configure(elevatorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     elevatorRightMotor.configure(elevatorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//   }

//   public void setElevatorPosition(double position) {
//     currentPosition = eRRelativeEncoder.getPosition();  
//     goalPosition = position;
//     //goalState = new TrapezoidProfile.State(position, 0);
//     double error = goalPosition - currentPosition;

//     double power = error * elevator_kP;
//     elevatorRightMotor.set(power);
//     elevatorLeftMotor.set(-power);
//     atBottom = false;

//     // make sure it doesn't go past max height
//     if (getElevatorPosition() > Constants.kElevatorMaxHeight) {
//       stop();
//     }
    
//     // when limit switch is clicked
//     // if (limitSwitch.get()) {
//     //   handleLimitSwitch();
//     // }
//     // ONLY DO SET POSITION WHEN ELEVATOR POSITION IS RESET
//     // if (atBottom) {
//     //   // double pidOutput = e_PidController.calculate(getElevatorPosition(), currentState.position);
//     //   // double ffValue = calculateFeedForward(currentState);
//     //   // double power = error * elevator_kP;
//     //   // elevatorRightMotor.set(power);
//     //   // elevatorLeftMotor.set(-power);
//     //   // atBottom = false;

//     //   // // make sure it doesn't go past max height
//     //   // if (getElevatorPosition() > Constants.kElevatorMaxHeight) {
//     //   //   stop();
//     //   // }
//     // }
//   }

//   // public void handleLimitSwitch() {
//   //   stop();
//   //   eRRelativeEncoder.setPosition(ElevatorReefPositions.INIT.height);
//   //   eLRelativeEncoder.setPosition(ElevatorReefPositions.INIT.height);
//   //   atBottom = true;
//   //   currentPosition = 0.0;
//   //   goalPosition = 0.0;
//   //   // currentState = new TrapezoidProfile.State(ElevatorReefPositions.INIT.height, 0);
//   //   // goalState = new TrapezoidProfile.State(ElevatorReefPositions.INIT.height, 0);
//   //   //e_PidController.reset();
//   // }

//   public double getElevatorPosition() {
//     return eRRelativeEncoder.getPosition();
//   }

//   public void setMotors(double leftPower, double rightPower) {
//     elevatorLeftMotor.set(leftPower);
//     elevatorRightMotor.set(rightPower);
//   }

//   // // public double calculateFeedForward(TrapezoidProfile.State state) {
//   // //   return Constants.ELEVATOR_FF_CONSTANTS[0] * Math.signum(state.velocity) + Constants.ELEVATOR_FF_CONSTANTS[1] + Constants.ELEVATOR_FF_CONSTANTS[2] * state.velocity;
//   // // }

//   public void stop() {
//     elevatorLeftMotor.set(0);
//     elevatorRightMotor.set(0);
//     // e_PidController.reset();
//   }
// }



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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    public void setElevatorPosition(double position) {
        currentPosition = getElevatorPosition();
        goalPosition = position;
        double error = goalPosition - currentPosition;

        // Calculate PID output
        double power = MathUtil.clamp(ePidController.calculate(currentPosition, goalPosition), -0.5, 0.5);

        elevatorRightMotor.set(power);
        elevatorLeftMotor.set(-power);
        atBottom = false;

        // Prevent exceeding max height
        if (getElevatorPosition() > Constants.kElevatorMaxHeight) {
            stop();
        }
    }

    public void handleLimitSwitch() {
        stop();
        eRRelativeEncoder.setPosition(ElevatorReefPositions.INIT.height);
        eLRelativeEncoder.setPosition(ElevatorReefPositions.INIT.height);
        atBottom = true;
        currentPosition = 0.0;
        goalPosition = 0.0;
    }

    public double getElevatorPosition() {
        System.out.println("R: " + eRRelativeEncoder.getPosition() + "L: " + eLRelativeEncoder.getPosition());
        return eRRelativeEncoder.getPosition();
        
        
    }

    public void setMotors(double leftPower, double rightPower) {
        elevatorLeftMotor.set(leftPower);
        elevatorRightMotor.set(rightPower);
    }

    public void stop() {
        elevatorLeftMotor.set(0);
        elevatorRightMotor.set(0);
        ePidController.reset();
    }
}