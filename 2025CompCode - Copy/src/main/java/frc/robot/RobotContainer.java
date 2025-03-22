// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.ElevatorReefPositions;
import frc.robot.commands.A_Center;
import frc.robot.commands.A_LeftL4;
import frc.robot.commands.A_RightL4;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.DriveMecanumCartesian;
import frc.robot.commands.IntakeIn;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.MecanumDriveTrain;

//import java.nio.Buffer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

//import frc.robot.subsystems.Elevator;
//import frc.robot.commands.DriveMecanumCartesian;
import frc.robot.commands.ScoreLevelHeight;
import frc.robot.commands.runelevatorsslow;
//import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;

public class RobotContainer {
  
  Elevator elevator = new Elevator();
  CoralIntake coralIntake = new CoralIntake();
  DigitalInput digitalInput = new DigitalInput(Constants.LINE_BREAKER_PORT);
  DigitalInput limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_PORT);
  XboxController xboxController = new XboxController(Constants.XBOX_CONTROLLER_PORT);
  XboxController xboxController2 = new XboxController(Constants.XBOX_CONTROLLER2_PORT);
  MecanumDriveTrain dDrive = new MecanumDriveTrain();
  LimeLight limeLight = new LimeLight();
  //DigitalInput limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_PORT);

  double intakeOutPower = 0.1;
  double elevatorDownPower = 0.2;



  SendableChooser<Command> chooser = new SendableChooser<>();

  A_Center a_Center = new A_Center(dDrive);
  A_RightL4 a_RightL4 = new A_RightL4(dDrive, coralIntake, elevator, intakeOutPower);
  A_LeftL4 a_Left4 = new A_LeftL4(dDrive, coralIntake, elevator, intakeOutPower);

  public RobotContainer() {
    configureBindings();
    
    dDrive.setDefaultCommand(new DriveMecanumCartesian(dDrive, xboxController));
    chooser.setDefaultOption("Auton Center", a_Center);
    SmartDashboard.putData(chooser);
  }

  private void configureBindings() {
    new JoystickButton(xboxController2, Button.kRightBumper.value).whileTrue(new IntakeIn( coralIntake, -0.2));
    new JoystickButton(xboxController2, Button.kLeftBumper.value).whileTrue(new IntakeOut( coralIntake, 0.2));
  //  new JoystickButton(xboxController2, Button.kY.value).whileTrue(new ScoreLevelHeight(coralIntake, elevator, ElevatorReefPositions.L4.height, intakeOutPower));
    new JoystickButton(xboxController2, Button.kX.value).whileTrue(new ScoreLevelHeight(coralIntake, elevator, ElevatorReefPositions.L2.height, intakeOutPower, elevatorDownPower,  limitSwitch));
  //  new JoystickButton(xboxController2, Button.kY.value).whileTrue(new ScoreLevelHeight(coralIntake, elevator, ElevatorReefPositions.L3.height, intakeOutPower));
    //new JoystickButton(xboxController2, Button.kB.value).whileTrue(new ScoreLevelHeight(coralIntake, elevator, ElevatorReefPositions.L4.height, intakeOutPower));
   // new JoystickButton(xboxController, XboxController.Button.kY.value).whileTrue(new AlignToAprilTag(dDrive, limeLight));
    new JoystickButton(xboxController, Button.kY.value).whileTrue(new runelevatorsslow(elevator, 0.5, limitSwitch));
    new JoystickButton(xboxController, Button.kA.value).whileTrue(new runelevatorsslow(elevator,-0.5, limitSwitch));

  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
