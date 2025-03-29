// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.A_Center;
import frc.robot.commands.A_Taxi;
import frc.robot.commands.AlignReefLeft;
import frc.robot.commands.AlignReefRight;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.DriveMecanumCartesian;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeInDelay;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.MecanumDriveTrain;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.commands.RunElevatorDown;
import frc.robot.commands.RunElevatorUp;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.SetElevatorPositionL2;
import frc.robot.commands.SetElevatorPositionL3;
import frc.robot.commands.SetElevatorPositionL4;

public class RobotContainer {

  Elevator elevator = new Elevator();
  CoralIntake coralIntake = new CoralIntake();
  DigitalInput digitalInput = new DigitalInput(Constants.LINE_BREAKER_PORT);
  DigitalInput limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_PORT);
  XboxController xboxController = new XboxController(Constants.XBOX_CONTROLLER_PORT);
  XboxController xboxController2 = new XboxController(Constants.XBOX_CONTROLLER2_PORT);
  MecanumDriveTrain mDrive = new MecanumDriveTrain();
  LimeLight limeLight = new LimeLight();

  SendableChooser<Command> chooser = new SendableChooser<>();

  A_Center a_Center = new A_Center(mDrive, elevator, coralIntake, limitSwitch);
  A_Taxi a_Taxi = new A_Taxi(mDrive);

  double intakePower = -0.55;
  double adjustPower = -0.7;
  double outtakePower = -0.7;
  double elevatorPower = 0.75;

  public RobotContainer() {
    configureBindings();
    
    mDrive.setDefaultCommand(new DriveMecanumCartesian(mDrive, xboxController));
    chooser.setDefaultOption("Auton Center L4", a_Center);
    chooser.addOption("Auton Taxi", a_Taxi);
    SmartDashboard.putData(chooser);
  }

  private void configureBindings() {
    // Xbox Controller 1 Bindings
    new JoystickButton(xboxController, Button.kRightBumper.value).whileTrue(new IntakeOut(coralIntake, outtakePower));
    new JoystickButton(xboxController, Button.kX.value).whileTrue(new AlignReefLeft(mDrive, limeLight));
    new JoystickButton(xboxController, Button.kB.value).whileTrue(new AlignReefRight(mDrive, limeLight));
    new JoystickButton(xboxController, Button.kY.value).whileTrue(new AlignToAprilTag(mDrive, limeLight));

    // Xbox Controller 2 Bindings
    new JoystickButton(xboxController2, Button.kRightBumper.value).whileTrue(new IntakeIn(coralIntake, intakePower, digitalInput));
    new JoystickButton(xboxController2, Button.kLeftBumper.value).whileTrue(new IntakeOut(coralIntake, -outtakePower));
    new JoystickButton(xboxController2, Button.kB.value).whileTrue(new SetElevatorPositionL4(elevator, elevatorPower));
    new JoystickButton(xboxController2, Button.kY.value).whileTrue(new SetElevatorPositionL3(elevator, elevatorPower));
    new JoystickButton(xboxController2, Button.kX.value).whileTrue(new SetElevatorPositionL2(elevator, elevatorPower));
    new JoystickButton(xboxController2, Button.kA.value).whileTrue(new IntakeInDelay(coralIntake, adjustPower));
    new POVButton(xboxController2, 0).whileTrue(new RunElevatorUp(elevator, elevatorPower)); // up button, going up
    new POVButton(xboxController2, 180).whileTrue(new RunElevatorDown(elevator, elevatorPower, limitSwitch)); // down button, going down
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
