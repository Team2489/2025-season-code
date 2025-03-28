// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.ElevatorReefPositions;
import frc.robot.commands.A_Center;
import frc.robot.commands.A_Taxi;
import frc.robot.commands.AlignReefLeft;
import frc.robot.commands.AlignReefRight;
import frc.robot.commands.MoveLeft;
import frc.robot.commands.MoveRight;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.DriveMecanumCartesian;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeInDelay;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.MecanumDriveTrain;

import java.util.Collection;
import java.util.ConcurrentModificationException;
import java.util.LinkedHashSet;

//import java.nio.Buffer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

//import frc.robot.subsystems.Elevator;
//import frc.robot.commands.DriveMecanumCartesian;
import frc.robot.commands.RunElevatorDown;
import frc.robot.commands.RunElevatorUp;
//import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.ScoreReef;
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
  //DigitalInput limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_PORT);

  SendableChooser<Command> chooser = new SendableChooser<>();

  A_Center a_Center = new A_Center(mDrive);
  A_Taxi a_Taxi = new A_Taxi(mDrive);
  // A_RightL4 a_RightL4 = new A_RightL4(dDrive, coralIntake, elevator, intakeOutPower);
  // A_LeftL4 a_Left4 = new A_LeftL4(dDrive, coralIntake, elevator, intakeOutPower);

  double intakePower = -0.6;
  double adjustPower = -0.8;
  double outtakePower = -0.8;
  double elevatorPower = 0.6;

  public RobotContainer() {
    configureBindings();
    
    mDrive.setDefaultCommand(new DriveMecanumCartesian(mDrive, xboxController));
    chooser.setDefaultOption("Auton Center L4", a_Center);
    chooser.addOption("Auton Taxi", a_Taxi);
    SmartDashboard.putData(chooser);
  }

  // public final class ElevatorEventLoop {
  //   private final Collection<Runnable> eBindings = new LinkedHashSet<>();
  //   private boolean isPolled;

  //   public ElevatorEventLoop() {}

  //   public void bind(Runnable action) {
  //     if (isPolled) {
  //       throw new ConcurrentModificationException("Cannot bind EventLoop while it is running");
  //     }
  //       eBindings.add(action);
  //   }
  // }

  private void configureBindings() {
    // Xbox Controller 1 Bindings
    //new JoystickButton(xboxController, Button.kRightBumper.value).whileTrue(new ScoreReef(elevator, coralIntake, intakePower, limitSwitch)); // driver 1 will position and score, eliminates communication error
    new JoystickButton(xboxController, Button.kRightBumper.value).whileTrue(new IntakeOut(coralIntake, outtakePower));
    new JoystickButton(xboxController, Button.kX.value).whileTrue(new AlignReefLeft(mDrive, limeLight));
    new JoystickButton(xboxController, Button.kB.value).whileTrue(new AlignReefRight(mDrive, limeLight));
    new JoystickButton(xboxController, Button.kY.value).whileTrue(new AlignToAprilTag(mDrive, limeLight));

    // Xbox Controller 2 Bindings
    new JoystickButton(xboxController2, Button.kRightBumper.value).whileTrue(new IntakeIn(coralIntake, intakePower, digitalInput));
   // new JoystickButton(xboxController2, Button.kLeftBumper.value).whileTrue(new IntakeOut(coralIntake, outtakePower));
    new JoystickButton(xboxController2, Button.kY.value).whileTrue(new SetElevatorPositionL4(elevator, ElevatorReefPositions.L4.height));
    new JoystickButton(xboxController2, Button.kX.value).whileTrue(new SetElevatorPositionL3(elevator, ElevatorReefPositions.L3.height));
    new JoystickButton(xboxController2, Button.kB.value).whileTrue(new SetElevatorPositionL2(elevator, ElevatorReefPositions.L2.height));
    new JoystickButton(xboxController2, Button.kA.value).whileTrue(new IntakeInDelay(coralIntake, adjustPower));
    new POVButton(xboxController2, 0).whileTrue(new RunElevatorUp(elevator, elevatorPower)); // up button, going up
    new POVButton(xboxController2, 180).whileTrue(new RunElevatorDown(elevator, elevatorPower, limitSwitch)); // down button, going down
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
