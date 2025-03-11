// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ReefScorePositions.ElevatorReefPositions;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.StadiaController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.MecanumDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.DriveMecanumCartesian;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.MecanumPolarCustomized;
import frc.robot.commands.ScoreLevelHeight;

public class RobotContainer {
  XboxController xboxController = new XboxController(Constants.XBOX_CONTROLLER_PORT);
  XboxController xboxController2 = new XboxController(Constants.XBOX_CONTROLLER2_PORT);

  MecanumDrivetrain mDrive = new MecanumDrivetrain();
  CoralIntake coralIntake = new CoralIntake();
  Elevator elevator = new Elevator();
  DigitalInput digitalInput = new DigitalInput(Constants.LINE_BREAKER_PORT);
  DigitalInput limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_PORT);

  SendableChooser<Command> chooser = new SendableChooser<>();

  double intakeOutPower = 0.1;

  public RobotContainer() {
    configureBindings();
    mDrive.setDefaultCommand(new DriveMecanumCartesian(mDrive, -xboxController.getLeftY(), -xboxController.getLeftX(), -xboxController.getRightX()));
   // mDrive.setDefaultCommand(new MecanumPolarCustomized(mDrive, Math.hypot(xboxController.getRawAxis(0), xboxController.getRawAxis(1)),  Math.atan2(xboxController.getRawAxis(1), xboxController.getRawAxis(0)), xboxController.getRawAxis(2)));
  }

  private void configureBindings() {
    new JoystickButton(xboxController2, Button.kRightBumper.value).whileTrue(new IntakeIn(coralIntake, 1, digitalInput));
    new JoystickButton(xboxController2, Button.kLeftBumper.value).whileTrue(new IntakeOut(coralIntake, 1));
    new JoystickButton(xboxController2, Button.kA.value).whileTrue(new ScoreLevelHeight(coralIntake, elevator, limitSwitch, ElevatorReefPositions.L1.height, intakeOutPower));
    new JoystickButton(xboxController2, Button.kX.value).whileTrue(new ScoreLevelHeight(coralIntake, elevator, limitSwitch, ElevatorReefPositions.L2.height, intakeOutPower));
    new JoystickButton(xboxController2, Button.kY.value).whileTrue(new ScoreLevelHeight(coralIntake, elevator, limitSwitch, ElevatorReefPositions.L3.height, intakeOutPower));
    new JoystickButton(xboxController2, Button.kB.value).whileTrue(new ScoreLevelHeight(coralIntake, elevator, limitSwitch, ElevatorReefPositions.L4.height, intakeOutPower));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
