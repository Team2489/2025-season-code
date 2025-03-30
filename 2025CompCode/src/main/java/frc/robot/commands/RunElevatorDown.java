// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorReefPositions;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunElevatorDown extends Command {
  Elevator elevator;
  double power;
  DigitalInput limitSwitch;
  public RunElevatorDown(Elevator elevator, double power, DigitalInput limitSwitch) {
    this.elevator = elevator;
    this.power = power;
    this.limitSwitch = limitSwitch;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setMotors(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limitSwitch.get()) {
      double pwr = (elevator.getElevatorPosition() > 10) ? power : 0.25 * power;
      elevator.setMotors(-pwr, pwr);
    } else {
      elevator.stop();
      elevator.handleLimitSwitch();
    }
    // if (((elevator.getElevatorPosition() - 0.5) < elevator.getElevatorPosition())) {
    //   elevator.ledLightsGreen();
    // } else {
    //   elevator.ledLightRed();
    // }
    // int height = (((ElevatorReefPositions.L2.height - 0.5) < elevator.getElevatorPosition()) && ((elevator.getElevatorPosition()) < (ElevatorReefPositions.L2.height))) ? 2 : (((ElevatorReefPositions.L3.height - 0.5) < elevator.getElevatorPosition()) && ((elevator.getElevatorPosition()) < (ElevatorReefPositions.L3.height))) ? 3 : (((ElevatorReefPositions.L3.height - 0.5) < elevator.getElevatorPosition()) && ((elevator.getElevatorPosition()) < (ElevatorReefPositions.L3.height))) ? 4 : 1;
    // switch (height) {
    //   case 2:
    //     if (((ElevatorReefPositions.L2.height - 0.5) <= elevator.getElevatorPosition())) {
    //       elevator.ledLightsGreen();
    //     } else {
    //       elevator.ledLightRed();
    //     }
    //     break;
    //   case 3:
    //     if (((ElevatorReefPositions.L3.height - 0.5) <= elevator.getElevatorPosition())) {
    //       elevator.ledLightsGreen();
    //     } else {
    //       elevator.ledLightRed();
    //     }
    //     break;
    //   case 4:
    //     if (((ElevatorReefPositions.L4.height - 0.5) <= elevator.getElevatorPosition())) {
    //       elevator.ledLightsGreen();
    //     } else {
    //       elevator.ledLightRed();
    //     }
    //     break;
    //   default:
    //  elevator.ledLightsOn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
