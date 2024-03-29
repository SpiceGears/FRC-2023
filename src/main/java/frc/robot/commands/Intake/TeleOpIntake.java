// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class TeleOpIntake extends CommandBase {
  /** Creates a new RollIntakeCommand. */

  private final IntakeSubsystem intakeSubsystem;

  public TeleOpIntake() {

    // Use addRequirements() here to declare subsystem dependencies.
    intakeSubsystem = RobotContainer.intakeSubsystem;
    addRequirements(intakeSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    intakeSubsystem.stopIntake();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(RobotContainer.driver.getRightTriggerAxis() > Constants.JOYSTICK.DEADBAND || RobotContainer.driver.getLeftTriggerAxis() > Constants.JOYSTICK.DEADBAND) {
      
      if((RobotContainer.driver.getLeftTriggerAxis()) > (RobotContainer.driver.getRightTriggerAxis())) {
        intakeSubsystem.setIntake(-Constants.INTAKE.SPEED_IN * RobotContainer.driver.getLeftTriggerAxis());
      } else {
        intakeSubsystem.setIntake(Constants.INTAKE.SPEED_OUT * RobotContainer.driver.getRightTriggerAxis());
      }

    } else {

      intakeSubsystem.stopIntake();

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;

  }

}
