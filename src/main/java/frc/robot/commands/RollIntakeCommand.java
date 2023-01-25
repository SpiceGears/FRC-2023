// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class RollIntakeCommand extends CommandBase {
  /** Creates a new RollIntakeCommand. */

  private final IntakeSubsystem intakeSubsystem;

  public RollIntakeCommand(IntakeSubsystem intakeSubsystem) {

    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(this.intakeSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(RobotContainer.driver.getLeftTriggerAxis() > 0.05 || RobotContainer.driver.getRightTriggerAxis() > 0.05) {
      
      if((RobotContainer.driver.getLeftTriggerAxis()) > (RobotContainer.driver.getRightTriggerAxis())) {
        intakeSubsystem.rollIntake(-RobotContainer.driver.getLeftTriggerAxis());
      } else {
        intakeSubsystem.rollIntake(RobotContainer.driver.getRightTriggerAxis());
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
