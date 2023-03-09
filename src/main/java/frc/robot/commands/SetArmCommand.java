// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;


public class SetArmCommand extends CommandBase {

  private final ArmSubsystem armSubsystem;
  private final double position;

  /** Creates a new SetArmCommand. */
  public SetArmCommand(ArmSubsystem armSubsystem, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.position = position;
    addRequirements(this.armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.enable();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Move the arm to [position] radians above horizontal when the button is pressed.
    armSubsystem.setGoal(position);
    armSubsystem.enable();
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
