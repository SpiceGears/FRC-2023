// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmTo0Power extends CommandBase {
  /** Creates a new MoveArmToAngleCommand. */

  private final ArmSubsystem armSubsystem;

  public SetArmTo0Power(ArmSubsystem armSubsystem) {

    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    addRequirements(this.armSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // disables PID control
    armSubsystem.setArmSetpoint(0, false);
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
