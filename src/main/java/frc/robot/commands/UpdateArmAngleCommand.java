// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class UpdateArmAngleCommand extends CommandBase {
  /** Creates a new MoveArmCommand. */

  private final ArmSubsystem armSubsystem;
  private double angle;

  public UpdateArmAngleCommand(ArmSubsystem armSubsystem, double angle) {

    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.angle = angle;
    addRequirements(this.armSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    armSubsystem.stopArm();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    armSubsystem.setArmAngle(angle);


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
