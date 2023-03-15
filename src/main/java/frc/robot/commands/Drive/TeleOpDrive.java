// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TeleOpDrive extends CommandBase {
  /** Creates a new ArcadeDriveCommand. */

  private final DriveTrainSubsystem driveTrainSubsystem;

  public TeleOpDrive() {

    // Use addRequirements() here to declare subsystem dependencies.
    driveTrainSubsystem = RobotContainer.driveTrainSubsystem;
    addRequirements(driveTrainSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    driveTrainSubsystem.stopDriving();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    driveTrainSubsystem.pidDrive(-RobotContainer.driver.getRawAxis(1), RobotContainer.driver.getRawAxis(4));

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
