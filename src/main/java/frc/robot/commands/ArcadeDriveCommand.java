// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class ArcadeDriveCommand extends CommandBase {
  /** Creates a new ArcadeDriveCommand. */

  private final DriveTrainSubsystem driveTrainSubsystem;

  public ArcadeDriveCommand(DriveTrainSubsystem driveTrainSubsystem) {

    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrainSubsystem = driveTrainSubsystem;
    addRequirements(this.driveTrainSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    driveTrainSubsystem.stopDriving();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    driveTrainSubsystem.arcadeDrive(-RobotContainer.driver.getRawAxis(1), RobotContainer.driver.getRawAxis(4));
    SmartDashboard.putNumber("DRIVEAXIS", -RobotContainer.driver.getRawAxis(1));
    SmartDashboard.putNumber("ROTATEAXIS", RobotContainer.driver.getRawAxis(4));


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
