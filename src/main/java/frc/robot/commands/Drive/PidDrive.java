// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;


public class PidDrive extends CommandBase {
  /** Creates a new ArcadeDriveCommand. */

  double speed;
  double rotation;
  final double speed_reduction = Constants.DRIVETRAIN.SPEED_REDUCTION;

  private final DriveTrainSubsystem driveTrainSubsystem;

  public PidDrive() {

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

    speed = -RobotContainer.driver.getRawAxis(1);
    rotation = RobotContainer.driver.getRawAxis(4);

    // driveTrainSubsystem.pidDrive(-RobotContainer.driver.getRawAxis(1), RobotContainer.driver.getRawAxis(4));

    // REDUCE DRIVE SPEED WHEN LEFTBUMPER PRESSED
    if(RobotContainer.driver.getLeftBumper()) {
      driveTrainSubsystem.pidDrive(speed * speed_reduction ,
                                      rotation * speed_reduction );
    } else {
      driveTrainSubsystem.pidDrive(speed, rotation);
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
