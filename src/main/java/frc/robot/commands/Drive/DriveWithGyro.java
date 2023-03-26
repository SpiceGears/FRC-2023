// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.Constants.DRIVETRAIN;

public class DriveWithGyro extends CommandBase {
  /** Creates a new RotateByAngle. */

  private DriveTrainSubsystem driveTrainSubsystem;
  private final double goalAngle;
  private final double speed;
  private final double distanceInMeters;

  public DriveWithGyro(double goalAngle, double speed, double distanceInMeters) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.goalAngle = goalAngle;
    this.speed = speed;
    this.distanceInMeters = distanceInMeters;
    driveTrainSubsystem = RobotContainer.driveTrainSubsystem;
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // driveTrainSubsystem.resetGyro();
    driveTrainSubsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // angle > getAngle()   => turn right
    double turnError = goalAngle -  driveTrainSubsystem.gyro.getAngle();
    turnError = turnError * Constants.DRIVETRAIN.DRIVE_WITH_GYRO.TURN_kP;
    if(distanceInMeters >0) {
        driveTrainSubsystem.arcadeDrive(Math.abs(speed), turnError);
    } else {
        driveTrainSubsystem.arcadeDrive(-Math.abs(speed), -turnError);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double error = Math.abs(distanceInMeters) - Math.abs(driveTrainSubsystem.getAverageDistance());
    if(Math.abs(error) < Constants.DRIVETRAIN.TURN_IN_PLACE.ACCEPTED_ERROR) {
      return true;
    } else {
      return false;
    }
  }
}

