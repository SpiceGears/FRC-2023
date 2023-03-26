// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class RotateByAngle extends CommandBase {
  /** Creates a new RotateByAngle. */

  private DriveTrainSubsystem driveTrainSubsystem;
  private final double angle;
  private final double speed;


  public RotateByAngle(double angle, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
    this.speed = speed;
    driveTrainSubsystem = RobotContainer.driveTrainSubsystem;
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrainSubsystem.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // angle > getAngle()   => turn right
    if (angle > driveTrainSubsystem.gyro.getAngle()) {
      driveTrainSubsystem.tankDrive(speed, -speed);
    }

    // angle < getAngle()  => turn left
    else {
      driveTrainSubsystem.tankDrive(-speed, speed);
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
    if(Math.abs(driveTrainSubsystem.gyro.getAngle()) > Math.abs(angle)) {
      return true;
    } else {
      return false;
    }
  }
}
