// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.Constants.DRIVETRAIN;

public class RotateByAngle extends CommandBase {
  /** Creates a new RotateByAngle. */

  private DriveTrainSubsystem driveTrainSubsystem;
  private final double goalAngle;
  private final double speed;


  public RotateByAngle(double goalAngle, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.goalAngle = goalAngle;
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
    double error = goalAngle -  driveTrainSubsystem.gyro.getAngle();
    double motorOutput = error * Constants.DRIVETRAIN.TURN_IN_PLACE.TURN_TO_ANGLE;
    if(0.0 < motorOutput && motorOutput < Constants.DRIVETRAIN.TURN_IN_PLACE.MINIMAL_MOTOR_OUTPUT) {
      motorOutput = Constants.DRIVETRAIN.TURN_IN_PLACE.MINIMAL_MOTOR_OUTPUT;
    } else if(0.0 > motorOutput && motorOutput > -Constants.DRIVETRAIN.TURN_IN_PLACE.MINIMAL_MOTOR_OUTPUT) {
      motorOutput = -Constants.DRIVETRAIN.TURN_IN_PLACE.MINIMAL_MOTOR_OUTPUT;
    }

    driveTrainSubsystem.tankDrive(-motorOutput, motorOutput);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double error = goalAngle -  driveTrainSubsystem.gyro.getAngle();
    if(Math.abs(error) < Constants.DRIVETRAIN.TURN_IN_PLACE.ACCEPTED_ERROR) {
      return true;
    } else {
      return false;
    }
  }
}
