// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.Constants.DRIVETRAIN;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;;

public class DriveWithGyro extends CommandBase {
  //creates new DriveWithGyro class

  private DriveTrainSubsystem driveTrainSubsystem;
  private final double goalAngle;
  private double maxSpeed;
  private final double distanceInMeters;
  private double startTimeForDrive;
  private double deltaTime;
  private final double accelerateInTime;

  public DriveWithGyro(double goalAngle, double maxSpeed, double distanceInMeters, double accelerateInTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.accelerateInTime = accelerateInTime;
    this.goalAngle = goalAngle;
    this.maxSpeed = maxSpeed;
    this.distanceInMeters = distanceInMeters;
    driveTrainSubsystem = RobotContainer.driveTrainSubsystem;
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // reset encoders
    driveTrainSubsystem.resetEncoders();
    startTimeForDrive = Timer.getFPGATimestamp();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // calculate the angle by which the robot is going to turn
    double turnError = goalAngle -  driveTrainSubsystem.gyro.getAngle();
    turnError = turnError * Constants.DRIVETRAIN.DRIVE_WITH_GYRO.TURN_kP;
    
    // slow speeding and slowing (speed for 0.5m, slow down for 1m)
    double distanceLeft = Math.abs(distanceInMeters - driveTrainSubsystem.getAverageDistance());
    double speed = maxSpeed;
    deltaTime = Timer.getFPGATimestamp() - startTimeForDrive;
    if(Math.abs(distanceLeft) <= 1.0) {
      speed = distanceLeft * (maxSpeed + Constants.DRIVETRAIN.DRIVE_WITH_GYRO.MINIMAL_SPEED) ;

    } else if(deltaTime< accelerateInTime) {
      speed = maxSpeed * (deltaTime/accelerateInTime);
    }
    
    if(speed > maxSpeed) {
      speed = maxSpeed;
    }

    if(speed< Constants.DRIVETRAIN.DRIVE_WITH_GYRO.MINIMAL_SPEED) {
      speed = Constants.DRIVETRAIN.DRIVE_WITH_GYRO.MINIMAL_SPEED;
    }

    if(distanceInMeters > 0.0) {
        speed = Math.abs(speed);
        turnError = turnError;
    } else {
        speed = -Math.abs(speed);
        turnError = turnError * 1;
    }

    driveTrainSubsystem.arcadeDrive(speed, turnError);
    SmartDashboard.putNumber("DRIVE/speed", speed);
    SmartDashboard.putNumber("DRIVE/turnError", turnError);
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
    if(Math.abs(error) < Constants.DRIVETRAIN.DRIVE_WITH_GYRO.ACCEPTED_ERROR_IN_METERS) {
      return true;
    } else {
      return false;
    }
  }
}

