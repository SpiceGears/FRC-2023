// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveForwardByGyro extends CommandBase {
  /** Creates a new DriveForwardByGyro. */

  private DriveTrainSubsystem driveTrainSubsystem;
  private final double distance;
  private final double speed;
  private double encoderSetpoint;
  
  /** Drive to given distance forward (in meters) with given speed (always speed > 0) using gyro to go straight */
  public DriveForwardByGyro(double distance, double speed) {

    // Use addRequirements() here to declare subsystem dependencies.
    this.distance = distance;
    this.speed = speed;
    driveTrainSubsystem = RobotContainer.driveTrainSubsystem;
    addRequirements(driveTrainSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    driveTrainSubsystem.resetGyro(); // reset gyro to get 0 heading when straight
    encoderSetpoint = driveTrainSubsystem.getLeftDistance() + distance;
    System.out.println("> DriveForwardByGyro( distance: [" + distance + "] , speed: [" + speed + "] ) started!");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    driveTrainSubsystem.driveStraightAtZeroHeading(speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    driveTrainSubsystem.tankDrive(0, 0);
    System.out.println(" > DriveForwardByGyro() ended!");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(driveTrainSubsystem.getLeftDistance() > encoderSetpoint) {
      return true;
    } else {
      return false;
    }

  }


}
