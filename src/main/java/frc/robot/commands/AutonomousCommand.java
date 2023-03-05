// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutonomousCommand extends CommandBase {
  /** Creates a new Autonomous. */

  private final DriveTrainSubsystem driveTrainSubsystem;
  private double startTime;
  
  public AutonomousCommand(DriveTrainSubsystem driveTrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrainSubsystem = driveTrainSubsystem;
    addRequirements(this.driveTrainSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double time = Timer.getFPGATimestamp();
    if(time-startTime<3){
      driveTrainSubsystem.arcadeDrive(0.5, 0);
      System.out.println("WYKONANE");
      } else {
        driveTrainSubsystem.arcadeDrive(0, 0);
      }
      if(time-startTime>3) {
        // shooter shoot
        System.out.println("STRZELANIE");
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
