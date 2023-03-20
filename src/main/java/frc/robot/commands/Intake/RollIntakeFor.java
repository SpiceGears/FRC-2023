// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class RollIntakeFor extends CommandBase {

  private final IntakeSubsystem intakeSubsystem;
  private final double speed;
  private final double seconds;
  private double initTime;

  /** Creates a new RollIntakeForCommand. */
  public RollIntakeFor(double speed, double seconds) {

    // Use addRequirements() here to declare subsystem dependencies.
    intakeSubsystem = RobotContainer.intakeSubsystem;
    this.speed = speed;
    this.seconds = seconds;
    addRequirements(intakeSubsystem);
    
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
    intakeSubsystem.stopIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.setIntake(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("> RollIntakeFor() ended!");
    intakeSubsystem.setIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Timer.getFPGATimestamp() - initTime >= seconds) { // end command when it runs for more than seconds
      return true;
    }
    return false;
  }
}
