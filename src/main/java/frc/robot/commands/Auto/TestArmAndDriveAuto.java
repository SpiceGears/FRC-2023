// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveBackwardCommand;
import frc.robot.commands.DriveForwardCommand;
import frc.robot.commands.SetArmCommand;


public class TestArmAndDriveAuto extends CommandBase {
  /** Creates a new TestArmAndDriveAuto. */
  public TestArmAndDriveAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    new SequentialCommandGroup(
      new SetArmCommand(RobotContainer.armSubsystem, 0),
      new WaitCommand(2),
      new SetArmCommand(RobotContainer.armSubsystem, 0.5),
      new WaitCommand(2),
      new SetArmCommand(RobotContainer.armSubsystem, 1),
      new WaitCommand(2),
      new SetArmCommand(RobotContainer.armSubsystem, 1.5),
      new WaitCommand(2),
      new SetArmCommand(RobotContainer.armSubsystem, 2),
      new WaitCommand(2),
      new DriveForwardCommand(RobotContainer.driveTrainSubsystem, 0.5, 0.3),
      new WaitCommand(1),
      new DriveBackwardCommand(RobotContainer.driveTrainSubsystem, 0.5, 0.3)
    );

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
