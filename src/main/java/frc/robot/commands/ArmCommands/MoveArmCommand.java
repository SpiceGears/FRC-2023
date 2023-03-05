// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmCommand extends CommandBase {
  /** Creates a new MoveArmCommand. */

  private final ArmSubsystem armSubsystem;

  public MoveArmCommand(ArmSubsystem armSubsystem) {

    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    addRequirements(this.armSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    armSubsystem.stopArm();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed = Constants.ARM.SPEED_MULTIPLIER;
    int pov;

    pov = RobotContainer.driver.getPOV();
    if(pov == 0) {
      armSubsystem.rotateArmBySpeed(speed); // if dpad is up -> rotate arm up

    } else if(pov == 180) {
      armSubsystem.rotateArmBySpeed(-speed); // if dpas is down -> rotate arm down

    } else {
      armSubsystem.rotateArmBySpeed(0); // else stop arm

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
