// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TestAutoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.SetArm;
import frc.robot.commands.Drive.DriveBackwardByGyro;
import frc.robot.commands.Drive.DriveForwardByGyro;
import frc.robot.commands.Drive.RotateByAngle;
import frc.robot.commands.Drive.RotateByAngle;
import frc.robot.commands.Intake.ShootCube;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueAllianceFromGrid extends SequentialCommandGroup {
  /** Creates a new TestDriveByGyro. */
  public BlueAllianceFromGrid() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    System.out.println("> Starting BlueAllianceFromGrid AutoPath");
    
    addCommands(
      new SetArm(Constants.ARM.POSITION.VERTICAL),
      new ShootCube(),
      new DriveBackwardByGyro(3.6, 0.8),
      new RotateByAngle(90, 0.8),
      new DriveForwardByGyro(1.2, 0.7)
    );

    System.out.println("> Ended BlueAllianceFromGrid AutoPath");
  }

}