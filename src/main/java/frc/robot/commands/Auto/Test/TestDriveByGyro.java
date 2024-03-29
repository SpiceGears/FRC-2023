// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drive.DriveBackwardByGyro;
import frc.robot.commands.Drive.DriveForwardByGyro;
import frc.robot.commands.Drive.DriveWithGyro;
import frc.robot.commands.Drive.ResetGyro;
import frc.robot.commands.Drive.RotateByAngle;
import frc.robot.commands.Drive.TurnToAngle;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestDriveByGyro extends SequentialCommandGroup {
  /** Creates a new TestDriveByGyro. */
  public TestDriveByGyro() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    System.out.println("> Starting TestDriveByGyro AutoPath");
    
    addCommands(
      new ResetGyro(),
      new RotateByAngle(-180, 0.6)
      // new DriveWithGyro(0, 0.6, 2.0, 1),
      // new WaitCommand(2),
      // new DriveWithGyro(0, 0.6, -2.0, 1)
    );

    System.out.println("> Ended TestDriveByGyro AutoPath");

  }
}
