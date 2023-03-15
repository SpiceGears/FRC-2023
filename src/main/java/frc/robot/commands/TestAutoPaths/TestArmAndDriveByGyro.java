// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TestAutoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.SetArm;
import frc.robot.commands.Drive.DriveBackwardByGyro;
import frc.robot.commands.Drive.DriveForwardByGyro;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestArmAndDriveByGyro extends SequentialCommandGroup {
  /** Creates a new TestDriveByGyro. */
  public TestArmAndDriveByGyro() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    System.out.println("> Starting TestArmAndDriveByGyro AutoPath");
    
    addCommands(
      new SetArm(0),
      new WaitCommand(2),
      new SetArm(0.5),
      new WaitCommand(2),
      new SetArm(1),
      new WaitCommand(2),
      new SetArm(1.5),
      new WaitCommand(2),
      new SetArm(0),
      new WaitCommand(2),
      new DriveForwardByGyro(2, .5),
      new WaitCommand(2),
      new DriveBackwardByGyro(2, .5)
    );

    System.out.println("> Ended TestArmAndDriveByGyro AutoPath");

  }
}
