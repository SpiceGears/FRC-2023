// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.SetArm;
import frc.robot.commands.Drive.DriveBackward;
import frc.robot.commands.Drive.DriveForward;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestArmAndDrive extends SequentialCommandGroup {
  /** Creates a new TestArm. */
  public TestArmAndDrive() {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

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
      new DriveForward(1, 0.5),
      new WaitCommand(1),
      new DriveBackward(1, 0.5)
    );

  }

}
