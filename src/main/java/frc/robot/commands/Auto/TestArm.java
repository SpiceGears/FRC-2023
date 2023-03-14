// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveBackwardCommand;
import frc.robot.commands.DriveForwardCommand;
import frc.robot.commands.SetArmCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestArm extends SequentialCommandGroup {
  /** Creates a new TestArm. */
  public TestArm() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetArmCommand(0),
      new WaitCommand(2),
      new SetArmCommand(0.5),
      new WaitCommand(2),
      new SetArmCommand(1),
      new WaitCommand(2),
      new SetArmCommand(1.5),
      new WaitCommand(2),
      new SetArmCommand(0),
      new WaitCommand(2),
      new DriveForwardCommand(1, 0.5),
      new WaitCommand(1),
      new DriveBackwardCommand(1, 0.5)
    );
  }
}
