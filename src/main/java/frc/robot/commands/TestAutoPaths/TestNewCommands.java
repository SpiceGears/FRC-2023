// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TestAutoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.ResetArmAtStart;
import frc.robot.commands.Arm.SetArm;
import frc.robot.commands.Drive.DriveBackwardByGyro;
import frc.robot.commands.Drive.DriveForwardByGyro;
import frc.robot.commands.Drive.RotateByAngle;
import frc.robot.commands.Intake.RollIntakeFor;
import frc.robot.commands.Intake.RollIntakeSeparateSpeedsFor;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestNewCommands extends SequentialCommandGroup {
  /** Creates a new TestNewCommands. */
  public TestNewCommands() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // RESETARM -> SETARM(0) -> INTAKETEST -> GO BACKWARD, ROTATETEST, GO FORWARD

    addCommands(
      new ResetArmAtStart(),
      new SetArm(0),
      new RollIntakeSeparateSpeedsFor(.5, .5, 1),
      new WaitCommand(.5),
      new RollIntakeSeparateSpeedsFor(-.5, .5, 1),
      new WaitCommand(.5),
      new RollIntakeSeparateSpeedsFor(.5, -.5, 1),
      new WaitCommand(.5),
      new RollIntakeSeparateSpeedsFor(.5, .5, 1),
      new DriveBackwardByGyro(.5, .5),
      new WaitCommand(.5),
      new RotateByAngle(90, .5),
      new WaitCommand(.5),
      new RotateByAngle(90, 0),
      new WaitCommand(.5),
      new RotateByAngle(-180, .5),
      new DriveForwardByGyro(.5, .5)
    );
  }
}
