// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.ResetArmAtStart;
import frc.robot.commands.Arm.SetArm;
import frc.robot.commands.Drive.DriveBackwardByGyro;
import frc.robot.commands.Drive.DriveForwardByGyro;
import frc.robot.commands.Intake.RollIntakeFor;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestStartFromTop extends SequentialCommandGroup {
  /** Creates a new TestStartFromTop. */
  public TestStartFromTop() {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new ResetArmAtStart(),
      new WaitCommand(.5),

      new SetArm(0),
      new WaitCommand(1),

      new SetArm(.8),
      new WaitCommand(1),

      new DriveForwardByGyro(.3, .5),
      new WaitCommand(.5),

      new RollIntakeFor(-.5, 1),
      new WaitCommand(.7),

      new DriveBackwardByGyro(.3, .5),
      new WaitCommand(.5),

      new SetArm(-0.2),
      new WaitCommand(1),

      new DriveBackwardByGyro(1, .6)
      
    );
  }
}
