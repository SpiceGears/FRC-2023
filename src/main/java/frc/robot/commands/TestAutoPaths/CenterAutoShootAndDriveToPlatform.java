// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TestAutoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Arm.SetArm;
import frc.robot.commands.Drive.DriveToPlatform;
import frc.robot.commands.Drive.ResetGyro;
import frc.robot.commands.Drive.RotateByAngle;
import frc.robot.commands.Drive.TurnToAngle;
import frc.robot.commands.Intake.ShootCube;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterAutoShootAndDriveToPlatform extends SequentialCommandGroup {
  /** Creates a new CenterAutoShootAndDriveToPlatform. */
  public CenterAutoShootAndDriveToPlatform() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetGyro(),
      new ShootCube(),
      new RotateByAngle(180, 0.7),
      new SetArm(Constants.ARM.POSITION.INTAKE),
      new WaitCommand(2),
      new DriveToPlatform(5, 0.6, 180)
    );
  }
}
