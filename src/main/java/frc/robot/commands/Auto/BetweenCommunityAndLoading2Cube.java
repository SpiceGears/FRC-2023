// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Arm.SetArm;
import frc.robot.commands.Drive.DriveBackward;
import frc.robot.commands.Drive.DriveForward;
import frc.robot.commands.Drive.ResetGyro;
import frc.robot.commands.Drive.RotateByAngle;
import frc.robot.commands.Intake.RollIntakeFor;
import frc.robot.commands.Intake.ShootCube;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BetweenCommunityAndLoading2Cube extends SequentialCommandGroup {
  /** Creates a new BetweenCommunityAndLoading2Cube. */
  public BetweenCommunityAndLoading2Cube() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetGyro(),
      new ShootCube(),
      new DriveBackward(Units.inchesToMeters(150), 0.7),
      new WaitCommand(.5),
      new RotateByAngle(180, .7),
      new SetArm(Constants.ARM.POSITION.INTAKE),
      new WaitCommand(.2),
      new DriveForward(Units.inchesToMeters(30), .5),
      new ParallelRaceGroup(new DriveForward(Units.inchesToMeters(50), .5), 
                            new RollIntakeFor(-0.5, 10)),
      new RotateByAngle(180, .7),
      new SetArm(Constants.ARM.POSITION.VERTICAL),
      new DriveForward(Units.inchesToMeters(230), .8),
      new ShootCube()

    );
  }
}
