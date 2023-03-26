package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.RollIntakeFor;

public class ShootCube extends SequentialCommandGroup {
    public ShootCube() {
        addCommands(
        new RollIntakeFor(-0.33, 0.2),
        new RollIntakeFor(1, 0.6)
 );  
    }
}
