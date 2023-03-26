package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Arm.SetArm;
import frc.robot.commands.Drive.DriveBackwardByGyro;
import frc.robot.commands.Drive.DriveForwardByGyro;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.RobotContainer;
import java.util.Timer;

public class TurnToAngle extends CommandBase  {

    private final DriveTrainSubsystem driveTrainSubsystem;
    private final double speed;
    private final double seconds;
    private double initTime;
  
    /** Creates a new RollIntakeForCommand. */
    public TurnToAngle(double speed, double seconds) {
  
      // Use addRequirements() here to declare subsystem dependencies.
      driveTrainSubsystem = RobotContainer.driveTrainSubsystem;
      this.speed = speed;
      this.seconds = seconds;
      addRequirements(driveTrainSubsystem);
      
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {      
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      System.out.println("> TurnToAngle() ended!");
    
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      
      return false;
    }
  }
  
