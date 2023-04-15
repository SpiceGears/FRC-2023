// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveToPlatform extends CommandBase {
  /** Creates a new DriveToPlatform. */

  private DriveTrainSubsystem driveTrainSubsystem;
  private final double maxDistance;
  private final double startSpeed;
  private double speed;
  private int state;
  private double turnError;
  private double angleSetpoint;
  private double encoderSetpoint;
  private double maxRollAngle;
  private double startTimeFor3thState;
  private double lastSpeed;
  private double errorSum;
  
  public DriveToPlatform(double maxDistance, double startSpeed, double angleSetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.maxDistance = maxDistance;
    this.startSpeed = startSpeed;
    this.state = 0;
    this.maxRollAngle = 0;
    this.lastSpeed = 0;
    this.angleSetpoint = angleSetpoint;
    driveTrainSubsystem = RobotContainer.driveTrainSubsystem;
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrainSubsystem.tankDrive(0, 0);
    driveTrainSubsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // driveTrainSubsystem.driveStraightAtZeroHeading(speed);
    turnError = angleSetpoint - driveTrainSubsystem.gyro.getAngle();
    turnError = turnError * Constants.DRIVETRAIN.DRIVE_TO_PLATFORM.DRIVE_TO_PLATFORM_TURN_KP;

    double rollAngle = driveTrainSubsystem.gyro.getRoll();
    
    // Nadpisujemy maxRollAngle
    if(Math.abs(maxRollAngle) < Math.abs(rollAngle)){
      maxRollAngle = rollAngle;
    }

    
    
    if(state == 0) {
      // JEDZIE DO PRZODU NA PLATROFME jesli roll angle jest wiekszy niz stala ROLL_ANGLE_FOR_1st_STAGE to zmien na stage 1
      speed = 0.73;
      if(Math.abs(maxRollAngle) > Constants.DRIVETRAIN.DRIVE_TO_PLATFORM.ROLL_ANGLE_1st_STATE) {
        state = 1;
      }
    } else if(state == 1) {
      // ZACZYNA WJEZDZAC NA PLATFORME
      // sprawdzamy czy robot zaczal opadac czyli czy max roll agle jest
      double rollAngleErrorWithMax = Math.abs(Math.abs(maxRollAngle) - Math.abs(rollAngle));
      rollAngleErrorWithMax = Math.abs(rollAngleErrorWithMax);
      if(rollAngleErrorWithMax > Constants.DRIVETRAIN.DRIVE_TO_PLATFORM.FALL_ERROR_TO_2nd_STATE) {
        state = 2;
      }
      
      speed = 0.77;
    } else if(state == 2) {
      // ZACZYNA OPADAC WIEC MUSIMY GO ZATRZYMAC
      speed = 0;
      // Odpalamy timer od kiedy zaczelismy zatrzymywac robota. 
      startTimeFor3thState = Timer.getFPGATimestamp();
      state = 3;
    } else if(state == 3) {

      double deltaTimeInSeconds= Timer.getFPGATimestamp() - startTimeFor3thState;
      if(deltaTimeInSeconds > Constants.DRIVETRAIN.DRIVE_TO_PLATFORM.FALL_TIME){
        state = 4;
      } else {
        speed = 0;
      }
    } else if(state == 4) {
      //su po zatrzmymaniu sprawdzamy czy jest na kącie 0 (+/ ACCEPTED_ERROR_FOR_LEVEL_IN_DEGREE stopnien) stopnii roll 
      //jesli nie to jedziemy albo do przodu albo do tylu (w zaleznosci od roll) i wrzucamy stage 4
      // jezeli rowno to nic jezeli nie rowno to 5 i autobalans
      
      // JEZELI NIE POZIOMO TO 5 i AUTOBALANS
      if(Math.abs(driveTrainSubsystem.gyro.getRoll()) > Constants.DRIVETRAIN.DRIVE_TO_PLATFORM.ACCEPTED_ERROR_FOR_LEVEL_IN_DEGREE) {
        state = 5;
      }
      
      
      
    } else if(state == 5) {
      // sprawdzamy w którą stronę ma jechać w zaleznosci od roll 
      double errorCorrection = .01;
      double robotSpeed = Math.abs(driveTrainSubsystem.leftEncoder.getRate());
      double robotMovingTresholdSetpoint =  0.04;
      double robotBalanceAngle = Math.abs(driveTrainSubsystem.gyro.getRoll());
      
      SmartDashboard.putNumber("AUTOBALANCE/robotBalanceAngle", robotBalanceAngle);
      
      if(robotBalanceAngle > 6) {
        robotMovingTresholdSetpoint = 0.175;
      }
      
      boolean isRobotMoving = robotSpeed > robotMovingTresholdSetpoint;
      boolean is_balanced_local = robotBalanceAngle < Constants.DRIVETRAIN.DRIVE_TO_PLATFORM.ACCEPTED_ERROR_FOR_LEVEL_IN_DEGREE;
      
      
      if( ! is_balanced_local) {
        if(isRobotMoving) {
          errorSum = errorSum - errorCorrection;
        } else {
          errorSum = errorSum + errorCorrection;
        }
      }
      
      if(is_balanced_local){
        speed = 0;
        errorSum = errorSum - errorCorrection;
        if(errorSum < 0) {
          errorSum = 0;
        }

      } else if(driveTrainSubsystem.gyro.getRoll() < 0) {
        // jazda na + z minimalna prędkościa 
        speed = Constants.DRIVETRAIN.DRIVE_TO_PLATFORM.DRIVE_TO_BALANCE_MIN_SPEED + errorSum;
        
      } else {
        // jazda na - z minimalna prędkościa 
        speed = -Constants.DRIVETRAIN.DRIVE_TO_PLATFORM.DRIVE_TO_BALANCE_MIN_SPEED - errorSum;
      } 

      
    }
    boolean is_balanced = Math.abs(driveTrainSubsystem.gyro.getRoll()) > Constants.DRIVETRAIN.DRIVE_TO_PLATFORM.ACCEPTED_ERROR_FOR_LEVEL_IN_DEGREE;
    

    SmartDashboard.putBoolean("AUTOBALANCE/is_balanced", is_balanced);
    SmartDashboard.putNumber("AUTOBALANCE/speed", speed);
    SmartDashboard.putNumber("AUTOBALANCE/state", state);
    SmartDashboard.putNumber("AUTOBALANCE/turnError", turnError);
    SmartDashboard.putNumber("AUTOBALANCE/rollAngle", rollAngle);
    SmartDashboard.putNumber("AUTOBALANCE/maxRollAngle", maxRollAngle);
    SmartDashboard.putNumber("AUTOBALANCE/errorSum", errorSum);
    System.out.println(errorSum);

    // if(speed == 0){
    //   driveTrainSubsystem.pidDrive(0, 0);
    // } else {
      driveTrainSubsystem.arcadeDrive(speed, turnError); // TODO - turn it on ;) 
      lastSpeed = speed;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      driveTrainSubsystem.tankDrive(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(Math.abs(driveTrainSubsystem.getAverageDistance())  > maxDistance) {
      return true;
    }
    return false;
  }
}
