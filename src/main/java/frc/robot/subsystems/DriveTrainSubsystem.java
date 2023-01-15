// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;

public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsystem. */

  public Spark leftMaster;
  public Spark rightMaster;
  public Spark leftSlave1;
  public Spark rightSlave1;

  public MotorControllerGroup leftDrive;
  public MotorControllerGroup rightDrive;

  public DifferentialDrive differentialDrive;

  public Encoder leftEncoder;
  public Encoder rightEncoder;

  private PIDController leftPIDController;
  private PIDController rightPIDController;

  public DriveTrainSubsystem() {

    Spark leftMaster = new Spark(PortMap.DRIVE.DRIVE_LEFT_MASTER_PORT);
    Spark rightMaster = new Spark(PortMap.DRIVE.DRIVE_RIGHT_MASTER_PORT);
    Spark leftSlave1 = new Spark(PortMap.DRIVE.DRIVE_LEFT_SLAVE1_PORT);
    Spark rightSlave1 = new Spark(PortMap.DRIVE.DRIVE_RIGHT_SLAVE1_PORT);

    leftDrive = new MotorControllerGroup(leftMaster, leftSlave1);
    rightDrive = new MotorControllerGroup(rightMaster, rightSlave1);

    differentialDrive = new DifferentialDrive(leftDrive, rightDrive);
    // differentialDrive.setSafetyEnabled(false);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    leftSlave1.setInverted(false);
    rightSlave1.setInverted(true);

    leftEncoder = new Encoder(PortMap.DRIVE.LEFT_ENCODER_PORT_A, PortMap.DRIVE.LEFT_ENCODER_PORT_B);
    leftEncoder.setDistancePerPulse(
            Constants.DRIVETRAIN.DISTANCE_PER_ROTATION / Constants.DRIVETRAIN.ENCODER_TICK_RATE);
    leftEncoder.setMaxPeriod(Constants.DRIVETRAIN.ENCODER_MIN_RATE);
    leftEncoder.setReverseDirection(Constants.DRIVETRAIN.ENCODER_LEFT_REVERSE);
    leftEncoder.setSamplesToAverage(Constants.DRIVETRAIN.ENCODER_SAMPLES_TO_AVERAGE);

    rightEncoder = new Encoder(PortMap.DRIVE.RIGHT_ENCODER_PORT_A, PortMap.DRIVE.RIGHT_ENCODER_PORT_B);
    rightEncoder.setDistancePerPulse(
            Constants.DRIVETRAIN.DISTANCE_PER_ROTATION / Constants.DRIVETRAIN.ENCODER_TICK_RATE);
    leftEncoder.setMaxPeriod(Constants.DRIVETRAIN.ENCODER_MIN_RATE);
    leftEncoder.setReverseDirection(Constants.DRIVETRAIN.ENCODER_LEFT_REVERSE);
    leftEncoder.setSamplesToAverage(Constants.DRIVETRAIN.ENCODER_SAMPLES_TO_AVERAGE);

    resetEncoders();

    leftPIDController = new PIDController(Constants.DRIVETRAIN.PID_LEFT_KP, 
                                          Constants.DRIVETRAIN.PID_LEFT_KI,
                                          Constants.DRIVETRAIN.PID_LEFT_KD);
    rightPIDController = new PIDController(Constants.DRIVETRAIN.PID_RIGHT_KP, 
                                          Constants.DRIVETRAIN.PID_RIGHT_KI,
                                          Constants.DRIVETRAIN.PID_RIGHT_KD);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  /** Controls the robot based on encoders reading and pid. 
   * Removes unintentional deadband.
   * */
  public void pidDrive(double speed, double turn) {

    if (Math.abs(speed) >= Constants.JOYSTICK.DEADBAND || Math.abs(turn) >= Constants.JOYSTICK.DEADBAND) {

      double leftSpeed = 0;
      double rightSpeed = 0;

          leftSpeed = speed + turn;
          rightSpeed = speed - turn;
          if(rightSpeed < -1) {rightSpeed = -1;}
          if(leftSpeed > 1) {leftSpeed = 1;}
          if(rightSpeed > 1) {rightSpeed = 1;}
          if(leftSpeed < -1) {leftSpeed = -1;}

      /*
      arcadeDrive(
        (sideSpeedToArcadeDriveSpeed(leftPIDController.calculate(getLeftEncoderMetersPerSecond(), leftSpeed * 3), rightPIDController.calculate(getRightEncoderMetersPerSecond(), rightSpeed * 3))),
        (sideSpeedToArcadeDriveRotation(leftPIDController.calculate(getLeftEncoderMetersPerSecond(), leftSpeed * 3), rightPIDController.calculate(getRightEncoderMetersPerSecond(), rightSpeed * 3))));
      */

      //                                      read speed (m/s)              setpoint (1 * x m/s)
      tankDrive(leftPIDController.calculate(-getLeftMetersPerSecond(), leftSpeed * 2),  // blad = getencoder * setpoint  |||||||         blad*kP
                rightPIDController.calculate(-getRightMetersPerSecond(), rightSpeed * 2));
      SmartDashboard.putNumber("leftspeed", leftSpeed);

    } else {
      stopDriving();
      leftPIDController.setSetpoint(0);
      rightPIDController.setSetpoint(0);
    }

  }

  /*

  public double sideSpeedToArcadeDriveSpeed(double leftSpeed, double rightSpeed) {
    double xSpeed = 0;
    xSpeed = Math.max(leftSpeed, rightSpeed);
    return xSpeed;
  
  }

  public double sideSpeedToArcadeDriveRotation(double leftSpeed, double rightSpeed) {
    double zRotation = 0; 
    zRotation = leftSpeed - rightSpeed;
    return zRotation;
  }

  */


  /** Controls the robot with simple speed and rotation values.
   * Removes unintentional deadband.
   * */  
  public void arcadeDrive(double xSpeed, double zRotation) {

    if(Math.abs(xSpeed) >= Constants.JOYSTICK.DEADBAND || Math.abs(zRotation) >= Constants.JOYSTICK.DEADBAND) {

      differentialDrive.arcadeDrive(Constants.DRIVETRAIN.SPEED_MULTIPLIER * xSpeed, Constants.DRIVETRAIN.ROTATION_MULTIPLIER * zRotation);

    } else {

      differentialDrive.arcadeDrive(0, 0);

    }

  }

  /** Sends raw speeds directly to motors.
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {

    // Bounds speeds in range -1 to 1
    leftSpeed = Math.min(1, Math.max(-1, leftSpeed));
    rightSpeed = Math.min(1, Math.max(-1, rightSpeed));

    differentialDrive.tankDrive(leftSpeed, rightSpeed);

  }

  /** Returns left distance in xxxxxxxxxx */
  public double getLeftDistance(){
    return leftEncoder.getDistance();
  }

  /** Returns right distance in xxxxxxxxxx */
  public double getRightDistance(){
    return rightEncoder.getDistance();
  }

  /** Returns left encoder speed in xxxxxxxxxx */
  public double getLeftMetersPerSecond() {
    return leftEncoder.getRate() * 0.0036;
  }

  /** Returns right encoder speed in xxxxxxxxxx */
  public double getRightMetersPerSecond() {
    return rightEncoder.getRate() * 0.0036;
  }

  /** Stops drive motors */
  public void stopDriving() {
    tankDrive(0, 0);
  }

  /** Resets endoders */
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /** Logs important values to Smart Dashboard */
  public void logSmartDashboard() {

    SmartDashboard.putNumber("LeftSpeed m/s", getLeftMetersPerSecond());
    SmartDashboard.putNumber("RighSpeed m/s", getRightMetersPerSecond());
    SmartDashboard.putNumber("Left Distance", getLeftDistance());
    SmartDashboard.putNumber("Right Distance", getRightDistance());

  }
}
