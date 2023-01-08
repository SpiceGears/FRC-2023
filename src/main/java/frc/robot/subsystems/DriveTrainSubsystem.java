// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;

public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsystem. */

  public WPI_TalonSRX leftMaster;
  public WPI_TalonSRX rightMaster;
  public WPI_TalonSRX leftSlave1;
  public WPI_TalonSRX leftSlave2;
  public WPI_TalonSRX rightSlave1;
  public WPI_TalonSRX rightSlave2;

  public MotorControllerGroup leftDrive;
  public MotorControllerGroup rightDrive;

  public DifferentialDrive differentialDrive;

  public DriveTrainSubsystem() {

    WPI_TalonSRX leftMaster = new WPI_TalonSRX(PortMap.DRIVE.DRIVE_LEFT_MASTER_PORT);
    WPI_TalonSRX rightMaster = new WPI_TalonSRX(PortMap.DRIVE.DRIVE_RIGHT_MASTER_PORT);
    WPI_TalonSRX leftSlave1 = new WPI_TalonSRX(PortMap.DRIVE.DRIVE_LEFT_SLAVE1_PORT);
    WPI_TalonSRX leftSlave2 = new WPI_TalonSRX(PortMap.DRIVE.DRIVE_LEFT_SLAVE2_PORT);
    WPI_TalonSRX rightSlave1 = new WPI_TalonSRX(PortMap.DRIVE.DRIVE_RIGHT_SLAVE1_PORT);
    WPI_TalonSRX rightSlave2 = new WPI_TalonSRX(PortMap.DRIVE.DRIVE_RIGHT_SLAVE2_PORT);

    leftDrive = new MotorControllerGroup(leftMaster, leftSlave1, leftSlave2);
    rightDrive = new MotorControllerGroup(rightMaster, rightSlave1, rightSlave2);

    differentialDrive = new DifferentialDrive(leftDrive, rightDrive);
    // differentialDrive.setSafetyEnabled(false);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    leftSlave1.follow(leftMaster);
    leftSlave2.follow(leftMaster);
    rightSlave1.follow(rightMaster);
    rightSlave2.follow(rightMaster);

    leftSlave1.setInverted(InvertType.FollowMaster);
    leftSlave2.setInverted(InvertType.FollowMaster);
    rightSlave1.setInverted(InvertType.FollowMaster);
    rightSlave2.setInverted(InvertType.FollowMaster);

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }


  /** Controls the robot with simple speed and rotation values.
   * Removes unintentional deadband. */  
  public void arcadeDrive(double xSpeed, double zRotation) {

    if(Math.abs(xSpeed) >= Constants.JOYSTICK.DEADBAND || Math.abs(zRotation) >= Constants.JOYSTICK.DEADBAND) {

      differentialDrive.arcadeDrive(Constants.DRIVETRAIN.SPEED_MULTIPLIER * xSpeed, Constants.DRIVETRAIN.ROTATION_MULTIPLIER * zRotation);

    } else {

      differentialDrive.arcadeDrive(0, 0);

    }

  }

  /** Controls the robot with inputs of left and right side values.
   * Removes unintentional deadband.
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {

    differentialDrive.tankDrive((leftSpeed < Constants.JOYSTICK.DEADBAND) ? (0) : (leftSpeed),
                               (rightSpeed < Constants.JOYSTICK.DEADBAND) ? (0) : (rightSpeed));
  
  }

  public void stopDriving() {

    differentialDrive.tankDrive(0, 0);

  }

}
