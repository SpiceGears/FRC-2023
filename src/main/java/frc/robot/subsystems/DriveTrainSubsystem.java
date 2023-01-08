// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    differentialDrive.setSafetyEnabled(false);

    leftSlave1.follow(leftMaster);
    leftSlave2.follow(leftMaster);
    rightSlave1.follow(rightMaster);
    rightSlave2.follow(rightMaster);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void arcadeDrive(double xSpeed, double zRotation) {

    if(Math.abs(xSpeed) >= 0.1 || Math.abs(zRotation) >= 0.1) {

      differentialDrive.arcadeDrive(0.8 * xSpeed, 0.5 * zRotation);

    } else {

      differentialDrive.arcadeDrive(0, 0);

    }

  }

  public void tankDrive(double leftSpeed, double rightSpeed) {

    differentialDrive.tankDrive((leftSpeed < 0.1) ? (0) : (leftSpeed), (rightSpeed < 0.05) ? (0) : (rightSpeed));
  
  }

  public void stopDriving() {

    differentialDrive.tankDrive(0, 0);

  }

}
