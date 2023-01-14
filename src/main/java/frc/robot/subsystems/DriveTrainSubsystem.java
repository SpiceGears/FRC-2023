// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
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
