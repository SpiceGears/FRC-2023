// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

public VictorSP leftArmMaster;
public VictorSP rightArmMaster;
public VictorSP leftArmSlave;
public VictorSP rightArmSlave;

public MotorControllerGroup armGroup;

public Encoder armEncoder;

public PIDController armPIDController;

public DigitalInput backLimitSwitch;
public DigitalInput frontLimitSwitch;

  public ArmSubsystem() {

    leftArmMaster = new VictorSP(PortMap.ARM.LEFT_MASTER_PORT);
    rightArmMaster = new VictorSP(PortMap.ARM.RIGHT_MASTER_PORT);
    leftArmSlave = new VictorSP(PortMap.ARM.LEFT_SLAVE_PORT);
    rightArmSlave = new VictorSP(PortMap.ARM.RIGHT_SLAVE_PORT);

    armGroup = new MotorControllerGroup(leftArmMaster, rightArmMaster, leftArmSlave, rightArmSlave);

    armEncoder = new Encoder(PortMap.ARM.ENCODER_PORT_A, PortMap.ARM.ENCODER_PORT_B);
    armEncoder.setDistancePerPulse(Constants.ARM.ENCODER_ANGLES_PER_ROTATION / Constants.ARM.ENCODER_TICK_RATE);  // 2048 ticks = 360 of arm
    armEncoder.setMaxPeriod(Constants.ARM.ENCODER_MIN_RATE);
    armEncoder.setReverseDirection(Constants.ARM.ENCODER_REVERSE); // TODO: check if arm going up -> armAngle ++
    armEncoder.setSamplesToAverage(Constants.ARM.ENCODER_SAMPLES_TO_AVERAGE);

    armPIDController = new PIDController(Constants.ARM.PID_ARM_KP, 
                                        Constants.ARM.PID_ARM_KI,
                                        Constants.ARM.PID_ARM_KD);


    backLimitSwitch = new DigitalInput(PortMap.ARM.BACK_LIMIT_SWITCH);
    frontLimitSwitch = new DigitalInput(PortMap.ARM.FRONT_LIMIT_SWITCH);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    logArm();

  }

  //* Rotate arm by speed, prevents rotating into deadzone. */
  public void rotateArmBySpeed(double speed) {
    // if(armEncoder.getDistance() <= Constants.ARM.DEADZONE_HIGH && speed > 0) { // lower than max deadzone -> move up
    //   armGroup.set(speed);
    // } if(armEncoder.getDistance() >= Constants.ARM.DEADZONE_LOW && speed < 0) { // higher than min deadzone -> move down
    //   armGroup.set(speed);
    // } else {
    //   armGroup.set(0); // don't move in deadzone direction
    // }

    if(speed>0) {

      if(backLimitSwitch.get()) {
        armGroup.set(0);
      } else {
        armGroup.set(speed);
      }

    } else if(speed<0) {

      if (frontLimitSwitch.get()) {
        armGroup.set(0);
        armEncoder.reset();
      } else {
        armGroup.set(speed);
      }

    } else {
      armGroup.set(0);
    }

    SmartDashboard.putNumber("arm input speed", speed);
  }
  
  //* Rotate arm to specific angle */
  public void setArmSetpoint(double angle, boolean working) {

    if (working) {
      armGroup.set(armPIDController.calculate(armEncoder.getDistance(), angle));
    } else {
      armGroup.set(0);
    }

  }

  public void stopArm() {
    armGroup.set(0);
    SmartDashboard.putNumber("arm input speed", 0);
  }

  /** Return true when is beyond arm's limit deadzones, false when in normal arm position. */
  public boolean isInDeadZone() {
    if(armEncoder.getDistance() <= 0 || armEncoder.getDistance() >= 100) {
      return true;
    } else
      return false;
  }

  public void resetEncoder() {
    armEncoder.reset();
    System.out.println("> Arm encoder reset");
  }

  public void logArm() {
    SmartDashboard.putNumber("arm angle", armEncoder.getDistance());
    SmartDashboard.putNumber("arm speed degrees per second", armEncoder.getRate());
  }


}
