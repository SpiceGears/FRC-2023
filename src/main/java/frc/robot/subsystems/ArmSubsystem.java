// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  // public Encoder leftEncoder;
  // public Encoder rightEncoder;

  public PIDController armPidController;
  // private PIDController leftPIDController;
  // private PIDController rightPIDController;

  public DigitalInput frontLimitSwitch;
  public DigitalInput backLimitSwitch;
  
  private final ArmFeedforward feedforward =
    new ArmFeedforward(
      Constants.ARM.kSVolts, Constants.ARM.kGVolts, Constants.ARM.kVoltSecondPerRad, Constants.ARM.kAVoltSecondSquaredPerRad);


  public ArmSubsystem() {

    leftArmMaster = new VictorSP(PortMap.ARM.LEFT_MASTER_PORT);
    rightArmMaster = new VictorSP(PortMap.ARM.RIGHT_MASTER_PORT);
    leftArmSlave = new VictorSP(PortMap.ARM.LEFT_SLAVE_PORT);
    rightArmSlave = new VictorSP(PortMap.ARM.RIGHT_SLAVE_PORT);
    
    armGroup = new MotorControllerGroup(leftArmMaster, rightArmMaster, leftArmSlave, rightArmSlave);

    armEncoder = new Encoder(PortMap.ARM.ENCODER_PORT_A, PortMap.ARM.ENCODER_PORT_B);
    armEncoder.setDistancePerPulse(Constants.ARM.ENCODER_ANGLES_PER_ROTATION / Constants.ARM.ENCODER_TICK_RATE);
    // armEncoder.setMaxPeriod(Constants.ARM.ENCODER_MIN_RATE);
    armEncoder.setReverseDirection(Constants.ARM.ENCODER_REVERSE);
    armEncoder.setSamplesToAverage(Constants.ARM.ENCODER_SAMPLES_TO_AVERAGE);

    frontLimitSwitch = new DigitalInput(6);
    backLimitSwitch = new DigitalInput(7);


    ProfiledPIDController profiledController = new ProfiledPIDController(
      Constants.ARM.kP,
      Constants.ARM.kI,
      Constants.ARM.kD,
      new TrapezoidProfile.Constraints(
        0.35, // in radians 0.35 == 20deg
        0.35)); // in radians 0.35 == 20deg

  }

  //* Rotate arm to specific angle */
  public void setArmAngle(double angle) {
    double maxVolts = 3; // double protection 0_0
    armGroup.setVoltage(MathUtil.clamp((maxVolts* feedforward.calculate(angle, 0.35, 0.35)), -maxVolts, maxVolts));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    logArm();

  }
  
  public void stopArm() {
    armGroup.set(0);
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
