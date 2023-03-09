// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.PortMap;

public class ArmSubsystem extends ProfiledPIDSubsystem {
  /** Creates a new ArmSubsystem. */

  // private final PWMSparkMax m_motor = new PWMSparkMax(ArmConstants.kMotorPort);
  // private final Encoder m_encoder =
  //     new Encoder(ArmConstants.kEncoderPorts[0], ArmConstants.kEncoderPorts[1]);
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
        Constants.ARM.kSVolts, Constants.ARM.kGVolts,
        Constants.ARM.kVVoltSecondPerRad, Constants.ARM.kAVoltSecondSquaredPerRad);
  
  private final VictorSP leftArmMaster = new VictorSP(PortMap.ARM.LEFT_MASTER_PORT);
  private final VictorSP rightArmMaster = new VictorSP(PortMap.ARM.RIGHT_MASTER_PORT);
  private final VictorSP leftArmSlave = new VictorSP(PortMap.ARM.LEFT_SLAVE_PORT);
  private final VictorSP rightArmSlave = new VictorSP(PortMap.ARM.RIGHT_SLAVE_PORT);

  public MotorControllerGroup armGroup = new MotorControllerGroup(leftArmMaster, rightArmMaster, leftArmSlave, rightArmSlave);

  public Encoder armEncoder;

  public DigitalInput frontLimitSwitch = new DigitalInput(6);
  public DigitalInput backLimitSwitch = new DigitalInput(7);

    public ArmSubsystem() {
      super(
        new ProfiledPIDController(
        Constants.ARM.KP,
        0,
        0,
        new TrapezoidProfile.Constraints(
          Constants.ARM.kMaxVelocityRadPerSecond,
          Constants.ARM.kMaxAccelerationRadPerSecSquared)),
          0);
          armEncoder = new Encoder(PortMap.ARM.ENCODER_PORT_A, PortMap.ARM.ENCODER_PORT_B);
          // armEncoder.setDistancePerPulse(Constants.ARM.ENCODER_ANGLES_PER_ROTATION / Constants.ARM.ENCODER_TICK_RATE);
          armEncoder.setMaxPeriod(Constants.ARM.ENCODER_MIN_RATE); //TODO check if it works without it
          armEncoder.setReverseDirection(Constants.ARM.ENCODER_REVERSE);
          armEncoder.setSamplesToAverage(Constants.ARM.ENCODER_SAMPLES_TO_AVERAGE);
          armEncoder.setDistancePerPulse(Constants.ARM.kEncoderDistancePerPulse);

          // Start arm at rest in neutral position
          setGoal(Constants.ARM.kArmOffsetRads);
          
 System.out.println("KONIEC ARMSUBSYSTEM()");
        }
        
  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {

    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);

    // Limit output voltage to this X/12V
    double maxVoltage = 4;

    // Add the feedforward to the PID output to get the motor output
    double finalOutput = MathUtil.clamp(output + feedforward, -maxVoltage, maxVoltage);
    System.out.println("finalOutput: " + finalOutput);
    System.out.println("feedforward: " + feedforward);
    System.out.println("output: " + output);
    SmartDashboard.putNumber("finalOutput", finalOutput);
    
    armGroup.setVoltage(finalOutput);

  }

  @Override
  public double getMeasurement() {
    return armEncoder.getDistance() + Constants.ARM.kArmOffsetRads;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    logArm();

    // if(frontLimitSwitch.get()) {
    //   armEncoder.reset();
    // }

  }

  //* Rotate arm by speed, prevents rotating into deadzone. */
  public void rotateArmBySpeed(double speed) {
    if(armEncoder.getDistance() >= Constants.ARM.DEADZONE_LOW && speed < 0) { 
      armGroup.set(speed);
    } else if(armEncoder.getDistance() <= Constants.ARM.DEADZONE_HIGH && speed > 0) { 
      armGroup.set(speed);
    } else {
      armGroup.set(0); // don't move in deadzone direction
    }

    if(frontLimitSwitch.get()) {
      resetEncoder();
    }
    if(backLimitSwitch.get()) {
      stopArm();
      
    }

    SmartDashboard.putNumber("arm input speed", speed);
  }


  public void stopArm() {
    armGroup.set(0);
  }

  public void resetEncoder() {
    armEncoder.reset();
    System.out.println("> Arm encoder reset [ resetEncoder() ]");
  }

  public void logArm() {
    SmartDashboard.putNumber("armEncoder.getDistance()!!!!!!!", armEncoder.getDistance());
    SmartDashboard.putNumber("getMeasurement()", getMeasurement());
;

  }


}
