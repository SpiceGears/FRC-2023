// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */

  //TODO change this to top/bottom names
  public VictorSP intakeMotor1;
  public VictorSP intakeMotor2;
  public DigitalInput touchSensor1;
  public DigitalInput touchSensor2;

  private final AnalogInput ultrasonic = new AnalogInput(PortMap.INTAKE.ULTRASONIC);
  double ultrasonicRawValue;
  double voltageScaleFactor;
  double voltageCurrent;
  double currentDistanceCentimeters;
  boolean isDetected;
  boolean lastIsDetected;
  double rumbleStartTime;

  public IntakeSubsystem() {

    intakeMotor1 = new VictorSP(PortMap.INTAKE.MOTOR_1);
    intakeMotor2 = new VictorSP(PortMap.INTAKE.MOTOR_2); 

    intakeMotor1.setInverted(true);
    intakeMotor2.setInverted(false);

    isDetected = false;

    touchSensor1 = new DigitalInput(8);
    touchSensor2 = new DigitalInput(9);

  }

  /** Rotates intake in desired direction, positive for in, negative for out. */
  public void setIntake(double speed) {

    intakeMotor1.set(speed);
    intakeMotor2.set(speed);

  }

  /** Rotates intake using separate speed for each motor in intake.  */
  public void setIntakeSeparateSpeeds(double speed1, double speed2) {

    intakeMotor1.set(speed1);
    intakeMotor2.set(speed2);

  }

  /** Sets intake power to 0 */
  public void stopIntake() {

    intakeMotor1.set(0);
    intakeMotor2.set(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    

    ultrasonicLoop();
    logIntake();


  }

  public void ultrasonicLoop() {

    ultrasonicRawValue = ultrasonic.getValue(); // returns 0-4095 for 0V-5V
    voltageCurrent = RobotController.getVoltage5V(); // returns RoboRIO 5V rail voltage
    voltageScaleFactor = 5 / voltageCurrent;
    currentDistanceCentimeters = ultrasonicRawValue * voltageScaleFactor * 0.125; // returns distance in centimeters

    lastIsDetected = isDetected; // set to last loop's value

    // if current distance < range => isDetected = true
    if(currentDistanceCentimeters < 35) {
      isDetected = true;
    } else {
      isDetected = false;
    }

    // watch if detection changed from false to true
    // and schedule rumbleGamepad()
    if (lastIsDetected == false && isDetected == true) {
      rumbleGamepad();
    }

    // if touch sensors in intake are on
    // schedule rumbleGamepad()
    if(touchSensor1.get() == false || touchSensor2.get() == false) {
      rumbleGamepad();
    }

    rumbleGamepadPeriodic(RobotContainer.driver, 0.3);

  }

  public void rumbleGamepad() {
    rumbleStartTime = Timer.getFPGATimestamp();
  }

  // rumble gamepad if rumbleGamepad() ran in last rumbleDuration seconds
  public void rumbleGamepadPeriodic(XboxController gamepad, double rumbleDuration) {
    
    double rumbleStrength = 0.8;
    double currentTime = Timer.getFPGATimestamp();
    double timeDifference = currentTime - rumbleStartTime;
    

    if (timeDifference < rumbleDuration) {
      RobotContainer.driver.setRumble(RumbleType.kBothRumble, rumbleStrength );
    } else {
      RobotContainer.driver.setRumble(RumbleType.kBothRumble, 0);
    }

    SmartDashboard.putNumber("INTAKE/time current", currentTime);
    SmartDashboard.putNumber("INTAKE/time init", rumbleStartTime);
    SmartDashboard.putNumber("INTAKE/time difference", timeDifference);
    SmartDashboard.putBoolean("INTAKE/1", touchSensor1.get());
    SmartDashboard.putBoolean("INTAKE/2", touchSensor2.get());

  }

  /** Logs important values to Smart Dashboard */
  public void logIntake() {

    SmartDashboard.putNumber("INTAKE/motor1 speed", intakeMotor1.get());
    SmartDashboard.putNumber("INTAKE/motor2 speed", intakeMotor2.get());
    SmartDashboard.putNumber("INTAKE/ultrasonic (cm)", currentDistanceCentimeters);
    SmartDashboard.putBoolean("INTAKE/ultrasonic detected", isDetected);

  }

  
}
