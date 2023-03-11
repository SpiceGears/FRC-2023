// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */

  //TODO change this to top/bottom names
  public VictorSP intakeMotor1;
  public VictorSP intakeMotor2;

  public IntakeSubsystem() {

    intakeMotor1 = new VictorSP(PortMap.INTAKE.MOTOR_1_PORT);
    intakeMotor2 = new VictorSP(PortMap.INTAKE.MOTOR_2_PORT); 

    intakeMotor1.setInverted(false);
    intakeMotor2.setInverted(true);

  }

  /** Rotates intake in desired direction, positive for in, negative for out. */
  public void setIntake(double speed) {

    intakeMotor1.set(speed);
    intakeMotor2.set(speed);

  }

  /** Sets intake power to 0 */
  public void stopIntake() {

    intakeMotor1.set(0);
    intakeMotor2.set(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    logIntake();

  }

  /** Logs important values to Smart Dashboard */
  public void logIntake() {

    SmartDashboard.putNumber("INTAKE/motor1 speed", intakeMotor1.get());
    SmartDashboard.putNumber("INTAKE/motor2 speed", intakeMotor2.get());

  }

  
}
