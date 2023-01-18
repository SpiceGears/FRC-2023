// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */

  public VictorSP motor1;
  public VictorSP motor2;

  public IntakeSubsystem() {


    // kontrolery są na:
    // can
    // pwm(5)

    motor1 = new VictorSP(4);
    motor2 = new VictorSP(5); 

    motor1.setInverted(false);
    motor2.setInverted(true);

  }

  /** Rotates intake in desired direction, positive for in, negative for out. */
  public void rollIntake(double speed) {

    motor1.set(speed * 0.3);
    motor2.set(speed * 0.3);

    System.out.println();

    SmartDashboard.putNumber("motor1 speed", motor1.get());
    SmartDashboard.putNumber("motor2 speed", motor2.get());

  }

  /** Sets intake power to 0 */
  public void stopIntake() {

    motor1.set(0);
    motor2.set(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
