// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.RobotContainer;

public class OtherLogs extends SubsystemBase {
  /** Creates a new OtherLogs. */
  
  public static final PowerDistribution pdp = new PowerDistribution();

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("pdpVoltage", pdp.getVoltage());
    SmartDashboard.putNumber("pdpTemperature", pdp.getTemperature());
    SmartDashboard.putNumber("pdpTotalCurrent", pdp.getTotalCurrent());
    SmartDashboard.putNumber("pdpTotalPower", pdp.getTotalPower());
    SmartDashboard.putNumber("pdpTotalEnergy", pdp.getTotalEnergy());

    SmartDashboard.putNumber("driveLeftMasterCurrent", pdp.getCurrent(PortMap.PDP_CHANNEL.DRIVE_LEFT_MASTER));
    SmartDashboard.putNumber("driveRightMasterCurrent", pdp.getCurrent(PortMap.PDP_CHANNEL.DRIVE_RIGHT_MASTER));
    SmartDashboard.putNumber("driveLeftSlaveCurrent", pdp.getCurrent(PortMap.PDP_CHANNEL.DRIVE_LEFT_SLAVE));
    SmartDashboard.putNumber("driveRightSlaveCurrent", pdp.getCurrent(PortMap.PDP_CHANNEL.DRIVE_RIGHT_SLAVE));

    SmartDashboard.putNumber("driverLeftStickX", -RobotContainer.driver.getRawAxis(1));
    SmartDashboard.putNumber("driverRightStickY", RobotContainer.driver.getRawAxis(4));

  }

  
}
