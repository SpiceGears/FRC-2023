// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class OtherLogs {

    public static PowerDistribution pdp = new PowerDistribution();

    static double pdpVoltage;
    static double pdpTemperature;
    static double pdpTotalCurrent; // Total current of all channels (Ampere)
    static double pdpTotalPower; // Power is the bus voltage multiplied by the current (Watt)
    static double pdpTotalEnergy; // Energy is the power summed over time (Joule)

    static double driveLeftMasterCurrent;
    static double driveRightMasterCurrent;
    static double driveLeftSlaveCurrent;
    static double driveRightSlaveCurrent;

    /** Logs values from PDP and others (joysticks) to SmartDashboard */
    public void logOther() {

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
