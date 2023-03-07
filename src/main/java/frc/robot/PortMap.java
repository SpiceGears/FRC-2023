// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class PortMap {

    // CONNECTIONS
    // https://imgur.com/a/EzDLGpW

    public class JOYSTICK {

    public static final int DRIVER_JOYSTICK = 0;
    public static final int OPERATOR_JOYSTICK = 1;   

    }
    
    public class DRIVE {

        // TODO: MAKE SURE THIS IS CORRECT

        public static final int LEFT_MASTER_PORT = 2; // left front
        public static final int RIGHT_MASTER_PORT = 1; // right front
        public static final int LEFT_SLAVE1_PORT = 3; // left rear
        public static final int RIGHT_SLAVE1_PORT = 0; // right rear

        public static final int LEFT_ENCODER_PORT_A = 0;
        public static final int LEFT_ENCODER_PORT_B = 1;
        public static final int RIGHT_ENCODER_PORT_A = 2;
        public static final int RIGHT_ENCODER_PORT_B = 3;

    }

    public class ARM {

        // TODO: MAKE SURE THIS IS CORRECT

        public static final int LEFT_MASTER_PORT = 4;
        public static final int RIGHT_MASTER_PORT = 5;
        public static final int LEFT_SLAVE_PORT = 6;
        public static final int RIGHT_SLAVE_PORT = 7;

        public static final int ENCODER_PORT_A = 4;	
        public static final int ENCODER_PORT_B = 5;
    }

    public class INTAKE {

        public static final int MOTOR_1_PORT = 8;
        public static final int MOTOR_2_PORT = 9;

    }

    public class PDP_CHANNEL {

        // TODO: MAKE SURE THIS IS CORRECT

        public static final int PDP_CAN = 0;

        public static final int DRIVE_LEFT_MASTER = 15;
        public static final int DRIVE_RIGHT_MASTER = 14;
        public static final int DRIVE_LEFT_SLAVE = 13;
        public static final int DRIVE_RIGHT_SLAVE = 12;
        // public static final int ARM_LEFT_MASTER = 0;
        // public static final int ARM_RIGHT_MASTER = 0;
        // public static final int ARM_LEFT_SLAVE = 0;
        // public static final int ARM_RIGHT_SLAVE = 0;
        // public static final int INTAKE_TOP = 0;
        // public static final int INTAKE_BOTTOM = 0;
        
    }
    
}
