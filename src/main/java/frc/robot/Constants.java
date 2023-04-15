// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

    public class JOYSTICK {

        public static final double DEADBAND = 0.05;

    }

    public class DRIVETRAIN {

        public static final double MAX_SPEED = 3; // m/s, used for PID setpoint in teleop

        public static final double SPEED_MULTIPLIER = 1.00;
        public static final double SPEED_REDUCTION = 0.6;

        public static final double ROTATION_MULTIPLIER = .9;

        public static final double ENCODER_DISTANCE_PER_ROTATION = .471; //mm (diameter 150mm*pi)
        public static final double ENCODER_TICK_RATE = 2048;
        public static final double ENCODER_MIN_RATE = 10;
        public static final int ENCODER_SAMPLES_TO_AVERAGE = 5; // Can be between 1 and 127

        public static final boolean ENCODER_RIGHT_REVERSE = true;
        public static final boolean ENCODER_LEFT_REVERSE = false;

        public static final double PID_LEFT_KP = .25;
        public static final double PID_LEFT_KI = 0;
        public static final double PID_LEFT_KD = 0;
        public static final double PID_RIGHT_KP = .25;
        public static final double PID_RIGHT_KI = 0;
        public static final double PID_RIGHT_KD = 0; 

        public class TURN_IN_PLACE {
            public static final double ACCEPTED_ERROR = 2.5; //accepted error when turning (in angles)
            public static final double TURN_TO_ANGLE = 0.09; 
            public static final double MINIMAL_MOTOR_OUTPUT = 0.5; //minimal percentage of output in motors
            public static final double MAX_MOTOR_OUTPUT = 0.80; //minimal percentage of output in motors
        }

        public class DRIVE_WITH_GYRO {
            public static final double TURN_kP = 0.07;
            public static final double ACCEPTED_ERROR_IN_METERS = 0.01; 
            public static final double MINIMAL_SPEED = 0.45; //minimal speed when driving in auto
        }

        public class DRIVE_TO_PLATFORM {
            public static final double DRIVE_TO_PLATFORM_TURN_KP = 0.07;
            public static final double ROLL_ANGLE_1st_STATE = 14.0; // do zmiany przy testach
            public static final double FALL_ERROR_TO_2nd_STATE = 4.8; // do zmiany przy testach
            public static final double FALL_TIME = 0;
            public static final double ACCEPTED_ERROR_FOR_LEVEL_IN_DEGREE = 6;
            public static final double DRIVE_TO_BALANCE_MIN_SPEED = 0.5;
            public static final double AUTO_DRIVE_SPEED_CM_PER_SEC_TRESHOLD_SETPOINT = 2 / 100;
        }

    }

    public class GYRO {

        // DRIVEBYGYRO CONSTANTS v
        public static final double MAX_ERROR_1 = 1.5;
        public static final double MAX_ERROR_2 = 10;
        public static final double MOTOR_SLOWDOWN_ON_ERROR2 = .9;
        public static final double ROTATION_SPEED_MULTIPLIER = 1;
        
    }

    public class ARM {

        public static final double KP = 8;
        public static final double kD = 0;
        public static final double kI = 0.6;


        public static final double REDUCTION_CHAIN = 2.3125; // 32t -> 74t = 2.3125:1

        public static final double DEADZONE_LOW = 0;
        public static final double DEADZONE_HIGH = 100;

        public static final double SPEED_MULTIPLIER = 0.3;

        public static final double ENCODER_ANGLES_PER_ROTATION = 360 / REDUCTION_CHAIN; // 1 rotation = 360 degrees
        public static final double ENCODER_TICK_RATE = 2048;
        public static final double ENCODER_MIN_RATE = 10;
        public static final int ENCODER_SAMPLES_TO_AVERAGE = 5; // Can be between 1 and 127
        public static final boolean ENCODER_REVERSE = true;

        public static final double kMaxVelocityRadPerSecond = 80 * Math.PI/180; //20 degrees
        public static final double kMaxAccelerationRadPerSecSquared = 80 * Math.PI/180;
        public static final double kEncoderDistancePerPulse = 2 * Math.PI / REDUCTION_CHAIN / ENCODER_TICK_RATE; // 2rad per full rotation
        public static final double kArmOffsetRads = -0.3; // arm rest position counting from horizontal

        public static final double MAX_VOLTAGE_OUTPUT_UP = 5.5;
        // public static final double MAX_VOLTAGE_OUTPUT_DOWN = 4.0;


        public static final double kSVolts = 0.1;
        public static final double kGVolts = 2.9;
        // 2.5v utrzymuje w horizontal, wyzej 2.5v to za duzo
        public static final double kVVoltSecondPerRad = 2.5;
        public static final double kAVoltSecondSquaredPerRad = 0.05;

        public class POSITION{
            public static final double INTAKE = -0.3;
            public static final double HORIZONTAL = 0.0;
            public static final double SECONDLEVEL = 0.7;
            public static final double VERTICAL = 1.4;


        }
        
    }

    public class INTAKE {
        public static final double SPEED_OUT = .99;
        public static final double SPEED_IN = .4;
    }

}
