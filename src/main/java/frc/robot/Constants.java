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

        public static final double SPEED_MULTIPLIER = .7;
        public static final double ROTATION_MULTIPLIER = .7;

        public static final double ENCODER_DISTANCE_PER_ROTATION = .471; //mm (diameter 150mm*pi)
        public static final double ENCODER_TICK_RATE = 2048;
        public static final double ENCODER_MIN_RATE = 10;
        public static final int ENCODER_SAMPLES_TO_AVERAGE = 5; // Can be between 1 and 127

        public static final boolean ENCODER_RIGHT_REVERSE = true;
        public static final boolean ENCODER_LEFT_REVERSE = false;

        public static final double PID_LEFT_KP = .3;
        public static final double PID_LEFT_KI = .2;
        public static final double PID_LEFT_KD = 0;
        public static final double PID_RIGHT_KP = .3;
        public static final double PID_RIGHT_KI = .2;
        public static final double PID_RIGHT_KD = 0;  

    }

    public class ARM {

        public static final double DEADZONE_LOW = 0;
        public static final double DEADZONE_HIGH = 90;

        public static final double SPEED_MULTIPLIER = 0.2;

        public static final double ENCODER_DISTANCE_PER_ROTATION = 360; // 1 rotation = 360 degrees
        public static final double ENCODER_TICK_RATE = 2048;
        public static final double ENCODER_MIN_RATE = 10;
        public static final int ENCODER_SAMPLES_TO_AVERAGE = 5; // Can be between 1 and 127
        public static final boolean ENCODER_REVERSE = true;

        public static final double REDUCTION_CHAIN = 2.3125; // 32t:74t        2.3125:1
        
    }

    public class INTAKE {
        public static final double SPEED_MULTIPLIER = 0.5;
    }

}
