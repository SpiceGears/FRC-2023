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

        public static final double DEADBAND = 0.1;

    }

    public class DRIVETRAIN {

        public static final double SPEED_MULTIPLIER = 1;
        public static final double ROTATION_MULTIPLIER = 1;

        public static final double DISTANCE_PER_ROTATION = 471; //mm (diameter 150mm*pi)
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

    }

}
