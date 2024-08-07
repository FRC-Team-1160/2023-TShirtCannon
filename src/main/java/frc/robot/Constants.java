/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class PortConstants{

        public static final int PITCH_MOTOR = 2;

        public static final int BACK_LEFT = 3;
        public static final int MID_LEFT = 1;
        public static final int FRONT_LEFT = 17;

        public static final int BACK_RIGHT = 10;
        public static final int MID_RIGHT = 5;
        public static final int FRONT_RIGHT = 4;


        //Valves still not controllable
        public static final int VALVE_1 = 0;
        public static final int VALVE_2 = 1; 
        public static final int VALVE_3 = 2;
        public static final int PCM = 4;
    }

    public static final class OIConstants { //LOGITECH
        public static final int mainStickPort = 2;

        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;

        public static final int LB = 5;
        public static final int RB = 6;

        public static final int LX = 0;
        public static final int LY = 1;
        public static final int LT = 2;
        public static final int RT = 3;
        public static final int RX = 4;
        public static final int RY = 5;
    }
}
