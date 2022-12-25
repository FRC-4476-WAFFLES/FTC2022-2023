package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;

public final class Constants {
    public static final class DriveConstants {
        public static final String FRONT_LEFT = "FL";
        public static final String FRONT_RIGHT = "FR";
        public static final String BACK_LEFT = "BL";
        public static final String BACK_RIGHT = "BR";

        public static final double WHEEL_DIAMETER = 101.6; // Wheel diameter in millimeters

        public static final double COUNTS_TO_MM = (WHEEL_DIAMETER * Math.PI) / Motor.GoBILDA.RPM_312.getCPR();
        public static final double METERS_TO_TICKS = (1 / COUNTS_TO_MM) * 1000; // Convert meters per second to ticks per second
        //public static final double MAX_SPEED_M_PER_S = Motor.GoBILDA.RPM_312.getAchievableMaxTicksPerSecond() * COUNTS_TO_MM;
        public static final double MAX_SPEED_M_PER_S = 1.5;

        public static final double MAX_ACCEL_M_PER_S_SQ = 2.0;

        public static final double MAX_ROT_SPEED_RAD_PER_SEC = 3.0;
        public static final double MAX_ROT_ACCEL_RAD_PER_SEC_SQ = 3.0;
    }
}
