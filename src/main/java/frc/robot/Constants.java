// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class Constants {
    public static class Motors {
        public static int kLeftMaster = 1;
        public static int kLeftFollower = 3;
        public static int kRightMaster = 2;
        public static int kRightFollower = 4;

        public static boolean kInvertLeftSide = true;
        public static boolean kInvertRightSide = false;

        public static double kDriveRampRate = 0.25; // 0.5s to full throttle
        public static double kTurnLimiter = 0.75;
        public static double kMaxSpeed = 0.5;
    }

    public static class IO {
        public static XboxController m_driverJoy = new XboxController(0);
        public static XboxController m_operatorJoy = new XboxController(1);
    }

    public static class Encoders {
        // TODO need to configure these!!!
        private static double kGearRatioLow = 15;
        private static double kGearRatioHigh = 9999;

        private static double kWheelCountsPerRotation = 0.15; //6in in meters

        public static int kCountsPerRev = 42;

        // divide by 4 because quadrature ppr (4 pulses per cpr)
        public static double kGearFactorLow = (kGearRatioLow * kWheelCountsPerRotation) / 8;
        public static double kGearFactorHigh = (kGearRatioHigh * kWheelCountsPerRotation) / 8;
    }

    public static class Autonomous {
        public static double kTrackWidthMeters = 0.546;

        // TODO tune these, keep meters everything
        public static final double kS = 1.69993;
        public static final double kV = 2.2741;
        public static final double kA = 1.1605;
        public static double kP = 1;
    }
}
