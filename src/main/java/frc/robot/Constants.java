// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class Constants {
    public static class Motors {
        public static int kLeftMaster = 0;
        public static int kLeftFollower = 1;
        public static int kRightMaster = 2;
        public static int kRightFollower = 3;

        public static boolean kInvertLeftSide = false;
        public static boolean kInvertRightSide = true;

        public static double kDriveRampRate = 0.5; // 0.5s to full throttle
    }

    public static class IO {
        public static XboxController m_driverJoy = new XboxController(0);
        public static XboxController m_operatorJoy = new XboxController(1);
    }

    public static class Encoders {
        // TODO need to configure these!!!
        private static double kGearRatioLow = 9999;
        private static double kGearRatioHigh = 9999;

        private static double kWheelDiameter = 0.1524; //6in in meters

        public static int kCountsPerRev = 42;

        public static double kGearFactorLow = kGearRatioLow * kWheelDiameter;
        public static double kGearFactorHigh = kGearRatioHigh * kWheelDiameter;
    }
}
