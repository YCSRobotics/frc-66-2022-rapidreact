// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class Constants {
    public static class Motors {
        public static int kLeftMaster = 1;
        public static int kLeftFollower = 2;
        public static int kRightMaster = 3;
        public static int kRightFollower = 4;

        // CTRE devices have a separate CAN ID
        public static int kIntakeMotor = 1;
        public static int kTowerMotor = 0;
        public static int kShooterMotor = 2;

        public static double kIntakePower = -1.0;
        public static double kTowerPower = -0.5;

        public static boolean kInvertLeftSide = true;
        public static boolean kInvertRightSide = false;

        public static double kDriveRampRate = 0.25; // 0.5s to full throttle
        public static double kTurnLimiter = 0.75;
        public static double kMaxSpeed = 0.5;

        public static double kTargetShooterRPM = 300; //TODO, change/tune
        public static double kTargetShooterRPMTolerance = 100; //acceptable error
        public static double kShootTimeout = 5; //time until the shooter stops shooting
    }

    public static class IO {
        public static XboxController m_driverJoy = new XboxController(0);
        public static XboxController m_operatorJoy = new XboxController(1);

        public static double kDeadBand = 0.20;
    }

    public static class Encoders {
        // TODO need to configure these!!!
        public static double kGearRatioLow = 1.0/15.0;
        private static double kGearRatioHigh = 9999;

        public static double kWheelCircumference = 0.152 * Math.PI; //6in in meters

        public static double kGearFactorLow = kGearRatioLow * kWheelCircumference; //0.032
        public static double kGearFactorHigh = 0;
    }

    public static class Autonomous {
        public static double kTrackWidthMeters = 0.546;

        public static final double kS = 0.129;
        public static final double kV = 3.9299;
        public static final double kA = 0.329;
        public static double kP = 4.4595;
    }

    public static class Sensors {
        public static int kBreakBeamDIO = 0;
    }

    public static class Solenoids {
        public static int kGearboxSolenoid = 0;
        public static int kIntakeSolenoid = 1;
    }
}
