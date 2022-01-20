// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/** Add your docs here. */
public class Drivetrain implements Loggable {
    private CANSparkMax m_leftMaster = new CANSparkMax(Constants.Motors.kLeftMaster, MotorType.kBrushless);
    private CANSparkMax m_leftFollower = new CANSparkMax(Constants.Motors.kLeftFollower, MotorType.kBrushless);
    private CANSparkMax m_rightMaster = new CANSparkMax(Constants.Motors.kRightMaster, MotorType.kBrushless);
    private CANSparkMax m_rightFollower = new CANSparkMax(Constants.Motors.kRightFollower, MotorType.kBrushless);

    private static RelativeEncoder m_leftEncoder;
    private static RelativeEncoder m_righEncoder;

    private DifferentialDrive m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

    private static AHRS m_gyro = new AHRS(SerialPort.Port.kMXP);

    public XboxController driverJoy = Constants.IO.m_driverJoy;
    public XboxController operatorJoy = Constants.IO.m_operatorJoy;

    private Solenoid m_shifter = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    private boolean isShifted = false;

    public Drivetrain() {
        m_leftFollower.follow(m_leftMaster);
        m_rightFollower.follow(m_rightMaster);
        
        // initialize our encoders
        m_leftEncoder = m_leftMaster.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.Encoders.kCountsPerRev);
        m_righEncoder = m_rightMaster.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.Encoders.kCountsPerRev);

        // set the encoder factor
        m_leftEncoder.setPositionConversionFactor(Constants.Encoders.kGearFactorLow);
        m_righEncoder.setPositionConversionFactor(Constants.Encoders.kGearFactorLow);

        // apply ramp rate to prevent burnout
        m_leftMaster.setOpenLoopRampRate(Constants.Motors.kDriveRampRate);
        m_rightMaster.setOpenLoopRampRate(Constants.Motors.kDriveRampRate);

        // no need to invert follower on spark maxes
        m_rightMaster.setInverted(Constants.Motors.kInvertLeftSide);
        m_rightMaster.setInverted(Constants.Motors.kInvertRightSide);

        // disable the stupid motor safety when you are not feeding the object (aka auton)
        m_drive.setSafetyEnabled(false);
    }

    // no need to implement a deadband as it is already implemented in XboxController (default 0.02)
    public void drive() {
        m_drive.arcadeDrive(-driverJoy.getLeftY(), driverJoy.getRightX());

        // this function only runs once until you need to "re-press" the bumper
        if (driverJoy.getLeftBumperPressed()) {
            m_shifter.toggle();

            // assuming solenoid on = high gear, off = low gear
            // we have to invert this statement because we get() in the same loop as we previously set via toggle()
            // TODO investigate if the above logic is correct
            if (!m_shifter.get()) {
                m_leftEncoder.setPositionConversionFactor(Constants.Encoders.kGearFactorHigh);
            } else {
                m_leftEncoder.setPositionConversionFactor(Constants.Encoders.kGearFactorLow);
            }
        }

    }

    @Log(name = "Left Distance Meters")
    public static double getLeftDistanceMeters() {
        return m_leftEncoder.getPosition();
    }

    @Log(name = "Right Distance Meters")
    public static double getRightDistanceMeters() {
        return m_righEncoder.getPosition();
    }

    @Log(name = "Gyro Yaw")
    public static double getAngle() {
        return m_gyro.getYaw();
    }
}
