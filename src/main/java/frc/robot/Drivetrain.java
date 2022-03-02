// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/** Add your docs here. */
public class Drivetrain implements Loggable {
    private static CANSparkMax m_leftMaster = new CANSparkMax(Constants.Motors.kLeftMaster, MotorType.kBrushless);
    private static CANSparkMax m_leftFollower = new CANSparkMax(Constants.Motors.kLeftFollower, MotorType.kBrushless);
    private static CANSparkMax m_rightMaster = new CANSparkMax(Constants.Motors.kRightMaster, MotorType.kBrushless);
    private static CANSparkMax m_rightFollower = new CANSparkMax(Constants.Motors.kRightFollower, MotorType.kBrushless);

    private static RelativeEncoder m_leftEncoder;
    private static RelativeEncoder m_rightEncoder;

    private DifferentialDrive m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

    private static AHRS m_gyro = new AHRS(SerialPort.Port.kMXP);

    public XboxController driverJoy = Constants.IO.m_driverJoy;
    public XboxController operatorJoy = Constants.IO.m_operatorJoy;

    private Solenoid m_shifter = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    // autonomous functions
    private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    private PIDController m_leftPIDController = new PIDController(Constants.Autonomous.kP, 0, 0);
    private PIDController m_rightPIDController = new PIDController(Constants.Autonomous.kP, 0, 0);

    private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Constants.Autonomous.kTrackWidthMeters);

    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.Autonomous.kS, Constants.Autonomous.kV);

    private boolean isShifted = false;

    @Log(name = "Field View")
    private Field2d m_field = new Field2d();

    public Drivetrain() {
        m_rightMaster.restoreFactoryDefaults();
        m_leftMaster.restoreFactoryDefaults();
        
        m_leftMaster.setIdleMode(IdleMode.kCoast);
        m_rightMaster.setIdleMode(IdleMode.kCoast);

        m_leftFollower.follow(m_leftMaster);
        m_rightFollower.follow(m_rightMaster);

        // limit max speed cuz zooom
        m_drive.setMaxOutput(Constants.Motors.kMaxSpeed);

        // initialize our encoders
        m_leftEncoder = m_leftMaster.getEncoder();
        m_rightEncoder = m_rightMaster.getEncoder();

        // set the encoder factor
        m_leftEncoder.setPositionConversionFactor(Constants.Encoders.kGearFactorLow);
        m_rightEncoder.setPositionConversionFactor(Constants.Encoders.kGearFactorLow);

        m_leftEncoder.setVelocityConversionFactor(Constants.Encoders.kGearFactorLow);
        m_rightEncoder.setVelocityConversionFactor(Constants.Encoders.kGearFactorLow);

        // reset the encoders
        m_leftEncoder.setPosition(0.0);
        m_rightEncoder.setPosition(0.0);

        // reset gyro
        m_gyro.calibrate();
        m_gyro.reset();

        // apply ramp rate to prevent burnout
        m_leftMaster.setOpenLoopRampRate(Constants.Motors.kDriveRampRate);
        m_rightMaster.setOpenLoopRampRate(Constants.Motors.kDriveRampRate);

        m_leftMaster.setClosedLoopRampRate(Constants.Motors.kDriveRampRate);
        m_rightMaster.setClosedLoopRampRate(Constants.Motors.kDriveRampRate);

        // no need to invert follower on spark maxes
        m_leftMaster.setInverted(Constants.Motors.kInvertLeftSide);
        m_rightMaster.setInverted(Constants.Motors.kInvertRightSide);

        // disable the stupid motor safety when you are not feeding the object (aka auton)
        m_drive.setSafetyEnabled(false);
        m_drive.setDeadband(0.20);
    }

    // no need to implement a deadband as it is already implemented in XboxController (default 0.02)
    public void drive() {
        m_drive.arcadeDrive(-driverJoy.getLeftY(), driverJoy.getRightX() * Constants.Motors.kTurnLimiter);
        
        if (driverJoy.getLeftBumper()) {
            m_shifter.toggle();
        }
    }

    // autonomous drive function, this should only be called in autonomous
    public void drive(double xSpeed, double rot) {
        var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
        setSpeeds(wheelSpeeds);
    }

    // set speeds function, should only be called by autonomous drive function
    private void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
        var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
    
        var leftOutput =
            m_leftPIDController.calculate(getLeftVelocityMeters(), speeds.leftMetersPerSecond);
        var rightOutput =
            m_rightPIDController.calculate(getRightVelocityMeters(), speeds.rightMetersPerSecond);

        m_leftMaster.setVoltage((leftOutput + leftFeedforward));
        m_rightMaster.setVoltage((rightOutput + rightFeedforward));
    }
    
    // the sole purpose of this function is to run continuously regardless of robot state IE, autonomous vs teleop
    // this is to ensure accurate updating of odometry and pose
    public void periodic() {
        m_odometry.update(m_gyro.getRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters());
    }

    @Log(name = "Left Distance Meters")
    public static double getLeftDistanceMeters() {
        return m_leftEncoder.getPosition();
    }

    @Log(name = "Right Distance Meters")
    public static double getRightDistanceMeters() {
        return m_rightEncoder.getPosition();
    }

    @Log(name = "Raw Units")
    public static double getRawUnits() {
        return m_leftEncoder.getCountsPerRevolution();
    }

    @Log(name = "Left Velocity Meters")
    public static double getLeftVelocityMeters() {
        return m_leftEncoder.getVelocity();
    }

    @Log(name = "Right Velocity Meters")
    public static double getRightVelocityMeters() {
        return m_rightEncoder.getVelocity();
    }

    @Log(name = "Gyro Yaw")
    public static double getAngle() {
        return m_gyro.getYaw();
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    // helper method to reset encoders
    // useful for trajectory following
    public static void resetEncoders() {
        m_leftEncoder.setPosition(0.0);
        m_rightEncoder.setPosition(0.0);
    }

    public static void resetGyro() {
        m_gyro.reset();
    }

    // reset odometry by resetting encoders and gyro
    public void resetOdometry() {
        resetEncoders();
        resetGyro();
    }

    // function to plot to dashboard
    public void plotTrajectory(Trajectory trajectory) {
        ArrayList<Pose2d> poses = new ArrayList<>();
    
        for (Trajectory.State pose : trajectory.getStates()) {
          poses.add(pose.poseMeters);
        }
    
        m_field.getObject("foo").setPoses(poses);
      }


    double kp = 0.25;

    public void runAutonomousSimple(double target) {
        var speed = (target - getLeftDistanceMeters()) * kp;

        m_leftMaster.set(speed); //1 = 100%, 0.5 = 50%
        m_rightMaster.set(speed);
    }
}
