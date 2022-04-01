// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import io.github.oblarg.oblog.annotations.Log;

/** State machine that handles switching from various autonomous states **/
public class AutoStates {
    private static STATE m_currentState = STATE.STOP_ALL;

    private Drivetrain m_drivetrain;
    private Shooter m_shooter;

    public static boolean autonomousInit = false;
    public static boolean autonomousPeriodic = false;

    private RamseteController m_controller = new RamseteController();

    private Trajectory m_trajectory;

    private Timer m_timer;

    public AutoStates(Drivetrain m_drivetrain, Shooter m_shooter) {
        this.m_drivetrain = m_drivetrain;
        this.m_shooter = m_shooter;
    }

    public static enum STATE {
        TRAJ_GO_STRAIGHT,
        TRAJ_GRABBALL_INIT,
        TRAJ_GRABBALL_FOLLOWING,
        TRAJ_GRABBALL_SHOOT,
        TRAJ_GRABBALL_END,
        ROTATE_TO_TARGET,
        ROTATE_TO_TARGET_END,
        SHOOT_TARGET,
        SHOOT_TARGET_END,
        STOP_ALL, GO_STRAIGHT_INIT,
    }

    public static STATE getState() {
        return m_currentState;
    }

    public static String getStateName() {
        return m_currentState.name();
    }

    public static void setState(STATE newState) {
        m_currentState = newState;
    }

    public void runStateMachine() {
        switch (m_currentState) {
            case GO_STRAIGHT_INIT:
                if (autonomousInit) { //this will only be called once
                    m_trajectory = initTrajectory("paths/GoStraight.wpilib.json");
                    m_currentState = STATE.TRAJ_GO_STRAIGHT;
                    autonomousInit = false;
                }
                break;
            case TRAJ_GO_STRAIGHT:
                if (autonomousPeriodic) { // this will be called continuously
                    followTrajectory();
                }

                if (isTrajectoryDone()) { // done following trajectory
                    m_currentState = STATE.STOP_ALL;
                }
                break;
            case ROTATE_TO_TARGET:
                break;
            case ROTATE_TO_TARGET_END:
                break;
            case SHOOT_TARGET:
                break;
            case SHOOT_TARGET_END:
                break;
            case STOP_ALL:
                // THIS STATE SHOULD JUST DO NOTHING!!!
                if (autonomousPeriodic) {
                    m_drivetrain.drive(0, 0);
                }
                break;
            case TRAJ_GRABBALL_END:
                m_currentState = STATE.STOP_ALL;
                break;
            case TRAJ_GRABBALL_SHOOT:
                if (m_timer.get() < 5) {
                    m_shooter.shoot(0.9);

                    if (m_timer.get() > 1) {
                        m_shooter.towerFeed(Constants.Motors.kTowerPower);
                    } else {
                        m_shooter.towerFeed(0.0);
                    }
                } else {
                    m_shooter.shoot(0.0);
                    m_shooter.towerFeed(0.0);
                    m_currentState = STATE.TRAJ_GRABBALL_END;
                }

                m_drivetrain.drive(0.0, 0.0);
            case TRAJ_GRABBALL_FOLLOWING:
                if (autonomousPeriodic) { // this will be called continuously
                    followTrajectory();
                }

                if (isTrajectoryDone()) { // done following 
                    m_timer.reset();
                    m_currentState = STATE.TRAJ_GRABBALL_SHOOT;
                }
                break;
            case TRAJ_GRABBALL_INIT:
                if (autonomousInit) { //this will only be called once
                    m_trajectory = initTrajectory("paths/GoStraight.wpilib.json");
                    m_currentState = STATE.TRAJ_GRABBALL_FOLLOWING;
                    autonomousInit = false;
                }
                break;
            default:
                break;

        }
    }

    private void goStraightInit() {
        m_drivetrain.resetOdometry();
        Drivetrain.resetEncoders();
    }

    private void goStraight() {
        if (!autonomousPeriodic) {
            return; //do nothing, we are not in periodic
        }

        var targetDistance = -3.0; // 3 meters
        if (Math.abs(Drivetrain.getLeftDistanceMeters()) <= targetDistance - 0.2) {
            m_drivetrain.runAutonomousSimple(-3.0);
        } else {
            m_currentState = STATE.STOP_ALL;
        }
    }

    private Trajectory initTrajectory(String trajectoryJson) {
        Trajectory trajectory = null;
        try {
          Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJson);

          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
          m_drivetrain.plotTrajectory(trajectory);

        } catch (IOException e) {
          DriverStation.reportError("Unable to open trajectory", false);
        }

        m_timer = new Timer();
        m_timer.start();

        m_drivetrain.resetOdometry();
        m_drivetrain.setPose(trajectory.getInitialPose());
        return trajectory;
    }

    @Log(name = "Angular Velocity")
    double expectedRotation = 0.0;

    private void followTrajectory() {
        if (m_timer.get() < m_trajectory.getTotalTimeSeconds()) {
            // Get the desired pose from the trajectory.
            var desiredPose = m_trajectory.sample(m_timer.get());
      
            // Get the reference chassis speeds from the Ramsete controller.
            var refChassisSpeeds = m_controller.calculate(m_drivetrain.getPose(), desiredPose);
      
            expectedRotation = refChassisSpeeds.omegaRadiansPerSecond;
      
            // Set the linear and angular speeds.
            m_drivetrain.drive(refChassisSpeeds.vxMetersPerSecond, expectedRotation);
        } else {
            m_drivetrain.drive(0, 0);
        }
    }

    private boolean isTrajectoryDone() {
        return m_timer.get() >= m_trajectory.getTotalTimeSeconds();
    }
}
