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
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private Drivetrain m_drivetrain;

  private RamseteController m_controller = new RamseteController();

  private Trajectory m_trajectory;

  private Timer m_timer;

  @Override
  public void robotInit() {
    m_drivetrain = new Drivetrain(); // necessary to be initialized before logger

    Logger.configureLoggingAndConfig(this, false);

    String trajectoryJson = "paths/SimplePath.wpilib.json";

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJson);
      m_trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      DriverStation.reportError("Unable to open trajectory", false);
    }

    m_drivetrain.plotTrajectory(m_trajectory);
  }

  @Override
  public void robotPeriodic() {
    Logger.updateEntries();
    m_drivetrain.periodic();
  }

  @Override
  public void autonomousInit() {
    m_timer = new Timer();
    m_timer.start();
    m_drivetrain.resetOdometry();

    m_drivetrain.setPose(m_trajectory.getInitialPose());
  }

  @Log (name = "Expected Angle")
  double expectedRotation = 0.0;

  @Override
  public void autonomousPeriodic() {
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

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    m_drivetrain.drive();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
