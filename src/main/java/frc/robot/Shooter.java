package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter implements Loggable {
    private Solenoid m_intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Solenoids.kIntakeSolenoid);
    private TalonSRX m_intakeMotor = new TalonSRX(Constants.Motors.kIntakeMotor);
    private TalonSRX m_towerMotor = new TalonSRX(Constants.Motors.kTowerMotor);
    private TalonSRX m_shooterMotor = new TalonSRX(Constants.Motors.kShooterMotor);
    
    private static DigitalInput m_breakBeamBottom = new DigitalInput(Constants.Sensors.kBreakBeamDIO);
    private Timer shootTimeout = new Timer();

    private boolean isTimerActive = false;
    private boolean isExtended = false;
    private boolean isCurrentlyShooting = false;

    private BangBangController m_shooterVelocityBB = new BangBangController();
    private XboxController operatorJoy = Constants.IO.m_operatorJoy;

    private Timer m_timer = new Timer();
    private InterpolatingTreeMap<Double, Number> m_lookupTable = new InterpolatingTreeMap<>();

    private PhotonCamera m_gloworm = new PhotonCamera("PhotonCam");

    public Shooter() {
        m_shooterMotor.configFactoryDefault();

        // convert our map into an interpolatingtreemap
        for (double key: Constants.kLookupTable.keySet()) {
            m_lookupTable.put(key, Constants.kLookupTable.get(key));
        }
    }

    // shooter controlling logic that is ran in teleop
    public void shooterTeleop() {
        // extend and begin intaking
        // retract if no longer intaking
        if (operatorJoy.getRightBumper()) {
            m_intakeSolenoid.set(true);
            
            intake(Constants.Motors.kIntakePower);
        } else if (operatorJoy.getLeftBumper()) {
            intake(-Constants.Motors.kIntakePower);
        } else {
            intake(0.0);
            m_intakeSolenoid.set(false);
        }

        // manual tower feed control
        if (!isCurrentlyShooting && Math.abs(operatorJoy.getRightTriggerAxis()) > 0.1) {
            towerFeed(Constants.Motors.kTowerPower);
        } else if (!isCurrentlyShooting && Math.abs(operatorJoy.getRightTriggerAxis()) <= 0.1) {
            towerFeed(0.0);
        }

        // start shooting motor and wait X seconds
        // then activate tower to shoot
        if (operatorJoy.getAButton()) {
            if (!isCurrentlyShooting) {
                isCurrentlyShooting = true;
                m_timer.reset();
                m_timer.start();
            }

            // wait for 1 second and then shoot
            if (m_timer.get() > 1) {
                if (m_gloworm.getLatestResult().hasTargets()) {
                    // assumes camera is facing straight forward
                    // may need to tune pitch for angled camera
                    // all values in meters
                    var distance = PhotonUtils.calculateDistanceToTargetMeters(
                        1.2192, // camera height: 48 inches up where camera is
                        2.56, // target height: 8 foot five inches up
                        0, // camera pitch
                        0  // target pitch
                    );

                    var power = calculateOptimalShootPower(distance);

                    shoot(power);
                } else {
                    shoot(0.9); //fallback shoot value
                }

                towerFeed(Constants.Motors.kTowerPower);
            } else {
                shoot(0.9);
            }
        } else {
            shoot(0.0);
            isCurrentlyShooting = false;
        }
    }

    public void intake(double power) {
        m_intakeMotor.set(ControlMode.PercentOutput, power);
    }

    public void towerFeed(double power) {
        m_towerMotor.set(ControlMode.PercentOutput, power);
    }

    public void shoot(double power) {
        m_shooterMotor.set(ControlMode.PercentOutput, power);
    }

    public double calculateOptimalShootPower(double distance) {
        return m_lookupTable.get(distance);
    }

    @Log.BooleanBox(name = "Target Detected", tabName = "Driver", width = 2, height = 2)
    public boolean isValidTarget() {
        return m_gloworm.getLatestResult().hasTargets();
    }
    // returns if the dio is "active" or not
    public static boolean isBottomCargoLoaded() {
        return m_breakBeamBottom.get();
    }

    // TODO, is 
    public double getShooterRPM() {
        return m_shooterMotor.getSelectedSensorVelocity();
    }
}
