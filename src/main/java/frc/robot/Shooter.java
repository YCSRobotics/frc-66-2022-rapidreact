package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter implements Loggable {
    private Solenoid m_intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Solenoids.kIntakeSolenoid);
    private WPI_TalonSRX m_intakeMotor = new WPI_TalonSRX(Constants.Motors.kIntakeMotor);
    private WPI_TalonSRX m_towerMotor = new WPI_TalonSRX(Constants.Motors.kTowerMotor);
    private WPI_TalonSRX m_shooterMotor = new WPI_TalonSRX(Constants.Motors.kShooterMotor);
    
    private static DigitalInput m_breakBeamBottom = new DigitalInput(Constants.Sensors.kBreakBeamDIO);
    private Timer shootTimeout = new Timer();

    private boolean isTimerActive = false;
    private boolean isExtended = false;
    private boolean isCurrentlyShooting = false;

    private BangBangController m_shooterVelocityBB = new BangBangController();
    private XboxController operatorJoy = Constants.IO.m_operatorJoy;

    private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(7, 12);

    private Timer m_timer = new Timer();
    private InterpolatingTreeMap<Double, Number> m_lookupTable = new InterpolatingTreeMap<>();

    private PhotonCamera m_gloworm = new PhotonCamera("gloworm");

    public Shooter() {
        m_shooterMotor.configFactoryDefault();
        m_shooterMotor.configNeutralDeadband(0.001);
        m_shooterMotor.config_kP(0, 0.0, 20);
        m_shooterMotor.config_kI(0, 0.0, 20);
        m_shooterMotor.config_kD(0, 0.0, 20);
        m_shooterMotor.config_kF(0, 1023.0 / 48000.0, 20);
        m_shooterMotor.configVoltageCompSaturation(10);
        m_shooterMotor.enableVoltageCompensation(true);

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

            if (isValidTarget()) {
                var distance = getDistanceToTarget();

                var velocity = calculateOptimalShootVelocity(distance);

                shoot(velocity);
            } else {
                shoot(40000);
            }

            // wait for 1 second and then shoot
            if (m_timer.get() > 1) {
                towerFeed(Constants.Motors.kTowerPower);
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

    public void shoot(double velocity) {
        m_shooterMotor.set(ControlMode.Velocity, velocity);
    }

    public double calculateOptimalShootVelocity(double distance) {
        System.out.println("Optimal Velocity" + m_lookupTable.get(distance));
        return m_lookupTable.get(distance);
    }

    @Log.BooleanBox(name = "Target Detected", tabName = "Driver", width = 2, height = 5)
    public boolean isValidTarget() {
        try {
            return m_gloworm.getLatestResult().hasTargets();
        } catch (NullPointerException ex) {
            return false;
        }
    }

    @Log(name = "Target Distance", tabName = "Driver", width = 3, height = 2)
    public double getDistanceToTarget() {
        // assumes camera is facing straight forward
        // may need to tune pitch for angled camera
        // all values in meters
        var currentTarget = m_gloworm.getLatestResult();
        var distance = -1.0;
    
        if (currentTarget.hasTargets()) {
            distance = PhotonUtils.calculateDistanceToTargetMeters(
                1.219, // camera height: 48 inches up where camera is
                2.565, // target height: 8 foot five inches up
                Units.degreesToRadians(43),
                Units.degreesToRadians(currentTarget.getBestTarget().getPitch()) 
            );
        }
        
        return distance;
    }
    // returns if the dio is "active" or not
    public static boolean isBottomCargoLoaded() {
        return m_breakBeamBottom.get();
    }

    // TODO 
    public double getShooterRPM() {
        return m_shooterMotor.getSelectedSensorVelocity();
    }
}
