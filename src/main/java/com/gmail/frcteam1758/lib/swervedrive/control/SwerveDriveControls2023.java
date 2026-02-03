package com.gmail.frcteam1758.lib.swervedrive.control;

import static com.gmail.frcteam1758.lib.enums.SwerveDriveMode.FIELD_ORIENTED;
import static com.gmail.frcteam1758.lib.enums.SwerveDriveMode.ROBOT_ORIENTED;

import java.util.function.Supplier;

import com.gmail.frcteam1758.lib.enums.SwerveDriveMode;
import com.gmail.frcteam1758.lib.swervedrive.MaxSwerveConstants;
import com.gmail.frcteam1758.lib.swervedrive.MaxSwerveConstants.DriveConstants;
import com.gmail.frcteam1758.lib.util.SwerveUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Joystick;

public class SwerveDriveControls2023 implements SwerveDriveInput {

    protected Joystick m_leftStick, m_rightStick;

    protected SwerveDriveMode m_mode;

    protected double
        m_maxSpeed,
        m_maxSpeedR;

    // variables for stolen code
    @SuppressWarnings("unused") // REV has "m_currentRotation" as a field, and it works, so ...
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;
    
    /** retrieves the robot's current rotation relative to the field,
    * for use in field-relative drive
    */
    protected Supplier<Rotation2d> m_rotationGetter;

    /** gets {@code 0d}, in the case that a gyroscope is not avalible for input */
    public static final Supplier<Rotation2d> GET_ZERO = () -> new Rotation2d();

    /**
     * constructs a {@code SwerveDriveControlls2023} using the USB port numbers of the joysticks.
     * Robot-oriented mode is selected by default and field-oriented mode is disabled do to the
     * {@link Supplier<Rotation2d>} that is used to get the robot's orientation not being provided. Both
     * speed multipliers are set to {@code 0}
     * @param p_leftPort the USB port used by the left stick 
     * @param p_rightPort the USB port used by the right stick
     */
    public SwerveDriveControls2023(int p_leftPort, int p_rightPort) {
        this(p_leftPort, p_rightPort, GET_ZERO, ROBOT_ORIENTED, 1d, 1d);
    }

    public SwerveDriveControls2023(int p_leftPort, int p_rightPort, Supplier<Rotation2d> p_rotGetter) {
        this(p_leftPort, p_rightPort, p_rotGetter, ROBOT_ORIENTED, 1d, 1d);
    }

    public SwerveDriveControls2023(int p_leftPort, int p_rightPort,
        Supplier<Rotation2d> p_rotGetter, SwerveDriveMode p_mode
    ) {
        this(p_leftPort, p_rightPort, p_rotGetter, p_mode, 1d, 1d);
    }

    public SwerveDriveControls2023(int p_leftPort, int p_rightPort, Supplier<Rotation2d> p_rotGetter,
        SwerveDriveMode p_mode, double p_spd, double p_spdR
    ) {
        m_leftStick  = new Joystick(p_leftPort );
        m_rightStick = new Joystick(p_rightPort);

        m_mode = p_mode;

        m_rotationGetter = p_rotGetter;

        m_maxSpeed = p_spd;
        m_maxSpeedR = p_spdR;
    }

    /**
     * permanently borrowed REV template code to limit slew rate
     * @param p_speeds chassisspeeds to limit
     */
    private void stolenRateLimitingCode(ChassisSpeeds p_speeds) {

      double xSpeed = p_speeds.vxMetersPerSecond, ySpeed = p_speeds.vyMetersPerSecond, rot = p_speeds.omegaRadiansPerSecond;
        // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      p_speeds.vxMetersPerSecond = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      p_speeds.vyMetersPerSecond = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      p_speeds.omegaRadiansPerSecond = m_rotLimiter.calculate(rot);

      m_currentRotation = p_speeds.omegaRadiansPerSecond;
    }

    private double addDeadband(double x) {
        return -MathUtil.applyDeadband(x, MaxSwerveConstants.OIConstants.kDriveDeadband);
    }

    @Override
    public SwerveDriveState getCommandedState() {

        if (m_rightStick.getTrigger()) {
            return SwerveDriveState.LOCKED;
        }
        
        ChassisSpeeds l_returnMe = new ChassisSpeeds(
            addDeadband(m_leftStick.getY()),
            addDeadband(m_leftStick.getX()),
            addDeadband(m_rightStick.getX())
        );

        stolenRateLimitingCode(l_returnMe);

        l_returnMe.vxMetersPerSecond *= m_maxSpeed;
        l_returnMe.vyMetersPerSecond *= m_maxSpeed;
        l_returnMe.omegaRadiansPerSecond *= m_maxSpeedR;

        if (m_mode == FIELD_ORIENTED)
            l_returnMe = ChassisSpeeds.fromFieldRelativeSpeeds(
                l_returnMe, m_rotationGetter.get()
            );
        
        return new SwerveDriveState(l_returnMe);
    }
}
