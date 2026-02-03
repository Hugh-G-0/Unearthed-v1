package com.gmail.frcteam1758.lib.swervedrive.control;

import static com.gmail.frcteam1758.lib.enums.SwerveDriveMode.FIELD_ORIENTED;
import java.util.function.Supplier;

import com.gmail.frcteam1758.lib.enums.SwerveDriveMode;
import com.gmail.frcteam1758.lib.swervedrive.MaxSwerveConstants;
import com.gmail.frcteam1758.lib.swervedrive.MaxSwerveConstants.DriveConstants;
import com.gmail.frcteam1758.lib.util.SwerveUtils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Joystick;

public class SwerveDriveControls2025 implements SwerveDriveInput {

    protected final Joystick m_leftStick, m_rightStick;

    protected double maxRotVel = 1, prevRot, rotAccel = 1;

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

    protected final TrapezoidProfile profile;

    protected final double period;

    protected Rotation2d targetRot = Rotation2d.kZero;

    /** gets {@code 0d}, in the case that a gyroscope is not avalible for input */
    public static final Supplier<Rotation2d> GET_ZERO = () -> new Rotation2d();

    public SwerveDriveControls2025(Joystick p_leftPort, Joystick p_rightPort, Supplier<Rotation2d> p_rotGetter,
        SwerveDriveMode p_mode, double p_spd, double maxRotVel, double rotAccel, double period
    ) {
        m_leftStick  = p_leftPort;
        m_rightStick = p_rightPort;

        m_mode = p_mode;

        m_rotationGetter = p_rotGetter;

        m_maxSpeed = p_spd;
        
        this.maxRotVel = maxRotVel;
        this.rotAccel = rotAccel;

        this.profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxRotVel, rotAccel)
        );

        this.period = period;

        this.prevRot = m_rotationGetter.get().getRadians();
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

        if (
            Math.abs(m_rightStick.getX()) > 0.75
            ||
            Math.abs(m_rightStick.getY()) > 0.75
        ) {
            this.targetRot = Rotation2d.fromRadians(
                Math.atan(
                    m_rightStick.getY() / m_rightStick.getX()
                )
                +
                ((m_rightStick.getX() < 0)? Math.PI : 0)
            );
        }
        Rotation2d deltaRot = targetRot.minus(m_rotationGetter.get());

        deltaRot = Rotation2d.fromRadians((deltaRot.getRadians() + 2 * Math.PI) % (2 * Math.PI));

        deltaRot = (deltaRot.getRadians() > Math.PI)? deltaRot.minus(Rotation2d.fromRadians(2 * Math.PI)) : deltaRot;

        double desiredRotVel = this.profile.calculate(
            this.period,
            new TrapezoidProfile.State(
                m_rotationGetter.get().getRadians(),
                (m_rotationGetter.get().getRadians() - this.prevRot)
            ),
            new TrapezoidProfile.State(deltaRot.getRadians(), 0)
        ).velocity;

        
        ChassisSpeeds l_returnMe = new ChassisSpeeds(
            addDeadband(m_leftStick.getY()),
            addDeadband(m_leftStick.getX()),
            desiredRotVel
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

    public static final Rotation2d PIx2 = Rotation2d.fromRadians(2 * Math.PI);
}
