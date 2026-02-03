package com.gmail.frcteam1758.lib.tankdrive.control;

import com.gmail.frcteam1758.lib.util.BoundedSet;

import edu.wpi.first.wpilibj.XboxController;

/**
 * implementation of TankDriveControls using a single XboxController.
 * 
 * uses the bindings from the 2023 season, found in 2023bindings.txt
 */
public class TankDriveControls2023 implements TankDriveControls {
    
    protected XboxController m_ctrl;

    protected BoundedSet<Double> m_speeds;

    protected char m_inversion = 1;

    protected boolean m_brakesEnabled;

    /**
     * constructs a TankDriveControls2023 from an XboxController and a BoundedSet<Double>
     * 
     * @param p_ctrl an XboxController to get input from
     * @param p_speeds a BoundedSet<Double> to use for speed multipliers
     */
    public TankDriveControls2023(XboxController p_ctrl, BoundedSet<Double> p_speeds) {
        m_ctrl = p_ctrl;
        m_speeds = p_speeds;
    }

    /**
     * constructs a TankDriveControls2023 from an XboxController and a Double[]
     * 
     * @param p_ctrl an XboxController to get input from
     * @param p_speeds a Double[] to use for speed multipliers
     */
    public TankDriveControls2023(XboxController p_ctrl, Double[] p_speeds) {
        m_ctrl = p_ctrl;
        m_speeds = new BoundedSet<>(p_speeds);
    }

    /** constructs a TankDriveControls2023 from an XboxController,
     *  with a default speed set of { 100% }
     * 
     * @param p_ctrl an XboxController to get input from
     */
    public TankDriveControls2023(XboxController p_ctrl) {
        m_ctrl = p_ctrl;
        m_speeds = new BoundedSet<>(new Double[] { 1.00 });
    }

    @Override
    public void processInputs() {
        if (m_ctrl.getYButtonPressed()) m_inversion *= -1;
        if (m_ctrl.getXButtonPressed()) m_brakesEnabled = !m_brakesEnabled;

        if      (m_ctrl.getLeftTriggerAxis()  > .5) m_speeds.toBeginning();
        else if (m_ctrl.getRightTriggerAxis() > .5) m_speeds.toEnd();
    }

    @Override
    public double getArcadeFwd()
        { return m_ctrl.getLeftY() * m_speeds.get().doubleValue() * m_inversion; }

    @Override
    public double getArcadeTurn()
        { return m_ctrl.getRightX() * m_speeds.get().doubleValue() * m_inversion; }
    
    @Override
    public double getTankLeft() {
        return (m_inversion > 0? m_ctrl.getLeftY() : m_ctrl.getRightY())
        * m_speeds.get().doubleValue() * m_inversion;
    }
    
    @Override
    public double getTankRight() {
        return (m_inversion > 0? m_ctrl.getRightY() : m_ctrl.getLeftY())
        * m_speeds.get().doubleValue() * m_inversion;
    }
    
    @Override
    public boolean getBrakes() { return m_brakesEnabled; }

    @Override
    public boolean getBooleanProperty(String name) { return false; }

    @Override
    public int getIntProperty(String name) { return 0; }

    /**
     * gets the current speed multiplier.
     * 
     * @param name unused
     * 
     * @return the current speed multiplier
     * 
     * @apiNote this method usually accesses multiple properties,
     * but TankDriveControls2023 only has one
     * 
     * @see TankDriveControls#getDoubleProperty(String)
     */
    @Override
    public double getDoubleProperty(String name) {
        return m_speeds.get().doubleValue();
    }

    @Override
    public String getStringProperty(String name) { return null; }
}
