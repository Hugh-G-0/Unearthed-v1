package com.gmail.frcteam1758.lib.tankdrive.control;


/**
 * the TankDriveControls interface should be implemented to create a class
 * containing inputs, ex. Xboxcontrollers, and sending their input to another class
 * through named methods. This allows users to use a different TankDriveControls to
 * change controls (like changing keybinds or even controller type)
 * and add functionalities (like cruise control)
 */
public interface TankDriveControls {

    /**
     * polls buttons, sticks, etc., and uses their input to operate toggles,
     * change multipliers, etc.
     * <p>
     * Should be called exactly once per cycle (run of Robot.activePeriodicMethod())
     */
    public void processInputs();

    /**
     * gets the fwd value commanded by the controller
     * 
     * <p>
     * Use: DifferentialDrive.arcadeDrive(m_ctrl.getFwd(), 0)
     * 
     * @return the fwd value commanded by the controller
     */
    public double getArcadeFwd();

    /**
     * gets the turn value commanded by the controller
     * 
     * <p>
     * Use: DifferentialDrive.arcadeDrive(0, m_ctrl.getTurn())
     * 
     * @return the turn value commanded by the controller
     */
    public double getArcadeTurn();

    /**
     * gets the left side speed commanded by the controller.
     * 
     * @return the left side speed commanded by the controller
     */
    public double getTankLeft();

    /**
     * gets the right side speed commanded by the controller.
     * 
     * @return the right side speed commanded by the controller
     */
    public double getTankRight();

    /**
     * gets whether or not brakes are enabled, if brakes are supported
     * 
     * @return whether or not brakes are enabled
     */
    public boolean getBrakes();


    /*
     * the methods below allow additional properties to be polled,
     * even though they don't have their own method.
     * this allows the TankDriveControls interface to be used with
     * a more sophisticated control system. For example,
     * <p>
     * For Exeample, getDoubleProperty("cruise_speed") could return
     * a cruise control speed the robot is using. */


    /**
     * gets a property of type boolean not included in the base TankDriveControls
     * 
     * @param name the name of the property
     * 
     * @return a boolean representing the property
     */
    public boolean getBooleanProperty(String name);

    /**
     * gets a property of type int not included in the base TankDriveControls
     * 
     * @param name the name of the property
     * 
     * @return an int representing the property
     */
    public int getIntProperty(String name);

    /**
     * gets a property of type double not included in the base TankDriveControls
     * 
     * @param name the name of the property
     * 
     * @return a double representing the property
     */
    public double getDoubleProperty(String name);

    /**
     * gets a property of type String not included in the base TankDriveControls
     * 
     * @param name the name of the property
     * 
     * @return a String representing the property
     */
    public String getStringProperty(String name);
}