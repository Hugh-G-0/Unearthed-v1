package com.gmail.frcteam1758.lib.annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/** indicates which CAN ids and USB ports an object uses */
@Target(ElementType.FIELD)
@Retention(RetentionPolicy.RUNTIME)
public @interface Resources {
    
    /** the CAN ids an object uses */
    public char[] canUsage();

    /** the USB ports an object uses */
    public char[] usbUsage();
}
