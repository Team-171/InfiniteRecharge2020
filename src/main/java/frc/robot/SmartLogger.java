/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.EnumSet;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.Function;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class SmartLogger {
    public enum LogLevel {
        MATCH,
        DEBUG,
        SHOOTER_TUNING,
        ELEVATOR_TUNING 
    }

    private static EnumSet<LogLevel> levels = Constants.robotLogLevel;

    private static boolean isLoggable(EnumSet<LogLevel> messageLevels)
    {
        return messageLevels.stream().filter(levels::contains).toArray().length != 0;
    }

    public static void put(String key, double number, EnumSet<LogLevel> flags)
    {
        if(!isLoggable(flags))
        {
            return;
        }

        SmartDashboard.putNumber(key, number);
    }

    public static void put(String key, boolean bool, EnumSet<LogLevel> flags)
    {
        if(!isLoggable(flags))
        {
            return;
        }

        SmartDashboard.putBoolean(key, bool);
    }

    public static void getNumber(String key, DoubleConsumer callback, EnumSet<LogLevel> flags, double initialValue)
    {
        if(!isLoggable(flags))
        {
            return;
        }

        if(!SmartDashboard.getKeys().contains(key))
        {
            SmartDashboard.putNumber(key, initialValue);
        }

        callback.accept(SmartDashboard.getNumber(key, initialValue));
    }

    public static void getBoolean(String key, Consumer<Boolean> callback, EnumSet<LogLevel> flags, Boolean initialValue)
    {
        if(!isLoggable(flags))
        {
            return;
        }

        if(!SmartDashboard.getKeys().contains(key))
        {
            SmartDashboard.putBoolean(key, initialValue);
        }

        callback.accept(SmartDashboard.getBoolean(key, initialValue));
    }
}
