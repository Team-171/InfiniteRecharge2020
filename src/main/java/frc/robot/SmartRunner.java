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
public class SmartRunner {
    public enum RunLevel {
        MATCH, 
        DEBUG, 
        SHOOTER_TUNING, 
        ELEVATOR_TUNING
    }

    private static EnumSet<RunLevel> levels = Constants.robotLogLevel;

    private static boolean isRunnable(EnumSet<RunLevel> messageLevels) {
        return messageLevels.stream().filter(levels::contains).toArray().length != 0;
    }

    public static void run(Runnable action, EnumSet<RunLevel> levels) {
        if (!isRunnable(levels)) {
            return;
        }

        action.run();
    }

    public static class Logger {
        public static void put(String key, double number, EnumSet<RunLevel> flags) {
            run(() -> {
                SmartDashboard.putNumber(key, number);
            }, flags);
        }

        public static void put(String key, boolean bool, EnumSet<RunLevel> flags) {
            run(() -> {
                SmartDashboard.putBoolean(key, bool);
            }, flags);
        }

        public static void getNumber(String key, DoubleConsumer callback, EnumSet<RunLevel> flags,
                double initialValue) {
            run(() -> {
                if (!SmartDashboard.getKeys().contains(key)) {
                    SmartDashboard.putNumber(key, initialValue);
                }

                callback.accept(SmartDashboard.getNumber(key, initialValue));
            }, flags);
        }

        public static void getBoolean(String key, Consumer<Boolean> callback, EnumSet<RunLevel> flags,
                Boolean initialValue) {
            run(() -> {
                if (!SmartDashboard.getKeys().contains(key)) {
                    SmartDashboard.putBoolean(key, initialValue);
                }

                callback.accept(SmartDashboard.getBoolean(key, initialValue));
            }, flags);
        }
    }
}
