package frc.robot.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * Centralized telemetry management for SmartDashboard.
 * Provides global enable/disable and efficient batch updates.
 * 
 * Usage:
 * <pre>
 * // Anywhere in your code:
 * TelemetryManager.putNumber("MyKey", value);
 * TelemetryManager.putBoolean("MyKey", value);
 * TelemetryManager.putString("MyKey", value);
 * 
 * // For complex objects (Field2d, Mechanism2d, etc):
 * TelemetryManager.putData("Field", field);
 * </pre>
 */
public class TelemetryManager {
    private static TelemetryManager instance;
    private boolean enabled = true;
    private final Map<String, Object> lastValues = new HashMap<>();
    // Track which Sendable keys have already been registered with SmartDashboard.
    // SmartDashboard.putData() on a Field2d serializes the entire object -- call it once only.
    private final Set<String> registeredSendables = new HashSet<>();
    
    private TelemetryManager() {}
    
    private static TelemetryManager getInstance() {
        if (instance == null) {
            instance = new TelemetryManager();
        }
        return instance;
    }
    
    // ==================== STATIC CONVENIENCE METHODS ====================
    // These allow direct calls like: TelemetryManager.putNumber(...)
    
    /**
     * Enable or disable all telemetry updates.
     * Set to false for competition to reduce CAN bus utilization.
     */
    public static void setEnabled(boolean enabled) {
        getInstance().enabled = enabled;
    }
    
    public static boolean isEnabled() {
        return getInstance().enabled;
    }
    
    /**
     * Put a number to SmartDashboard only if telemetry is enabled
     * and the value has changed (unless force is true).
     */
    public static void putNumber(String key, double value) {
        getInstance().putNumberImpl(key, value, false);
    }
    
    public static void putNumber(String key, double value, boolean force) {
        getInstance().putNumberImpl(key, value, force);
    }
    
    /**
     * Put a boolean to SmartDashboard only if telemetry is enabled
     * and the value has changed (unless force is true).
     */
    public static void putBoolean(String key, boolean value) {
        getInstance().putBooleanImpl(key, value, false);
    }
    
    public static void putBoolean(String key, boolean value, boolean force) {
        getInstance().putBooleanImpl(key, value, force);
    }
    
    /**
     * Put a string to SmartDashboard only if telemetry is enabled
     * and the value has changed (unless force is true).
     */
    public static void putString(String key, String value) {
        getInstance().putStringImpl(key, value, false);
    }
    
    public static void putString(String key, String value, boolean force) {
        getInstance().putStringImpl(key, value, force);
    }
    
    /**
     * Put complex data objects (Field2d, Mechanism2d, SendableChooser, etc).
     * SmartDashboard.putData() serializes the whole object every call, so we
     * only register it once. After the first call the NT publisher handles updates.
     */
    public static void putData(String key, Sendable value) {
        if (!getInstance().enabled) return;
        // Only call SmartDashboard.putData() on the first registration.
        // Subsequent calls would re-serialize the whole Sendable needlessly.
        if (getInstance().registeredSendables.add(key)) {
            SmartDashboard.putData(key, value);
        }
    }
    
    /**
     * Clear the cache for a specific key to force next update.
     */
    public static void invalidate(String key) {
        getInstance().lastValues.remove(key);
    }
    
    /**
     * Clear all cached values.
     */
    public static void invalidateAll() {
        getInstance().lastValues.clear();
    }
    
    // ==================== INTERNAL IMPLEMENTATION ====================
    
    private void putNumberImpl(String key, double value, boolean force) {
        if (!enabled) return;
        
        if (force) {
            SmartDashboard.putNumber(key, value);
            lastValues.put(key, value);
            return;
        }
        
        Object lastValue = lastValues.get(key);
        if (lastValue == null || !lastValue.equals(value)) {
            SmartDashboard.putNumber(key, value);
            lastValues.put(key, value);
        }
    }
    
    private void putBooleanImpl(String key, boolean value, boolean force) {
        if (!enabled) return;
        
        if (force) {
            SmartDashboard.putBoolean(key, value);
            lastValues.put(key, value);
            return;
        }
        
        Object lastValue = lastValues.get(key);
        if (lastValue == null || !lastValue.equals(value)) {
            SmartDashboard.putBoolean(key, value);
            lastValues.put(key, value);
        }
    }
    
    private void putStringImpl(String key, String value, boolean force) {
        if (!enabled) return;
        
        if (force) {
            SmartDashboard.putString(key, value);
            lastValues.put(key, value);
            return;
        }
        
        Object lastValue = lastValues.get(key);
        if (lastValue == null || !lastValue.equals(value)) {
            SmartDashboard.putString(key, value);
            lastValues.put(key, value);
        }
    }
}
