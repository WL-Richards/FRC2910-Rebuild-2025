package frc.robot.util.phoenix6;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.config.talonFX.TalonFXConfigEquality;

import java.util.EnumMap;
import java.util.Map;
import java.util.StringJoiner;
import java.util.function.Supplier;

/**
 * This code is almost directly from Jack in the Bot
 * 
 * Utility helpers for common Phoenix 6 (CTRE) patterns such as
 * - Robust error checking/throwing with optional retries
 * - Apply/refresh configuration with verification round‑trip
 * - Fault and sticky‑fault aggregation/reporting
 *
 * <p>Design goals:
 * <ul>
 *   <li>Small, focused surface area</li>
 *   <li>Clear, consistent logging to DriverStation</li>
 *   <li>Safe defaults (bounded retries, null‑safety for descriptions)</li>
 *   <li>Non‑instantiable utility class</li>
 * </ul>
 */
public final class Phoenix6Util {

    private Phoenix6Util() { /* no instances */ }

    private static final String kDashboardConfigKey = "Talon Configuration state";

    // Tracks aggregate success across all calls to applyAndCheckConfiguration(...)
    private static boolean sConfigAggregateResult = true;

    // ---------------------------------------------------------------------
    // Error handling helpers
    // ---------------------------------------------------------------------

    /**
     * Logs a non‑OK Phoenix StatusCode to DriverStation as an error.
     *
     * @param statusCode the Phoenix status/result code
     * @param message    context message to accompany the error
     */
    public static void checkError(final StatusCode statusCode, final String message) {
        if (statusCode != StatusCode.OK) {
            DriverStation.reportError(message + " " + statusCode, false);
        }
    }

    /**
     * Same as {@link #checkError(StatusCode, String)} but throws an exception on error.
     *
     * @param statusCode the Phoenix status/result code
     * @param message    context message
     * @throws RuntimeException if {@code statusCode != StatusCode.OK}
     */
    public static void checkErrorWithThrow(final StatusCode statusCode, final String message) {
        if (statusCode != StatusCode.OK) {
            throw new RuntimeException(message + " " + statusCode);
        }
    }

    /**
     * Executes a Phoenix call with bounded retry logic.
     * The supplied function is executed at least once and up to {@code maxAttempts} times
     * until {@link StatusCode#OK} is returned.
     *
     * @param function    a supplier that invokes a Phoenix 6 API and returns a StatusCode
     * @param maxAttempts maximum number of attempts (must be >= 1)
     * @return true if the final call returned {@link StatusCode#OK}, false otherwise
     */
    public static boolean checkErrorAndRetry(final Supplier<StatusCode> function, final int maxAttempts) {
        final int attempts = Math.max(1, maxAttempts);
        StatusCode code = function.get();
        int tryIndex = 1;

        while (code != StatusCode.OK && tryIndex < attempts) {
            DriverStation.reportWarning("Retrying CTRE Device Config " + code.getName(), false);
            code = function.get();
            tryIndex++;
        }

        if (code != StatusCode.OK) {
            DriverStation.reportError(
                    "Failed to execute Phoenix6 API call after " + attempts + " attempts. " + code.getDescription(),
                    false);
            return false;
        }
        return true;
    }

    /** Default retry count = 5. */
    public static boolean checkErrorAndRetry(final Supplier<StatusCode> function) {
        return checkErrorAndRetry(function, 5);
    }

    // ---------------------------------------------------------------------
    // Configuration helpers
    // ---------------------------------------------------------------------

    /**
     * Applies a configuration to a TalonFX with retries, then reads back the config and
     * verifies it matches the expected values.
     *
     * @param talon     the motor controller
     * @param config    desired configuration to apply
     * @param maxTries  number of attempts for the apply/verify loop
     * @return true if configuration was applied and verified; false otherwise
     */
    public static boolean applyAndCheckConfiguration(final TalonFX talon,
                                                     final TalonFXConfiguration config,
                                                     final int maxTries) {
        final int attempts = Math.max(1, maxTries);
        final String desc = safeDescription(talon);

        for (int attempt = 1; attempt <= attempts; attempt++) {
            if (checkErrorAndRetry(() -> talon.getConfigurator().apply(config))) {
                // API reports success — verify by reading back
                if (readAndVerifyConfiguration(talon, config)) {
                    return true;
                }
                DriverStation.reportWarning(
                        "Failed to verify config for talon [" + desc + "] (attempt " + attempt + " of " + attempts + ")",
                        false);
            } else {
                DriverStation.reportWarning(
                        "Failed to apply config for talon [" + desc + "] (attempt " + attempt + " of " + attempts + ")",
                        false);
            }
        }

        DriverStation.reportError("Failed to apply config for talon after " + attempts + " attempts", false);
        return false;
    }

    /**
     * Overload that uses a default of 5 attempts and also publishes cumulative state to
     * SmartDashboard under {@value #kDashboardConfigKey}.
     */
    public static boolean applyAndCheckConfiguration(final TalonFX talon, final TalonFXConfiguration config) {
        final boolean result = applyAndCheckConfiguration(talon, config, 5);
        sConfigAggregateResult &= result; // accumulate across calls
        SmartDashboard.putBoolean(kDashboardConfigKey, sConfigAggregateResult);
        return result;
    }

    /**
     * Refreshes the TalonFX configuration and verifies it matches {@code expected}.
     *
     * @param talon    the motor controller
     * @param expected expected configuration
     * @return true if the read config matches; false if refresh failed or values differ
     */
    public static boolean readAndVerifyConfiguration(final TalonFX talon, final TalonFXConfiguration expected) {
        final TalonFXConfiguration readback = new TalonFXConfiguration();
        if (!checkErrorAndRetry(() -> talon.getConfigurator().refresh(readback))) {
            DriverStation.reportWarning("Failed to read config for talon [" + safeDescription(talon) + "]", false);
            return false;
        }
        if (!TalonFXConfigEquality.isEqual(expected, readback)) {
            DriverStation.reportWarning(
                    "Configuration verification failed for talon [" + safeDescription(talon) + "]", false);
            return false;
        }
        return true;
    }

    /** @return aggregated success state of prior configuration applications. */
    public static boolean getAggregateConfigResult() {
        return sConfigAggregateResult;
    }

    private static String safeDescription(final TalonFX talon) {
        final String desc = talon.getDescription();
        return (desc == null || desc.isBlank()) ? ("TalonFX ID " + talon.getDeviceID()) : desc;
    }

    // ---------------------------------------------------------------------
    // Fault helpers
    // ---------------------------------------------------------------------

    /** Subset of real‑time fault flags surfaced as convenience keys. */
    public enum Fault {
        Hardware,
        OverSupplyV,
        Undervoltage,
        UnstableSupplyV,
        StatorCurrLimit,
        SupplyCurrLimit,
        UnlicensedFeatureInUse,
        BridgeBrownout,
        RemoteSensorReset,
        RemoteSensorPosOverflow,
        RemoteSensorDataInvalid,
        FusedSensorOutOfSync,
        UsingFusedCANcoderWhileUnlicensed,
        MissingDifferentialFX,
        ReverseHardLimit,
        ForwardHardLimit,
        ReverseSoftLimit,
        ForwardSoftLimit,
        ProcTemp,
        DeviceTemp,
    }

    /** Sticky (latched) versions of a subset of faults. */
    public enum StickyFault {
        BootDuringEnable,
        BridgeBrownout,
        DeviceTemp,
        ForwardHardLimit,
        ForwardSoftLimit,
        Hardware,
        OverSupplyV,
        ProcTemp,
        ReverseHardLimit,
        ReverseSoftLimit,
        RemoteSensorReset,
        Undervoltage,
        UnstableSupplyV,
        UnlicensedFeatureInUse
    }

    /**
     * Aggregates and reports any asserted real‑time faults for the given TalonFX.
     *
     * @param subsystemName for context in the DS log
     * @param talon         the device to query
     */
    public static void checkFaults(final String subsystemName, final TalonFX talon) {
        final EnumMap<Fault, Boolean> faults = new EnumMap<>(Fault.class);
        faults.put(Fault.Hardware, talon.getFault_Hardware().getValue());
        faults.put(Fault.OverSupplyV, talon.getFault_OverSupplyV().getValue());
        faults.put(Fault.Undervoltage, talon.getFault_Undervoltage().getValue());
        faults.put(Fault.UnstableSupplyV, talon.getFault_UnstableSupplyV().getValue());
        // faults.put(Fault.StatorCurrLimit, talon.getFault_StatorCurrLimit().getValue());
        // faults.put(Fault.SupplyCurrLimit, talon.getFault_SupplyCurrLimit().getValue());
        faults.put(Fault.UnlicensedFeatureInUse, talon.getFault_UnlicensedFeatureInUse().getValue());
        faults.put(Fault.BridgeBrownout, talon.getFault_BridgeBrownout().getValue());
        faults.put(Fault.RemoteSensorReset, talon.getFault_RemoteSensorReset().getValue());
        faults.put(Fault.RemoteSensorPosOverflow, talon.getFault_RemoteSensorPosOverflow().getValue());
        faults.put(Fault.RemoteSensorDataInvalid, talon.getFault_RemoteSensorDataInvalid().getValue());
        faults.put(Fault.FusedSensorOutOfSync, talon.getFault_FusedSensorOutOfSync().getValue());
        faults.put(Fault.UsingFusedCANcoderWhileUnlicensed, talon.getFault_UsingFusedCANcoderWhileUnlicensed().getValue());
        faults.put(Fault.MissingDifferentialFX, talon.getFault_MissingDifferentialFX().getValue());
        faults.put(Fault.ReverseHardLimit, talon.getFault_ReverseHardLimit().getValue());
        faults.put(Fault.ForwardHardLimit, talon.getFault_ForwardHardLimit().getValue());
        faults.put(Fault.ReverseSoftLimit, talon.getFault_ReverseSoftLimit().getValue());
        faults.put(Fault.ForwardSoftLimit, talon.getFault_ForwardSoftLimit().getValue());
        faults.put(Fault.ProcTemp, talon.getFault_ProcTemp().getValue());
        faults.put(Fault.DeviceTemp, talon.getFault_DeviceTemp().getValue());

        final StringJoiner joiner = new StringJoiner(", ");
        for (Map.Entry<Fault, Boolean> e : faults.entrySet()) {
            if (Boolean.TRUE.equals(e.getValue())) {
                joiner.add(e.getKey().name());
            }
        }
        final String msg = joiner.toString();
        if (!msg.isEmpty()) {
            DriverStation.reportError(subsystemName + ": Talon Faults! " + msg, false);
        }
    }

    /**
     * Aggregates and reports asserted sticky faults, then clears sticky faults on the device.
     *
     * @param subsystemName for context in the DS log
     * @param talon         the device to query and clear
     */
    public static void checkStickyFaults(final String subsystemName, final TalonFX talon) {
        final EnumMap<StickyFault, Boolean> faults = new EnumMap<>(StickyFault.class);
        faults.put(StickyFault.BootDuringEnable, talon.getStickyFault_BootDuringEnable().getValue());
        faults.put(StickyFault.BridgeBrownout, talon.getStickyFault_BridgeBrownout().getValue());
        faults.put(StickyFault.DeviceTemp, talon.getStickyFault_DeviceTemp().getValue());
        faults.put(StickyFault.ForwardHardLimit, talon.getStickyFault_ForwardHardLimit().getValue());
        faults.put(StickyFault.ForwardSoftLimit, talon.getStickyFault_ForwardSoftLimit().getValue());
        faults.put(StickyFault.Hardware, talon.getStickyFault_Hardware().getValue());
        faults.put(StickyFault.OverSupplyV, talon.getStickyFault_OverSupplyV().getValue());
        faults.put(StickyFault.ProcTemp, talon.getStickyFault_ProcTemp().getValue());
        faults.put(StickyFault.ReverseHardLimit, talon.getStickyFault_ReverseHardLimit().getValue());
        faults.put(StickyFault.ReverseSoftLimit, talon.getStickyFault_ReverseSoftLimit().getValue());
        faults.put(StickyFault.Undervoltage, talon.getStickyFault_Undervoltage().getValue());
        faults.put(StickyFault.UnstableSupplyV, talon.getStickyFault_UnstableSupplyV().getValue());
        faults.put(StickyFault.UnlicensedFeatureInUse, talon.getStickyFault_UnlicensedFeatureInUse().getValue());
        faults.put(StickyFault.RemoteSensorReset, talon.getStickyFault_RemoteSensorReset().getValue());

        final StringJoiner joiner = new StringJoiner(", ");
        for (Map.Entry<StickyFault, Boolean> e : faults.entrySet()) {
            if (Boolean.TRUE.equals(e.getValue())) {
                joiner.add(e.getKey().name());
            }
        }
        final String msg = joiner.toString();
        if (!msg.isEmpty()) {
            DriverStation.reportError(subsystemName + ": Talon StickyFaults! " + msg, false);
        }

        // Clear sticky faults after reporting so we only see new occurrences next time.
        talon.clearStickyFaults();
    }
}