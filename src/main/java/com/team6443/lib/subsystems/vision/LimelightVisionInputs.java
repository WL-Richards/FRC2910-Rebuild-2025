// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.vision;

import java.util.function.ToDoubleFunction;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.team6443.lib.subsystems.vision.util.FiducialObservation;
import com.team6443.lib.subsystems.vision.util.MegatagPoseEstimate;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N6;

/**
 * Snapshot of all vision-related measurements that can be logged or replayed via AdvantageKit.
 * Every field is public so subsystem implementations can populate them before forwarding to the
 * logger.
 */
public class LimelightVisionInputs implements LoggableInputs {
    private static final String MT1_PREFIX = "Megatag1PoseEstimate";
    private static final String MT2_PREFIX = "Megatag2PoseEstimate";

    // Vision Standard Deviation configuration
    public static final int StandardDeviationArrayLength = 12;
    public static final double[] DefaultStandardDeviations = new double[StandardDeviationArrayLength];

    /** True if any AprilTag detection is currently reported by the sensor stack. */
    public boolean hasTag = false;

    /** Individual AprilTag observations returned by the vision pipeline. */
    public FiducialObservation[] fiducialObservations = new FiducialObservation[0];

    /** Megatag pipeline estimate using single-tag solves (if available). */
    public MegatagPoseEstimate megatag1PoseEstimate = new MegatagPoseEstimate(
        Pose2d.kZero, 
        0.0, 
        0.0, 
        0.0, 
        0.0, 
        new Matrix<>(Nat.N6(), Nat.N1()),
        new int[0]
    );

    /** Megatag pipeline estimate using two or more tags (if available). */
    public MegatagPoseEstimate megatag2PoseEstimate = new MegatagPoseEstimate(
        Pose2d.kZero, 
        0.0, 
        0.0, 
        0.0, 
        0.0,
        new Matrix<>(Nat.N6(), Nat.N1()),
        new int[0]
    );

    /** Number of tags contributing to the single-tag solve. */
    public int megatag1TagCount = 0;

    /** Number of tags contributing to the multi-tag solve. */
    public int megatag2TagCount = 0;

    /** Fused 3D robot pose reported by the camera's onboard estimator. */
    public Pose3d estimatedRobotPose3d = Pose3d.kZero;

    /**
     * Vision-estimated standard deviations for each Megatag solve.
     * [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
     */
    public double[] standardDeviations = new double[StandardDeviationArrayLength];

    /**
     * Serializes all inputs into the provided log table so AdvantageKit can persist them for later
     * playback. Multi-field objects are flattened so they can be represented using built-in log
     * types.
     */
    @Override
    public void toLog(LogTable table) {
        table.put("HasTag", hasTag);
        FiducialLogHelper.write(table, fiducialObservations);
        MegatagLogHelper.write(table, MT1_PREFIX, megatag1PoseEstimate);
        MegatagLogHelper.write(table, MT2_PREFIX, megatag2PoseEstimate);
        table.put("Megatag1TagCount", megatag1TagCount);
        table.put("Megatag2TagCount", megatag2TagCount);
        table.put("EstimatedRobotPose3d", estimatedRobotPose3d);
        table.put("StandardDeviations", standardDeviations);
    }

    /**
     * Restores input values from a log table entry. This enables AdvantageKit replay or unit tests
     * to feed historical data back into subsystems.
     */
    @Override
    public void fromLog(LogTable table) {
        hasTag = table.get("HasTag", hasTag);
        fiducialObservations = FiducialLogHelper.read(table, fiducialObservations);
        megatag1PoseEstimate = MegatagLogHelper.read(table, MT1_PREFIX, megatag1PoseEstimate);
        megatag2PoseEstimate = MegatagLogHelper.read(table, MT2_PREFIX, megatag2PoseEstimate);
        megatag1TagCount = table.get("Megatag1TagCount", megatag1TagCount);
        megatag2TagCount = table.get("Megatag2TagCount", megatag2TagCount);
        estimatedRobotPose3d = table.get("EstimatedRobotPose3d", estimatedRobotPose3d);
        standardDeviations = table.get("StandardDeviations", standardDeviations);
    }

    /**
     * Utility methods that translate {@link FiducialObservation} arrays to/from primitive arrays so
     * they can be represented inside an AdvantageKit {@link LogTable}.
     */
    private static final class FiducialLogHelper {
        private static final String PREFIX = "FiducialObservations";

        private FiducialLogHelper() {}

        /**
         * Writes every field of the fiducial observations into primitive arrays so that AdvantageKit
         * can store them.
         */
        static void write(LogTable table, FiducialObservation[] observations) {
            FiducialObservation[] sanitized = sanitize(observations);
            table.put(PREFIX + "/Ids", map(sanitized, obs -> obs.id()));
            table.put(PREFIX + "/Txnc", map(sanitized, FiducialObservation::txnc));
            table.put(PREFIX + "/Tync", map(sanitized, FiducialObservation::tync));
            table.put(PREFIX + "/Ambiguity", map(sanitized, FiducialObservation::ambiguity));
            table.put(PREFIX + "/Area", map(sanitized, FiducialObservation::area));
        }

        /**
         * Reads the primitive arrays back out of the log table and rehydrates them into structured
         * {@link FiducialObservation} records. The previous state is used as the default value.
         */
        static FiducialObservation[] read(
                LogTable table, FiducialObservation[] fallbackObservations) {
            FiducialObservation[] sanitizedFallback = sanitize(fallbackObservations);
            double[] ids = table.get(PREFIX + "/Ids", map(sanitizedFallback, obs -> obs.id()));
            double[] txncs = table.get(PREFIX + "/Txnc", map(sanitizedFallback, FiducialObservation::txnc));
            double[] tyncs = table.get(PREFIX + "/Tync", map(sanitizedFallback, FiducialObservation::tync));
            double[] ambiguities =
                    table.get(
                            PREFIX + "/Ambiguity",
                            map(sanitizedFallback, FiducialObservation::ambiguity));
            double[] areas =
                    table.get(PREFIX + "/Area", map(sanitizedFallback, FiducialObservation::area));
            return rebuild(ids, txncs, tyncs, ambiguities, areas);
        }

        private static FiducialObservation[] sanitize(FiducialObservation[] observations) {
            return observations == null ? new FiducialObservation[0] : observations;
        }

        private static double[] map(
                FiducialObservation[] observations, ToDoubleFunction<FiducialObservation> mapper) {
            double[] values = new double[observations.length];
            for (int i = 0; i < observations.length; i++) {
                values[i] = mapper.applyAsDouble(observations[i]);
            }
            return values;
        }

        private static FiducialObservation[] rebuild(
                double[] ids,
                double[] txncs,
                double[] tyncs,
                double[] ambiguities,
                double[] areas) {
            int count =
                    Math.min(
                            ids.length,
                            Math.min(
                                    txncs.length,
                                    Math.min(
                                            tyncs.length,
                                            Math.min(ambiguities.length, areas.length))));
            FiducialObservation[] rebuilt = new FiducialObservation[count];
            for (int i = 0; i < count; i++) {
                rebuilt[i] =
                        new FiducialObservation(
                                (int) Math.round(ids[i]), txncs[i], tyncs[i], ambiguities[i], areas[i]);
            }
            return rebuilt;
        }
    }

    /**
     * Handles serialization for {@link MegatagPoseEstimate} records by flattening the struct into
     * primitive log entries.
     */
    private static final class MegatagLogHelper {
        private MegatagLogHelper() {}

        /**
         * Stores the Megatag struct into discrete log entries keyed by the provided prefix.
         */
        static void write(LogTable table, String prefix, MegatagPoseEstimate estimate) {
            MegatagPoseEstimate sanitized = sanitize(estimate);
            table.put(prefix + "/FieldToRobot", sanitized.fieldToRobot());
            table.put(prefix + "/TimestampSeconds", sanitized.timestampSeconds());
            table.put(prefix + "/Latency", sanitized.latency());
            table.put(prefix + "/AvgTagArea", sanitized.avgTagArea());
            table.put(prefix + "/Quality", sanitized.quality());
            table.put(prefix + "/FiducialIds", encodeIds(sanitized.fiducialIds()));
        }

        /**
         * Restores a {@link MegatagPoseEstimate} from the flattened log representation, falling back
         * to the provided estimate when an entry is missing.
         */
        static MegatagPoseEstimate read(
                LogTable table, String prefix, MegatagPoseEstimate fallback) {
            MegatagPoseEstimate sanitizedFallback = sanitize(fallback);
            Pose2d fieldToRobot =
                    table.get(prefix + "/FieldToRobot", sanitizedFallback.fieldToRobot());
            double timestampSeconds =
                    table.get(prefix + "/TimestampSeconds", sanitizedFallback.timestampSeconds());
            double latency = table.get(prefix + "/Latency", sanitizedFallback.latency());
            double avgTagArea = table.get(prefix + "/AvgTagArea", sanitizedFallback.avgTagArea());
            double quality = table.get(prefix + "/Quality", sanitizedFallback.quality());
            double[] fiducialIds =
                    table.get(prefix + "/FiducialIds", encodeIds(sanitizedFallback.fiducialIds()));

            Matrix<N6, N1> stddevs = table.get(prefix + "/StdDevs", sanitizedFallback.stdDevs());
            return new MegatagPoseEstimate(
                    fieldToRobot,
                    timestampSeconds,
                    latency,
                    avgTagArea,
                    quality,
                    stddevs,
                    decodeIds(fiducialIds));
        }

        private static MegatagPoseEstimate sanitize(MegatagPoseEstimate estimate) {
            if (estimate == null) {
                return new MegatagPoseEstimate(
                    Pose2d.kZero, 
                    0.0, 
                    0.0, 
                    0.0, 
                    0.0, 
                    new Matrix<>(Nat.N6(), Nat.N1()), 
                    new int[0]
                );
            }
            if (estimate.fiducialIds() == null) {
                return new MegatagPoseEstimate(
                        estimate.fieldToRobot(),
                        estimate.timestampSeconds(),
                        estimate.latency(),
                        estimate.avgTagArea(),
                        estimate.quality(),
                        estimate.stdDevs(),
                        new int[0]);
            }
            return estimate;
        }

        private static double[] encodeIds(int[] ids) {
            double[] encoded = new double[ids.length];
            for (int i = 0; i < ids.length; i++) {
                encoded[i] = ids[i];
            }
            return encoded;
        }

        private static int[] decodeIds(double[] encoded) {
            int[] ids = new int[encoded.length];
            for (int i = 0; i < encoded.length; i++) {
                ids[i] = (int) Math.round(encoded[i]);
            }
            return ids;
        }
    }
}
