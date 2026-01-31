package com.team6443.lib.subsystems.vision.util;

import java.nio.ByteBuffer;

import com.team6443.lib.subsystems.vision.util.limelight.LimelightHelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

/**
 * Mostly yoinked from 254
 * Represents a robot pose estimate using multiple AprilTags (Megatag).
 *
 * @param fieldToRobot The estimated robot pose on the field
 * @param timestampSeconds The timestamp when this estimate was captured
 * @param latency Processing latency in seconds
 * @param avgTagArea Average area of detected tags
 * @param quality Quality score of the pose estimate (0-1)
 * @param fiducialIds IDs of fiducials used for this estimate
 */
public record MegatagPoseEstimate(
    Pose2d fieldToRobot,
    double timestampSeconds,
    double latency,
    double avgTagArea,
    double quality,
    int[] fiducialIds
)
implements StructSerializable {
    /**
     * Canonical constructor that backfills optional fields when they are omitted by the source
     * sensor data.
     *
     * @param fieldToRobot Pose reported by the vision pipeline (defaults to {@link Pose2d#kZero})
     * @param timestampSeconds Capture timestamp supplied by the vision pipeline
     * @param latency Measured processing latency
     * @param avgTagArea Average AprilTag area from the detection set
     * @param quality Overall quality score from the upstream estimator
     * @param fiducialIds IDs of the AprilTags contributing to the estimate (defaults to empty array)
     */
    public MegatagPoseEstimate {
        if (fieldToRobot == null) {
            fieldToRobot = Pose2d.kZero;
        }
        if (fiducialIds == null) {
            fiducialIds = new int[0];
        }
    }

    /**
     * Builds a {@link MegatagPoseEstimate} from the raw data provided by Limelight's helper class.
     * The robot pose and fiducial IDs are sanitized so that downstream consumers do not need to
     * null-check the Limelight outputs.
     *
     * @param poseEstimate Pose estimate returned by {@link LimelightHelpers}
     * @return Canonicalized {@link MegatagPoseEstimate} ready for logging or network transport
     */
    public static MegatagPoseEstimate fromLimelight(LimelightHelpers.PoseEstimate poseEstimate) {
        Pose2d fieldToRobot = poseEstimate.pose;
        if (fieldToRobot == null) {
            fieldToRobot = Pose2d.kZero;
        }
        int[] fiducialIds = new int[poseEstimate.rawFiducials.length];
        for (int i = 0; i < poseEstimate.rawFiducials.length; i++) {
            if (poseEstimate.rawFiducials[i] != null) {
                fiducialIds[i] = poseEstimate.rawFiducials[i].id;
            }
        }
        return new MegatagPoseEstimate(
                fieldToRobot,
                poseEstimate.timestampSeconds,
                poseEstimate.latency,
                poseEstimate.avgTagArea,
                fiducialIds.length > 1 ? 1.0 : 1.0 - poseEstimate.rawFiducials[0].ambiguity,
                fiducialIds);
    }

    public static final MegatagPoseEstimateStruct struct = new MegatagPoseEstimateStruct();

    /**
     * WPILib {@link Struct} implementation that defines how to serialize the pose estimate to a
     * {@link ByteBuffer}. This enables logging and network transport using the WPILib struct APIs.
     */
    public static class MegatagPoseEstimateStruct implements Struct<MegatagPoseEstimate> {

        @Override
        public Class<MegatagPoseEstimate> getTypeClass() {
            return MegatagPoseEstimate.class;
        }

        @Override
        public String getTypeString() {
            return "record:MegatagPoseEstimate";
        }

        /**
         * Calculates the total serialized byte size by combining the nested {@link Pose2d} struct
         * with the primitive fields.
         */
        @Override
        public int getSize() {
            return Pose2d.struct.getSize() + 3 * Double.BYTES;
        }

        /** Returns the schema string used by NetworkTables/Struct tools for discovery. */
        @Override
        public String getSchema() {
            return "Pose2d fieldToRobot; double timestampSeconds; double latency; double avgTagArea";
        }

        /** Indicates that this struct nests a {@link Pose2d} as its first field. */
        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] {Pose2d.struct};
        }

        /**
         * Reconstructs a {@link MegatagPoseEstimate} from data stored in a {@link ByteBuffer}. The
         * fiducial IDs are not presently serialized, so they default to an empty array.
         */
        @Override
        public MegatagPoseEstimate unpack(ByteBuffer bb) {
            Pose2d fieldToRobot = Pose2d.struct.unpack(bb);
            double timestampSeconds = bb.getDouble();
            double latency = bb.getDouble();
            double avgTagArea = bb.getDouble();
            double quality = bb.getDouble();
            int[] fiducialIds = new int[0];
            return new MegatagPoseEstimate(
                    fieldToRobot, timestampSeconds, latency, avgTagArea, quality, fiducialIds);
        }
 
        /**
         * Packs every serialized field of a {@link MegatagPoseEstimate} into the provided
         * {@link ByteBuffer} so the data can be transmitted via the struct API.
         */
        @Override
        public void pack(ByteBuffer bb, MegatagPoseEstimate value) {
            Pose2d.struct.pack(bb, value.fieldToRobot());
            bb.putDouble(value.timestampSeconds());
            bb.putDouble(value.latency());
            bb.putDouble(value.avgTagArea());
            bb.putDouble(value.quality());
        }

        /** Human-readable type name used by the struct registry. */
        @Override
        public String getTypeName() {
            return "MegatagPoseEstimate";
        }
    }
}
