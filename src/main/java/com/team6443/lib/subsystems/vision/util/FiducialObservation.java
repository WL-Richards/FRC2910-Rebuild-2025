package com.team6443.lib.subsystems.vision.util;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Objects;

import com.team6443.lib.subsystems.vision.util.limelight.LimelightHelpers;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

/**
 * Represents an observation of a fiducial marker (AprilTag) with position and quality data.
 *
 * @param id The fiducial marker ID
 * @param txnc Normalized horizontal offset (-1 to 1)
 * @param tync Normalized vertical offset (-1 to 1)
 * @param ambiguity Pose ambiguity score (0 = confident, 1 = ambiguous)
 * @param area Target area as percentage of image
 */
public record FiducialObservation(
    int id, 
    double txnc, 
    double tync, 
    double ambiguity, 
    double area)
    implements StructSerializable {

    /**
     * Converts a Limelight raw fiducial to this project's canonical observation representation. This
     * helper shields callers from dealing with Limelight-specific value names or null checks.
     *
     * @param fiducial Raw fiducial information reported by {@link LimelightHelpers}
     * @return Populated observation, or {@code null} if the Limelight entry was null
     */
    public static FiducialObservation fromLimelight(LimelightHelpers.RawFiducial fiducial) {
        if (fiducial == null) {
            return null;
        }
        return new FiducialObservation(
                fiducial.id, fiducial.txnc, fiducial.tync, fiducial.ambiguity, fiducial.ta);
    }

    /**
     * Converts and filters a collection of Limelight fiducial reports into an array of structured
     * observations.
     *
     * @param fiducials Array returned by {@link LimelightHelpers}
     * @return Array containing only non-null converted entries
     */
    public static FiducialObservation[] fromLimelight(LimelightHelpers.RawFiducial[] fiducials) {
        if (fiducials == null) {
            return new FiducialObservation[0];
        }
        return Arrays.stream(fiducials)
                .map(FiducialObservation::fromLimelight)
                .filter(Objects::nonNull)
                .toArray(FiducialObservation[]::new);
    }

    public static final Struct<FiducialObservation> struct =
            new Struct<FiducialObservation>() {
                @Override
                public Class<FiducialObservation> getTypeClass() {
                    return FiducialObservation.class;
                }

                @Override
                public String getTypeString() {
                    return "record:FiducialObservation";
                }

                /**
                 * Total serialized size: four doubles plus the integer tag ID.
                 */
                @Override
                public int getSize() {
                    return Integer.BYTES + 4 * Double.BYTES;
                }

                /**
                 * NetworkTables schema string used to describe the layout to subscribers.
                 */
                @Override
                public String getSchema() {
                    return "int id;double txnc;double tync;double ambiguity";
                }

                /**
                 * Recreates a {@link FiducialObservation} from a {@link ByteBuffer} payload.
                 */
                @Override
                public FiducialObservation unpack(ByteBuffer bb) {
                    int id = bb.getInt();
                    double txnc = bb.getDouble();
                    double tync = bb.getDouble();
                    double ambiguity = bb.getDouble();
                    double area = bb.getDouble();
                    return new FiducialObservation(id, txnc, tync, ambiguity, area);
                }

                /**
                 * Serializes the observation to a {@link ByteBuffer} for logging or NT transport.
                 */
                @Override
                public void pack(ByteBuffer bb, FiducialObservation value) {
                    bb.putInt(value.id());
                    bb.putDouble(value.txnc());
                    bb.putDouble(value.tync());
                    bb.putDouble(value.ambiguity());
                    bb.putDouble(value.area());
                }

                @Override
                public String getTypeName() {
                    return "FiducialObservation";
                }
            };
}
