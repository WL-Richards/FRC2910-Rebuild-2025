// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;

/** 
 * Constants specific to the current game year in this case 2025 Reefscape
 */
public class FieldConstants {

    /**
     * Define layout of the april tags on the given field
     */
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;
    static {
        APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        APRIL_TAG_FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    }

    public static final double APRIL_TAG_HEIGHT_METERS = Units.inchesToMeters(6.5);
    public static final double APRIL_TAG_WIDTH_METERS = Units.inchesToMeters(6.5);

    

    // --- April Tags ---
    public static final double MIN_APRIL_TAG_ID = 0;
    public static final double MAX_APRIL_TAG_ID = 22;
    
    // April tag IDs
    public static final int RED_LEFT_CORAL_STATION = 1;
    public static final int RED_RIGHT_CORAL_STATION = 2;
    public static final int RED_PROCESSOR = 3;
    public static final int RED_RIGHT_NET = 4;
    public static final int RED_LEFT_NET = 5;
    public static final int RED_REEF_LEFT_DRIVER_STATION = 6;
    public static final int RED_REEF_CENTER_DRIVER_STATION = 7;
    public static final int RED_REEF_RIGHT_DRIVER_STATION = 8;
    public static final int RED_REEF_RIGHT_BARGE = 9;
    public static final int RED_REEF_CENTER_BARGE = 10;
    public static final int RED_REEF_LEFT_BARGE = 11;
    public static final int BLUE_RIGHT_CORAL_STATION = 12;
    public static final int BLUE_LEFT_CORAL_STATION = 13;
    public static final int BLUE_LEFT_BARGE = 14;
    public static final int BLUE_RIGHT_BARGE = 15;
    public static final int BLUE_PROCESSOR = 16;
    public static final int BLUE_REEF_RIGHT_DRIVER_STATION = 17;
    public static final int BLUE_REEF_CENTER_DRIVER_STATION = 18;
    public static final int BLUE_REEF_LEFT_DRIVER_STATION = 19;
    public static final int BLUE_REEF_RIGHT_BARGE = 20;
    public static final int BLUE_REEF_CENTER_BARGE = 21;
    public static final int BLUE_REEF_LEFT_BARGE = 22;
    
    public static Pose3d getTagPose3d(int id) {
        if (id < RED_LEFT_CORAL_STATION || id > BLUE_REEF_LEFT_BARGE) {
            throw new IllegalArgumentException("id must be between 1 and 22");
        }

        return APRIL_TAG_FIELD_LAYOUT.getTagPose(id).orElseThrow(() -> {
            final String message = String.format("getTagPose called for unexpected tag %d", id);
            return new RuntimeException(message);
        });
    }

    public static Pose2d getTagPose2d(int id) {
        return getTagPose3d(id).toPose2d();
    }
}
