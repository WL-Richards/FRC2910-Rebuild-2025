// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.constants;

import com.team6443.lib.constants.fields.Field2025;
import com.team6443.lib.constants.fields.interfaces.YearFieldConstantable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;

/** 
 * Constants that are agnostic to the year
 */
public class FieldConstants {

    // The height and width of the april tag itself 
    public static final double APRIL_TAG_HEIGHT_METERS = Units.inchesToMeters(6.5); 
    public static final double APRIL_TAG_WIDTH_METERS = Units.inchesToMeters(6.5);

    // Field constants for the 2025 game field "Reefscape"
    public static final Field2025 k2025FieldConstants = new Field2025();
    
    public static Pose3d getTagPose3d(int id, YearFieldConstantable fieldConstants) {
        if (id < fieldConstants.getMinAprilTagID() || id > fieldConstants.getMaxAprilTagID()) {
            final String message = String.format("id must be between 1 and 22 was %d", id);
            throw new IllegalArgumentException(message);
        }

        return fieldConstants.getFieldLayout().getTagPose(id).orElseThrow(() -> {
            final String message = String.format("getTagPose called for unexpected tag %d", id);
            return new RuntimeException(message);
        });
    }

    public static Pose2d getTagPose2d(int id, YearFieldConstantable fieldConstants) {
        return getTagPose3d(id, fieldConstants).toPose2d();
    }
}
