package com.team6443.lib.constants.fields;

import com.team6443.lib.constants.fields.interfaces.YearFieldConstantable;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


public class Field2025 implements YearFieldConstantable{
    /**
     * Define layout of the april tags on the given field
     */
    public final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // --- April Tags ---

    // April tag IDs
    public final int RED_LEFT_CORAL_STATION = 1;
    public final int RED_RIGHT_CORAL_STATION = 2;
    public final int RED_PROCESSOR = 3;
    public final int RED_RIGHT_NET = 4;
    public final int RED_LEFT_NET = 5;
    public final int RED_REEF_LEFT_DRIVER_STATION = 6;
    public final int RED_REEF_CENTER_DRIVER_STATION = 7;
    public final int RED_REEF_RIGHT_DRIVER_STATION = 8;
    public final int RED_REEF_RIGHT_BARGE = 9;
    public final int RED_REEF_CENTER_BARGE = 10;
    public final int RED_REEF_LEFT_BARGE = 11;
    public final int BLUE_RIGHT_CORAL_STATION = 12;
    public final int BLUE_LEFT_CORAL_STATION = 13;
    public final int BLUE_LEFT_BARGE = 14;
    public final int BLUE_RIGHT_BARGE = 15;
    public final int BLUE_PROCESSOR = 16;
    public final int BLUE_REEF_RIGHT_DRIVER_STATION = 17;
    public final int BLUE_REEF_CENTER_DRIVER_STATION = 18;
    public final int BLUE_REEF_LEFT_DRIVER_STATION = 19;
    public final int BLUE_REEF_RIGHT_BARGE = 20;
    public final int BLUE_REEF_CENTER_BARGE = 21;
    public final int BLUE_REEF_LEFT_BARGE = 22;

    public final int MIN_APRIL_TAG_ID = RED_LEFT_CORAL_STATION;
    public final int MAX_APRIL_TAG_ID = BLUE_REEF_LEFT_BARGE;

    public Field2025(){
        APRIL_TAG_FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    }

    @Override
    public int getMinAprilTagID() {
        return MIN_APRIL_TAG_ID;
    }

    @Override
    public int getMaxAprilTagID() {
       return MAX_APRIL_TAG_ID;
    }
   
    @Override
    public AprilTagFieldLayout getFieldLayout() {
        return APRIL_TAG_FIELD_LAYOUT;
    }
}
