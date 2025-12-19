package com.team6443.lib.constants.interfaces;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

/**
 * Interface to define common info getters for year agnostic constant retrieval
 */
public interface YearFieldConstantable {
    public int getMaxAprilTagID();
    public int getMinAprilTagID();

    public AprilTagFieldLayout getFieldLayout();

}