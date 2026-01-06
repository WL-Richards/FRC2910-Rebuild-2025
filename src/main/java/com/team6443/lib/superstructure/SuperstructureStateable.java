// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.superstructure;

import java.util.List;

import com.team6443.lib.superstructure.config.BaseSuperstructureStateCollisionStateable;

/** 
 * Interface to be used with a super structure state to provide basic information
 */
public interface SuperstructureStateable {

    /**
     * What states can this state actually transition to from here
     * @return List of SuperstructureStates that we can transition to from the current
     */
    public List<SuperstructureStateable> getValidNextStates();

    /**
     * Get list of collision states that this superstructure state has
     * @return List of possible state collisions we could run into
     */
    public List<BaseSuperstructureStateCollisionStateable<?>> getCollisionStates();

    /*
     * Get the name of this superstructure state
     */
    public String getName();
}
