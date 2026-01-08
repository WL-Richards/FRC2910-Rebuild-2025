// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.config.robots.nautilus;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.team6443.lib.config.superstructure.BaseSuperstructureConfiguration;
import com.team6443.lib.config.superstructure.BaseSuperstructureStateCollisionStateable;
import com.team6443.lib.superstructure.SuperstructureStateable;

/**
 * Enum to represent the different collision states that can be encountered when attempting to transiton between staes
 */
enum NautilusSuperstructureCollisionStates implements BaseSuperstructureStateCollisionStateable<NautilusSuperstructureCollisionStates>{
    HOLDING_ALGAE("Algae_Holding")
    ;

    private NautilusSuperstructureCollisionStates(String name){
        this.name = name;
    }

    // --- BaseSuperstructureStateCollisionStateable Implementation
    private final String name;

    @Override
    public String getName() {
        return name;
    }

    // ------ State collision implementation
    private BooleanSupplier isCollidingSupplier = null;

    @Override
    public NautilusSuperstructureCollisionStates withColliderSupplier(BooleanSupplier isCollidingSupplier) {
        this.isCollidingSupplier = isCollidingSupplier;
        return this;
    }

    @Override
    public BooleanSupplier getColliderSupplier() {
        return isCollidingSupplier;
    }
}

/**
 * Enumeration of the different states that this robot can possibly be in
 */
enum NautilusSuperstructureState implements SuperstructureStateable {
    STOW_CORAL("Coral_Stow"),
    STAGE_CORAL_L1("Coral_Stage_L1"),
    STAGE_CORAL_L2("Coral_Stage_L2"),
    STAGE_CORAL_L3("Coral_Stage_L3"),
    STAGE_CORAL_L4("Coral_Stage_L4")
    ;

    /// --- Enum State Information
    public final String kStateName;

    private NautilusSuperstructureState(String stateName){
        this.kStateName = stateName;
    }

    // --- SuperstructureStateable Implementation ---
    private List<SuperstructureStateable> kValidNextStates = new ArrayList<>();
    private List<BaseSuperstructureStateCollisionStateable<?>> kCollisionStates = new ArrayList<>();

    // Configure this enum with specific valid next states
    public NautilusSuperstructureState withValidNextStates(SuperstructureStateable... validNextStates){
        kValidNextStates = List.of(validNextStates);
        return this;
    }

    public NautilusSuperstructureState withCollisionStates(NautilusSuperstructureCollisionStates... collisionStates){
        kCollisionStates = List.of(collisionStates);
        return this;
    }

    @Override
    public List<SuperstructureStateable> getValidNextStates() {
        return kValidNextStates;
    }

    @Override
    public List<BaseSuperstructureStateCollisionStateable<?>> getCollisionStates() {
        return kCollisionStates;
    }

    @Override
    public String getName() {
        return kStateName;
    }

}

/** Superstructure configuration for Nautilus */
public class NautilusSuperstructureConfiguration extends BaseSuperstructureConfiguration<NautilusSuperstructureState>{

    // --- Superstructure Collision States ---
    // Collision state that is active when algae is held
    private static final NautilusSuperstructureCollisionStates kHoldingAlgaeCollisionState = 
        NautilusSuperstructureCollisionStates.HOLDING_ALGAE.
            withColliderSupplier(
                () -> { return true; }
            );  

    // --- States ---

    // ------ Coral States ------
    // // Add stow coral state and allow it to transition to test, all states we can transition too from this state should be listed below
    // private static final NautilusSuperstructureState kCoralStowState = NautilusSuperstructureState.STOW_CORAL
    //     .withValidNextStates(
    //         NautilusSuperstructureState.STAGE_CORAL_L1,
    //     );

    // Add stow coral state and allow it to transition to test, all states we can transition too from this state should be listed below
    private static final NautilusSuperstructureState kCoralStageL1State = NautilusSuperstructureState.STAGE_CORAL_L1
        .withValidNextStates(
            NautilusSuperstructureState.STAGE_CORAL_L2
        )
        .withCollisionStates(                               
            kHoldingAlgaeCollisionState
        );

    // Add stow coral state and allow it to transition to test, all states we can transition too from this state should be listed below
    private static final NautilusSuperstructureState kCoralStageL2State = NautilusSuperstructureState.STAGE_CORAL_L2
        .withValidNextStates(
            NautilusSuperstructureState.STAGE_CORAL_L3
        );

    // Add stow coral state and allow it to transition to test, all states we can transition too from this state should be listed below
    private static final NautilusSuperstructureState kCoralStageL3State = NautilusSuperstructureState.STAGE_CORAL_L3;

    // // Add stow coral state and allow it to transition to test, all states we can transition too from this state should be listed below
    // private static final NautilusSuperstructureState kCoralStageL4State = NautilusSuperstructureState.STAGE_CORAL_L4
    //     .withValidNextStates(
    //         NautilusSuperstructureState.STAGE_CORAL_L3
    //     );
        
    public NautilusSuperstructureConfiguration(String configName){
        super(
            configName,
            // kCoralStowState,
            kCoralStageL1State,
            kCoralStageL2State,
            kCoralStageL3State
            // kCoralStageL4State
        );
    }
}
