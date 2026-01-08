package com.team6443.lib.config.superstructure;

import java.util.function.BooleanSupplier;

/**
 * Defines the basis for a collision state that could occur with in the superstructure system
 */
public interface BaseSuperstructureStateCollisionStateable<T> {

    public T withColliderSupplier(BooleanSupplier isCollidingSupplier);
    public BooleanSupplier getColliderSupplier();

    public String getName();

    /**
     * Check if this collision state is actively occurring
     * @return True if isCollidingSupplier is not null and collision is occurring, false otherwise
     */
    public default boolean isColliding(){
        if(getColliderSupplier() != null){
            return getColliderSupplier().getAsBoolean();
        }

        return false;
    }
}
