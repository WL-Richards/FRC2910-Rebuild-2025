package com.team6443.lib.superstructure.config;

import java.util.function.BooleanSupplier;

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
