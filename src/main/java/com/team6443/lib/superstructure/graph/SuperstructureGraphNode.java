// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.superstructure.graph;

import java.util.Objects;

/** A node in the superstructure graph to represent a single state */
public class SuperstructureGraphNode {
    public final String kStateName;         // The name of this node also the name of the state within the superstructure state machine

    public SuperstructureGraphNode(String stateName){
        this.kStateName = stateName;
    }

    // Use the state name as the hash for the map
    @Override
    public int hashCode() {
        return Objects.hash(kStateName); 
    }

    @Override
    public boolean equals(Object o) {
        // If it's the exact same object in memory, return true
        if (this == o) return true;
        // If it's null or a different class type, return false
        if (o == null || getClass() != o.getClass()) return false;
        
        // Cast and compare the state name
        SuperstructureGraphNode that = (SuperstructureGraphNode) o;
        return Objects.equals(kStateName, that.kStateName);
    }

    // 3. Optional but highly recommended for debugging (Logger output)
    @Override
    public String toString() {
        return kStateName;
    }
}
