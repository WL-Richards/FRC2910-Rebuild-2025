// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.superstructure.config;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.Logger;

import com.team6443.lib.superstructure.SuperstructureStateable;
import com.team6443.lib.superstructure.graph.AStarSolver;
import com.team6443.lib.superstructure.graph.Graph;
import com.team6443.lib.superstructure.graph.SuperstructureGraphNode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

/** Base class for superstructure configures */
public abstract class BaseSuperstructureConfiguration<SS extends SuperstructureStateable> {

    // If kValidNextStates is empty then all next states are valid
    protected final List<SS> kSuperstructureStates;

    /**
     * Effectively a dicitionary of format {
     *     "default" is when no collision states are active
     *     "CollisionStateName" : {
     *          "FROM_STATE_NAME->TO_STATE_NAME": [
     *              SUPERSTRUCTURE GRAPH NODE
     *          ]
     *      }
     * }
     * Maps a stringified state transition representation "FROM_STATE_NAME->TO_STATE_NAME" to the most optimal set of nodes to achieve this path given the conditions
     */
    protected final Map<String, Map<String, List<SuperstructureGraphNode>>> kStateTransitionToPath = new HashMap<>();
    protected final String kConfigurationName;

    private final String graphFileName;

    // --- Constructors ---
    @SafeVarargs
    public BaseSuperstructureConfiguration(String name, SS... validStates){
        this.kConfigurationName = name;
        this.kSuperstructureStates = Arrays.asList(validStates);

        this.graphFileName = "Superstructure_" + this.kConfigurationName + "_Graph.txt";
        if(!validateStates()){
            throw new IllegalStateException("Bad superstructure state configuration!! One more referenced states in not included in the list of valid states");
        }
    }

    // --- Graph Creation ---
    public Graph<SuperstructureGraphNode> loadGraph(){
        return this.loadGraph(this.graphFileName);
    }

    /**
     * !!!! THIS FUNCTION MAY TAKE SOME TIME TO RUN, THUS LOAD GRAPH SHOULD ONLY BE RUN ON STARTUP !!!! 
     * Load the Superstructure graph from a file passed in and precompute all possible transition configurations
     * @param filePathRelativeToDeploy File path relative to the deploy directory as to where we are loading from
     * @return A object representation of the superstructure graph that can be solved with A*
     */
    public Graph<SuperstructureGraphNode> loadGraph(String filePathRelativeToDeploy){
        File graphFile = new File(Filesystem.getDeployDirectory(), filePathRelativeToDeploy);

        try {
            // 3. Read the file (using standard Java NIO for example)
            if (graphFile.exists()) {
                List<String> lines = Files.readAllLines(graphFile.toPath());

                Graph<SuperstructureGraphNode> superstructureGraph = new Graph<>();

                for (String line : lines) {
                    String[] lineSplit = line.split(" ");

                    // If there is more than one entry we know that its a link
                    if (lineSplit.length > 1){
                        double edgeWeight = Double.parseDouble(lineSplit[2]);

                        SuperstructureGraphNode from = new SuperstructureGraphNode(lineSplit[0]);
                        SuperstructureGraphNode to = new SuperstructureGraphNode(lineSplit[1]);

                        superstructureGraph.addEdge(
                            from,
                            to,
                            edgeWeight
                        );

                        // Log Edge
                        Logger.recordOutput("Superstructure/Graph/Edges/" + from.toString() + "->" + to.toString(), edgeWeight);

                        Logger.recordOutput("Superstructure/Graph/Nodes/" + from.toString() + "/NumEdges", superstructureGraph.getNumEdges(from));
                        Logger.recordOutput("Superstructure/Graph/Nodes/" + to.toString() + "/NumEdges", superstructureGraph.getNumEdges(to));
                    }

                    // If there is only 1 or less entries then we know this is just a node definition
                    else if (lineSplit.length == 1){
                        superstructureGraph.addNode(new SuperstructureGraphNode(lineSplit[0]));


                        Logger.recordOutput("Superstructure/Graph/Nodes/" + lineSplit[0], true);
                    }
                }

                // !!!! THIS FUNCTION MAY TAKE SOME TIME TO RUN, THUS LOAD GRAPH SHOULD ONLY BE RUN ON STARTUP !!!! 
                precacheSuperstructureStateTransitions(superstructureGraph);
                return superstructureGraph;
            } else {
                DriverStation.reportError("File not found in deploy directory!", false);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        return null;
    }

    private void precacheSuperstructureStateTransitions(Graph<SuperstructureGraphNode> superstructureGraph){
        AStarSolver<SuperstructureGraphNode> solver = new AStarSolver<>();

        // Really just makes it Dijkstra's algo but its chill
        AStarSolver.Heuristic<SuperstructureGraphNode> aStarHeuristic = (from, to) -> 0.0;

        kStateTransitionToPath.putIfAbsent("default", new HashMap<>());
        for (SS stateA : kSuperstructureStates) {
            for (SS stateB : kSuperstructureStates) {
                // Skip if it is the exact same instance
                if (stateA == stateB) { 
                    continue;
                }

                // Solve the path and
                String stateTransitionName = stateA.getName() + "->" + stateB.getName();

                List<SuperstructureGraphNode> path = solver.findPath(
                    superstructureGraph, 
                    new SuperstructureGraphNode(stateA.getName()), 
                    new SuperstructureGraphNode(stateB.getName()), 
                    aStarHeuristic
                );

                kStateTransitionToPath.get("default").putIfAbsent(
                    stateTransitionName, 
                    // Use path if has solve if not its just null
                    path.size() > 0 ? path : null
                );


                Logger.recordOutput("Superstructure/Graph/Routes/default/" + stateTransitionName, path.size() > 0);
                for(SuperstructureGraphNode node : path){
                    Logger.recordOutput("Superstructure/Graph/Routes/default/" + stateTransitionName + "/" + node.toString(), "");
                }
            }
        }
    }

    /**
     * Generate a template for the superstructure graph representation, output is easily visualized here: https://csacademy.com/app/graph_editor/,
     * Use directed mode and just copy paste the output file contents into the "Graph Data" section
     * (you can use the config on the site to make edge lengths longer and change the node color/text color) 
     */
    public void generateSuperstructureGraphTemplate(){
        // Only generate the template if we are running in simulation
        if (RobotBase.isSimulation()) {
            String nodes = "";
            String connections = "";

            // Build the graph representation
            for(SS state : kSuperstructureStates){
                nodes += state.getName() + "\n";

                for (SuperstructureStateable nextState : state.getValidNextStates()){
                    connections += state.getName() + " " + nextState.getName() + " 1\n";
                }
            }

            try {
                // Saves to project root directory
                PrintWriter writer = new PrintWriter(new FileWriter("src/main/deploy/" + this.graphFileName, false));
                writer.print(nodes + connections);
                writer.flush(); // Ensure data is written immediately
                writer.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        else{
            DriverStation.reportWarning("Attempted to generate superstructure graph template on live robot, this action was blocked!", false);
        }
        
    }

    private boolean validateStates(){

        // ----------------- Check 1: Ensure that connected states are in valid states -----------------
        for(SS state : kSuperstructureStates){
            for (SuperstructureStateable connectedState : state.getValidNextStates()){

                // If connected state is not in total states list we want to fail validation
                if (!kSuperstructureStates.contains(connectedState)){
                    return false;
                }
            }
        }

        return true;
    }
}
