package com.team6443.lib.superstructure.graph;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;



public class AStarSolver<T> {

    // Functional Interface for the Heuristic (h cost)
    @FunctionalInterface
    public static interface Heuristic<T> {
        double calculate(T current, T goal);
    }

    // Internal wrapper to prioritize nodes in the Open Set
    private static class NodeWrapper<T> implements Comparable<NodeWrapper<T>> {
        private final T node;
        private final double fScore; // f = g + h

        public NodeWrapper(T node, double fScore) {
            this.node = node;
            this.fScore = fScore;
        }

        @Override
        public int compareTo(NodeWrapper<T> other) {
            return Double.compare(this.fScore, other.fScore);
        }
    }

    public List<T> findPath(Graph<T> graph, T start, T goal, Heuristic<T> heuristic) {
        // Priority Queue for Open Set (ordered by lowest fScore)
        PriorityQueue<NodeWrapper<T>> openSet = new PriorityQueue<>();
        
        // Maps to track path and costs
        Map<T, T> cameFrom = new HashMap<>();
        Map<T, Double> gScore = new HashMap<>(); // Cost from start to node

        // Initialize
        gScore.put(start, 0.0);
        openSet.add(new NodeWrapper<>(start, heuristic.calculate(start, goal)));

        while (!openSet.isEmpty()) {
            // Get node with lowest fScore
            T current = openSet.poll().node;

            // Goal Reached
            if (current.equals(goal)) {
                return reconstructPath(cameFrom, current);
            }

            // Explore neighbors
            for (Edge<T> edge : graph.getNeighbors(current)) {
                T neighbor = edge.getTarget();
            

                double tentativeG = gScore.get(current) + edge.getWeight();

                // If we found a cheaper path to this neighbor
                if (tentativeG < gScore.getOrDefault(neighbor, Double.POSITIVE_INFINITY)) {
                    // Update path and scores
                    cameFrom.put(neighbor, current);
                    gScore.put(neighbor, tentativeG);
                    
                    double fScore = tentativeG + heuristic.calculate(neighbor, goal);
                    
                    // Add to Open Set (Duplication is handled by processing usually, 
                    // or you can implement remove/update logic if strict optimization is needed)
                    openSet.add(new NodeWrapper<>(neighbor, fScore));
                }
            }
        }

        // Return empty list if no path found
        return Collections.emptyList();
    }

    private List<T> reconstructPath(Map<T, T> cameFrom, T current) {
        List<T> path = new ArrayList<>();
        path.add(current);
        while (cameFrom.containsKey(current)) {
            current = cameFrom.get(current);
            path.add(current);
        }
        Collections.reverse(path);
        return path;
    }
}