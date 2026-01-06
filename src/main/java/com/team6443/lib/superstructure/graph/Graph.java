package com.team6443.lib.superstructure.graph;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;


// 2. The Edge Class
class Edge<T> {
    private final T target;
    private final double weight;

    public Edge(T target, double weight) {
        this.target = target;
        this.weight = weight;
    }

    public T getTarget() { return target; }
    public double getWeight() { return weight; }
}

// 3. Superstructure Graph Class
public class Graph<T> {

    // Adjacency List: Node -> List of Edges
    private final Map<T, List<Edge<T>>> adjacencyList = new HashMap<>();

    // Add a node (optional if you just want to add edges directly)
    public void addNode(T node) {
        adjacencyList.putIfAbsent(node, new ArrayList<>());
    }

    // Get all the nodes within the graph
    public Set<T> getNodes() {
        return adjacencyList.keySet();
    }

    // Get the number of nodes in the graph
    public int getNumNodes(){
        return adjacencyList.size();
    }

    // Get the number of nodes in the graph
    public int getNumEdges(T node){
        return adjacencyList.get(node).size();
    }

    // Add a directed edge
    public void addEdge(T source, T target, double weight) {
        addNode(source);
        addNode(target);
        adjacencyList.get(source).add(new Edge<>(target, weight));
    }

    public List<Edge<T>> getNeighbors(T node) {
        return adjacencyList.getOrDefault(node, Collections.emptyList());
    }
    
}