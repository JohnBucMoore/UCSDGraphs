/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	private List<MapNode> vertices; //adjacencyList
	private HashMap<GeographicPoint, List<MapNode>> nodeMap;
	private List<MapEdge> edges;
	
	//pointMap
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		vertices = new ArrayList<MapNode>();
		nodeMap = new HashMap<GeographicPoint, List<MapNode>>();
		edges = new LinkedList<MapEdge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return vertices.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		Set<GeographicPoint> intersections = new HashSet<GeographicPoint>();
		for (MapNode vertice : vertices) {
			intersections.add(vertice.getLocation());
		}
		return intersections;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return edges.size();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if (!nodeMap.containsKey(location) && location != null) {
			MapNode vertex = new MapNode(location);
			int v = getNumVertices();
			vertices.add(v,vertex);
			nodeMap.put(location, new ArrayList());
			return true;
		}
		return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		if (!(nodeMap.containsKey(from) && nodeMap.containsKey(to))) {
			throw new IllegalArgumentException();
		}
		MapEdge edge = new MapEdge(from, to, roadName, roadType, length);
		edges.add(edge);
		MapNode f = null;
		MapNode t = null;
		for (MapNode vertex : vertices) {
			if (vertex.getLocation().equals(from)) {
				f = vertex;
			}
			if (vertex.getLocation().equals(to)) {
				t = vertex;
			}
			if (f != null && t != null) {
				break;
			}
		}
		if (t != null) {
			nodeMap.get(from).add(t);
		}
		if (f != null) {
			f.addEdge(edge);
		}
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		Queue<GeographicPoint> q = new LinkedList<>();
		Set<GeographicPoint> visited = new HashSet<>();
		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
		q.add(start);
		visited.add(start);
		GeographicPoint curr = start;
		boolean found = false;
		
		while (!q.isEmpty()) {
			curr = q.remove();
			nodeSearched.accept(curr);
			if (curr.equals(goal)) {
				found = true;
				break;
			}
			
			List<MapNode> neighbors = nodeMap.get(curr);
			for (MapNode n : neighbors) {
				if (!visited.contains(n.getLocation())) {
					visited.add(n.getLocation());
					parentMap.put(n.getLocation(), curr);
					q.add(n.getLocation());
					
				}
			}
		}

		if (!found) {
			System.out.println("No path exists");
			return new LinkedList<GeographicPoint>();
		}

		// reconstruct the path
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		GeographicPoint currpt = goal;
		while (currpt != start) {
			path.addFirst(currpt);
			currpt = parentMap.get(currpt);
		}
		path.addFirst(start);
		return path;
	}
	

	public HashMap<MapNode, Double> initializeDistances(GeographicPoint start) {
		HashMap<MapNode, Double> distances = new HashMap<>();
		for (MapNode vertex : vertices) {
			if (!vertex.getLocation().equals(start)) {
				vertex.setDist(Double.POSITIVE_INFINITY);
				distances.put(vertex, Double.POSITIVE_INFINITY);
			} else {
				vertex.setDist(0.0);
				distances.put(vertex, 0.0);
			}
		}
		return distances;
	}
	
	public void updateDistance(Double newDist, MapNode neighborNode, MapNode endNode, Map<MapNode, Double> currDistFromStart) {
			neighborNode.setDist(newDist + heuristicEstimatedCost(neighborNode, endNode));
			currDistFromStart.replace(neighborNode, newDist);
	}
	
	/** Reconstruct a path from start to goal using the parentMap
	 *
	 * @param parentMap the HashNode map of children and their parents
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	private List<GeographicPoint>
	reconstructPath(HashMap<MapNode,MapNode> parentMap,
					MapNode start, MapNode goal)
	{
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode current = goal;

		while (!current.equals(start)) {
			path.addFirst(current.getLocation());
			current = parentMap.get(current);
		}

		// add start
		path.addFirst(start.getLocation());
		return path;
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		PriorityQueue<MapNode> q = new PriorityQueue<MapNode>();
		Set<MapNode> visited = new HashSet<>();
		HashMap<MapNode, MapNode> parentMap = new HashMap<>();
		List<GeographicPoint> path = new LinkedList<>();
		Map<MapNode, Double> currDistFromStart = initializeDistances(start);
		MapNode s = null;
		MapNode g = null;
		for (MapNode vertex : vertices) {
			if (vertex.getLocation().equals(start)) {
				s = vertex;
			} else if (vertex.getLocation().equals(goal)) {
				g = vertex;
			}
		}
		q.add(s);
		MapNode curr = null;
		
		while(!q.isEmpty()) {
			curr = q.poll();
			if(!visited.contains(curr)) {
				visited.add(curr);
				GeographicPoint loc = curr.getLocation();
				nodeSearched.accept(loc);
				if(loc.equals(goal)) {
					System.out.println("Total number visited: "+visited.size());
					path = reconstructPath(parentMap, s, g);
					return path;
				}
				double currDist = currDistFromStart.get(curr);
				List<MapEdge> neighbors = curr.getEdges();
				List<MapNode> nNodes = nodeMap.get(curr.getLocation());
				for(MapEdge neighbor : neighbors) {
					GeographicPoint nGP = neighbor.getEnd();
					MapNode n = null;
					for (MapNode node : nNodes) {
						if (node.getLocation().equals(nGP)) {
							n = node;
						}
					}
					if (!visited.contains(n)) {
						double neighborDist = currDist + neighbor.getDistance();
						 if(neighborDist < currDistFromStart.get(n)) {
							updateDistance(neighborDist, n, n, currDistFromStart);
						 	parentMap.put(n, curr);
						 	q.add(n);
						 }
					}
				}
			}
		}
		
		return null;
	}

	public double heuristicEstimatedCost(MapNode n, MapNode g) {
		return n.getLocation().distance(g.getLocation());
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		PriorityQueue<MapNode> q = new PriorityQueue<MapNode>();
		Set<MapNode> visited = new HashSet<>();
		HashMap<MapNode, MapNode> parentMap = new HashMap<>();
		List<GeographicPoint> path = new LinkedList<>();
		Map<MapNode, Double> currDistFromStart = initializeDistances(start);
		MapNode s = null;
		MapNode g = null;
		for (MapNode vertex : vertices) {
			if (vertex.getLocation().equals(start)) {
				s = vertex;
			} else if (vertex.getLocation().equals(goal)) {
				g = vertex;
			}
		}
		
		s.setDist(heuristicEstimatedCost(s,g));
		q.add(s);
		MapNode curr = null;
		
		while(!q.isEmpty()) {
			curr = q.poll();
			if(!visited.contains(curr)) {
				visited.add(curr);
				GeographicPoint loc = curr.getLocation();
				nodeSearched.accept(loc);
				if(loc.equals(goal)) {
					System.out.println("Total number visited: "+visited.size());
					path = reconstructPath(parentMap, s, g);
					return path;
				}
				double currDist = currDistFromStart.get(curr);
				List<MapEdge> neighbors = curr.getEdges();
				List<MapNode> nNodes = nodeMap.get(curr.getLocation());
				for(MapEdge neighbor : neighbors) {
					GeographicPoint nGP = neighbor.getEnd();
					MapNode n = null;
					for (MapNode node : nNodes) {
						if (node.getLocation().equals(nGP)) {
							n = node;
						}
					}
					if (!visited.contains(n)) {
						double neighborDist = currDist + neighbor.getDistance();
						 if(neighborDist < currDistFromStart.get(n)) {
							updateDistance(neighborDist, n, g, currDistFromStart);
						 	parentMap.put(n, curr);
						 	q.add(n);
						 }
					}
				}
			}
		}
		
		return null;
	}

	
	public void printNodeMap() {
		for (GeographicPoint pt : nodeMap.keySet()) {
			List<MapNode> nodes = nodeMap.get(pt);
			String s = "";
			
			for (MapNode node : nodes) {
				
				s += node.getLocation()+" | ";
			}
			System.out.println(pt+"\t"+s);
		}
	}
	
	public void printEdges() {
		for (MapEdge edge : edges) {
			System.out.println(edge.getStart() + " | " + edge.getEnd() + "\t" + edge.getDistance());
		}
	}
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		System.out.println(simpleTestMap.getNumVertices());
		System.out.println(simpleTestMap.getNumEdges());
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		List<GeographicPoint> testroute = simpleTestMap.bfs(testStart,testEnd);
		System.out.println(testroute);
		
		Set<GeographicPoint> v = simpleTestMap.getVertices();
		
		simpleTestMap.printNodeMap();
		simpleTestMap.printEdges();
		// You can use this method for testing.  
		*/
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		System.out.println(simpleTestMap.getVertices());
		System.out.println(simpleTestMap.getNumVertices());
		System.out.println(simpleTestMap.getNumEdges());
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println(testStart);
		System.out.println(testEnd);
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);

		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);

		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
