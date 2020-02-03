package roadgraph;

import java.util.LinkedList;
import java.util.List;

import geography.GeographicPoint;

public class MapNode implements Comparable<Object>{
	private GeographicPoint location;
	private List<MapEdge> edges;
	private double distanceFrom;
	
	public MapNode(GeographicPoint location) {
		this.location = location;
		edges = new LinkedList<MapEdge>();
		distanceFrom = 0.0;
	}
	
	public GeographicPoint getLocation() {
		return this.location;
	}
	
	public boolean addEdge(MapEdge edge) {
		if (edge != null) {
			edges.add(edge);
			return true;
		}
		return false;
	}
	
	public List<MapEdge> getEdges() {
		return this.edges;
	}
	
	public void setDist(double dist) {
			distanceFrom = dist;
	}
	
	public double getDist() {
		return distanceFrom;
	}
	
	public int compareTo(Object o) {
		if (this.getDist() < ((MapNode) o).getDist()) {
			return -1;
		} else if (this.getDist() > ((MapNode) o).getDist()) {
			return 1;
		} else {
			return 0;
		}
	}
}
