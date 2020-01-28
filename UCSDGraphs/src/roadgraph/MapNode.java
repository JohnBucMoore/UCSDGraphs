package roadgraph;

import java.util.LinkedList;
import java.util.List;

import geography.GeographicPoint;

public class MapNode {
	private GeographicPoint location;
	private List<MapEdge> edges;
	
	public MapNode(GeographicPoint location) {
		this.location = location;
		edges = new LinkedList<MapEdge>();
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
}
