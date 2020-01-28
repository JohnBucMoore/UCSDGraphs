package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	private GeographicPoint start;
	private GeographicPoint end;
	private String streetName;
	private String streetType;
	private double distance;
	
	public MapEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) {
		this.start = from;
		this.end = to;
		this.streetName = roadName;
		this.streetType = roadType;
		this.distance = length;
	}
	
	public GeographicPoint getStart() {
		return start;
	}
	
	public GeographicPoint getEnd() {
		return end;
	}
	
	public String getStreetName() {
		return streetName;
	}
	
	public String getStreetType() {
		return streetType;
	}
	
	public double getDistance() {
		return distance;
	}
}
