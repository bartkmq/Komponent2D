package komponent.components.physics;

import kha.Color;

import hxcollision.shapes.Polygon in HxPolygon;

import komponent.components.Collider;
import komponent.ds.Point;

class Polygon extends Collider
{
	
	override public function added()
	{
		super.added();
		setToRegularPolygon(25, 5);
	}
	
	public function setVertices(vertices:Array<Point>):Void
	{
		shape = new HxPolygon(transform.x, transform.y, vertices);
	}
	
	public function setToRegularPolygon(radius:Float, edges:Int):Void
	{
		shape = HxPolygon.create(transform.x, transform.y, edges, radius);
	}
	
	override public function loadConfig(data:Dynamic)
	{
		super.loadConfig(data);
		if (data.regular != null)
		{
			setToRegularPolygon(data.radius.parse(25.0), data.edges.parse(5));
		}
		else
		{
			var vertices:Array<Float> = data.vertices;
			if (vertices == null || (vertices.length % 2) != 0)
				trace("Error loading Polygon vertices");
			else
			{
				var points:Array<Point> = [];
				for (n in 0...Std.int(vertices.length / 2))
				{
					points.push(new Point(vertices[n], vertices[n + 1]));
				}
				setVertices(points);
			}
		}
	}
	
}