package komponent.components.physics;

import hxcollision.shapes.Polygon in HxPolygon;

import komponent.components.Collider;
import komponent.utils.Painter;
import komponent.ds.Point;

class Polygon extends Collider
{
	
	override public function added()
	{
		super.added();
		setToRegularPolygon(25, 25, 5);
	}
	
	override public function debugDraw()
	{
		var vertices:Array<Point> = cast(shape, HxPolygon).transformedVertices;
		var i = 0;
		var v0 = vertices[i++];
		var v1 = v0;
		
		while (vertices[i] != null)
		{
			var v2 = vertices[i++];
			Painter.drawLine2(v1.x, v1.y, v2.x, v2.y, 1);
			v1 = v2;
		}
		Painter.drawLine2(v1.x, v1.y, v0.x, v0.y, 1);
	}
	
	public function setVertices(vertices:Array<Point>):Void
	{
		shape = new hxPolygon(transform.x, transform.y, vertices);
	}
	
	public function setToRegularPolygon(radius:Float, edges:Int):Void
	{
		shape = new hxPolygon.create(transform.x, transform.y, radius, edges);
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