package komponent.components.physics;

import hxcollision.shapes.Polygon in hxPolygon;

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
		/*
		var vertices:Array<Point> = cast(shape, hxPolygon).transformedVertices;
		var count:Int = vertices.length;
		
		Painter.drawLine2(vertices[0].x, vertices[0].y, vertices[1].x, vertices[1].y);
		for (i in 1...count - 1)
			Painter.drawLine2(vertices[i].x, vertices[i].y, vertices[i + 1].x, vertices[i + 1].y);
		Painter.drawLine2(vertices[count].x, vertices[count].y, vertices[0].x, vertices[0].y);
		*/
		
	}
	
	public function setVertices(vertices:Array<Point>):Void
	{
		shape = new hxPolygon(0, 0, vertices);
	}
	
	public function setToRegularPolygon(radius:Float, edges:Int):Void
	{
		shape = new hxPolygon.create(0, 0, radius, edges);
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