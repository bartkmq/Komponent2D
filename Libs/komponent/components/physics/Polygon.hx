package komponent.components.physics;

import kha.Color;

import nape.geom.GeomPoly;
import nape.geom.GeomPolyList;
import nape.phys.BodyType;
import nape.geom.Vec2;
import nape.shape.Polygon in NapePolygon;
 
import komponent.components.Collider;
import komponent.extension.Nape;
import komponent.utils.Painter;
import komponent.utils.Screen;

using komponent.utils.Parser;

class Polygon extends Collider
{
	
	override public function debugDraw()
	{
		super.debugDraw();
		
		Painter.set(Color.fromBytes(91, 194, 54), 1);
		for (camera in Screen.cameras)
		{
			Painter.camera = camera;
			Painter.drawPolygon(cast shape, -Screen.camera.x, -Screen.camera.y);
		}
	}
	
	override private inline function setDefaultShape()
	{
		setToRegularPolygon(50, 50, 5);
	}
	
	public inline function setVertices(vertices:Array<Vec2>):Void
	{
		if (shape != null)
			shape.body = null;
		shape = new NapePolygon(vertices);
		shape.body = body;
	}
	
	public inline function setToRegularPolygon(RadiusX:Float, RadiusY:Float, edges:Int, angle:Float = 0):Void
	{
		if (shape != null)
			shape.body = null;
		shape = new NapePolygon(NapePolygon.regular(RadiusX, RadiusY, edges, angle, true));
		shape.body = body;
	}
	
	override public function loadConfig(data:Dynamic)
	{
		super.loadConfig(data);
		if (data.regular != null)
		{
			setToRegularPolygon(data.radiusX.parse(25.0), data.radiusY.parse(25.0), data.edges.parse(5), data.angle.parse(0.0));
		}
		else
		{
			var vertices:Array<Float> = data.vertices;
			if (vertices == null || (vertices.length % 2) != 0)
				trace("Error loading Polygon vertices");
			else
			{
				var points:Array<Vec2> = [];
				for (n in 0...Std.int(vertices.length / 2))
				{
					points.push(Vec2.weak(vertices[n], vertices[n + 1]));
				}
				setVertices(points);
			}
		}
	}
	
}