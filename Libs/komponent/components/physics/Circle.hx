package komponent.components.physics;

import kha.Color;

import nape.phys.BodyType;
import nape.geom.Vec2;
import nape.shape.Circle in NapeCircle;

import komponent.components.Collider;
import komponent.extension.Nape;
import komponent.utils.Painter;
import komponent.utils.Screen;

using komponent.utils.Parser;

class Circle extends Collider
{
	
	public var radius(default, null):Float;
	
	public inline function new()
	{
		radius = 0;
	}
	
	override private function setDefaultShape():Void
	{
		setRadius(25);
	}
	
	override public function debugDraw():Void
	{
		super.debugDraw();
		Painter.set(Color.fromBytes(91, 194, 54), 1);
		
		if (Screen.camera != null)
			Painter.drawCircle(shape.worldCOM.x - Screen.camera.x, shape.worldCOM.y - Screen.camera.y, radius * Screen.fullScaleX, Screen.fullScaleX, Screen.fullScaleY);
		else
			Painter.drawCircle(shape.worldCOM.x, shape.worldCOM.y, radius);
	}
	
	public inline function setRadius(radius:Float):Void
	{
		if (shape != null)
			shape.body = null;
		shape = new NapeCircle(radius);
		this.radius = radius;
		shape.body = body;
	}
	
	override public function loadConfig(data:Dynamic):Void
	{
		super.loadConfig(data);
		setRadius(data.radius.parse(25.0));
	}
	
}