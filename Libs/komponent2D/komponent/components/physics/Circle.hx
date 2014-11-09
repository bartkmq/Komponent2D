package komponent.components.physics;

import kha.Color;

import komponent.components.Collider;
import komponent.utils.Painter;
import komponent.utils.Painter2;
import komponent.utils.Screen;
import komponent.utils.Misc;

class Circle extends Collider
{

	public var radius(default, null):Float;
	
	override public function added()
	{
		super.added();
		setRadius(25);
	}
	
	override public function debugDraw():Void
	{	
		Painter.set(Color.White, 1);
		for (camera in Screen.cameras)
		{
			if (!Painter.fallbackPainter)
			{
				Painter.matrix = transform.matrix * camera.matrix;
				Painter.drawCircle(0, 0, radius);
			}
			else
			{
				Painter2.camera = camera;
				Painter2.drawCross(transform.x, transform.y, 10, 10, 2);
				Painter2.drawCircle(transform.x, transform.y, radius);
			}
		}
		Painter.matrix = null;
	}
	
	public inline function setRadius(radius:Float):Void
	{
		shape = new HxCircle(transform.x, transform.y, radius);
		this.radius = radius;
	}
	
	override public function loadConfig(data:Dynamic):Void
	{
		super.loadConfig(data);
		setRadius(data.radius.parse(25.0));
	}
	
}