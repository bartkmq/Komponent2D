package komponent.components.physics;

import kha.Color;

import komponent.components.Collider;
import komponent.utils.Painter;
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
			Painter.matrix = transform.matrix * camera.matrix;
			Painter.drawCircle(0, 0, radius);
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