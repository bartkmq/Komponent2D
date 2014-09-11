package komponent.components.physics;

import komponent.components.Collider;
import komponent.utils.Painter;

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
		Painter.drawCircle(x, y, radius);
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