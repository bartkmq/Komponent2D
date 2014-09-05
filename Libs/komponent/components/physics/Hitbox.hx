package komponent.components.physics;

import komponent.components.graphic.Image;
import komponent.components.Collider;
import komponent.utils.Painter;

typedef HxPolygon = hxcollision.shapes.Polygon;

class Hitbox extends Collider
{

	public var width(default, null):Float;
	public var height(default, null):Float;
	
	override public function added()
	{
		super.added();
		setSize(50, 50);
	}
	
	override public function debugDraw():Void
	{	
		Painter.drawRect2(shape.x, shape.y, width, height, shape.rotation);
	}
	
	public inline function setSize(width:Float, height:Float):Void
	{
		shape = HxPolygon.rectangle(0, 0, width, height);
		this.width = width;
		this.height = height;
	}
	
	public inline function setTo(image:Image):Void
	{
		setSize(image.width, image.height);
	}
	
	override public function loadConfig(data:Dynamic)
	{
		super.loadConfig(data);
		setSize(data.width.parse(25.0), data.height.parse(25.0));
	}
	
}