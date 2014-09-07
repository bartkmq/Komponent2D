package komponent.components.physics;

import komponent.components.graphic.Image;
import komponent.components.Collider;
import komponent.utils.Painter;
import komponent.ds.Point;

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
	
	public inline function setSize(width:Float, height:Float):Void
	{
		shape = HxPolygon.rectangle(transform.x, transform.y, width, height);
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