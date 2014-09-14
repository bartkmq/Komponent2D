package komponent.components.physics;

import kha.Color;

import komponent.components.graphic.Image;
import komponent.components.Collider;
import komponent.utils.Painter;
import komponent.ds.Point;
import komponent.ds.Matrix;
import komponent.utils.Screen;
import komponent.utils.Misc;
import komponent.components.misc.Camera;

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
		//var vertices:Array<Point> = cast(shape, HxPolygon).vertices;
		
		Painter.set(Color.White, 1);
		for (camera in Screen.cameras)
		{
			Painter.matrix = /*transform.matrix */ camera.matrix;
			Painter.drawPolygon(0, 0, Misc.pointsToVector2(vertices));
		}
		Painter.matrix = null;
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