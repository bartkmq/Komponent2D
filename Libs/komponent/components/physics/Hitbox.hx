package komponent.components.physics;

import kha.Color;

import nape.phys.BodyType;
import nape.geom.Vec2;
import nape.shape.Polygon in NapePolygon;

import komponent.components.Collider;
import komponent.components.graphic.Image;
import komponent.extension.Nape;
import komponent.utils.Painter;
import komponent.utils.Screen;

using komponent.utils.Parser;


class Hitbox extends Collider
{
	
	public var width(default, null):Float;
	public var height(default, null):Float;
	
	public function new()
	{
		width = 0;
		height = 0;
	}
	
	override private function setDefaultShape():Void
	{
		setSize(50, 50);
	}
	
	override public function debugDraw():Void
	{	
		super.debugDraw();
		var polygon:NapePolygon = cast shape;
		Painter.set(Color.fromBytes(91, 194, 54), 1);
		
		if (Screen.camera != null)
			Painter.drawPolygon(polygon, -Screen.camera.x, -Screen.camera.y, Screen.fullScaleX, Screen.fullScaleY);
		else
			Painter.drawPolygon(polygon);
	}
	
	public inline function setSize(width:Float, height:Float):Void
	{
		var _centerX = 0.0;
		var _centerY = 0.0;
		
		if (shape != null)
		{
			_centerX = centerX;
			_centerY = centerY;
			shape.body = null;
		}
		
		shape = new NapePolygon(NapePolygon.box(width, height, true));
		this.width = width;
		this.height = height;
		shape.body = body;
		setCenter(_centerX, _centerY);
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