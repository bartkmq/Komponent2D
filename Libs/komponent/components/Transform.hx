package komponent.components;

import kha.Color;
import kha.math.Vector2;
import kha.Rotation;
import komponent.GameObject;

import komponent.utils.Painter;
import komponent.utils.Screen;

using komponent.utils.Parser;

class Transform extends Component
{
	// in world space
	public var x:Float;
	public var y:Float;
	
	// in local space, offset from the parent
	public var localX:Float;
	public var localY:Float;
	
	public var rotation(get, set):Float;
	
	public var centerX(get, set):Float;
	public var centerY(get, set):Float;
	
	public var scaleX:Float;
	public var scaleY:Float;
	
	public var parent:Transform;
	public var children:Array<Transform>;
	public var root(get, never):Transform;
	
	public var khaRotation:Rotation;

	private function new() 
	{
		x = 0;
		y = 0;
		
		khaRotation = new Rotation(new Vector2(), 0);
		
		scaleX = 1;
		scaleY = 1;
	}
	
	override public function update()
	{
		/*
		if (attachedTransform != null)
		{
			x = attachedTransform.x;
			y = attachedTransform.y;
			rotation = attachedTransform.rotation;
			scaleX = attachedTransform.scaleX;
			scaleY = attachedTransform.scaleY;
		}
		*/
	}
	
	override public function debugDraw()
	{
		Painter.set(Color.fromBytes(0, 0, 255), 1);
		
		if (Screen.camera != null)
			Painter.drawCross(x - Screen.camera.x, y - Screen.camera.y, 10 * Screen.fullScaleX, 10 * Screen.fullScaleY, 2);
		else
			Painter.drawCross(x, y, 10, 10, 2);
	}
	
	public inline function setPos(x:Float, y:Float):Void
	{
		this.x = x;
		this.y = y;
	}
	
	public inline function attachTo(otherGameObject:GameObject):Void
	{
		if (gameObject != null)
		{
			parent = otherGameObject.transform;
		}
	}
	
	public inline function attach(otherGameObject:GameObject):Void
	{
		children.push(otherGameObject.transform);
		otherGameObject.transform.parent = this;
	}
	
	public inline function detach(otherGameObject:GameObject)
	{
		children.remove(otherGameObject.transform);
		otherGameObject.transform.parent = null;
	}
	
	override public function loadConfig(data:Dynamic):Void
	{
		khaRotation = new Rotation(new Vector2(data.centerX.parse(0.0), data.centerY.parse(0.0)), data.rotation.parse(0.0));
		
		x = data.x.parse(0.0);
		y = data.y.parse(0.0);
		scaleX = data.scaleX.parse(1.0);
		scaleY = data.scaleY.parse(1.0);
	}
	
	private inline function get_rotation():Float { return khaRotation.angle; }
	private inline function set_rotation(value:Float):Float { return khaRotation.angle = value; }
	
	private inline function get_centerX():Float { return khaRotation.center.x; }
	private inline function set_centerX(value:Float):Float { return khaRotation.center.x = value; }
	
	private inline function get_centerY():Float { return khaRotation.center.y; }
	private inline function set_centerY(value:Float):Float { return khaRotation.center.y = value; }
	
	private inline function get_root():Transform
	{
		var transform:Transform = this;
		while (transform.parent != null)
		{
			transform = transform.parent;
		}
		return transform;
	}
}