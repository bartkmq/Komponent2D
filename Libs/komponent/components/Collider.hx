package komponent.components;

import kha.Color;

import nape.phys.Material;
import nape.phys.Body;
import nape.shape.Shape;
import nape.phys.BodyType;
import nape.geom.Vec2;
import nape.space.Space;

import komponent.extension.Nape;
import komponent.utils.Painter;
import komponent.utils.Screen;

using komponent.utils.Parser;

typedef EnterCallback = Collider -> Collider -> Void;
typedef ExitCallback = Collider -> Collider -> Void;
typedef StayCallback = Collider -> Collider -> Void;

class Collider extends Component
{
	
	public var centerX(get, set):Float;
	public var centerY(get, set):Float;
	
	public var body:Body;
	public var shape:Shape;
	public var material(get, set):Material;
	
	public var isTrigger(get, set):Bool;
	
	override public function added():Void
	{
		checkBody();
		setDefaultShape();
	}
	
	override public function debugDraw():Void
	{
		Painter.set(Color.fromBytes(91, 194, 54), 1); // green
		for (camera in Screen.cameras)
		{
			Painter.camera = camera;
			Painter.drawCross(shape.worldCOM.x, shape.worldCOM.y, 10, 10, 2);
		}
	}
	
	public inline function setCenter(x:Float, y:Float)
	{
		shape.localCOM.setxy(x, y);
	}
	
	private inline function checkBody():Void
	{	
		var nape = scene.getExtension(Nape);
		body = nape.bodies[gameObject];
		
		if (body == null)
		{
			var _transform = transform;
			body = new Body(BodyType.KINEMATIC, Vec2.weak(_transform.x, _transform.y));
			body.space = nape.space;
			nape.bodies[gameObject] = body;
		}
		else
		{
			if (!hasComponent(Physics))
				body.type = BodyType.KINEMATIC;
		}
	}
	
	override public function loadConfig(data:Dynamic):Void
	{
		centerX = data.centerX.parse(0.0);
		centerY = data.centerY.parse(0.0);
		isTrigger = data.isTrigger.parse(true);
	}
	
	private function setDefaultShape():Void { }
	
	private inline function get_isTrigger():Bool { return shape.sensorEnabled; }
	private inline function set_isTrigger(value:Bool):Bool { return shape.sensorEnabled = value; }
	
	private inline function get_centerX():Float { return shape.localCOM.x; }
	private inline function set_centerX(value:Float):Float { return shape.localCOM.x = value; }
	
	private inline function get_centerY():Float { return shape.localCOM.y; }
	private inline function set_centerY(value:Float):Float { return shape.localCOM.y = value; }
	
	private inline function get_material():Material { return shape.material; }
	private inline function set_material(value:Material):Material { return shape.material = value; }
}