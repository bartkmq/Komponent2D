package komponent.components;

import kha.Color;

import nape.geom.Vec2;
import nape.phys.Material;
import nape.phys.BodyType;
import nape.phys.Body;
import nape.shape.Polygon;
import nape.shape.Shape;

import komponent.extension.Nape;
import komponent.utils.Painter;
import komponent.utils.Screen;

using nape.hacks.ForcedSleep;
using komponent.utils.Parser;

class Physics extends Component
{

	public var velocityX(get, set):Float;
	public var velocityY(get, set):Float;

	public var angularVelocity(get, set):Float;

	public var mass(get, set):Float;
	public var type(get, set):BodyType;

	public var allowRotation(get, set):Bool;
	public var allowMovement(get, set):Bool;
	public var bullet(get, set):Bool;
	public var awake(get, never):Bool;
	public var sleeping(get, never):Bool;

	public var body(default, null):Body;

	/**
	 * A body needs atleast one shape.
	 * This shape is used when the body has no shape.
	 */
	private var temporaryShape:Polygon;

	override public function added()
	{
		checkBody();
		
		// if the body has no shapes add temporaryShape
		checkTemporaryShape();
	}

	override public function update():Void
	{
		checkTemporaryShape();
		transform.x = body.position.x;
		transform.y = body.position.y;
		transform.rotation = body.rotation;
		transform.ignoreParents = true;
	}

	override public function debugDraw():Void
	{
		if (temporaryShape != null)
		{
			Painter.set(Color.fromBytes(132, 31, 39), 1);
			for (camera in Screen.cameras)
			{
				Painter.camera = camera;
				Painter.drawPolygon(temporaryShape, -Screen.camera.x, -Screen.camera.y);
			}
		}
	}

	public inline function applyImpulse(forceX:Float, forceY:Float, sleepable:Bool = false):Void
	{
		body.applyImpulse(Vec2.weak(forceX, forceY), null, sleepable);
	}

	public inline function applyImpulseAtPosition(forceX:Float, forceY:Float, x:Float, y:Float, sleepable:Bool = false):Void
	{
		body.applyImpulse(Vec2.weak(forceX, forceY), Vec2.weak(x, y), sleepable);
	}

	public inline function applyAngularImpulse(impulse:Float, sleepable:Bool):Void
	{
		body.applyAngularImpulse(impulse, sleepable);
	}

	public inline function sleep():Void
	{
		body.sleepBody();
	}

	private inline function checkBody()
	{
		var nape = scene.getExtension(Nape);
		body = nape.bodies[gameObject];
		
		if (body == null)
		{
			body = new Body(BodyType.DYNAMIC, Vec2.weak(transform.x, transform.y));
			body.space = nape.space;
			nape.bodies[gameObject] = body;
		}
		else
		{
			body.type = BodyType.DYNAMIC;
		}
	}

	private inline function checkTemporaryShape():Void
	{
		if (temporaryShape != null)
		{
			// remove tempShape if the body has another shape
			if (body.shapes.length >= 2)
			{
				//temporaryShape.body = null;
				temporaryShape.body = null;
				temporaryShape = null;
			}
		}
		else
		{
			// add tempShape if the body has no shape
			if (body.shapes.empty())
			{
				temporaryShape = new Polygon(Polygon.box(5, 5, true));
				temporaryShape.body = body;
			}
		}
	}
	
	override public function loadConfig(data:Dynamic):Void
	{
		velocityX = data.velocityX.parse(0.0);
		velocityY = data.velocityY.parse(0.0);
		angularVelocity = data.angularVelocity.parse(0.0);
		
		var mass:Null<Float> = data.mass;
		if (mass != null) this.mass = mass;
		
		type = data.bodyType.parseBodyType(BodyType.DYNAMIC);
		allowRotation = data.allowRotation.parse(true);
		allowMovement = data.allowMovement.parse(true);
		bullet = data.bullet.parse(false);
		
		if (data.sleeping.parse(false))
			sleep();
	}

	private inline function get_velocityX():Float { return body.velocity.x; }
	private inline function set_velocityX(value:Float):Float { return body.velocity.x = value; }

	private inline function get_velocityY():Float { return body.velocity.y; }
	private inline function set_velocityY(value:Float):Float { return body.velocity.y = value; }

	private inline function get_angularVelocity():Float { return body.angularVel; }
	private inline function set_angularVelocity(value:Float):Float { return body.angularVel = value; }

	private inline function get_mass():Float { return body.mass; }
	private inline function set_mass(value:Float):Float { return body.mass = value; }

	private inline function get_type():BodyType { return body.type; }
	private inline function set_type(value:BodyType):BodyType { return body.type = value; }

	private inline function get_allowRotation():Bool { return body.allowRotation; }
	private inline function set_allowRotation(value:Bool):Bool { return body.allowRotation = value; }

	private inline function get_allowMovement():Bool { return body.allowMovement; }
	private inline function set_allowMovement(value:Bool):Bool { return body.allowMovement = value; }

	private inline function get_bullet():Bool { return body.isBullet; }
	private inline function set_bullet(value:Bool):Bool { return body.isBullet = value; }

	private inline function get_sleeping():Bool { return body.isSleeping; }

	private inline function get_awake():Bool { return !body.isSleeping; }
}