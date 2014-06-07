package komponent.extension;

import kha.Color;

import nape.geom.Vec2;
import nape.phys.Body;
import nape.space.Space;
import nape.util.Debug;
import nape.util.ShapeDebug;
import nape.geom.Vec2List;

import komponent.components.misc.Camera;
import komponent.components.Transform;
import komponent.utils.Time;
import komponent.utils.Painter;

class Nape extends Extension
{

	public var space:Space;
	public var velocityIterations:Int;
	public var positionIterations:Int;
	
	public var bodies:Map<GameObject, Body>;
	//public var painterDebug:PainterDebug;
	
	public var active:Bool;
	
	public function new()
	{
		space = new Space(Vec2.weak(0, 0));
		active = true;
		velocityIterations = 10;
		positionIterations = 10;
		bodies = new Map();
		
		//painterDebug = new PainterDebug();
	}
	
	override public function update():Void
	{
		if (active)
		{
			space.step(Time.elapsed, velocityIterations, positionIterations);
			
			for (gameObject in bodies.keys())
			{
				var body = bodies[gameObject];
				if (body.isKinematic())
				{
					var transform = gameObject.transform;
					body.position.x = transform.x;
					body.position.y = transform.y;
					body.rotation = transform.rotation;
				}
			}
		}
	}
	
	override public function render()
	{
		for (body in bodies)
		{
			Painter.set(Color.fromBytes(91, 194, 54), 1); // green
			//Painter.drawCross(body.worldCOM.x, body.worldCOM.y, 10, 10, 2);
		}
	}
	
	public inline function setGravity(gravityX:Float = 0, gravityY:Float = 100)
	{
		space.gravity.setxy(gravityX, gravityY);
	}
	
}
