package komponent.components.navigation;

import komponent.Component;
import komponent.GameObject;
import komponent.ds.Point;
import komponent.components.Physics;
import komponent.extension.Nape;

class Navigator<T> extends Component
{
	
	public var target(default, set):Point;
	public var targetObject:GameObject;
	
	// The nodes of the path.
	public var path:Array<Point>;
	
	// Total length of the path.
	public var distance(default, null):Float;
	
	// Remaining distance to the target.
	public var remainingDistance(default, null):Float;
	
	// Distance to the target when the Navigator starts to deaccelerate.
	public var stoppingDistance:Float;
	
	// If the path should be recalculated when the Navigators GameObjects Transform was moved manually.
	public var autoRepath:Bool;
	
	// If the Navigator should deaccelerate to avoid overshooting the target.
	public var autoBraking:Bool;
	
	// Current status of the Navigator
	public var status(default, null):NavigatorStatus;
	
	// Called when the Navigator should move.
	// Navigator, NavgiationData
	public var movementHandler:Navigator->T->Void;
	
	public var maxVelocity:Float;
	public var acceleration:Float;
	
	private var physics:Physics;

	public function new() 
	{
		path = [];
		distance = 0;
		remainingDistance = 0;
		stoppingDistance = 10;
		
		autoRepath = true;
		autoBraking = true;
		
		status = NavigatorStatus.STOPPED;
		
		movementHandler = physicalMovement;
		
		maxVelocity = 50;
		acceleration = 5;
	}
	
	override public function added()
	{
		physics = getComponent(Physics);
		if (physics == null)
			physics = addComponent(Physics);
	}
	
	override public function update()
	{
		//remainingDistance = 
		
		movementHandler(this, remainingDistance, target);
	}
	
	public function follow(gameObject:GameObject):Void
	{
		if (targetObject != gameObject)
		{
			target.setPos(gameObject.transform.x, gameObject.transform.y);
			calculatePath();
		}
	}
	
	public function moveTo(x:Float, y:Float):Void
	{
		if (!target.equalPos(x, y))
		{
			target.setPos(x, y);
			calculatePath();
		}
	}
	
	public function pause(instantStop:Bool = false):Void
	{
		if (instantStop)
			status = PAUSED;
		else
			status = STOPPING;
	}
	
	public function resume(resetVelocity:Bool = false):Void
	{
		status = NAVIGATING;
	}
	
	public function stop(instantStop:Bool = false):Void
	{
		if (instantStop)
			status = STOPPED;
		else
			status = STOPPING;
	}
	
	// moves by modifying the objects Transform
	private function simpleMovement():Void
	{
		
	}
	
	// moves by appling a impulse to the objects Physics
	private function physicalMovement():Void
	{
		
	}
	
	// returns if path to target was found
	private function calculatePath():Bool
	{
		clearPath();
		path.push(target);
		status = NAVIGATING;
	}
	
	private function clearPath():Void
	{
		for (n in 0...path.length)
		{
			path.pop();
		}
	}
	
	private inline function set_target(value:Point):Point
	{
		moveTo(value.x, value.y);
		return value;
	}
}

enum NavigatorStatus
{
	// Currently navigating to the target.
	NAVIGATING
	// The navigation was stopped/paused and the Navigator is deaccelerating.
	STOPPING
	// The navigation is stopped.
	STOPPED
	// The navigation is paused and can be resumed.
	PAUSED
}

class NavigationData
{
	
}