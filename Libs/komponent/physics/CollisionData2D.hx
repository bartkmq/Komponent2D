package komponent.physics;

import komponent.components.Collider;
import komponent.components.Physics;
import komponent.ds.Point;

import hxcollision.shapes.Shape;
import hxcollision.data.CollisionData;

/**
 * CollisionData with added collider information.
 * See documention of hxCollision.
 */ 
class CollisionData2D
{
	// The collider component of shape1
	public var collider1:Collider;
	
	// The collider component of shape2
	public var collider2:Collider;
	
	// The Physics component of collider1
	public var physics(get, null):Physics;
	
	// The Collision event
	public var event:CollisionEvent;
	
    public var overlap(get, never):Float;
    public var separation(get, never):Point;
    
    public var shape1(get, never):Shape;
    public var shape2(get, never):Shape;
    public var unitVector(get, never):Point; 
	
	private var data:CollisionData;
	
	public function new(data:CollisionData, coll1:Collider, coll2:Collider, physics:Physics, event:CollisionEvent)
	{
		this.data = data;
		collider1 = coll1;
		collider2 = coll2;
		this.physics = physics;
		this.event = event;
	}
	
	private inline function get_physics():Physics
	{
		if (physics == null)
		{
			physics = collider1.gameObject.getComponent(Physics);
		}
		return physics;
	}
	
	private inline function get_overlap():Float { return data.overlap; }
	private inline function get_separation():Point { return data.separation; }
	private inline function get_shape1():Shape { return data.shape1; }
	private inline function get_shape2():Shape { return data.shape2; }
	private inline function get_unitVector():Point { return data.unitVector; }
}