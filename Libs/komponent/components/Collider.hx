package komponent.components;

import hxcollision.data.CollisionData;
import hxcollision.Collision;
import hxcollision.data.RayData;
import hxcollision.shapes.Ray;
import hxcollision.shapes.Shape;

using komponent.utils.Parser;

enum CollisionType
{
	BEGIN;
	ONGOING;
	END;
}

class Collider extends Component
{
	/**
	* Base class for all colliders.
	* 
	* Dispatches:
	* -> onCollision(collider, otherCollider, collisionType) when this colliders collides with another collider.
	*/
	
	public var active:Bool;
	public var shape:Shape;
	
	public static var colliders:Array<Collider> = [];
	
	override public function added()
	{
		Collider.colliders.push(this);
	}
	
	override public function update()
	{
		shape.x = transform.x;
		shape.y = transform.y;
		shape.rotation = transform.rotation;
		shape.scaleX = transform.scaleX;
		shape.scaleY = transform.scaleY;
		
		var collisions = collideGroup(colliders, true);
		for (collision in collisions)
			trace(collision);
	}
	
	public function collide(otherCollider:Collider):CollisionData
	{
		return Collision.test(shape, otherCollider.shape);
	}
	
	public function collideGroup(otherColliders:Array<Collider>, ignoreItself:Bool = false):Array<CollisionData>
	{
		var results:Array<CollisionData> = [];
		for (collider in otherColliders)
		{
			if (ignoreItself && collider.shape == shape) continue;
			var result = Collision.test(shape, collider.shape);
			if (result != null) results.push(result);
		}
		return results;
	}
	
	public function raycast(ray:Ray):RayData
	{
		return shape.testRay(ray);
	}
	
	override public function loadConfig(data:Dynamic):Void
	{
		
	}
	
}