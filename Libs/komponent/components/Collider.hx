package komponent.components;

import hxcollision.Collision;
import hxcollision.shapes.Ray;
import hxcollision.shapes.Shape;

import komponent.physics.CollisionData2D;
import komponent.physics.RaycastData2D;
import komponent.physics.CollisionEvent;
import komponent.ds.Point;

using komponent.utils.Parser;

typedef HxPolygon = hxcollision.shapes.Polygon;
typedef HxCircle = hxcollision.shapes.Circle;

class Collider extends Component
{
	/**
	* Base class for all colliders.
	* 
	* Dispatches:
	* -> onCollision(CollisionData2D) when this colliders collides with another collider.
	*/
	
	public var active:Bool;
	public var shape:Shape;
	public var type(default, set):String;
	
	public static var colliders:Array<Collider> = [];
	public static var typeColliders:Map<String, Array<Collider>> = new Map();
	
	override public function added()
	{
		Collider.colliders.push(this);
		gameObject.colliders.push(this);
		active = true;
		type = gameObject.type;
	}
	
	override public function update()
	{
		
	}
	
	override public function removed()
	{
		Collider.colliders.remove(this);
		gameObject.colliders.remove(this);
	}
	
	public function collideType(type:String):Array<CollisionData2D>
	{
		return collideWithGroup(typeColliders[type]);
	}
	
	public function collideTypes(types:Array<String>):Array<CollisionData2D>
	{
		var result:Array<CollisionData2D> = [];
		for (type in types)
		{
			for (collisionData in collideWithGroup(typeColliders[type]))
				result.push(collisionData);
		}
		return result;
	}
	
	public function collideWith(otherCollider:Collider):CollisionData2D
	{
		var result = Collision.test(shape, otherCollider.shape);
		if (result != null)
			return new CollisionData2D(result, this, otherCollider, null, CollisionEvent.BEGIN);
		else
			return null;
	}
	
	public function collideWithGroup(otherColliders:Array<Collider>, ignoreItself:Bool = false):Array<CollisionData2D>
	{
		var results:Array<CollisionData2D> = [];
		for (collider in otherColliders)
		{
			if (ignoreItself && collider.shape == shape) continue;
			var result = Collision.test(shape, collider.shape);
			if (result != null)
				results.push(new CollisionData2D(result, this, collider, null, CollisionEvent.BEGIN));
		}
		return results;
	}
	
	public function raycast(ray:Ray):RaycastData2D
	{
		return new RaycastData2D(shape.testRay(ray), this);
	}
	
	override public function loadConfig(data:Dynamic):Void
	{
		
	}
	
	private function onTransformChange(transform:Transform):Void
	{
		shape.x = transform.x;
		shape.y = transform.y;
		shape.rotation = transform.rotation;
		shape.scaleX = transform.scaleX;
		shape.scaleY = transform.scaleY;
	}
	
	private inline function set_type(value:String):String
	{
		if (value != type)
		{
			if (value == null)
			{
				trace("Collider.type can't be set to null.");
				return null;
			}
			else
			{
				typeColliders.remove(type);
				type = value;
				var colliders = typeColliders[value];
				if (colliders != null)
				{
					colliders.push(this);
				}
				else
				{
					colliders = new Array();
					colliders.push(this);
					typeColliders[value] = colliders;
				}
			}
		}
		return value;
	}
	
}