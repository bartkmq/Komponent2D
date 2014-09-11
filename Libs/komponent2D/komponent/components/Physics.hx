package komponent.components;

import hxcollision.Collision;

import komponent.Component;
import komponent.GameObject;
import komponent.utils.Time;
import komponent.utils.Misc;
import komponent.physics.CollisionData2D;
import komponent.physics.CollisionEvent;

enum VelocityLimit
{
	SEPERATE(velocityX:Float, velocityY:Float);
	REALISTIC(velocity:Float);
}

/**
 * Removes the parent of this GameObject!
 * 
 * Dispatches:
 * -> onCollision(CollisionData2D) when a collision happens while moving.
 */
class Physics extends Component
{

	public var velocityX:Float;
	public var velocityY:Float;
	
	public var angularVelocity:Float;
	
	/*
	 * Gravity is added to the velocity every update.
	 */
	public var gravityX:Float;
	public var gravityY:Float;
	
	/**
	 * Determines how much of the original velocity is retained after bouncing against another collider.
	 * Should be between 0 and 1.
	 */
	public var elasticityX:Float;
	public var elasticityY:Float;
	
	/**
	 * Maximal Velocity.
	 */
	public var maxVelocity:VelocityLimit;
	
	/**
	 * If the velocity is below this value it will be set to 0.
	 */
	public var minVelocity:VelocityLimit;
	
	/**
	 * Maximal angular velocity.
	 */
	public var maxAngularVelocity:Float;
	
	/*
	 * If false the GameObjects velocity will be applied by directly modifying its position
	 * instead of using moveBy() wich causes it to ignore collisions.
	 */
	public var useMoveBy:Bool;
	
	/*
	 * Collision types to collide against when applying velocity.
	 */
	public var moveByTypes(default, null):Array<String>;
	
	/**
	 * This Callback is used to determine if 2 Colliders should collide.
	 * By default a collision is always accepted.
	 * If the function returns true a onCollision message will be dispatched.
	 */
	public var moveCollideX:(CollisionData2D->Bool);
	public var moveCollideY:(CollisionData2D->Bool);
	
	/**
	 * All Colliders currently colliding with this Collider.
	 */
	public var currentColliders:Array<Collider>;
	
	/**
	 * Internal data to increase precision of moveBy()
	 */
	private var moveX:Float;
	private var moveY:Float;
	
	/**
	 * The default values of a new Physics component.
	 */
	public static var defaultGravityX = 0;
	public static var defaultGravityY = 100;
	public static var defaultElasticity = 0;
	
	override public function added()
	{
		velocityX = velocityY = 0;
		angularVelocity = 0;
		gravityX = defaultGravityX;
		gravityY = defaultGravityY;
		elasticityX = elasticityY = defaultElasticity;
		maxVelocity = null;
		minVelocity = SEPERATE(0.01, 0.01);
		useMoveBy = true;
		moveByTypes = [];
		currentColliders = [];
		moveX = moveY = 0;
		moveCollideX = moveCollideY = moveCollideXY;
		
		gameObject.transform.parent = null;
	}
	
	override public function update()
	{
		velocityX += gravityX * Time.elapsed;
		velocityY += gravityY * Time.elapsed;
		
		if (useMoveBy)
		{
			moveBy(velocityX * Time.elapsed, velocityY * Time.elapsed);
		}
		else
		{
			transform.localX += velocityX * Time.elapsed;
			transform.localY += velocityY * Time.elapsed;
		}
		transform.localRotation += angularVelocity * Time.elapsed;
	}
	
	/**
	 * This is the default function for moveCollideX/Y.
	 */
	public static function moveCollideXY(collision:CollisionData2D):Bool
	{	
		var physics = collision.physics;
		physics.velocityX *= -physics.elasticityX;
		physics.velocityY *= -physics.elasticityY;
		return true;
	}
	
	public function moveBy(deltaX:Float, deltaY:Float, sweep:Bool = false):Void
	{
		var types = (moveByTypes.length == 0) ? [""] : moveByTypes;
		
		moveX += deltaX;
		moveY += deltaY;
		deltaX = Math.fround(moveX);
		deltaY = Math.fround(moveY);
		moveX -= deltaX;
		moveY -= deltaY;
		
		var newX = transform.x;
		var newY = transform.y;
		
		if (types != null)
		{
			var sign:Int;
			var result:CollisionData2D;
			if (deltaX != 0)
			{
				if (gameObject.active && (sweep || collideTypes(newX + deltaX, newY, types) != null))
				{
					sign = (deltaX > 0) ? 1 : -1;
					while (deltaX != 0)
					{
						if ((result = collideTypes(newX + sign, newY, types)) != null)
						{
							if (moveCollideX == null || moveCollideX(result))
							{
								if (Misc.contains(currentColliders, result.collider2))
									result.event = ONGOING;
								else
									currentColliders.push(result.collider2);
								sendMessage("onCollision", result);
								break;
							}
							else newX += sign;
						}
						else
						{
							newX += sign;
						}
						deltaX -= sign;
					}
				}
				else newX += deltaX;
			}
			if (deltaY != 0)
			{
				if (gameObject.active && (sweep || collideTypes(newX, newY + deltaY, types) != null))
				{
					sign = (deltaY > 0) ? 1 : -1;
					while (deltaY != 0)
					{
						if ((result = collideTypes(newX, newY + sign, types)) != null)
						{
							if (moveCollideY == null || moveCollideY(result))
							{
								if (Misc.contains(currentColliders, result.collider2))
									result.event = ONGOING;
								else
									currentColliders.push(result.collider2);
								sendMessage("onCollision", result);
								break;
							}
							else newY += sign;
						}
						else
						{
							newY += sign;
						}
						deltaY -= sign;
					}
				}
				else newY += deltaY;
			}
			transform.localX = newX;
			transform.localY = newY;
		}
		else
		{
			transform.localX += deltaY;
			transform.localY += deltaY;
		}
	}
	
	public function collideTypes(x:Float, y:Float, types:Array<String>):CollisionData2D
	{
		for (type in types)
		{
			var typeColliders = Collider.typeColliders[type];
			for (collider in gameObject.colliders)
			{
				if (collider.active)
				{
					for (otherCollider in typeColliders)
					{
						if (otherCollider.active)
						{
							var oldX:Float = collider.shape.x;
							var oldY:Float = collider.shape.y;
							collider.shape.x = x;
							collider.shape.y = y;
							
							var result = Collision.test(collider.shape, otherCollider.shape);
							
							collider.shape.x = oldX;
							collider.shape.y = oldY;
							
							if (result != null)
								return new CollisionData2D(result, collider, otherCollider, this, CollisionEvent.BEGIN);
							else
								return null;
						}
					}
				}
			}
		}
		return null;
	}
	
	public function collideTypesInto(x:Float, y:Float, types:Array<String>):Array<CollisionData2D>
	{
		var results = [];
		for (type in types)
		{
			var typeColliders = Collider.typeColliders[type];
			for (collider in gameObject.colliders)
			{
				if (collider.active)
				{
					for (otherCollider in typeColliders)
					{
						if (otherCollider.active)
						{
							var result = Collision.test(collider.shape, otherCollider.shape);
							if (result != null)
							{
								results.push(new CollisionData2D(result, collider, otherCollider, this, CollisionEvent.BEGIN));
							}
						}
					}
				}
			}
		}
		return results;
	}
	
	public function resetVelocity()
	{
		velocityX = 0;
		velocityY = 0;
	}
	
}