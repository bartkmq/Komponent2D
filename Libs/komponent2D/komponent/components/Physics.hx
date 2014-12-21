package komponent.components;

import hxcollision.Collision;

import komponent.ds.Point;
import komponent.Component;
import komponent.GameObject;
import komponent.utils.Time;
import komponent.utils.Misc;
import komponent.physics.CollisionData2D;
import komponent.physics.CollisionEvent;

enum VelocityLimit
{
	UNLIMITED;
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

	/**
	 * The velocity of this GameObject.
	 */
	public var velocityX:Float;
	public var velocityY:Float;
	public var angularVelocity:Float;
	
	/**
	 * Acceleration added to the velocity every frame.
	 */
	public var accelerationX:Float;
	public var accelerationY:Float;
	public var angularAcceleration:Float;
	
	/*
	 * Gravity is added to the velocity every update.
	 */
	public var gravityX:Float;
	public var gravityY:Float;
	
	/**
	 * Drag is subtracted from the velocity every update.
	 */
	public var dragX:Float;
	public var dragY:Float;
	public var angularDrag:Float;
	
	/**
	 * Determines how much of the original velocity is retained after bouncing against another collider.
	 * Should be between 0 and 1.
	 */
	public var elasticityX:Float;
	public var elasticityY:Float;
	
	/**
	 * Maximal Velocity.
	 * Default/Disable maxAngularVelocity: Set to -1.
	 */
	public var maxVelocity:VelocityLimit;
	public var maxAngularVelocity:Float;
	
	/**
	 * If the velocity is below this value it will be set to 0.
	 * Default/Disable minAngularVelocity: Set to -1.
	 */
	public var minVelocity:VelocityLimit;
	public var minAngularVelocity:Float;
	
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
	public static var defaultGravityX:Float = 0;
	public static var defaultGravityY:Float = 100;
	public static var defaultElasticity:Float = 0;
	
	public static var defaultMaxVelocity:VelocityLimit = UNLIMITED;
	public static var defaultMinVelocity:VelocityLimit = UNLIMITED;
	
	public static var defaultMaxAngularVelocity:Float = -1;
	public static var defaultMinAngularVelocity:Float = -1;
	
	public static var defaultDragX:Float = 0;
	public static var defaultDragY:Float = 0;
	public static var defaultAngularDrag:Float = 0;
	
	override public function added()
	{
		velocityX = velocityY = 0;
		angularVelocity = 0;
		gravityX = defaultGravityX;
		gravityY = defaultGravityY;
		accelerationX = accelerationY = 0;
		angularAcceleration = 0;
		
		dragX = defaultDragX;
		dragY = defaultDragY;
		angularDrag = defaultAngularDrag;
		elasticityX = elasticityY = defaultElasticity;
		maxVelocity = defaultMaxVelocity;
		minVelocity = defaultMinVelocity;
		
		useMoveBy = true;
		moveByTypes = [];
		currentColliders = [];
		moveX = moveY = 0;
		moveCollideX = moveCollideY = moveCollideXY;
		
		gameObject.transform.parent = null;
	}
	
	override public function update()
	{
		velocityX = computeVelocity(velocityX, accelerationX, gravityX, dragX);
		velocityY = computeVelocity(velocityY, accelerationY, gravityY, dragY);
		angularVelocity = computeVelocity(angularVelocity, angularAcceleration, 0, angularDrag);
		
		limitVelocity();
		
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
	
	/**
	 * Moves the gameObject by the specified amount and checks for collisions.
	 * Dispatches onCollision() when a collision happened.
	 * It's reccomment to modify acceleration/velococity instead of using this.
	 * @param	deltaX The amount to move on the x axis.
	 * @param	deltaY The amount to move on the y axis.
	 * @param	additional checks to prevent tunneling
	 */
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
	
	public function moveTo(x:Float, y:Float, sweep:Bool = false)
	{
		moveBy(x - transform.x, y - transform.y, sweep);
	}
	
	public function moveTowards(x:Float, y:Float, amount:Float, sweep:Bool = false)
	{
		var p = new Point(x - transform.x, y - transform.y);
		if (p.lengthSq > amount * amount)
			p.normalize(amount);
		moveBy(p.x, p.y, sweep);
	}
	
	public function collideTypes(x:Float, y:Float, types:Array<String>):CollisionData2D
	{
		for (type in types)
		{
			var typeColliders = Collider.typeColliders[type];
			if (typeColliders == null) continue;
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
			if (typeColliders == null) continue;
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
	
	private function computeVelocity(velocity:Float, acceleration:Float, gravity:Float, drag:Float)
	{
		velocity += acceleration * Time.elapsed;
		velocity += gravity * Time.elapsed;
		
		if (drag != 0)
		{
			drag *= Time.elapsed;
			if (velocity - drag > 0)
				velocity -= drag;
			else if (velocity + drag < 0)
				velocity += drag;
			else
				velocity = 0;
		}
		return velocity;
	}
	
	private function limitVelocity()
	{
		switch(maxVelocity)
		{
			case UNLIMITED: 0; // do nothing
			case SEPERATE(maxVelX, maxVelY):
				{
					velocityX = Misc.clampAbs(velocityX, maxVelX);
					velocityY = Misc.clampAbs(velocityY, maxVelY);
				}
			case REALISTIC(maxVel):
				{
					var velocity = new Point(velocityX, velocityY);
					var length = velocity.length;
					if (length > maxVel)
					{
						velocity.length = maxVel;
						velocityX = velocity.x;
						velocityY = velocity.y;
					}
				}
		}
		
		switch(minVelocity)
		{
			case UNLIMITED: 0; // do nothing
			case SEPERATE(minVelX, minVelY):
				{
					if (velocityX < minVelX) velocityX = 0;
					if (velocityY < minVelY) velocityY = 0;
				}
			case REALISTIC(minVel):
				{
					var length = new Point(velocityX, velocityY).length;
					if (length < minVel)
					{
						velocityX = 0;
						velocityY = 0;
					}
				}
		}
		
		if (maxAngularVelocity >= 0)
			angularVelocity = Misc.clampAbs(angularVelocity, maxAngularVelocity);
		if (minAngularVelocity >= 0 && (angularVelocity < minAngularVelocity)) angularVelocity = 0;
	}
	
}