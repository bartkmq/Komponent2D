package komponent.ds;

import hxcollision.math.Vector;

@:forward(x, y, clone, normalise, length)
abstract Point(Vector) from Vector to Vector
{

	public inline function new(x:Float = 0, y:Float = 0) 
	{
		this = new Vector(x, y);
	}
	
	public inline function distanceTo(point:Point):Float
	{
		return Math.sqrt(Math.pow(this.x - point.x, 2) + Math.pow(this.y - point.y, 2));
	}
	
	public inline function setTo(point:Point):Void
	{
		setPos(point.x, point.y);
	}
	
	public inline function setPos(x:Float, y:Float):Void
	{
		this.x = x;
		this.y = y;
	}
	
	public inline function equals(point:Point):Bool
	{
		return equalPos(point.x, point.y);
	}
	
	public inline function equalPos(x:Float, y:Float):Bool
	{
		return (this.x == x && this.y == y);
	}
	
	@:op(A + B)
	private inline function add(point:Point)
	{
		this.x += point.x;
		this.y += point.y;
		return this;
	}
	
	@:op(A - B)
	private inline function sub(point:Point)
	{
		this.x -= point.x;
		this.y -= point.y;
		return this;
	}
	
	@:op(A * B)
	private inline function mul(point:Point)
	{
		this.x *= point.x;
		this.y *= point.y;
		return this;
	}
	
	@:op(A / B)
	private inline function div(point:Point)
	{
		this.x /= point.x;
		this.y /= point.y;
		return this;
	}
	
}
