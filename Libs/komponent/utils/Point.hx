package komponent.utils;

import nape.geom.Vec2;

@:forward(x, y, copy, normalise)
abstract Point(Vec2) from Vec2 to Vec2
{

	public inline function new(x:Float = 0, y:Float = 0) 
	{
		this = new Vec2(x, y);
	}
	
	public inline function distance(point:Point):Float
	{
		return Math.sqrt(Math.pow(this.x - point.x, 2) + Math.pow(this.y - point.y, 2));
	}
	
	public inline function setTo(point:Point):Void
	{
		this.set(point);
	}
	
	public inline function setPos(x:Float, y:Float):Void
	{
		this.setxy(x, y);
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
