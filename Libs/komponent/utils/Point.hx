package komponent.utils;

import nape.geom.Vec2;

@:forward(x, y, copy, normalise)
abstract Point(Vec2) from Vec2 to Vec2
{

	public function new(x:Float = 0, y:Float = 0) 
	{
		this = new Vec2(x, y);
	}
	
	public function distance(point:Point):Float
	{
		return Math.sqrt(Math.pow(this.x - point.x, 2) + Math.pow(this.y - point.y, 2));
	}
	
	@:op(A + B)
	private function add(point:Point)
	{
		this.x += point.x;
		this.y += point.y;
		return this;
	}
	
	@:op(A - B)
	private function sub(point:Point)
	{
		this.x -= point.x;
		this.y -= point.y;
		return this;
	}
	
	@:op(A * B)
	private function mul(point:Point)
	{
		this.x *= point.x;
		this.y *= point.y;
		return this;
	}
	
	@:op(A / B)
	private function div(point:Point)
	{
		this.x /= point.x;
		this.y /= point.y;
		return this;
	}
	
}
