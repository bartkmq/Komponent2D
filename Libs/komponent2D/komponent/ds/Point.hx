package komponent.ds;

import kha.math.Matrix3;
import kha.math.Vector2 in KhaVector;

typedef PointData = hxcollision.math.Vector;

abstract Point(PointData) from PointData to PointData
{
	
	public static var ZERO:Point = new Point(0, 0);
	public static var INVERSE = new Point( -1, -1);
	
	public var x(get, set):Float;
	public var y(get, set):Float;
	
	public var length(get, set):Float;
	public var lengthSq(get, never):Float;

	public inline function new(x:Float = 0, y:Float = 0) 
	{
		this = new PointData(x, y);
	}
	
	public inline function distanceTo(point:Point):Float
	{
		return Math.sqrt(Math.pow(point.x - x, 2) + Math.pow(point.y - y, 2));
	}
	
	public inline function normalize(length:Float = 1):Void
	{
		if (x == 0 && y == 0)
			return;
		else
		{
			var norm = length / this.length;
			x *= norm;
			y *= norm;
		}
	}
	
	public inline function negate():Void
	{
		x *= -1;
		y *= -1;
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
	
	public function toString():String
	{
		return 'Point[$x, $y]';
	}
	
	@:op(A += B) public static inline function add(a:Point, b:Point):Point
	{
		a.x += b.x;
		a.y += b.y;
		return a;
	}

	@:op(A + B) private static inline function _add(a:Point, b:Point):Point
	{
		return new Point(a.x + b.x, a.y + b.y);
	}

	@:op(A -= B) private static inline function subtract(a:Point, b:Point):Point
	{
		a.x -= b.x;
		a.y -= b.y;
		return a;
	}

	@:op(A - B) private static inline function _subtract(a:Point, b:Point):Point
	{
		return new Point(a.x - b.x, a.y - b.y);
	}

	@:commutative @:op(A * B) private static inline function _multiplyByScalar(a:Point, b:Float):Point
	{
		return new Point(a.x * b, a.y * b);
	}

	@:op(A *= B) private static inline function _multiplyEquals(a:Point, b:Float):Point
	{
		a.x *= b;
		a.y *= b;
		return a;
	}

	@:op(A *= B) public static inline function multiply(a:Point, b:Point):Point
	{
		a.x *= b.x;
		a.y *= b.y;
		return a;
	}

	@:op(A / B) private static inline function _divideByScalar(a:Point, b:Float):Point
	{
		b = 1 / b;
		return new Point(a.x * b, a.y * b);
	}

	@:op(A /= B) private static inline function _divideEquals(a:Point, b:Float):Point
	{
		b = 1 / b;
		a.x *= b;
		a.y *= b;
		return a;
	}

	@:op(A /= B) public static inline function divide(a:Point, b:Point):Point
	{
		a.x /= b.x;
		a.y /= b.y;
		return a;
	}

	@:op(A *= B) private static inline function _multiplyEqualsMatrix(v:Point, m:Matrix3):Point
	{
		var f:Float = 0;
		f += m[Matrix3.index(0, 2)] * v.x;
		f += m[Matrix3.index(1, 2)] * v.y;
		f += m[Matrix3.index(2, 2)] * 1;
		var w = f;
		f = 0;
		f += m[Matrix3.index(0, 0)] * v.x;
		f += m[Matrix3.index(1, 0)] * v.y;
		f += m[Matrix3.index(2, 0)] * 1;
		v.x = f / w;
		f = 0;
		f += m[Matrix3.index(0, 1)] * v.x;
		f += m[Matrix3.index(1, 1)] * v.y;
		f += m[Matrix3.index(2, 1)] * 1;
		v.y = f / w;
		return v;
	}

	@:op(A * B) private static inline function _multiplyMatrix(v:Point, m:Matrix3):Point
	{
		var point = new Point();
		var f:Float = 0;
		f += m[Matrix3.index(0, 2)] * v.x;
		f += m[Matrix3.index(1, 2)] * v.y;
		f += m[Matrix3.index(2, 2)] * 1;
		var w = f;
		f = 0;
		f += m[Matrix3.index(0, 0)] * v.x;
		f += m[Matrix3.index(1, 0)] * v.y;
		f += m[Matrix3.index(2, 0)] * 1;
		point.x = f / w;
		f = 0;
		f += m[Matrix3.index(0, 1)] * v.x;
		f += m[Matrix3.index(1, 1)] * v.y;
		f += m[Matrix3.index(2, 1)] * 1;
		point.y = f / w;
		return point;
	}
	
	/*
	@:op(A * B) private static inline function _multiplyInverseMatrix(m:Matrix3, v:Point):Point
	{
		return new Point(
			m._11 * v.x + m._21 * v.y + m._31 * v.z + m._41,
			m._12 * v.x + m._22 * v.y + m._32 * v.z + m._42,
			m._13 * v.x + m._23 * v.y + m._33 * v.z + m._43
		);
	}
	*/
	
	@:op(A * B) public static inline function dot(a:Point, b:Point):Float
	{
		return a.x * b.x + a.y * b.y;
	}

	@:op(A == B) private static inline function _equals(a:Point, b:Point):Bool
	{
		return (a == null ? b == null : (b != null && a.x == b.x && a.y == b.y));
	}

	@:op(A != B) private static inline function _notEquals(a:Point, b:Point):Bool
	{
		return !_equals(a, b);
	}

	@:op(-A) private static inline function _negativeVector(a:Point):Point
	{
		return new Point(-a.x, -a.y);
	}
	
	@:from private static inline function fromKhaVector(v:KhaVector)
	{
		return new Point(v.x, v.y);
	}
	
	@:to private inline function toKhaVector()
	{
		return new KhaVector(x, y);
	}
	
	private inline function get_length():Float
	{
		return Math.sqrt(x * x + y * y);
	}
	
	private inline function set_length(value:Float):Float
	{
		normalize();
		x *= value;
		y *= value;
		return value;
	}
	
	private inline function get_lengthSq():Float
	{
		return x * x + y * y;
	}
	
	private inline function get_x():Float { return this.x; }
	private inline function set_x(value:Float):Float {return this.x = value; }
	private inline function get_y():Float { return this.y; }
	private inline function set_y(value:Float):Float { return this.y = value; }
}


/**
 * A Point that notifies it's owner about changes.
 */
/*
abstract CbPoint(Point)
{

	public function new(x:Float = 0, y:Float = 0, callback:Point->Void = null) 
	{
		this.x = x;
		this.y = y;
		this.cb = callback;
		
		if (this.cb == null)
			callback = emptyCallback;
	}
	
	private static function emptyCallback(point:Point) {}
	
	private inline function get_x():Float { return this.x; }
	private inline function set_x(value:Float):Float { this.x = value; this.cb(this); return value; }
	private inline function get_y():Float { return this.y; }
	private inline function set_y(value:Float):Float { this.y = value; this.cb(this); return value; }
}
*/
