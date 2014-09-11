package komponent.input;

import kha.input.Surface;
import kha.Rectangle;

import komponent.utils.Screen;
import komponent.utils.Time;

@:allow(komponent.input.Input)
class Touch
{
	
	public static var touches:Map<Int, TouchData>;
	
	private static var definitions:Map<String, Rectangle>;
	
	public static function define(name:String, area:Rectangle)
	{
		if (name == null || area == null || area.width * area.height <= 0)
			trace('Invalid Touch Definition: name=$name, area=${area.width * area.height}');
		definitions[name] = area;
	}
	
	public function check(name:String):Bool
	{
		var area = definitions[name];
		if (area != null)
		{
			var tempRect = new Rectangle(0, 0, 1, 1);
			for (touch in touches)
			{
				if (!touch.released)
				{
					tempRect.setPos(touch.x, touch.y);
					if (area.collision(tempRect))
						return true;
				}
			}
		}
		return false;
	}
	
	public static function pressed(name:String):Bool
	{
		var area = definitions[name];
		if (area != null)
		{
			var tempRect = new Rectangle(0, 0, 1, 1);
			for (touch in touches)
			{
				if (touch.pressed)
				{
					tempRect.setPos(touch.x, touch.y);
					if (area.collision(tempRect))
						return true;
				}
			}
		}
		return false;
	}
	
	public static function released(name:String):Bool
	{
		var area = definitions[name];
		if (area != null)
		{
			var tempRect = new Rectangle(0, 0, 1, 1);
			for (touch in touches)
			{
				if (touch.released)
				{
					tempRect.setPos(touch.x, touch.y);
					if (area.collision(tempRect))
						return true;
				}
			}
		}
		return false;
	}
	
	private static function init()
	{
		touches = new Map();
		definitions = new Map();
		if (Surface.get() != null)
			Surface.get().notify(onTouchStart, onTouchEnd, onTouchMove);
	}
	
	private static function update()
	{
		for (touch in touches)
		{
			if (touch.released && !touch.pressed)
			{
				touches.remove(touch.id);
			}
		}
	}
	
	private static function onTouchStart(touchID:Int, x:Int, y:Int)
	{
		touches[touchID] = new TouchData(x, y, touchID);
	}
	
	private static function onTouchEnd(touchID:Int, x:Int, y:Int)
	{
		touches[touchID].released = true;
	}
	
	private static function onTouchMove(touchID:Int, x:Int, y:Int)
	{
		touches[touchID].setXY(x, y);
	}
}

class TouchData
{
	/**
	 * Touch id used for multiple touches
	 */
	public var id(default, null):Int;
	/**
	 * X-Axis coord in window
	 */
	public var x:Int;
	/**
	 * Y-Axis coord in window
	 */
	public var y:Int;
	/**
	 * Starting X position of touch
	 */
	public var startX:Int;
	/**
	 * Starting Y position of touch
	 */
	public var startY:Int;
	/**
	 * The time this touch has been held
	 */
	public var time(default, null):Float;

	/**
	 * Creates a new touch object
	 * @param  x  x-axis coord in window
	 * @param  y  y-axis coord in window
	 * @param  id touch id
	 */
	public function new(x:Int, y:Int, id:Int)
	{
		this.startX = this.x = x;
		this.startY = this.y = y;
		this.id = id;
		this.time = 0;
	}

	/**
	 * The touch x-axis coord in the scene.
	 */
	public var sceneX(get, never):Float;
	private inline function get_sceneX():Float { return x + Screen.camera.x; }

	/**
	 * The touch y-axis coord in the scene.
	 */
	public var sceneY(get, never):Float;
	private inline function get_sceneY():Float { return y + Screen.camera.y; }

	/**
	 * If the touch was pressed this frame.
	 */
	public var pressed(get, never):Bool;
	private inline function get_pressed():Bool { return time == 0; }

	/**
	 * If the touch is not longer held
	 */
	public var released:Bool = false;
	
	public inline function setXY(x:Int, y:Int)
	{
		this.x = x;
		this.y = y;
	}

	/**
	 * Updates the touch state.
	 */
	public function update()
	{
		time += Time.elapsed;
	}
}