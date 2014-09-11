package komponent.utils;

import kha.Scheduler;

class Time
{
	
	// The current time in ms.
	public static var current(get, never):Float;
	
	// The delta time in ms.
	public static var elapsed(get, never):Float;
	
	// The timescale applied to Time.elapsed.
	public static var scale(get, set):Float;
	
	// Update frames since the game started
	public static var frames:Int = 0;
	
	// Timestamps for start() and stop().
	private static var timestamps:Map<String, Float> = new Map();
	
	// delta times
	private static var times:Map<String, Float> = new Map();
	
	// Creates a timestamp.
	public static inline function start(name:String)
	{
		timestamps[name] = current;
	}
	
	// Returns the delta between the timestamp and the current time
	public static inline function stop(name:String):Float
	{
		var delta = current - timestamps[name];
		timestamps.remove(name);
		return times[name] = delta;
	}
	
	// Returns the delta time of a previous measured interval
	public static inline function get(name:String):Float
	{
		return times[name];
	}
	
	private static inline function get_current():Float
	{
		#if sys
		return Sys.time();
		#else
		return Date.now().getTime() / 60;
		#end
	}
	private static inline function get_elapsed():Float { return Scheduler.deltaTime; }
	
	private static inline function get_scale():Float { return Scheduler.deltaScale; }
	private static inline function set_scale(value:Float):Float { return Scheduler.deltaScale = value; }
}