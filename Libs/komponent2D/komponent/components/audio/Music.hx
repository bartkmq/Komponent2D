package komponent.components.audio;

import kha.Music in KhaMusic;
import kha.Loader;

class Music extends Component
{
	
	// volume as float in range 0-1
	public var volume(get, set):Float;
	
	// length of the music in miliseconds
	public var length(get, never):Int;
	
	// current playback position in miliseconds
	public var currentPosition(get, never):Int;
	
	// if the music should loop
	public var loop(get, set):Bool;
	
	// if the music is playing
	public var finished(get, never):Bool;
	
	private var _music:KhaMusic;
	private var _loop:Bool;
	
	override public function added() 
	{
		_loop = false;
	}
	
	public inline function load(music:String)
	{
		_music = Loader.the.getMusic(music);
	}
	
	public inline function play(loop:Bool = true)
	{
		_loop = loop;
		_music.play(_loop);
	}
	
	public inline function pause()
	{
		_music.pause();
	}
	
	public inline function stop()
	{
		_music.stop();
	}
	
	public inline function unload()
	{
		_music.unload();
	}
	
	private inline function get_volume():Float { return _music.getVolume(); }
	private inline function set_volume(value:Float):Float { _music.setVolume(value); return value; }
	
	private inline function get_length():Int { return _music.getLength(); }
	
	private inline function get_currentPosition():Int { return _music.getCurrentPos(); }
	
	private inline function get_loop():Bool { return _loop; }
	private inline function set_loop(value:Bool):Bool
	{
		_loop = value;
		if (!finished)
		{
			pause();
			play(value);
		}
		return value;
	}
	
	private inline function get_finished():Bool { return _music.isFinished(); }
}