package komponent.components.audio;

import kha.Sound in KhaSound;
import kha.SoundChannel;
import kha.Loader;

class Sound extends Component
{
	
	// volume as float in range 0-1
	public var volume(get, set):Float;
	
	// length of the sound in miliseconds
	public var length(get, never):Int;
	
	// current playback position in miliseconds
	public var currentPosition(get, never):Int;
	
	// if the sound should loop
	public var loop:Bool;
	
	// if the sound is playing
	public var finished(get, never):Bool;
	
	private var _sound:KhaSound;
	private var _soundchannel:SoundChannel;
	private var _stopped:Bool;
	
	private function new()
	{
		loop = false;
		_stopped = false;
	}
	
	override public function update()
	{
		if (_soundchannel != null && finished && loop && !_stopped)
			play(loop);
	}
	
	public inline function load(sound:String)
	{
		_sound = Loader.the.getSound(sound);
		_soundchannel = _sound.play();
		_soundchannel.stop();
	}
	
	public inline function play(loop:Bool)
	{
		this.loop = loop;
		_soundchannel.play();
		_stopped = false;
	}
	
	public inline function pause()
	{
		_soundchannel.pause();
	}
	
	public inline function stop()
	{
		_soundchannel.stop();
		_stopped = true;
	}
	
	public inline function unload()
	{
		_sound.unload();
	}
	
	private inline function get_volume():Float { return _soundchannel.getVolume(); }
	private inline function set_volume(value:Float):Float { _soundchannel.setVolume(value); return value; }
	
	private inline function get_length():Int { return _soundchannel.getLength(); }
	
	private inline function get_currentPosition():Int { return _soundchannel.getCurrentPos(); }
	
	private inline function get_finished():Bool { return _soundchannel.isFinished(); }
	
}