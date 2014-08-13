package komponent;

import nape.geom.Vec2;

import yaml.Parser;
import yaml.Renderer;
import yaml.Yaml;

import kha.Configuration;
import kha.Game;
import kha.Loader;
import kha.LoadingScreen;
import kha.Scheduler;

import komponent.extension.Nape;
import komponent.utils.Misc;
import komponent.utils.Time;
import komponent.utils.Painter;
import komponent.utils.Screen;
import komponent.input.Keyboard;
import komponent.input.Mouse;
import komponent.input.Gamepad;
import komponent.input.Touch;

using komponent.utils.Misc;

@:keepSub
class Engine extends Game
{
	
	public var currentScene(default, set):Scene;
	
	public var fps(get, never):Float;
	public var paused:Bool;
	
	// Debug draw
	public var debug:Bool;
	
	// Config
	public var config:Dynamic;
	
	/**
	 * Constructor. Can be used to set the name of this game and to load Rooms.
	 * @param	name			Name to assign to the game.
	 * @param	loadRooms		Rooms that should be loaded before the engine starts
	 * @param	scene			Scene that will be created. Should have no constructor arguments. Default: Scene.
	 * @param	loadingScreen	Loading Screen that is displayed while loading the rooms. Should have no constructor arguments. Default: LoadingScreen.
	 */
	public function new(name:String = "", loadRooms:Array<String> = null, scene:Class<Scene> = null, loadingScreen:Class<Game> = null)
	{
		super(name);
		_loadRooms = loadRooms;
		paused = false;
		
		#if debug
		debug = true;
		#else
		debug = false;
		#end
		
		if (scene == null)
			_startScene = Scene;
		else
			_startScene = scene;
		
		if (loadingScreen == null)
			_loadingScreen = LoadingScreen;
		else
			_loadingScreen = loadingScreen;
	}
	
	override public function init():Void
	{	
		Configuration.setScreen(Type.createInstance(_loadingScreen, []));
		_loadingScreen = null;

		#if debug
		Time.start("loading");
		#end
		for (room in _loadRooms)
			Loader.the.loadRoom(room, roomLoaded);
	}
	
	private inline function roomLoaded():Void
	{
		_loadRooms.pop();
		
		if (_loadRooms.length == 0)
		{
			_loadRooms = null;
			#if debug
			trace('Rooms loaded in ${Misc.round(Time.stop("loading"), 2)}ms');
			#end
			initEngine();
		}
	}
	
	private function initEngine():Void
	{
		Misc.engine = this;
		
		// init Input
		Keyboard.init();
		Mouse.init();
		Touch.init();
		
		if (_config != null)
		{
			config = Yaml.parse(Loader.the.getBlob(_config).toString(), Parser.options().useObjects());
			currentScene = Type.createInstance(config.Engine.scene.type.loadClass(_startScene), []);
			currentScene.loadPrefab(config.Engine.scene);
			
			name = config.Engine.name.loadDefault("");
			paused = config.Engine.paused.loadDefault(false);
			debug = config.Engine.debug.loadDefault(#if debug true #else false #end);
		}
		else
		{
			currentScene = Type.createInstance(_startScene, []);
		}		
		_startScene = null;
		
		Configuration.setScreen(this);
	}
	
	override public function update():Void
	{
		Time.start("updating");
		currentScene.update();
		
		Keyboard.update();
		Mouse.update();
		Gamepad.update();
		Touch.update();
		
		Time.frames++;
		Time.stop("updating");
	}
	
	override public function render(khaPainter:kha.Painter):Void
	{
		Time.start("rendering");
		startRender(khaPainter);
		Painter.painter = khaPainter;
		currentScene.render();
		endRender(khaPainter);
		Time.stop("rendering");
	}
	
	override public function onClose():Void
	{
		currentScene.quit();
	}
	
	public static function fromConfig(configfile:String, loadRooms:Array<String> = null, loadingScreen:Class<Game> = null):Engine
	{
		var engine:Engine = new Engine("", loadRooms, null, loadingScreen);
		engine._config = configfile;
		return engine;
	}
	
	private inline function set_currentScene(value:Scene):Scene
	{
		if (currentScene != null)
			currentScene.end();
		if (currentScene == value)
			return value;
			
		currentScene = value;
		currentScene.engine = this;
		currentScene.begin();
		return value;
	}
	
	private inline function get_fps():Float { return 1 / Time.elapsed; }
	
	private var _startScene:Class<Scene>;
	private var _loadRooms:Array<String>;
	private var _loadingScreen:Class<Game>;
	
	private var _config:String;
}