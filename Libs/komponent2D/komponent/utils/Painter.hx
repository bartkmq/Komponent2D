package komponent.utils;

import kha.graphics2.Graphics in Graphics2;
import kha.graphics4.Graphics in Graphics4;
import kha.Image;
import kha.Color;
import kha.Font;
import kha.Video;
import kha.Rectangle;
import kha.Framebuffer;
import kha.math.Vector2;

import komponent.ds.Matrix;

using kha.graphics2.GraphicsExtension;

class Painter
{
	
	public static var backbuffer(never, set):Image;
	public static var g2(default, null):Graphics2;
	public static var g4(default, null):Graphics4;
	
	public static var color(get, set):Color;
	public static var alpha(get, set):Float;
	public static var font(get, set):Font;
	
	public static var matrix(get, set):Matrix;
	
	public static inline function drawImage5(image:Image, x:Float, y:Float, flipX:Bool = false, flipY:Bool = false, sourceRect:Rectangle = null, tiledWidth:Int = 0, tiledHeight:Int = 0, fillScreen:Bool = false):Void
	{
		var sx:Float, sy:Float, sw:Float, sh:Float, dw:Float, dh:Float;
		
		if (sourceRect == null)
		{
			dw = image.width;
			dh = image.height;
			
			sx = sy = 0;
			sw = image.width;
			sh = image.height;
		}
		else
		{
			dw = sourceRect.width;
			dh = sourceRect.height;
			
			sx = sourceRect.x;
			sy = sourceRect.y;
			sw = sourceRect.width;
			sh = sourceRect.height;
		}
		
		if (flipX)
		{
			//x += dw;
			dw *= -1;
		}
		if (flipY)
		{
			//y += dh;
			dh *= -1;
		}
		
		
		if (tiledWidth > 0 && tiledHeight > 0 || fillScreen)
		{
			var tx = 0.0;
			var ty = 0.0;
			
			if (fillScreen)
			{
				x = 0;
				y = 0;
				tiledWidth = Screen.width;
				tiledHeight = Screen.height;
			}
			
			while (ty < tiledHeight)
			{
				while (tx < tiledWidth)
				{
					g2.drawScaledSubImage(image, sx, sy, sw, sh, x + tx, y + ty, dw, dh); 
					tx += dw;
				}
				tx = 0;
				ty += dh;
			}
		}
		else
		{
			g2.drawScaledSubImage(image, sx, sy, sw, sh, x, y, dw, dh);
		}
		
	}
	
	// draws a cross
	public static inline function drawCross(x:Float, y:Float, width:Float, height:Float, strength:Float = 1):Void
	{		
		g2.drawLine(x, y, x + width, y + height, strength);
		g2.drawLine(x + width, y, x, y + height, strength);
	}
	
	public static inline function drawCircle(cx:Float, cy:Float, radius:Float, strength:Float = 1, segments:Int = 0):Void
	{
		g2.drawCircle(cx, cy, radius, strength, segments);
	}
	
	public static inline function fillCircle(cx:Float, cy:Float, radius:Float, segments:Int = 0):Void
	{
		g2.fillCircle(cx, cy, radius, segments);
	}
	
	public static inline function drawPolygon(x:Float, y:Float, vertices:Array<Vector2>, strength:Float = 1):Void
	{
		g2.drawPolygon(x, y, vertices, strength);
	}
	
	public static inline function fillPolygon(x:Float, y:Float, vertices:Array<Vector2>):Void
	{
		g2.fillPolygon(x, y, vertices);
	}
	
	public static inline function set(color:Color, alpha:Float, font:Font = null):Void
	{
		Painter.color = color;
		Painter.alpha = alpha;
		Painter.font = font;
	}
	
	private static inline function set_backbuffer(value:Image):Image
	{
		g2 = value.g2;
		g4 = value.g4;
		return value;
	}
	
	public static inline function clear():Void { g2.clear; }
	public static inline function drawImage(img: Image, x: Float, y: Float):Void { g2.drawImage(img, x, y); }
	public static inline function drawSubImage(img: Image, x: Float, y: Float, sx: Float, sy: Float, sw: Float, sh: Float):Void { g2.drawSubImage(img, x, y, sx, sy, sw, sh); }
	public static inline function drawScaledImage(img: Image, dx: Float, dy: Float, dw: Float, dh: Float):Void { g2.drawScaledImage(img, dx, dy, dw, dh); }
	public static inline function drawScaledSubImage(image: Image, sx: Float, sy: Float, sw: Float, sh: Float, dx: Float, dy: Float, dw: Float, dh: Float):Void { g2.drawScaledSubImage(image, sx, sy, sw, sh, dx, dy, dw, dh); }
	public static inline function drawRect(x: Float, y: Float, width: Float, height: Float, strength: Float = 1.0):Void { g2.drawRect(x, y, width, height, strength); }
	public static inline function fillRect(x: Float, y: Float, width: Float, height: Float):Void { g2.fillRect(x, y, width, height); }
	public static inline function drawString(text: String, x: Float, y: Float):Void { g2.drawString(text, x, y); }
	public static inline function drawLine(x1: Float, y1: Float, x2: Float, y2: Float, strength: Float = 1.0):Void { g2.drawLine(x1, y1, x2, y2, strength); }
	public static inline function drawVideo(video: Video, x: Float, y: Float, width: Float, height: Float):Void { g2.drawVideo(video, x, y, width, height); }
	public static inline function fillTriangle(x1: Float, y1: Float, x2: Float, y2: Float, x3: Float, y3: Float):Void { g2.fillTriangle(x1, y1, x2, y2, x3, y3); }
	
	private static inline function get_color():Color { return g2.color; }
	private static inline function set_color(value:Color):Color { return g2.color = value; }
	
	private static inline function get_alpha():Float { return g2.opacity; }
	private static inline function set_alpha(value:Float):Float { return g2.opacity = value; }
	
	private static inline function get_font():Font { return g2.font; }
	private static inline function set_font(value:Font):Font { return g2.font = value; }
	
	private static inline function get_matrix():Matrix { return g2.transformation; }
	private static inline function set_matrix(value:Matrix):Matrix
	{
		if (value == null) g2.transformation = Matrix.identity();
		else g2.transformation = value;
		return value;
	}
}