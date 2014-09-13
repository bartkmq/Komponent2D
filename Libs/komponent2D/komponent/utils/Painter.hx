package komponent.utils;

import kha.graphics2.Graphics in Graphics2;
import kha.graphics4.Graphics in Graphics4;
import kha.Image;
import kha.Color;
import kha.Rectangle;
import kha.Font;
import kha.Video;
import kha.Framebuffer;

import komponent.components.misc.Camera;

class Painter
{
	
	public static var backbuffer(never, set):Image;
	public static var g2(default, null):Graphics2;
	public static var g4(default, null):Graphics4;
	
	public static var color(never, set):Color;
	public static var alpha(get, set):Float;
	public static var font(never, set):Font;
	
	public static var camera:Camera;
	public static var scaleX:Float = 1;
	public static var scaleY:Float = 1;
	
	public static inline function drawCircle(cx:Float, cy:Float, r:Float, strength:Float = 1, segments:Int = 0):Void
	{
		cx = (cx - Screen.halfWidth) * camera.fullScaleX + Screen.halfWidth;
		cy = (cy - Screen.halfHeight) * camera.fullScaleY + Screen.halfHeight;
		strength *= (camera.fullScaleX * camera.fullScaleY) / 2;
		r *= (camera.fullScaleX * camera.fullScaleY) / 2;
		
		if (segments <= 0)
			segments = Std.int(10 * Math.sqrt(r));
			
		var theta = 2 * Math.PI / segments;
		var c = Math.cos(theta);
		var s = Math.sin(theta);
		
		var x = r;
		var y = 0.0;
		
		for (n in 0...segments)
		{
			var px = x + cx;
			var py = y + cy;
			
			var t = x;
			x = c * x - s * y;
			y = c * y + s * t;
			
			g2.drawLine(px, py, x + cx, y + cy, strength);
		}
	}
	
	public static inline function fillCircle(cx:Float, cy:Float, r:Float, segments:Int = 0):Void
	{
		cx = (cx - Screen.halfWidth) * camera.fullScaleX + Screen.halfWidth;
		cy = (cy - Screen.halfHeight) * camera.fullScaleY + Screen.halfHeight;
		r *= (camera.fullScaleX * camera.fullScaleY) / 2;
		
		if (segments <= 0)
			segments = Std.int(10 * Math.sqrt(r));
			
		var theta = 2 * Math.PI / segments;
		var c = Math.cos(theta);
		var s = Math.sin(theta);
		
		var x = r;
		var y = 0.0;
		
		for (n in 0...segments)
		{
			var px = x + cx;
			var py = y + cy;
			
			var t = x;
			x = c * x - s * y;
			y = c * y + s * t;
			
			g2.fillTriangle(px, py, x + cx, y + cy, cx, cy);
		}
	}
	
	public static inline function drawImage5(image:Image, x:Float, y:Float, rotation:Float, centerX:Float, centerY:Float, flipX:Bool = false, flipY:Bool = false, sourceRect:Rectangle = null, tiledWidth:Int = 0, tiledHeight:Int = 0, fillScreen:Bool = false):Void
	{
		var sx:Float, sy:Float, sw:Float, sh:Float, dw:Float, dh:Float;
		
		if (sourceRect == null)
		{
			dw = image.width * Painter.scaleX;
			dh = image.height * Painter.scaleY;
			
			sx = sy = 0;
			sw = image.width;
			sh = image.height;
		}
		else
		{
			dw = sourceRect.width * Painter.scaleX;
			dh = sourceRect.height * Painter.scaleY;
			
			sx = sourceRect.x;
			sy = sourceRect.y;
			sw = sourceRect.width;
			sh = sourceRect.height;
		}
		
		// center image
		x -= dw / 2;
		y -= dh / 2;
		
		if (flipX)
		{
			x += dw;
			dw *= -1;
		}
		if (flipY)
		{
			y += dh;
			dh *= -1;
		}
		
		// camera transformations
		x -= camera.x;
		y -= camera.y;
			
		x = (x - Screen.halfWidth) * camera.fullScaleX + Screen.halfWidth;
		y = (y - Screen.halfHeight) * camera.fullScaleY + Screen.halfHeight;
		
		dw *= camera.fullScaleX;
		dh *= camera.fullScaleY;
		
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
			else
			{
				tiledWidth = Std.int(tiledWidth * camera.fullScaleX);
				tiledHeight = Std.int(tiledHeight * camera.fullScaleY);
			}
			
			while (ty < tiledHeight)
			{
				while (tx < tiledWidth)
				{
					//Painter.drawImage2(image, sx, sy, sw, sh, x + tx, y + ty, dw, dh, rotation, centerX, centerY);
					// TODO: rotation
					g2.drawScaledSubImage(image, sx, sy, sw, sh, x + tx, y + ty, dw, dh); 
					tx += dw;
				}
				tx = 0;
				ty += dh;
			}
		}
		else
		{
			//drawImage2(image, sx, sy, sw, sh, x, y, dw, dh, rotation, centerX, centerY);
			// TODO: rotation
			g2.drawScaledSubImage(image, sx, sy, sw, sh, x, y, dw, dh);
		}
	}
	
	public static inline function drawRect2(x:Float, y:Float, width:Float, height:Float, angle:Float = 0, centerX:Float = null, centerY:Float = null, strength:Float = 1.0):Void
	{
		if (centerX == null)
			centerX = x;
		if (centerY == null)
			centerY = y;
			
		x = (x - Screen.halfWidth) * camera.fullScaleX + Screen.halfWidth;
		y = (y - Screen.halfHeight) * camera.fullScaleY + Screen.halfHeight;
		
		width *= camera.fullScaleX * Painter.scaleX;
		height *= camera.fullScaleY * Painter.scaleY;
		
		var rad = angle * Misc.toRad;
		var sin = Math.sin(rad);
        var cos = Math.cos(rad);
		
		var x1 = cos * (x - centerX) - sin * (y - centerY) + centerX;
		var y1 = sin * (x - centerX) + cos * (y - centerY) + centerY;
		
		var x2 = cos * (x + width - centerX) - sin * (y - centerY) + centerX;
		var y2 = sin * (x + width - centerX) + cos * (y - centerY) + centerY;
		
		var x3 = cos * (x + width - centerX) - sin * (y + height - centerY) + centerX;
		var y3 = sin * (x + width - centerX) + cos * (y + height - centerY) + centerY;
		
		var x4 = cos * (x - centerX) - sin * (y + height - centerY) + centerX;
		var y4 = sin * (x - centerX) + cos * (y + height - centerY) + centerY;
		
		g2.drawLine(x1, y1, x2, y2, strength);
		g2.drawLine(x2, y2, x3, y3, strength);
		g2.drawLine(x3, y3, x4, y4, strength);
		g2.drawLine(x4, y4, x1, y1, strength);
	}
	
	public static inline function fillRect2(x:Float, y:Float, width:Float, height:Float, angle:Float = 0, centerX:Float = null, centerY:Float = null):Void
	{
		if (centerX == null)
			centerX = x;
		if (centerY == null)
			centerY = y;
			
		x = (x - Screen.halfWidth) * camera.fullScaleX + Screen.halfWidth;
		y = (y - Screen.halfHeight) * camera.fullScaleY + Screen.halfHeight;
		
		width *= camera.fullScaleX * Painter.scaleX;
		height *= camera.fullScaleY * Painter.scaleY;
		
		var rad = angle * Misc.toRad;
		var sin = Math.sin(rad);
        var cos = Math.cos(rad);
		
		var x1 = cos * (x - centerX) - sin * (y - centerY) + centerX;
		var y1 = sin * (x - centerX) + cos * (y - centerY) + centerY;
		
		var x2 = cos * (x + width - centerX) - sin * (y - centerY) + centerX;
		var y2 = sin * (x + width - centerX) + cos * (y - centerY) + centerY;
		
		var x3 = cos * (x + width - centerX) - sin * (y + height - centerY) + centerX;
		var y3 = sin * (x + width - centerX) + cos * (y + height - centerY) + centerY;
		
		var x4 = cos * (x - centerX) - sin * (y + height - centerY) + centerX;
		var y4 = sin * (x - centerX) + cos * (y + height - centerY) + centerY;
		
		g2.fillTriangle(x1, y1, x2, y2, x3, y3);
		g2.fillTriangle(x3, y3, x4, y4, x1, y1);
	}
	
	// draws a cross with center at x/y
	public static inline function drawCross(x:Float, y:Float, width:Float, height:Float, strength:Float = 1):Void
	{
		width *= camera.scaleX * Painter.scaleX;
		height *= camera.scaleY * Painter.scaleY;
		// use average scale
		strength *= (camera.fullScaleX + camera.fullScaleY) / 2;
		
		x = (x - Painter.camera.x - Screen.halfWidth) * camera.fullScaleX + Screen.halfWidth;
		y = (y - Painter.camera.y - Screen.halfHeight) * camera.fullScaleY + Screen.halfHeight;
		
		x -= width / 2;
		y -= height / 2;
		
		g2.drawLine(x, y, x + width, y + height, strength);
		g2.drawLine(x + width, y, x, y + height, strength);
	}
	
	public static inline function drawString(text: String, x: Float, y: Float, scaleCenterX: Float = 0.0, scaleCenterY: Float = 0.0)
	{
		x = (x - Screen.halfWidth) * camera.fullScaleX + Screen.halfWidth;
		y = (y - Screen.halfHeight) * camera.fullScaleY + Screen.halfHeight;
		//g2.drawString(text, x, y, Painter.scaleX * camera.fullScaleX, Painter.scaleY * camera.fullScaleY, scaleCenterX, scaleCenterY);
		// TODO: implement scale/rotation
		g2.drawString(text, x, y);
	}
	
	public static inline function drawLine2(x1: Float, y1: Float, x2: Float, y2: Float, strength: Float = 1.0)
	{
		x1 = (x1 - Screen.halfWidth) * Painter.scaleX + Screen.halfWidth;
		y1 = (y1 - Screen.halfHeight) * Painter.scaleY + Screen.halfHeight;
		
		x2 = (x2 - Screen.halfWidth) * Painter.scaleX + Screen.halfWidth;
		y2 = (y2 - Screen.halfHeight) * Painter.scaleY + Screen.halfHeight;
		
		g2.drawLine(x1, y1, x2, y2, strength);
	}
	
	public static inline function set(color:Color, alpha:Float, font:Font = null):Void
	{
		Painter.color = color;
		Painter.alpha = alpha;
		Painter.setScale(scaleX, scaleY);
		Painter.font = font;
	}
	
	public static inline function setScale(scaleX:Float, scaleY:Float)
	{
		Painter.scaleX = scaleX;
		Painter.scaleY = scaleY;
	}
	
	private static inline function set_backbuffer(value:Image):Image
	{
		g2 = value.g2;
		g4 = value.g4;
		return value;
	}
	
	public static inline function clear() { g2.clear(); }
		
	private static inline function set_color(value:Color) { return g2.color = value; }
	private static inline function get_alpha() { return g2.opacity; }
	private static inline function set_alpha(value:Float) { return g2.opacity = value; }
	private static inline function set_font(value:Font) { return g2.font = value; }
}