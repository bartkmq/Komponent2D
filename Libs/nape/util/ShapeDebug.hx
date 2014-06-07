package nape.util;
import zpp_nape.Const;
import zpp_nape.constraint.PivotJoint;
import zpp_nape.ID;
import zpp_nape.constraint.Constraint;
import zpp_nape.constraint.WeldJoint;
import zpp_nape.constraint.UserConstraint;
import zpp_nape.constraint.LineJoint;
import zpp_nape.constraint.DistanceJoint;
import zpp_nape.constraint.LinearJoint;
import zpp_nape.constraint.MotorJoint;
import zpp_nape.constraint.AngleJoint;
import zpp_nape.phys.Interactor;
import zpp_nape.phys.FeatureMix;
import zpp_nape.phys.Material;
import zpp_nape.constraint.PulleyJoint;
import zpp_nape.phys.FluidProperties;
import zpp_nape.phys.Compound;
import zpp_nape.callbacks.OptionType;
import zpp_nape.phys.Body;
import zpp_nape.callbacks.CbSetPair;
import zpp_nape.callbacks.CbType;
import zpp_nape.callbacks.Callback;
import zpp_nape.callbacks.CbSet;
import zpp_nape.callbacks.Listener;
import zpp_nape.geom.GeomPoly;
import zpp_nape.geom.Mat23;
import zpp_nape.geom.ConvexRayResult;
import zpp_nape.geom.Cutter;
import zpp_nape.geom.Vec2;
import zpp_nape.geom.Ray;
import zpp_nape.geom.Convex;
import zpp_nape.geom.MatMath;
import zpp_nape.geom.Triangular;
import zpp_nape.geom.PartitionedPoly;
import zpp_nape.geom.Simplify;
import zpp_nape.geom.AABB;
import zpp_nape.geom.SweepDistance;
import zpp_nape.geom.Simple;
import zpp_nape.geom.VecMath;
import zpp_nape.geom.Monotone;
import zpp_nape.geom.Vec3;
import zpp_nape.geom.MatMN;
import zpp_nape.geom.PolyIter;
import zpp_nape.geom.MarchingSquares;
import zpp_nape.geom.Geom;
import zpp_nape.shape.Circle;
import zpp_nape.geom.Collide;
import zpp_nape.shape.Shape;
import zpp_nape.shape.Edge;
import zpp_nape.space.Broadphase;
import zpp_nape.shape.Polygon;
import zpp_nape.space.SweepPhase;
import zpp_nape.space.DynAABBPhase;
import zpp_nape.dynamics.Contact;
import zpp_nape.space.Space;
import zpp_nape.dynamics.InteractionGroup;
import zpp_nape.dynamics.InteractionFilter;
import zpp_nape.dynamics.SpaceArbiterList;
import zpp_nape.dynamics.Arbiter;
import zpp_nape.util.Array2;
import zpp_nape.util.Lists;
import zpp_nape.util.Flags;
import zpp_nape.util.Queue;
import zpp_nape.util.Debug;
import zpp_nape.util.RBTree;
import zpp_nape.util.FastHash;
import zpp_nape.util.WrapLists;
import zpp_nape.util.Pool;
import zpp_nape.util.Names;
import zpp_nape.util.Math;
import zpp_nape.util.UserData;
import zpp_nape.util.DisjointSetForest;
import nape.TArray;
import nape.Config;
import zpp_nape.util.Circular;
import nape.constraint.PivotJoint;
import nape.constraint.WeldJoint;
import nape.constraint.UserConstraint;
import nape.constraint.Constraint;
import nape.constraint.DistanceJoint;
import nape.constraint.LineJoint;
import nape.constraint.LinearJoint;
import nape.constraint.ConstraintList;
import nape.constraint.AngleJoint;
import nape.constraint.MotorJoint;
import nape.constraint.ConstraintIterator;
import nape.phys.GravMassMode;
import nape.phys.BodyList;
import nape.phys.Interactor;
import nape.phys.InertiaMode;
import nape.phys.InteractorList;
import nape.constraint.PulleyJoint;
import nape.phys.MassMode;
import nape.phys.Material;
import nape.phys.InteractorIterator;
import nape.phys.FluidProperties;
import nape.phys.BodyIterator;
import nape.phys.Compound;
import nape.phys.CompoundList;
import nape.phys.BodyType;
import nape.phys.CompoundIterator;
import nape.callbacks.InteractionListener;
import nape.callbacks.OptionType;
import nape.callbacks.PreListener;
import nape.callbacks.BodyListener;
import nape.callbacks.ListenerIterator;
import nape.callbacks.CbType;
import nape.callbacks.ListenerType;
import nape.callbacks.PreFlag;
import nape.callbacks.CbEvent;
import nape.callbacks.InteractionType;
import nape.callbacks.PreCallback;
import nape.callbacks.InteractionCallback;
import nape.phys.Body;
import nape.callbacks.ListenerList;
import nape.callbacks.BodyCallback;
import nape.callbacks.CbTypeList;
import nape.callbacks.Callback;
import nape.callbacks.ConstraintListener;
import nape.callbacks.CbTypeIterator;
import nape.callbacks.ConstraintCallback;
import nape.callbacks.Listener;
import nape.geom.Mat23;
import nape.geom.ConvexResultIterator;
import nape.geom.GeomPoly;
import nape.geom.Ray;
import nape.geom.GeomPolyIterator;
import nape.geom.Vec2Iterator;
import nape.geom.RayResult;
import nape.geom.Winding;
import nape.geom.Vec2List;
import nape.geom.RayResultIterator;
import nape.geom.AABB;
import nape.geom.IsoFunction;
import nape.geom.GeomVertexIterator;
import nape.geom.ConvexResult;
import nape.geom.GeomPolyList;
import nape.geom.Vec2;
import nape.geom.RayResultList;
import nape.geom.Vec3;
import nape.geom.MatMN;
import nape.geom.MarchingSquares;
import nape.geom.ConvexResultList;
import nape.shape.Circle;
import nape.shape.ValidationResult;
import nape.geom.Geom;
import nape.shape.ShapeIterator;
import nape.shape.Polygon;
import nape.shape.Edge;
import nape.shape.EdgeList;
import nape.shape.Shape;
import nape.shape.EdgeIterator;
import nape.shape.ShapeList;
import nape.shape.ShapeType;
import nape.space.Broadphase;
import nape.dynamics.Contact;
import nape.dynamics.InteractionGroupList;
import nape.dynamics.Arbiter;
import nape.dynamics.InteractionGroup;
import nape.dynamics.ContactIterator;
import nape.dynamics.InteractionFilter;
import nape.dynamics.ArbiterList;
import nape.space.Space;
import nape.dynamics.ArbiterIterator;
import nape.dynamics.InteractionGroupIterator;
import nape.dynamics.FluidArbiter;
import nape.dynamics.ContactList;
import nape.dynamics.ArbiterType;
import nape.dynamics.CollisionArbiter;
import nape.util.Debug;
import nape.util.BitmapDebug;
#if(flash9||openfl||nme)/**
 * Implementation of nape debug draw using flash/openfl||nme graphics API.
 * <br/><br/>
 * This debug draw is slower than BitmapDebug which is available in flash10+
 * however the BitmapDebug draw makes use of Alchemy opcodes so you may wish
 * not to use it if you are also using Stage3D and do not wish to be subject
 * to Adobe licensing.
 */
@:final#if nape_swc@:keep #end
class ShapeDebug extends Debug{
    /**
     * @private
     */
    public var zpp_inner_zn:ZPP_ShapeDebug=null;
    /**
     * Thickness to draw lines with.
     * @default 0.1
     */
    public var thickness:Float=0.0;
    /**
     * Construct new ShapeDebug with given viewport size and backgruond.
     * <br/><br/>
     * Background colour does not have much weight for a ShapeDebug which
     * always has a transparent background, but serves to bias the colours
     * chosen for drawing objects.
     *
     * @param width The width of Debug draw viewport.
     * @param height The height of Debug draw viewport.
     * @param bgColour the background colour for debug draw. (default 0x333333)
     * @return The constructed ShapeDebug.
     * @throws # If width or height are not strictly positive.
     */
    #if flib@:keep function flibopts_1(){}
    #end
    public function new(width:Int,height:Int,bgColour=0x333333){
        #if(!NAPE_RELEASE_BUILD)
        if(width<=0)throw "Error: Debug width must be > 0";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(height<=0)throw "Error: Debug height must be > 0";
        #end
        #if(!NAPE_RELEASE_BUILD)
        ZPP_Debug.internal=true;
        #end
        super();
        #if(!NAPE_RELEASE_BUILD)
        ZPP_Debug.internal=false;
        #end
        zpp_inner_zn=new ZPP_ShapeDebug(width,height);
        zpp_inner_zn.outer_zn=this;
        zpp_inner=zpp_inner_zn;
        zpp_inner.outer=this;
        this.bgColour=bgColour;
        this.thickness=0.1;
    }
    /**
     * @inheritDoc
     */
    public override function clear(){
        zpp_inner_zn.graphics.clear();
    }
    /**
     * @inheritDoc
     */
    public override function drawLine(start:Vec2,end:Vec2,colour:Int){
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.xform!=null&&!zpp_inner.xform.outer.equiorthogonal())throw "Error: Debug draw can only operate with an equiorthogonal transform!";
        #end
        {
            #if(!NAPE_RELEASE_BUILD)
            if(start!=null&&start.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(end!=null&&end.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(start==null)throw "Error: drawLine::start cannot be null";
        if(end==null)throw "Error: drawLine::end cannot be null";
        #end
        var g=zpp_inner_zn.graphics;
        g.lineStyle(thickness,colour&0xffffff,1);
        if(zpp_inner.xnull){
            g.moveTo(start.x,start.y);
            g.lineTo(end.x,end.y);
            ({
                if(({
                    start.zpp_inner.weak;
                })){
                    start.dispose();
                    true;
                }
                else{
                    false;
                }
            });
            ({
                if(({
                    end.zpp_inner.weak;
                })){
                    end.dispose();
                    true;
                }
                else{
                    false;
                }
            });
        }
        else{
            var v=zpp_inner.xform.outer.transform(start);
            g.moveTo(v.x,v.y);
            v.dispose();
            v=zpp_inner.xform.outer.transform(end);
            g.lineTo(v.x,v.y);
            v.dispose();
        }
    }
    /**
     * @inheritDoc
     */
    public override function drawCurve(start:Vec2,control:Vec2,end:Vec2,colour:Int):Void{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.xform!=null&&!zpp_inner.xform.outer.equiorthogonal())throw "Error: Debug draw can only operate with an equiorthogonal transform!";
        #end
        {
            #if(!NAPE_RELEASE_BUILD)
            if(start!=null&&start.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(control!=null&&control.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(end!=null&&end.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(start==null)throw "Error: drawCurve::start cannot be null";
        if(control==null)throw "Error: drawCurve::control cannot be null";
        if(end==null)throw "Error: drawCurve::end cannot be null";
        #end
        var g=zpp_inner_zn.graphics;
        g.lineStyle(thickness,colour&0xffffff,1);
        if(zpp_inner.xnull){
            g.moveTo(start.x,start.y);
            g.curveTo(control.x,control.y,end.x,end.y);
            ({
                if(({
                    start.zpp_inner.weak;
                })){
                    start.dispose();
                    true;
                }
                else{
                    false;
                }
            });
            ({
                if(({
                    control.zpp_inner.weak;
                })){
                    control.dispose();
                    true;
                }
                else{
                    false;
                }
            });
            ({
                if(({
                    end.zpp_inner.weak;
                })){
                    end.dispose();
                    true;
                }
                else{
                    false;
                }
            });
        }
        else{
            var u=zpp_inner.xform.outer.transform(start);
            var v=zpp_inner.xform.outer.transform(control);
            var q=zpp_inner.xform.outer.transform(end);
            g.moveTo(u.x,u.y);
            g.curveTo(v.x,v.y,q.x,q.y);
            u.dispose();
            v.dispose();
            q.dispose();
        }
    }
    /**
     * @inheritDoc
     */
    public override function drawCircle(position:Vec2,radius:Float,colour:Int){
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.xform!=null&&!zpp_inner.xform.outer.equiorthogonal())throw "Error: Debug draw can only operate with an equiorthogonal transform!";
        #end
        {
            #if(!NAPE_RELEASE_BUILD)
            if(position!=null&&position.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(position==null)throw "Error: drawCircle::position cannot be null";
        if((radius!=radius)||radius<0)throw "Error: drawCircle::radius must be >=0";
        #end
        var g=zpp_inner_zn.graphics;
        g.lineStyle(thickness,colour&0xffffff,1);
        if(zpp_inner.xnull){
            g.drawCircle(position.x,position.y,radius);
            ({
                if(({
                    position.zpp_inner.weak;
                })){
                    position.dispose();
                    true;
                }
                else{
                    false;
                }
            });
        }
        else{
            var v=zpp_inner.xform.outer.transform(position);
            g.drawCircle(v.x,v.y,radius*zpp_inner.xdet);
            v.dispose();
        }
    }
    /**
     * @inheritDoc
     */
    public override function drawAABB(aabb:AABB,colour:Int){
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.xform!=null&&!zpp_inner.xform.outer.equiorthogonal())throw "Error: Debug draw can only operate with an equiorthogonal transform!";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(aabb==null)throw "Error: drawAABB::aabb cannot be null";
        #end
        var g=zpp_inner_zn.graphics;
        g.lineStyle(thickness,colour&0xffffff,1);
        if(zpp_inner.xnull)g.drawRect(aabb.x,aabb.y,aabb.width,aabb.height);
        else{
            var v=zpp_inner.xform.outer.transform(aabb.min);
            var w=Vec2.get(aabb.width,0);
            var w2=zpp_inner.xform.outer.transform(w,true);
            var h=Vec2.get(0,aabb.height);
            var h2=zpp_inner.xform.outer.transform(h,true);
            g.moveTo(v.x,v.y);
            g.lineTo(v.x+w2.x,v.y+w2.y);
            g.lineTo(v.x+w2.x+h2.x,v.y+w2.y+h2.y);
            g.lineTo(v.x+h2.x,v.y+h2.y);
            g.lineTo(v.x,v.y);
            v.dispose();
            w.dispose();
            w2.dispose();
            h.dispose();
            h2.dispose();
        }
    }
    /**
     * @inheritDoc
     */
    public override function drawFilledTriangle(p0:Vec2,p1:Vec2,p2:Vec2,colour:Int){
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.xform!=null&&!zpp_inner.xform.outer.equiorthogonal())throw "Error: Debug draw can only operate with an equiorthogonal transform!";
        #end
        {
            #if(!NAPE_RELEASE_BUILD)
            if(p0!=null&&p0.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(p1!=null&&p1.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(p2!=null&&p2.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(p0==null||p1==null||p2==null)throw "Error: drawFilledTriangle can't use null points";
        #end
        var g=zpp_inner_zn.graphics;
        g.lineStyle(0,0,0);
        g.beginFill(colour&0xffffff,1);
        if(zpp_inner.xnull){
            g.moveTo(p0.x,p0.y);
            g.lineTo(p1.x,p1.y);
            g.lineTo(p2.x,p2.y);
            ({
                if(({
                    p0.zpp_inner.weak;
                })){
                    p0.dispose();
                    true;
                }
                else{
                    false;
                }
            });
            ({
                if(({
                    p1.zpp_inner.weak;
                })){
                    p1.dispose();
                    true;
                }
                else{
                    false;
                }
            });
            ({
                if(({
                    p2.zpp_inner.weak;
                })){
                    p2.dispose();
                    true;
                }
                else{
                    false;
                }
            });
        }
        else{
            var v=zpp_inner.xform.outer.transform(p0);
            g.moveTo(v.x,v.y);
            v.dispose();
            v=zpp_inner.xform.outer.transform(p1);
            g.lineTo(v.x,v.y);
            v.dispose();
            v=zpp_inner.xform.outer.transform(p2);
            g.lineTo(v.x,v.y);
            v.dispose();
        }
        g.endFill();
    }
    /**
     * @inheritDoc
     */
    public override function drawFilledCircle(position:Vec2,radius:Float,colour:Int){
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.xform!=null&&!zpp_inner.xform.outer.equiorthogonal())throw "Error: Debug draw can only operate with an equiorthogonal transform!";
        #end
        {
            #if(!NAPE_RELEASE_BUILD)
            if(position!=null&&position.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(position==null)throw "Error: drawFilledCircle::position cannot be null";
        if((radius!=radius)||radius<0)throw "Error: drawFilledCircle::radius must be >=0";
        #end
        var g=zpp_inner_zn.graphics;
        g.lineStyle(0,0,0);
        g.beginFill(colour&0xffffff,1);
        if(zpp_inner.xnull){
            g.drawCircle(position.x,position.y,radius);
            ({
                if(({
                    position.zpp_inner.weak;
                })){
                    position.dispose();
                    true;
                }
                else{
                    false;
                }
            });
        }
        else{
            var v=zpp_inner.xform.outer.transform(position);
            g.drawCircle(v.x,v.y,radius*zpp_inner.xdet);
            v.dispose();
        }
        g.endFill();
    }
    /**
     * @inheritDoc
     */
    public override function drawPolygon(polygon:Dynamic,colour:Int){
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.xform!=null&&!zpp_inner.xform.outer.equiorthogonal())throw "Error: Debug draw can only operate with an equiorthogonal transform!";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(polygon==null)throw "Error: Cannot draw null polygon";
        #end
        var g=zpp_inner_zn.graphics;
        g.lineStyle(thickness,colour&0xffffff,1.0);
        var fst=null;
        var fsttime=true;
        if(zpp_inner.xnull){
            {
                if(#if flash untyped __is__(polygon,Array)#else Std.is(polygon,Array)#end){
                    var lv:Array<Dynamic>=polygon;
                    for(vite in lv){
                        #if(!NAPE_RELEASE_BUILD)
                        if(vite==null)throw "Error: Array<Vec2> contains null objects";
                        #end
                        #if(!NAPE_RELEASE_BUILD)
                        if(!#if flash untyped __is__(vite,Vec2)#else Std.is(vite,Vec2)#end)throw "Error: Array<Vec2> contains non Vec2 objects";
                        #end
                        var p:Vec2=vite;
                        {
                            #if(!NAPE_RELEASE_BUILD)
                            if(p!=null&&p.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                            #end
                        };
                        {
                            if(fsttime){
                                fst=p.copy();
                                g.moveTo(p.x,p.y);
                            }
                            else g.lineTo(p.x,p.y);
                            fsttime=false;
                        };
                    }
                }
                else if(#if flash10 untyped __is__(polygon,ZPP_Const.vec2vector)#else false #end){
                    #if flash10 var lv:flash.Vector<Vec2>=polygon;
                    for(vite in lv){
                        #if(!NAPE_RELEASE_BUILD)
                        if(vite==null)throw "Error: flash.Vector<Vec2> contains null objects";
                        #end
                        var p:Vec2=vite;
                        {
                            #if(!NAPE_RELEASE_BUILD)
                            if(p!=null&&p.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                            #end
                        };
                        {
                            if(fsttime){
                                fst=p.copy();
                                g.moveTo(p.x,p.y);
                            }
                            else g.lineTo(p.x,p.y);
                            fsttime=false;
                        };
                    }
                    #end
                }
                else if(#if flash untyped __is__(polygon,Vec2List)#else Std.is(polygon,Vec2List)#end){
                    var lv:Vec2List=polygon;
                    for(p in lv){
                        #if(!NAPE_RELEASE_BUILD)
                        if(p==null)throw "Error: Vec2List contains null objects";
                        #end
                        {
                            #if(!NAPE_RELEASE_BUILD)
                            if(p!=null&&p.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                            #end
                        };
                        {
                            if(fsttime){
                                fst=p.copy();
                                g.moveTo(p.x,p.y);
                            }
                            else g.lineTo(p.x,p.y);
                            fsttime=false;
                        };
                    }
                }
                else if(#if flash untyped __is__(polygon,GeomPoly)#else Std.is(polygon,GeomPoly)#end){
                    var lv:GeomPoly=polygon;
                    {
                        #if(!NAPE_RELEASE_BUILD)
                        if(lv!=null&&lv.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
                        #end
                    };
                    var verts:ZPP_GeomVert=lv.zpp_inner.vertices;
                    if(verts!=null){
                        var vite=verts;
                        do{
                            var p=Vec2.get(vite.x,vite.y);
                            vite=vite.next;
                            {
                                if(fsttime){
                                    fst=p.copy();
                                    g.moveTo(p.x,p.y);
                                }
                                else g.lineTo(p.x,p.y);
                                fsttime=false;
                            };
                            p.dispose();
                        }
                        while(vite!=verts);
                    }
                }
                else{
                    #if(!NAPE_RELEASE_BUILD)
                    throw "Error: Invalid type for polygon object, should be Array<Vec2>, Vec2List, GeomPoly or for flash10+ flash.Vector<Vec2>";
                    #end
                }
            };
            g.lineTo(fst.x,fst.y);
            fst.dispose();
        }
        else{
            {
                if(#if flash untyped __is__(polygon,Array)#else Std.is(polygon,Array)#end){
                    var lv:Array<Dynamic>=polygon;
                    for(vite in lv){
                        #if(!NAPE_RELEASE_BUILD)
                        if(vite==null)throw "Error: Array<Vec2> contains null objects";
                        #end
                        #if(!NAPE_RELEASE_BUILD)
                        if(!#if flash untyped __is__(vite,Vec2)#else Std.is(vite,Vec2)#end)throw "Error: Array<Vec2> contains non Vec2 objects";
                        #end
                        var p:Vec2=vite;
                        {
                            #if(!NAPE_RELEASE_BUILD)
                            if(p!=null&&p.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                            #end
                        };
                        {
                            var v=zpp_inner.xform.outer.transform(p);
                            if(fsttime){
                                fst=v;
                                g.moveTo(v.x,v.y);
                            }
                            else g.lineTo(v.x,v.y);
                            if(!fsttime)v.dispose();
                            fsttime=false;
                        };
                    }
                }
                else if(#if flash10 untyped __is__(polygon,ZPP_Const.vec2vector)#else false #end){
                    #if flash10 var lv:flash.Vector<Vec2>=polygon;
                    for(vite in lv){
                        #if(!NAPE_RELEASE_BUILD)
                        if(vite==null)throw "Error: flash.Vector<Vec2> contains null objects";
                        #end
                        var p:Vec2=vite;
                        {
                            #if(!NAPE_RELEASE_BUILD)
                            if(p!=null&&p.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                            #end
                        };
                        {
                            var v=zpp_inner.xform.outer.transform(p);
                            if(fsttime){
                                fst=v;
                                g.moveTo(v.x,v.y);
                            }
                            else g.lineTo(v.x,v.y);
                            if(!fsttime)v.dispose();
                            fsttime=false;
                        };
                    }
                    #end
                }
                else if(#if flash untyped __is__(polygon,Vec2List)#else Std.is(polygon,Vec2List)#end){
                    var lv:Vec2List=polygon;
                    for(p in lv){
                        #if(!NAPE_RELEASE_BUILD)
                        if(p==null)throw "Error: Vec2List contains null objects";
                        #end
                        {
                            #if(!NAPE_RELEASE_BUILD)
                            if(p!=null&&p.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                            #end
                        };
                        {
                            var v=zpp_inner.xform.outer.transform(p);
                            if(fsttime){
                                fst=v;
                                g.moveTo(v.x,v.y);
                            }
                            else g.lineTo(v.x,v.y);
                            if(!fsttime)v.dispose();
                            fsttime=false;
                        };
                    }
                }
                else if(#if flash untyped __is__(polygon,GeomPoly)#else Std.is(polygon,GeomPoly)#end){
                    var lv:GeomPoly=polygon;
                    {
                        #if(!NAPE_RELEASE_BUILD)
                        if(lv!=null&&lv.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
                        #end
                    };
                    var verts:ZPP_GeomVert=lv.zpp_inner.vertices;
                    if(verts!=null){
                        var vite=verts;
                        do{
                            var p=Vec2.get(vite.x,vite.y);
                            vite=vite.next;
                            {
                                var v=zpp_inner.xform.outer.transform(p);
                                if(fsttime){
                                    fst=v;
                                    g.moveTo(v.x,v.y);
                                }
                                else g.lineTo(v.x,v.y);
                                if(!fsttime)v.dispose();
                                fsttime=false;
                            };
                            p.dispose();
                        }
                        while(vite!=verts);
                    }
                }
                else{
                    #if(!NAPE_RELEASE_BUILD)
                    throw "Error: Invalid type for polygon object, should be Array<Vec2>, Vec2List, GeomPoly or for flash10+ flash.Vector<Vec2>";
                    #end
                }
            };
            g.lineTo(fst.x,fst.y);
            fst.dispose();
        }
        {
            if(#if flash untyped __is__(polygon,Array)#else Std.is(polygon,Array)#end){
                var lv:Array<Vec2>=polygon;
                var i=0;
                while(i<lv.length){
                    var cur=lv[i];
                    if(({
                        if(({
                            cur.zpp_inner.weak;
                        })){
                            cur.dispose();
                            true;
                        }
                        else{
                            false;
                        }
                    })){
                        lv.splice(i,1);
                        continue;
                    }
                    i++;
                }
            }
            else if(#if flash10 untyped __is__(polygon,ZPP_Const.vec2vector)#else false #end){
                #if flash10 var lv:flash.Vector<Vec2>=polygon;
                if(!lv.fixed){
                    var i:Int=0;
                    while(i<cast lv.length){
                        var cur=lv[i];
                        if(({
                            if(({
                                cur.zpp_inner.weak;
                            })){
                                cur.dispose();
                                true;
                            }
                            else{
                                false;
                            }
                        })){
                            lv.splice(i,1);
                            continue;
                        }
                        i++;
                    }
                }
                #end
            }
            else if(#if flash untyped __is__(polygon,Vec2List)#else Std.is(polygon,Vec2List)#end){
                var lv:Vec2List=polygon;
                if(lv.zpp_inner._validate!=null)lv.zpp_inner._validate();
                var ins=lv.zpp_inner.inner;
                var pre=null;
                var cur=ins.begin();
                while(cur!=null){
                    var x=cur.elem();
                    if(({
                        x.outer.zpp_inner.weak;
                    })){
                        cur=ins.erase(pre);
                        ({
                            if(({
                                x.outer.zpp_inner.weak;
                            })){
                                x.outer.dispose();
                                true;
                            }
                            else{
                                false;
                            }
                        });
                    }
                    else{
                        pre=cur;
                        cur=cur.next;
                    }
                }
            }
        };
    }
    /**
     * @inheritDoc
     */
    public override function drawFilledPolygon(polygon:Dynamic,colour:Int){
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.xform!=null&&!zpp_inner.xform.outer.equiorthogonal())throw "Error: Debug draw can only operate with an equiorthogonal transform!";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(polygon==null)throw "Error: Cannot draw null polygon!";
        #end
        var g=zpp_inner_zn.graphics;
        g.beginFill(colour&0xffffff,1.0);
        g.lineStyle(0,0,0);
        var fst=null;
        var fsttime=true;
        if(zpp_inner.xnull){
            {
                if(#if flash untyped __is__(polygon,Array)#else Std.is(polygon,Array)#end){
                    var lv:Array<Dynamic>=polygon;
                    for(vite in lv){
                        #if(!NAPE_RELEASE_BUILD)
                        if(vite==null)throw "Error: Array<Vec2> contains null objects";
                        #end
                        #if(!NAPE_RELEASE_BUILD)
                        if(!#if flash untyped __is__(vite,Vec2)#else Std.is(vite,Vec2)#end)throw "Error: Array<Vec2> contains non Vec2 objects";
                        #end
                        var p:Vec2=vite;
                        {
                            #if(!NAPE_RELEASE_BUILD)
                            if(p!=null&&p.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                            #end
                        };
                        {
                            if(fsttime){
                                fst=p.copy();
                                g.moveTo(p.x,p.y);
                            }
                            else g.lineTo(p.x,p.y);
                            fsttime=false;
                        };
                    }
                }
                else if(#if flash10 untyped __is__(polygon,ZPP_Const.vec2vector)#else false #end){
                    #if flash10 var lv:flash.Vector<Vec2>=polygon;
                    for(vite in lv){
                        #if(!NAPE_RELEASE_BUILD)
                        if(vite==null)throw "Error: flash.Vector<Vec2> contains null objects";
                        #end
                        var p:Vec2=vite;
                        {
                            #if(!NAPE_RELEASE_BUILD)
                            if(p!=null&&p.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                            #end
                        };
                        {
                            if(fsttime){
                                fst=p.copy();
                                g.moveTo(p.x,p.y);
                            }
                            else g.lineTo(p.x,p.y);
                            fsttime=false;
                        };
                    }
                    #end
                }
                else if(#if flash untyped __is__(polygon,Vec2List)#else Std.is(polygon,Vec2List)#end){
                    var lv:Vec2List=polygon;
                    for(p in lv){
                        #if(!NAPE_RELEASE_BUILD)
                        if(p==null)throw "Error: Vec2List contains null objects";
                        #end
                        {
                            #if(!NAPE_RELEASE_BUILD)
                            if(p!=null&&p.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                            #end
                        };
                        {
                            if(fsttime){
                                fst=p.copy();
                                g.moveTo(p.x,p.y);
                            }
                            else g.lineTo(p.x,p.y);
                            fsttime=false;
                        };
                    }
                }
                else if(#if flash untyped __is__(polygon,GeomPoly)#else Std.is(polygon,GeomPoly)#end){
                    var lv:GeomPoly=polygon;
                    {
                        #if(!NAPE_RELEASE_BUILD)
                        if(lv!=null&&lv.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
                        #end
                    };
                    var verts:ZPP_GeomVert=lv.zpp_inner.vertices;
                    if(verts!=null){
                        var vite=verts;
                        do{
                            var p=Vec2.get(vite.x,vite.y);
                            vite=vite.next;
                            {
                                if(fsttime){
                                    fst=p.copy();
                                    g.moveTo(p.x,p.y);
                                }
                                else g.lineTo(p.x,p.y);
                                fsttime=false;
                            };
                            p.dispose();
                        }
                        while(vite!=verts);
                    }
                }
                else{
                    #if(!NAPE_RELEASE_BUILD)
                    throw "Error: Invalid type for polygon object, should be Array<Vec2>, Vec2List, GeomPoly or for flash10+ flash.Vector<Vec2>";
                    #end
                }
            };
            g.lineTo(fst.x,fst.y);
            fst.dispose();
        }
        else{
            {
                if(#if flash untyped __is__(polygon,Array)#else Std.is(polygon,Array)#end){
                    var lv:Array<Dynamic>=polygon;
                    for(vite in lv){
                        #if(!NAPE_RELEASE_BUILD)
                        if(vite==null)throw "Error: Array<Vec2> contains null objects";
                        #end
                        #if(!NAPE_RELEASE_BUILD)
                        if(!#if flash untyped __is__(vite,Vec2)#else Std.is(vite,Vec2)#end)throw "Error: Array<Vec2> contains non Vec2 objects";
                        #end
                        var p:Vec2=vite;
                        {
                            #if(!NAPE_RELEASE_BUILD)
                            if(p!=null&&p.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                            #end
                        };
                        {
                            var v=zpp_inner.xform.outer.transform(p);
                            if(fsttime){
                                fst=v;
                                g.moveTo(v.x,v.y);
                            }
                            else g.lineTo(v.x,v.y);
                            if(!fsttime)v.dispose();
                            fsttime=false;
                        };
                    }
                }
                else if(#if flash10 untyped __is__(polygon,ZPP_Const.vec2vector)#else false #end){
                    #if flash10 var lv:flash.Vector<Vec2>=polygon;
                    for(vite in lv){
                        #if(!NAPE_RELEASE_BUILD)
                        if(vite==null)throw "Error: flash.Vector<Vec2> contains null objects";
                        #end
                        var p:Vec2=vite;
                        {
                            #if(!NAPE_RELEASE_BUILD)
                            if(p!=null&&p.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                            #end
                        };
                        {
                            var v=zpp_inner.xform.outer.transform(p);
                            if(fsttime){
                                fst=v;
                                g.moveTo(v.x,v.y);
                            }
                            else g.lineTo(v.x,v.y);
                            if(!fsttime)v.dispose();
                            fsttime=false;
                        };
                    }
                    #end
                }
                else if(#if flash untyped __is__(polygon,Vec2List)#else Std.is(polygon,Vec2List)#end){
                    var lv:Vec2List=polygon;
                    for(p in lv){
                        #if(!NAPE_RELEASE_BUILD)
                        if(p==null)throw "Error: Vec2List contains null objects";
                        #end
                        {
                            #if(!NAPE_RELEASE_BUILD)
                            if(p!=null&&p.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                            #end
                        };
                        {
                            var v=zpp_inner.xform.outer.transform(p);
                            if(fsttime){
                                fst=v;
                                g.moveTo(v.x,v.y);
                            }
                            else g.lineTo(v.x,v.y);
                            if(!fsttime)v.dispose();
                            fsttime=false;
                        };
                    }
                }
                else if(#if flash untyped __is__(polygon,GeomPoly)#else Std.is(polygon,GeomPoly)#end){
                    var lv:GeomPoly=polygon;
                    {
                        #if(!NAPE_RELEASE_BUILD)
                        if(lv!=null&&lv.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
                        #end
                    };
                    var verts:ZPP_GeomVert=lv.zpp_inner.vertices;
                    if(verts!=null){
                        var vite=verts;
                        do{
                            var p=Vec2.get(vite.x,vite.y);
                            vite=vite.next;
                            {
                                var v=zpp_inner.xform.outer.transform(p);
                                if(fsttime){
                                    fst=v;
                                    g.moveTo(v.x,v.y);
                                }
                                else g.lineTo(v.x,v.y);
                                if(!fsttime)v.dispose();
                                fsttime=false;
                            };
                            p.dispose();
                        }
                        while(vite!=verts);
                    }
                }
                else{
                    #if(!NAPE_RELEASE_BUILD)
                    throw "Error: Invalid type for polygon object, should be Array<Vec2>, Vec2List, GeomPoly or for flash10+ flash.Vector<Vec2>";
                    #end
                }
            };
            g.lineTo(fst.x,fst.y);
            fst.dispose();
        }
        g.endFill();
        {
            if(#if flash untyped __is__(polygon,Array)#else Std.is(polygon,Array)#end){
                var lv:Array<Vec2>=polygon;
                var i=0;
                while(i<lv.length){
                    var cur=lv[i];
                    if(({
                        if(({
                            cur.zpp_inner.weak;
                        })){
                            cur.dispose();
                            true;
                        }
                        else{
                            false;
                        }
                    })){
                        lv.splice(i,1);
                        continue;
                    }
                    i++;
                }
            }
            else if(#if flash10 untyped __is__(polygon,ZPP_Const.vec2vector)#else false #end){
                #if flash10 var lv:flash.Vector<Vec2>=polygon;
                if(!lv.fixed){
                    var i:Int=0;
                    while(i<cast lv.length){
                        var cur=lv[i];
                        if(({
                            if(({
                                cur.zpp_inner.weak;
                            })){
                                cur.dispose();
                                true;
                            }
                            else{
                                false;
                            }
                        })){
                            lv.splice(i,1);
                            continue;
                        }
                        i++;
                    }
                }
                #end
            }
            else if(#if flash untyped __is__(polygon,Vec2List)#else Std.is(polygon,Vec2List)#end){
                var lv:Vec2List=polygon;
                if(lv.zpp_inner._validate!=null)lv.zpp_inner._validate();
                var ins=lv.zpp_inner.inner;
                var pre=null;
                var cur=ins.begin();
                while(cur!=null){
                    var x=cur.elem();
                    if(({
                        x.outer.zpp_inner.weak;
                    })){
                        cur=ins.erase(pre);
                        ({
                            if(({
                                x.outer.zpp_inner.weak;
                            })){
                                x.outer.dispose();
                                true;
                            }
                            else{
                                false;
                            }
                        });
                    }
                    else{
                        pre=cur;
                        cur=cur.next;
                    }
                }
            }
        };
    }
    /**
     * @inheritDoc
     */
    public override function draw(object:Dynamic){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner.xform!=null&&!zpp_inner.xform.outer.equiorthogonal())throw "Error: Debug draw can only operate with an equiorthogonal transform!";
            #end
            if(zpp_inner.xnull){
                if(#if flash untyped __is__(object,Space)#else Std.is(object,Space)#end)zpp_inner_zn.draw_space(cast(object,Space).zpp_inner,null,1.0,true);
                else if(#if flash untyped __is__(object,Compound)#else Std.is(object,Compound)#end)zpp_inner_zn.draw_compound(cast(object,Compound).zpp_inner,null,1.0,true);
                else if(#if flash untyped __is__(object,Body)#else Std.is(object,Body)#end)zpp_inner_zn.draw_body(cast(object,Body).zpp_inner,null,1.0,true);
                else if(#if flash untyped __is__(object,Shape)#else Std.is(object,Shape)#end)zpp_inner_zn.draw_shape(cast(object,Shape).zpp_inner,null,1.0,true);
                else if(#if flash untyped __is__(object,Constraint)#else Std.is(object,Constraint)#end)cast(object,Constraint).zpp_inner.draw(this);
                else{
                    #if(!NAPE_RELEASE_BUILD)
                    throw "Error: Unhandled object type for Debug draw";
                    #end
                }
            }
            else{
                if(#if flash untyped __is__(object,Space)#else Std.is(object,Space)#end)zpp_inner_zn.draw_space(cast(object,Space).zpp_inner,zpp_inner.xform,zpp_inner.xdet,false);
                else if(#if flash untyped __is__(object,Body)#else Std.is(object,Body)#end)zpp_inner_zn.draw_body(cast(object,Body).zpp_inner,zpp_inner.xform,zpp_inner.xdet,false);
                else if(#if flash untyped __is__(object,Shape)#else Std.is(object,Shape)#end)zpp_inner_zn.draw_shape(cast(object,Shape).zpp_inner,zpp_inner.xform,zpp_inner.xdet,false);
                else if(#if flash untyped __is__(object,Constraint)#else Std.is(object,Constraint)#end)cast(object,Constraint).zpp_inner.draw(this);
                else{
                    #if(!NAPE_RELEASE_BUILD)
                    throw "Error: Unhandled object type for Debug draw";
                    #end
                }
            }
        };
    }
    /**
     * @inheritDoc
     */
    public override function drawSpring(start:Vec2,end:Vec2,colour:Int,coils=3,radius=3.0){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(start!=null&&start.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        }
        {
            #if(!NAPE_RELEASE_BUILD)
            if(end!=null&&end.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        }
        #if(!NAPE_RELEASE_BUILD)
        if(start==null)throw "Error: drawCurve::start cannot be null";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(end==null)throw "Error: drawCurve::end cannot be null";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(coils<0)throw "Error: drawCurve::coils must be >= 0";
        #end
        if(coils==0)drawLine(start,end,colour);
        else{
            var dx:Float=end.x-start.x;
            var dy:Float=end.y-start.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((dx!=dx));
                };
                if(!res)throw "assert("+"!assert_isNaN(dx)"+") :: "+("vec_new(in n: "+"d"+",in x: "+"end.x-start.x"+",in y: "+"end.y-start.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((dy!=dy));
                };
                if(!res)throw "assert("+"!assert_isNaN(dy)"+") :: "+("vec_new(in n: "+"d"+",in x: "+"end.x-start.x"+",in y: "+"end.y-start.y"+")");
                #end
            };
            {
                var t=(1.0/(coils*4));
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"d"+",in s: "+"1.0/(coils*4)"+")");
                    #end
                };
                dx*=t;
                dy*=t;
            };
            var nx:Float=0.0;
            var ny:Float=0.0;
            {
                nx=dx;
                ny=dy;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((nx!=nx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"dx"+",in y: "+"dy"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((ny!=ny));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"dx"+",in y: "+"dy"+")");
                    #end
                };
            };
            if((nx*nx+ny*ny)<0.1)return;
            {
                {
                    var d=(nx*nx+ny*ny);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            d!=0.0;
                        };
                        if(!res)throw "assert("+"d!=0.0"+") :: "+("vec_normalise(in n: "+"n"+")");
                        #end
                    };
                    var imag=ZPP_Math.invsqrt(d);
                    {
                        var t=(imag);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"n"+",in s: "+"imag"+")");
                            #end
                        };
                        nx*=t;
                        ny*=t;
                    };
                };
                {
                    var t=nx;
                    nx=-ny;
                    ny=t;
                };
            };
            {
                var t=(radius*2);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"n"+",in s: "+"radius*2"+")");
                    #end
                };
                nx*=t;
                ny*=t;
            };
            var u=start.copy();
            var v=Vec2.get();
            var q=Vec2.get();
            for(i in 0...coils){
                v.x=u.x+dx+nx;
                v.y=u.y+dy+ny;
                q.x=u.x+dx*2;
                q.y=u.y+dy*2;
                drawCurve(u,v,q,colour);
                u.x=q.x;
                u.y=q.y;
                v.x=u.x+dx-nx;
                v.y=u.y+dy-ny;
                q.x=u.x+dx*2;
                q.y=u.y+dy*2;
                drawCurve(u,v,q,colour);
                u.x=q.x;
                u.y=q.y;
            }
            u.dispose();
            v.dispose();
            q.dispose();
        }
        ({
            if(({
                start.zpp_inner.weak;
            })){
                start.dispose();
                true;
            }
            else{
                false;
            }
        });
        ({
            if(({
                end.zpp_inner.weak;
            })){
                end.dispose();
                true;
            }
            else{
                false;
            }
        });
    }
}
#end
