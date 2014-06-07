package zpp_nape.util;
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
import nape.util.ShapeDebug;
#if(flash9||openfl||nme)#if nape_swc@:keep #end
class ZPP_Debug{
    public static var internal=false;
    public var outer:Debug=null;
    public var isbmp:Bool=false;
    #if flash10 public var d_bmp:ZPP_BitmapDebug=null;
    #end
    public var d_shape:ZPP_ShapeDebug=null;
    public var bg_r:Float=0.0;
    public var bg_g:Float=0.0;
    public var bg_b:Float=0.0;
    public var bg_col:Int=0;
    public var xform:ZPP_Mat23=null;
    public var xnull:Bool=false;
    public var xdet:Float=0.0;
    public var width:Int=0;
    public var height:Int=0;
    public var viewport:ZPP_AABB=null;
    public var iport:ZPP_AABB=null;
    public function new(width:Int,height:Int){
        xnull=true;
        xdet=1.0;
        this.width=width;
        this.height=height;
        viewport=ZPP_AABB.get(0,0,width,height);
        iport=ZPP_AABB.get(0,0,width,height);
        tmpab=new ZPP_AABB();
    }
    private function xform_invalidate(){
        xdet=ZPP_Math.sqrt(({
            var x=xform.outer.determinant;
            x<0?-x:x;
        }));
        xnull=xform.a==1.0&&xform.b==0.0&&xform.c==0.0&&xform.d==1.0&&xform.tx==0.0&&xform.ty==0.0;
        var qmat=xform.outer.inverse();
        var q=Vec2.get();
        var v=qmat.transform(q);
        {
            iport.minx=v.x;
            iport.miny=v.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((iport.minx!=iport.minx));
                };
                if(!res)throw "assert("+"!assert_isNaN(iport.minx)"+") :: "+("vec_set(in n: "+"iport.min"+",in x: "+"v.x"+",in y: "+"v.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((iport.miny!=iport.miny));
                };
                if(!res)throw "assert("+"!assert_isNaN(iport.miny)"+") :: "+("vec_set(in n: "+"iport.min"+",in x: "+"v.x"+",in y: "+"v.y"+")");
                #end
            };
        };
        {
            iport.maxx=iport.minx;
            iport.maxy=iport.miny;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((iport.maxx!=iport.maxx));
                };
                if(!res)throw "assert("+"!assert_isNaN(iport.maxx)"+") :: "+("vec_set(in n: "+"iport.max"+",in x: "+"iport.minx"+",in y: "+"iport.miny"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((iport.maxy!=iport.maxy));
                };
                if(!res)throw "assert("+"!assert_isNaN(iport.maxy)"+") :: "+("vec_set(in n: "+"iport.max"+",in x: "+"iport.minx"+",in y: "+"iport.miny"+")");
                #end
            };
        };
        v.dispose();
        q.x=width;
        v=qmat.transform(q);
        iport.setExpandPoint(v.x,v.y);
        v.dispose();
        q.y=height;
        v=qmat.transform(q);
        iport.setExpandPoint(v.x,v.y);
        v.dispose();
        q.x=0;
        v=qmat.transform(q);
        iport.setExpandPoint(v.x,v.y);
        v.dispose();
        q.dispose();
    }
    public function setform(){
        xform=new Mat23().zpp_inner;
        xform._invalidate=xform_invalidate;
    }
    public var tmpab:ZPP_AABB=null;
    public function cull(aabb:ZPP_AABB){
        if(xnull)return aabb.intersect(viewport);
        else{
            var qx:Float=0.0;
            var qy:Float=0.0;
            var vx:Float=0.0;
            var vy:Float=0.0;
            {
                vx=aabb.minx;
                vy=aabb.miny;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((vx!=vx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(vx)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"aabb.minx"+",in y: "+"aabb.miny"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((vy!=vy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(vy)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"aabb.minx"+",in y: "+"aabb.miny"+")");
                    #end
                };
            };
            if(false){
                tmpab.minx=vx;
                tmpab.miny=vy;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((tmpab.minx!=tmpab.minx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(tmpab.minx)"+") :: "+("vec_set(in n: "+"tmpab.min"+",in x: "+"vx"+",in y: "+"vy"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((tmpab.miny!=tmpab.miny));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(tmpab.miny)"+") :: "+("vec_set(in n: "+"tmpab.min"+",in x: "+"vx"+",in y: "+"vy"+")");
                    #end
                };
            }
            else{
                tmpab.minx=xform.a*vx+xform.b*vy+xform.tx;
                tmpab.miny=xform.c*vx+xform.d*vy+xform.ty;
            };
            {
                tmpab.maxx=tmpab.minx;
                tmpab.maxy=tmpab.miny;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((tmpab.maxx!=tmpab.maxx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(tmpab.maxx)"+") :: "+("vec_set(in n: "+"tmpab.max"+",in x: "+"tmpab.minx"+",in y: "+"tmpab.miny"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((tmpab.maxy!=tmpab.maxy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(tmpab.maxy)"+") :: "+("vec_set(in n: "+"tmpab.max"+",in x: "+"tmpab.minx"+",in y: "+"tmpab.miny"+")");
                    #end
                };
            };
            vx=aabb.maxx;
            if(false){
                qx=vx;
                qy=vy;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((qx!=qx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(qx)"+") :: "+("vec_set(in n: "+"q"+",in x: "+"vx"+",in y: "+"vy"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((qy!=qy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(qy)"+") :: "+("vec_set(in n: "+"q"+",in x: "+"vx"+",in y: "+"vy"+")");
                    #end
                };
            }
            else{
                qx=xform.a*vx+xform.b*vy+xform.tx;
                qy=xform.c*vx+xform.d*vy+xform.ty;
            };
            tmpab.setExpandPoint(qx,qy);
            vy=aabb.maxy;
            if(false){
                qx=vx;
                qy=vy;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((qx!=qx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(qx)"+") :: "+("vec_set(in n: "+"q"+",in x: "+"vx"+",in y: "+"vy"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((qy!=qy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(qy)"+") :: "+("vec_set(in n: "+"q"+",in x: "+"vx"+",in y: "+"vy"+")");
                    #end
                };
            }
            else{
                qx=xform.a*vx+xform.b*vy+xform.tx;
                qy=xform.c*vx+xform.d*vy+xform.ty;
            };
            tmpab.setExpandPoint(qx,qy);
            vx=aabb.minx;
            if(false){
                qx=vx;
                qy=vy;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((qx!=qx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(qx)"+") :: "+("vec_set(in n: "+"q"+",in x: "+"vx"+",in y: "+"vy"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((qy!=qy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(qy)"+") :: "+("vec_set(in n: "+"q"+",in x: "+"vx"+",in y: "+"vy"+")");
                    #end
                };
            }
            else{
                qx=xform.a*vx+xform.b*vy+xform.tx;
                qy=xform.c*vx+xform.d*vy+xform.ty;
            };
            tmpab.setExpandPoint(qx,qy);
            return tmpab.intersect(viewport);
        }
    }
    public function sup_setbg(bgcol:Int){
        bg_r=(bgcol>>16)&0xff;
        bg_g=(bgcol>>8)&0xff;
        bg_b=(bgcol)&0xff;
        this.bg_col=bgcol;
    }
}
#if nape_swc@:keep #end
class ZPP_ShapeDebug extends ZPP_Debug{
    public var outer_zn:ShapeDebug=null;
    public var shape:flash.display.Shape=null;
    public var graphics:flash.display.Graphics=null;
    public function new(width:Int,height:Int){
        super(width,height);
        shape=new flash.display.Shape();
        shape.scrollRect=new flash.geom.Rectangle(0,0,width,height);
        graphics=shape.graphics;
        isbmp=false;
        d_shape=this;
    }
    public function setbg(bgColor:Int){
        sup_setbg(bgColor);
    }
    public var compoundstack:ZNPList_ZPP_Compound=null;
    public function draw_compound(compound:ZPP_Compound,xform:ZPP_Mat23,xdet:Float,xnull:Bool){
        {
            var cx_ite=compound.compounds.begin();
            while(cx_ite!=null){
                var c=cx_ite.elem();
                draw_compound(c,xform,xdet,xnull);
                cx_ite=cx_ite.next;
            }
        };
        {
            var cx_ite=compound.bodies.begin();
            while(cx_ite!=null){
                var b=cx_ite.elem();
                if(b.outer.debugDraw)draw_body(b,xform,xdet,xnull);
                cx_ite=cx_ite.next;
            }
        };
        {
            var cx_ite=compound.constraints.begin();
            while(cx_ite!=null){
                var c=cx_ite.elem();
                if(c.active&&c.outer.debugDraw)c.draw(outer);
                cx_ite=cx_ite.next;
            }
        };
    }
    var shapeList:ShapeList=null;
    var bodyList:BodyList=null;
    public function draw_space(space:ZPP_Space,xform:ZPP_Mat23,xdet:Float,xnull:Bool){
        if(outer.cullingEnabled){
            if(outer.drawBodies){
                if(outer.drawBodyDetail){
                    var bods=bodyList=space.bphase.bodiesInAABB(iport,false,false,null,bodyList);
                    while(!bods.empty()){
                        var b=bods.shift();
                        if(b.debugDraw)draw_body(b.zpp_inner,xform,xdet,xnull);
                    }
                }
                else{
                    var shapes=shapeList=space.bphase.shapesInAABB(iport,false,false,null,shapeList);
                    while(!shapes.empty()){
                        var s=shapes.shift();
                        if(s.body.debugDraw)draw_shape(s.zpp_inner,xform,xdet,xnull);
                    }
                }
            }
        }
        else{
            if(outer.drawBodies){
                if(compoundstack==null)compoundstack=new ZNPList_ZPP_Compound();
                {
                    var cx_ite=space.bodies.begin();
                    while(cx_ite!=null){
                        var b=cx_ite.elem();
                        if(b.outer.debugDraw)draw_body(b,xform,xdet,xnull);
                        cx_ite=cx_ite.next;
                    }
                };
                {
                    var cx_ite=space.compounds.begin();
                    while(cx_ite!=null){
                        var c=cx_ite.elem();
                        compoundstack.add(c);
                        cx_ite=cx_ite.next;
                    }
                };
                while(!compoundstack.empty()){
                    var x=compoundstack.pop_unsafe();
                    {
                        var cx_ite=x.bodies.begin();
                        while(cx_ite!=null){
                            var b=cx_ite.elem();
                            if(b.outer.debugDraw)draw_body(b,xform,xdet,xnull);
                            cx_ite=cx_ite.next;
                        }
                    };
                    {
                        var cx_ite=x.compounds.begin();
                        while(cx_ite!=null){
                            var c=cx_ite.elem();
                            compoundstack.add(c);
                            cx_ite=cx_ite.next;
                        }
                    };
                }
            }
        }
        if(outer.drawCollisionArbiters||outer.drawFluidArbiters||outer.drawSensorArbiters)for(arb in space.outer.arbiters)draw_arbiter(arb.zpp_inner,xform,xdet,xnull);
        if(outer.drawConstraints){
            if(compoundstack==null)compoundstack=new ZNPList_ZPP_Compound();
            {
                var cx_ite=space.constraints.begin();
                while(cx_ite!=null){
                    var c=cx_ite.elem();
                    if(c.active&&c.outer.debugDraw)c.draw(outer);
                    cx_ite=cx_ite.next;
                }
            };
            {
                var cx_ite=space.compounds.begin();
                while(cx_ite!=null){
                    var c=cx_ite.elem();
                    compoundstack.add(c);
                    cx_ite=cx_ite.next;
                }
            };
            while(!compoundstack.empty()){
                var x=compoundstack.pop_unsafe();
                {
                    var cx_ite=x.constraints.begin();
                    while(cx_ite!=null){
                        var c=cx_ite.elem();
                        if(c.active&&c.outer.debugDraw)c.draw(outer);
                        cx_ite=cx_ite.next;
                    }
                };
                {
                    var cx_ite=x.compounds.begin();
                    while(cx_ite!=null){
                        var c=cx_ite.elem();
                        compoundstack.add(c);
                        cx_ite=cx_ite.next;
                    }
                };
            }
        }
    }
    public function draw_body(body:ZPP_Body,xform:ZPP_Mat23,xdet:Float,xnull:Bool){
        {
            var cx_ite=body.shapes.begin();
            while(cx_ite!=null){
                var s=cx_ite.elem();
                draw_shape(s,xform,xdet,xnull);
                cx_ite=cx_ite.next;
            }
        };
        if(outer.drawBodyDetail){
            var col={
                var idc:Int;
                if(outer.colour==null)idc=(#if flash9 untyped __int__(0xffffff*Math.exp(-(body.id%500)/1500))#else Std.int(0xffffff*Math.exp(-(body.id%500)/1500))#end);
                else idc=outer.colour(body.id);
                var _r=(((idc&0xff0000)>>16))*0.7;
                var _g=(((idc&0xff00)>>8))*0.7;
                var _b=(idc&0xff)*0.7;
                if(body.space!=null&&body.outer.isSleeping){
                    _r=0.4*_r+0.6*bg_r;
                    _g=0.4*_g+0.6*bg_g;
                    _b=0.4*_b+0.6*bg_b;
                }
                0xff000000|(((#if flash9 untyped __int__(_r)#else Std.int(_r)#end))<<16)|(((#if flash9 untyped __int__(_g)#else Std.int(_g)#end))<<8)|((#if flash9 untyped __int__(_b)#else Std.int(_b)#end));
            };
            graphics.lineStyle(outer_zn.thickness,{
                var col=col;
                var ncol=0xff0000;
                var f=0.8;
                var _r:Int=(#if flash9 untyped __int__(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#else Std.int(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#end);
                var _g:Int=(#if flash9 untyped __int__(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#else Std.int(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#end);
                var _b:Int=(#if flash9 untyped __int__(((col)&0xff)*f+((ncol)&0xff)*(1-f))#else Std.int(((col)&0xff)*f+((ncol)&0xff)*(1-f))#end);
                0xff000000|(_r<<16)|(_g<<8)|(_b);
            },
            1);
            var px:Float=0.0;
            var py:Float=0.0;
            var qx:Float=0.0;
            var qy:Float=0.0;
            if(!body.shapes.empty()){
                body.validate_worldCOM();
                if(xnull){
                    px=body.worldCOMx;
                    py=body.worldCOMy;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((px!=px));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(px)"+") :: "+("vec_set(in n: "+"p"+",in x: "+"body.worldCOMx"+",in y: "+"body.worldCOMy"+")");
                        #end
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((py!=py));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(py)"+") :: "+("vec_set(in n: "+"p"+",in x: "+"body.worldCOMx"+",in y: "+"body.worldCOMy"+")");
                        #end
                    };
                }
                else{
                    px=xform.a*body.worldCOMx+xform.b*body.worldCOMy+xform.tx;
                    py=xform.c*body.worldCOMx+xform.d*body.worldCOMy+xform.ty;
                };
                graphics.drawCircle(px,py,1);
                body.validate_aabb();
                {
                    if(xnull)graphics.drawRect(body.aabb.minx,body.aabb.miny,body.aabb.width(),body.aabb.height());
                    else{
                        var ox:Float=0.0;
                        var oy:Float=0.0;
                        if(false){
                            ox=body.aabb.minx;
                            oy=body.aabb.miny;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((ox!=ox));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(ox)"+") :: "+("vec_set(in n: "+"o"+",in x: "+"body.aabb.minx"+",in y: "+"body.aabb.miny"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((oy!=oy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(oy)"+") :: "+("vec_set(in n: "+"o"+",in x: "+"body.aabb.minx"+",in y: "+"body.aabb.miny"+")");
                                #end
                            };
                        }
                        else{
                            ox=xform.a*body.aabb.minx+xform.b*body.aabb.miny+xform.tx;
                            oy=xform.c*body.aabb.minx+xform.d*body.aabb.miny+xform.ty;
                        };
                        var wx:Float=body.aabb.width();
                        var wy:Float=0;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((wx!=wx));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(wx)"+") :: "+("vec_new(in n: "+"w"+",in x: "+"body.aabb.width()"+",in y: "+"0"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((wy!=wy));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(wy)"+") :: "+("vec_new(in n: "+"w"+",in x: "+"body.aabb.width()"+",in y: "+"0"+")");
                            #end
                        };
                        if(!false){
                            var t=xform.a*wx+xform.b*wy;
                            wy=xform.c*wx+xform.d*wy;
                            wx=t;
                        };
                        var hx:Float=0;
                        var hy:Float=body.aabb.height();
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((hx!=hx));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(hx)"+") :: "+("vec_new(in n: "+"h"+",in x: "+"0"+",in y: "+"body.aabb.height()"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((hy!=hy));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(hy)"+") :: "+("vec_new(in n: "+"h"+",in x: "+"0"+",in y: "+"body.aabb.height()"+")");
                            #end
                        };
                        if(!false){
                            var t=xform.a*hx+xform.b*hy;
                            hy=xform.c*hx+xform.d*hy;
                            hx=t;
                        };
                        graphics.moveTo(ox,oy);
                        graphics.lineTo(ox+wx,oy+wy);
                        graphics.lineTo(ox+wx+hx,oy+wy+hy);
                        graphics.lineTo(ox+hx,oy+hy);
                        graphics.lineTo(ox,oy);
                    }
                };
            }
            if(xnull){
                qx=body.pre_posx;
                qy=body.pre_posy;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((qx!=qx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(qx)"+") :: "+("vec_set(in n: "+"q"+",in x: "+"body.pre_posx"+",in y: "+"body.pre_posy"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((qy!=qy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(qy)"+") :: "+("vec_set(in n: "+"q"+",in x: "+"body.pre_posx"+",in y: "+"body.pre_posy"+")");
                    #end
                };
            }
            else{
                qx=xform.a*body.pre_posx+xform.b*body.pre_posy+xform.tx;
                qy=xform.c*body.pre_posx+xform.d*body.pre_posy+xform.ty;
            };
            if(xnull){
                px=body.posx;
                py=body.posy;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((px!=px));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(px)"+") :: "+("vec_set(in n: "+"p"+",in x: "+"body.posx"+",in y: "+"body.posy"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((py!=py));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(py)"+") :: "+("vec_set(in n: "+"p"+",in x: "+"body.posx"+",in y: "+"body.posy"+")");
                    #end
                };
            }
            else{
                px=xform.a*body.posx+xform.b*body.posy+xform.tx;
                py=xform.c*body.posx+xform.d*body.posy+xform.ty;
            };
            graphics.moveTo(qx,qy);
            graphics.lineTo(px,py);
            graphics.drawCircle(px,py,1);
        }
    }
    public function draw_shape(shape:ZPP_Shape,xform:ZPP_Mat23,xdet:Float,xnull:Bool){
        var col={
            var idc:Int;
            if(outer.colour==null)idc=(#if flash9 untyped __int__(0xffffff*Math.exp(-(shape.id%500)/1500))#else Std.int(0xffffff*Math.exp(-(shape.id%500)/1500))#end);
            else idc=outer.colour(shape.id);
            var _r=(((idc&0xff0000)>>16))*0.7;
            var _g=(((idc&0xff00)>>8))*0.7;
            var _b=(idc&0xff)*0.7;
            if(false){
                _r=0.4*_r+0.6*bg_r;
                _g=0.4*_g+0.6*bg_g;
                _b=0.4*_b+0.6*bg_b;
            }
            0xff000000|(((#if flash9 untyped __int__(_r)#else Std.int(_r)#end))<<16)|(((#if flash9 untyped __int__(_g)#else Std.int(_g)#end))<<8)|((#if flash9 untyped __int__(_b)#else Std.int(_b)#end));
        };
        var body=shape.body;
        if(body!=null){
            var bcol={
                var idc:Int;
                if(outer.colour==null)idc=(#if flash9 untyped __int__(0xffffff*Math.exp(-(body.id%500)/1500))#else Std.int(0xffffff*Math.exp(-(body.id%500)/1500))#end);
                else idc=outer.colour(body.id);
                var _r=(((idc&0xff0000)>>16))*0.7;
                var _g=(((idc&0xff00)>>8))*0.7;
                var _b=(idc&0xff)*0.7;
                if(body.space!=null&&body.outer.isSleeping){
                    _r=0.4*_r+0.6*bg_r;
                    _g=0.4*_g+0.6*bg_g;
                    _b=0.4*_b+0.6*bg_b;
                }
                0xff000000|(((#if flash9 untyped __int__(_r)#else Std.int(_r)#end))<<16)|(((#if flash9 untyped __int__(_g)#else Std.int(_g)#end))<<8)|((#if flash9 untyped __int__(_b)#else Std.int(_b)#end));
            };
            col={
                var col=col;
                var ncol=bcol;
                var f=0.2;
                var _r:Int=(#if flash9 untyped __int__(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#else Std.int(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#end);
                var _g:Int=(#if flash9 untyped __int__(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#else Std.int(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#end);
                var _b:Int=(#if flash9 untyped __int__(((col)&0xff)*f+((ncol)&0xff)*(1-f))#else Std.int(((col)&0xff)*f+((ncol)&0xff)*(1-f))#end);
                0xff000000|(_r<<16)|(_g<<8)|(_b);
            };
            graphics.lineStyle(outer_zn.thickness,col,1.0);
            if(shape.isCircle()){
                var circ=shape.circle;
                circ.validate_worldCOM();
                var vx=circ.worldCOMx;
                var vy=circ.worldCOMy;
                if(!xnull){
                    var t=xform.a*vx+xform.b*vy+xform.tx;
                    vy=xform.c*vx+xform.d*vy+xform.ty;
                    vx=t;
                };
                graphics.drawCircle(vx,vy,circ.radius*xdet);
                if(outer.drawShapeAngleIndicators){
                    var v0x=circ.worldCOMx+0.3*circ.radius*body.axisy;
                    var v0y=circ.worldCOMy+0.3*circ.radius*body.axisx;
                    var v1x=circ.worldCOMx+circ.radius*body.axisy;
                    var v1y=circ.worldCOMy+circ.radius*body.axisx;
                    if(!xnull){
                        var t=xform.a*v0x+xform.b*v0y+xform.tx;
                        v0y=xform.c*v0x+xform.d*v0y+xform.ty;
                        v0x=t;
                    };
                    if(!xnull){
                        var t=xform.a*v1x+xform.b*v1y+xform.tx;
                        v1y=xform.c*v1x+xform.d*v1y+xform.ty;
                        v1x=t;
                    };
                    graphics.moveTo(v0x,v0y);
                    graphics.lineTo(v1x,v1y);
                }
            }
            else{
                var poly=shape.polygon;
                poly.validate_gverts();
                var u=poly.gverts.front();
                var vx=u.x;
                var vy=u.y;
                if(!xnull){
                    var t=xform.a*vx+xform.b*vy+xform.tx;
                    vy=xform.c*vx+xform.d*vy+xform.ty;
                    vx=t;
                };
                graphics.moveTo(vx,vy);
                var vox=vx;
                var voy=vy;
                {
                    var cx_ite=poly.gverts.begin().next;
                    while(cx_ite!=null){
                        var u=cx_ite.elem();
                        {
                            vx=u.x;
                            vy=u.y;
                            if(!xnull){
                                var t=xform.a*vx+xform.b*vy+xform.tx;
                                vy=xform.c*vx+xform.d*vy+xform.ty;
                                vx=t;
                            };
                            graphics.lineTo(vx,vy);
                        };
                        cx_ite=cx_ite.next;
                    }
                };
                graphics.lineTo(vox,voy);
                if(outer.drawShapeAngleIndicators){
                    poly.validate_worldCOM();
                    if(xnull){
                        vx=poly.worldCOMx;
                        vy=poly.worldCOMy;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((vx!=vx));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(vx)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"poly.worldCOMx"+",in y: "+"poly.worldCOMy"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((vy!=vy));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(vy)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"poly.worldCOMx"+",in y: "+"poly.worldCOMy"+")");
                            #end
                        };
                    }
                    else{
                        vx=xform.a*poly.worldCOMx+xform.b*poly.worldCOMy+xform.tx;
                        vy=xform.c*poly.worldCOMx+xform.d*poly.worldCOMy+xform.ty;
                    };
                    graphics.moveTo(vx,vy);
                    graphics.lineTo(vox,voy);
                }
            }
            if(outer.drawShapeDetail){
                shape.validate_worldCOM();
                graphics.lineStyle(outer_zn.thickness,{
                    var col=col;
                    var ncol=0xff0000;
                    var f=0.8;
                    var _r:Int=(#if flash9 untyped __int__(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#else Std.int(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#end);
                    var _g:Int=(#if flash9 untyped __int__(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#else Std.int(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#end);
                    var _b:Int=(#if flash9 untyped __int__(((col)&0xff)*f+((ncol)&0xff)*(1-f))#else Std.int(((col)&0xff)*f+((ncol)&0xff)*(1-f))#end);
                    0xff000000|(_r<<16)|(_g<<8)|(_b);
                },
                1);
                var vx:Float=0.0;
                var vy:Float=0.0;
                if(xnull){
                    vx=shape.worldCOMx;
                    vy=shape.worldCOMy;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((vx!=vx));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(vx)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"shape.worldCOMx"+",in y: "+"shape.worldCOMy"+")");
                        #end
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((vy!=vy));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(vy)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"shape.worldCOMx"+",in y: "+"shape.worldCOMy"+")");
                        #end
                    };
                }
                else{
                    vx=xform.a*shape.worldCOMx+xform.b*shape.worldCOMy+xform.tx;
                    vy=xform.c*shape.worldCOMx+xform.d*shape.worldCOMy+xform.ty;
                };
                graphics.drawCircle(vx,vy,1);
                shape.validate_aabb();
                {
                    if(xnull)graphics.drawRect(shape.aabb.minx,shape.aabb.miny,shape.aabb.width(),shape.aabb.height());
                    else{
                        var ox:Float=0.0;
                        var oy:Float=0.0;
                        if(false){
                            ox=shape.aabb.minx;
                            oy=shape.aabb.miny;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((ox!=ox));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(ox)"+") :: "+("vec_set(in n: "+"o"+",in x: "+"shape.aabb.minx"+",in y: "+"shape.aabb.miny"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((oy!=oy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(oy)"+") :: "+("vec_set(in n: "+"o"+",in x: "+"shape.aabb.minx"+",in y: "+"shape.aabb.miny"+")");
                                #end
                            };
                        }
                        else{
                            ox=xform.a*shape.aabb.minx+xform.b*shape.aabb.miny+xform.tx;
                            oy=xform.c*shape.aabb.minx+xform.d*shape.aabb.miny+xform.ty;
                        };
                        var wx:Float=shape.aabb.width();
                        var wy:Float=0;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((wx!=wx));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(wx)"+") :: "+("vec_new(in n: "+"w"+",in x: "+"shape.aabb.width()"+",in y: "+"0"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((wy!=wy));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(wy)"+") :: "+("vec_new(in n: "+"w"+",in x: "+"shape.aabb.width()"+",in y: "+"0"+")");
                            #end
                        };
                        if(!false){
                            var t=xform.a*wx+xform.b*wy;
                            wy=xform.c*wx+xform.d*wy;
                            wx=t;
                        };
                        var hx:Float=0;
                        var hy:Float=shape.aabb.height();
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((hx!=hx));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(hx)"+") :: "+("vec_new(in n: "+"h"+",in x: "+"0"+",in y: "+"shape.aabb.height()"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((hy!=hy));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(hy)"+") :: "+("vec_new(in n: "+"h"+",in x: "+"0"+",in y: "+"shape.aabb.height()"+")");
                            #end
                        };
                        if(!false){
                            var t=xform.a*hx+xform.b*hy;
                            hy=xform.c*hx+xform.d*hy;
                            hx=t;
                        };
                        graphics.moveTo(ox,oy);
                        graphics.lineTo(ox+wx,oy+wy);
                        graphics.lineTo(ox+wx+hx,oy+wy+hy);
                        graphics.lineTo(ox+hx,oy+hy);
                        graphics.lineTo(ox,oy);
                    }
                };
            }
        }
    }
    public function draw_arbiter(arb:ZPP_Arbiter,xform:ZPP_Mat23,xdet:Float,xnull:Bool){
        var vx:Float=0.0;
        var vy:Float=0.0;
        if(arb.outer.isSensorArbiter()){
            if(outer.drawSensorArbiters){
                var sarb=arb.outer;
                graphics.lineStyle(outer_zn.thickness,{
                    var col=0xff00;
                    var ncol=~bg_col;
                    var f=0.7;
                    var _r:Int=(#if flash9 untyped __int__(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#else Std.int(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#end);
                    var _g:Int=(#if flash9 untyped __int__(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#else Std.int(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#end);
                    var _b:Int=(#if flash9 untyped __int__(((col)&0xff)*f+((ncol)&0xff)*(1-f))#else Std.int(((col)&0xff)*f+((ncol)&0xff)*(1-f))#end);
                    0xff000000|(_r<<16)|(_g<<8)|(_b);
                },
                1);
                if(xnull){
                    vx=sarb.shape1.worldCOM.x;
                    vy=sarb.shape1.worldCOM.y;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((vx!=vx));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(vx)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"sarb.shape1.worldCOM.x"+",in y: "+"sarb.shape1.worldCOM.y"+")");
                        #end
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((vy!=vy));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(vy)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"sarb.shape1.worldCOM.x"+",in y: "+"sarb.shape1.worldCOM.y"+")");
                        #end
                    };
                }
                else{
                    vx=xform.a*sarb.shape1.worldCOM.x+xform.b*sarb.shape1.worldCOM.y+xform.tx;
                    vy=xform.c*sarb.shape1.worldCOM.x+xform.d*sarb.shape1.worldCOM.y+xform.ty;
                };
                graphics.moveTo(vx,vy);
                if(xnull){
                    vx=sarb.shape2.worldCOM.x;
                    vy=sarb.shape2.worldCOM.y;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((vx!=vx));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(vx)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"sarb.shape2.worldCOM.x"+",in y: "+"sarb.shape2.worldCOM.y"+")");
                        #end
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((vy!=vy));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(vy)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"sarb.shape2.worldCOM.x"+",in y: "+"sarb.shape2.worldCOM.y"+")");
                        #end
                    };
                }
                else{
                    vx=xform.a*sarb.shape2.worldCOM.x+xform.b*sarb.shape2.worldCOM.y+xform.tx;
                    vy=xform.c*sarb.shape2.worldCOM.x+xform.d*sarb.shape2.worldCOM.y+xform.ty;
                };
                graphics.lineTo(vx,vy);
            }
        }
        else if(arb.outer.isFluidArbiter()){
            if(outer.drawFluidArbiters){
                var farb=arb.outer.fluidArbiter;
                graphics.lineStyle(outer_zn.thickness,{
                    var col=0xff;
                    var ncol=~bg_col;
                    var f=0.7;
                    var _r:Int=(#if flash9 untyped __int__(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#else Std.int(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#end);
                    var _g:Int=(#if flash9 untyped __int__(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#else Std.int(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#end);
                    var _b:Int=(#if flash9 untyped __int__(((col)&0xff)*f+((ncol)&0xff)*(1-f))#else Std.int(((col)&0xff)*f+((ncol)&0xff)*(1-f))#end);
                    0xff000000|(_r<<16)|(_g<<8)|(_b);
                },
                1);
                if(xnull){
                    vx=farb.position.x;
                    vy=farb.position.y;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((vx!=vx));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(vx)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"farb.position.x"+",in y: "+"farb.position.y"+")");
                        #end
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((vy!=vy));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(vy)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"farb.position.x"+",in y: "+"farb.position.y"+")");
                        #end
                    };
                }
                else{
                    vx=xform.a*farb.position.x+xform.b*farb.position.y+xform.tx;
                    vy=xform.c*farb.position.x+xform.d*farb.position.y+xform.ty;
                };
                graphics.drawCircle(vx,vy,0.75);
            }
        }
        else{
            if(outer.drawCollisionArbiters){
                var carb=arb.outer.collisionArbiter;
                if(!carb.contacts.empty()){
                    var px:Float=0.0;
                    var py:Float=0.0;
                    if(carb.contacts.length==2){
                        var c1=carb.contacts.at(0).position;
                        var c2=carb.contacts.at(1).position;
                        var n=carb.normal;
                        var x=0.661437828;
                        var y=0.75;
                        if((n.y*c1.x-n.x*c1.y)<(n.y*c2.x-n.x*c2.y)){
                            x=-x;
                            y=-y;
                        }
                        graphics.lineStyle(outer_zn.thickness,{
                            var col=0xff;
                            var ncol=~bg_col;
                            var f=0.7;
                            var _r:Int=(#if flash9 untyped __int__(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#else Std.int(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#end);
                            var _g:Int=(#if flash9 untyped __int__(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#else Std.int(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#end);
                            var _b:Int=(#if flash9 untyped __int__(((col)&0xff)*f+((ncol)&0xff)*(1-f))#else Std.int(((col)&0xff)*f+((ncol)&0xff)*(1-f))#end);
                            0xff000000|(_r<<16)|(_g<<8)|(_b);
                        },
                        1);
                        {
                            vx=c1.x+n.x*y-n.y*x;
                            vy=c1.y+n.y*y+n.x*x;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((vx!=vx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(vx)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"c1.x+n.x*y-n.y*x"+",in y: "+"c1.y+n.y*y+n.x*x"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((vy!=vy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(vy)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"c1.x+n.x*y-n.y*x"+",in y: "+"c1.y+n.y*y+n.x*x"+")");
                                #end
                            };
                        };
                        if(!xnull){
                            var t=xform.a*vx+xform.b*vy+xform.tx;
                            vy=xform.c*vx+xform.d*vy+xform.ty;
                            vx=t;
                        };
                        graphics.moveTo(vx,vy);
                        {
                            vx=c2.x+n.x*y+n.y*x;
                            vy=c2.y+n.y*y-n.x*x;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((vx!=vx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(vx)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"c2.x+n.x*y+n.y*x"+",in y: "+"c2.y+n.y*y-n.x*x"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((vy!=vy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(vy)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"c2.x+n.x*y+n.y*x"+",in y: "+"c2.y+n.y*y-n.x*x"+")");
                                #end
                            };
                        };
                        if(!xnull){
                            var t=xform.a*vx+xform.b*vy+xform.tx;
                            vy=xform.c*vx+xform.d*vy+xform.ty;
                            vx=t;
                        };
                        graphics.lineTo(vx,vy);
                        graphics.lineStyle(outer_zn.thickness,{
                            var col=0xff0000;
                            var ncol=~bg_col;
                            var f=0.7;
                            var _r:Int=(#if flash9 untyped __int__(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#else Std.int(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#end);
                            var _g:Int=(#if flash9 untyped __int__(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#else Std.int(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#end);
                            var _b:Int=(#if flash9 untyped __int__(((col)&0xff)*f+((ncol)&0xff)*(1-f))#else Std.int(((col)&0xff)*f+((ncol)&0xff)*(1-f))#end);
                            0xff000000|(_r<<16)|(_g<<8)|(_b);
                        },
                        1);
                        {
                            vx=c1.x-n.x*y-n.y*x;
                            vy=c1.y-n.y*y+n.x*x;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((vx!=vx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(vx)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"c1.x-n.x*y-n.y*x"+",in y: "+"c1.y-n.y*y+n.x*x"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((vy!=vy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(vy)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"c1.x-n.x*y-n.y*x"+",in y: "+"c1.y-n.y*y+n.x*x"+")");
                                #end
                            };
                        };
                        if(!xnull){
                            var t=xform.a*vx+xform.b*vy+xform.tx;
                            vy=xform.c*vx+xform.d*vy+xform.ty;
                            vx=t;
                        };
                        graphics.moveTo(vx,vy);
                        {
                            vx=c2.x-n.x*y+n.y*x;
                            vy=c2.y-n.y*y-n.x*x;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((vx!=vx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(vx)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"c2.x-n.x*y+n.y*x"+",in y: "+"c2.y-n.y*y-n.x*x"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((vy!=vy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(vy)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"c2.x-n.x*y+n.y*x"+",in y: "+"c2.y-n.y*y-n.x*x"+")");
                                #end
                            };
                        };
                        if(!xnull){
                            var t=xform.a*vx+xform.b*vy+xform.tx;
                            vy=xform.c*vx+xform.d*vy+xform.ty;
                            vx=t;
                        };
                        graphics.lineTo(vx,vy);
                        {
                            px=0.5*(c1.x+c2.x);
                            py=0.5*(c1.y+c2.y);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((px!=px));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(px)"+") :: "+("vec_set(in n: "+"p"+",in x: "+"0.5*(c1.x+c2.x)"+",in y: "+"0.5*(c1.y+c2.y)"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((py!=py));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(py)"+") :: "+("vec_set(in n: "+"p"+",in x: "+"0.5*(c1.x+c2.x)"+",in y: "+"0.5*(c1.y+c2.y)"+")");
                                #end
                            };
                        };
                        if(!xnull){
                            var t=xform.a*px+xform.b*py+xform.tx;
                            py=xform.c*px+xform.d*py+xform.ty;
                            px=t;
                        };
                    }
                    else{
                        {
                            px=carb.contacts.at(0).position.x;
                            py=carb.contacts.at(0).position.y;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((px!=px));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(px)"+") :: "+("vec_set(in n: "+"p"+",in x: "+"carb.contacts.at(0).position.x"+",in y: "+"carb.contacts.at(0).position.y"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((py!=py));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(py)"+") :: "+("vec_set(in n: "+"p"+",in x: "+"carb.contacts.at(0).position.x"+",in y: "+"carb.contacts.at(0).position.y"+")");
                                #end
                            };
                        };
                        if(!xnull){
                            var t=xform.a*px+xform.b*py+xform.tx;
                            py=xform.c*px+xform.d*py+xform.ty;
                            px=t;
                        };
                        graphics.lineStyle(outer_zn.thickness,{
                            var col=0xff00ff;
                            var ncol=~bg_col;
                            var f=0.7;
                            var _r:Int=(#if flash9 untyped __int__(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#else Std.int(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#end);
                            var _g:Int=(#if flash9 untyped __int__(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#else Std.int(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#end);
                            var _b:Int=(#if flash9 untyped __int__(((col)&0xff)*f+((ncol)&0xff)*(1-f))#else Std.int(((col)&0xff)*f+((ncol)&0xff)*(1-f))#end);
                            0xff000000|(_r<<16)|(_g<<8)|(_b);
                        },
                        1);
                        graphics.drawCircle(px,py,1);
                    }
                    graphics.lineStyle(outer_zn.thickness,{
                        var col=~bg_col;
                        var ncol=bg_col;
                        var f=0.7;
                        var _r:Int=(#if flash9 untyped __int__(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#else Std.int(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#end);
                        var _g:Int=(#if flash9 untyped __int__(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#else Std.int(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#end);
                        var _b:Int=(#if flash9 untyped __int__(((col)&0xff)*f+((ncol)&0xff)*(1-f))#else Std.int(((col)&0xff)*f+((ncol)&0xff)*(1-f))#end);
                        0xff000000|(_r<<16)|(_g<<8)|(_b);
                    },
                    1);
                    graphics.moveTo(px,py);
                    {
                        vx=carb.normal.x*5;
                        vy=carb.normal.y*5;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((vx!=vx));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(vx)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"carb.normal.x*5"+",in y: "+"carb.normal.y*5"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((vy!=vy));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(vy)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"carb.normal.x*5"+",in y: "+"carb.normal.y*5"+")");
                            #end
                        };
                    };
                    if(!xnull){
                        var t=xform.a*vx+xform.b*vy;
                        vy=xform.c*vx+xform.d*vy;
                        vx=t;
                    };
                    graphics.lineTo(px+vx,py+vy);
                }
            }
        }
    }
}
#end
#if flash10#if nape_swc@:keep #end
class ZPP_BitmapDebug extends ZPP_Debug{
    public var outer_zn:BitmapDebug=null;
    public var bitmap:flash.display.Bitmap=null;
    public var rect:flash.geom.Rectangle=null;
    public var bit:flash.display.BitmapData=null;
    public var bytes:flash.utils.ByteArray=null;
    public var bgbytes:flash.utils.ByteArray=null;
    public var transp:Bool=false;
    public var filledVertices:Array<Vec2>;
    public var filledXs:Array<Int>;
    public function setbg(bgColor:Int){
        sup_setbg(bgColor);
        if(!transp){
            bgbytes.position=0;
            flash.Memory.select(bgbytes);
            for(i in 0...(bytes.length>>2))flash.Memory.setI32(i<<2,bgColor);
            flash.Memory.select(bytes);
        }
    }
    public function new(w:Int,h:Int,bgcol:Int,transparent:Bool){
        super(w,h);
        this.transp=transparent;
        filledVertices=[];
        filledXs=[];
        bytes=new flash.utils.ByteArray();
        bytes.length=(w*h)<<2;
        bytes.endian=flash.utils.Endian.LITTLE_ENDIAN;
        bit=new flash.display.BitmapData(w,h,transparent,transparent?0:bgcol);
        rect=bit.rect;
        bgbytes=new flash.utils.ByteArray();
        bgbytes.length=bytes.length;
        bgbytes.endian=flash.utils.Endian.LITTLE_ENDIAN;
        setbg(bgcol);
        if(transparent){
            flash.Memory.select(bgbytes);
            for(i in 0...(bytes.length>>3))flash.Memory.setDouble(i<<3,0);
            flash.Memory.select(bytes);
        }
        bitmap=new flash.display.Bitmap(bit,flash.display.PixelSnapping.NEVER,false);
        isbmp=true;
        d_bmp=this;
    }
    public function clear(){
        bytes.position=0;
        bgbytes.position=0;
        bytes.writeBytes(bgbytes);
    }
    public function prepare(){
        flash.Memory.select(bytes);
    }
    public function flush(){
        bit.lock();
        bytes.position=0;
        bit.setPixels(rect,bytes);
        bit.unlock();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function index(x:Int,y:Int){
        return y*width+x;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setpix(ind:Int,col:Int){
        flash.Memory.setI32(ind<<2,col);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setpixel(x:Int,y:Int,col:Int){
        if(x>=0&&x<width&&y>=0&&y<height)setpix(index(x,y),col);
    }
    public function __line(x0:Int,y0:Int,x1:Int,y1:Int,col:Int){
        if(x0<x1){
            var dx=x1-x0;
            if(y0<y1){
                var dy=y1-y0;
                var off=width;
                var err=dx-dy;
                var ind=index(x0,y0);
                while(true){
                    if(x0>=0&&x0<width&&y0>=0&&y0<height)setpix(ind,col);
                    if(x0==x1&&y0==y1)break;
                    var e2=err<<1;
                    if(e2>-dy){
                        err-=dy;
                        x0++;
                        ind++;
                    }
                    if(e2<dx){
                        err+=dx;
                        y0++;
                        ind+=off;
                    }
                };
            }
            else{
                var dy=y0-y1;
                var off=-width;
                var err=dx-dy;
                var ind=index(x0,y0);
                while(true){
                    if(x0>=0&&x0<width&&y0>=0&&y0<height)setpix(ind,col);
                    if(x0==x1&&y0==y1)break;
                    var e2=err<<1;
                    if(e2>-dy){
                        err-=dy;
                        x0++;
                        ind++;
                    }
                    if(e2<dx){
                        err+=dx;
                        y0--;
                        ind+=off;
                    }
                };
            }
        }
        else{
            var dx=x0-x1;
            if(y0<y1){
                var dy=y1-y0;
                var off=width;
                var err=dx-dy;
                var ind=index(x0,y0);
                while(true){
                    if(x0>=0&&x0<width&&y0>=0&&y0<height)setpix(ind,col);
                    if(x0==x1&&y0==y1)break;
                    var e2=err<<1;
                    if(e2>-dy){
                        err-=dy;
                        x0--;
                        ind--;
                    }
                    if(e2<dx){
                        err+=dx;
                        y0++;
                        ind+=off;
                    }
                };
            }
            else{
                var dy=y0-y1;
                var off=-width;
                var err=dx-dy;
                var ind=index(x0,y0);
                while(true){
                    if(x0>=0&&x0<width&&y0>=0&&y0<height)setpix(ind,col);
                    if(x0==x1&&y0==y1)break;
                    var e2=err<<1;
                    if(e2>-dy){
                        err-=dy;
                        x0--;
                        ind--;
                    }
                    if(e2<dx){
                        err+=dx;
                        y0--;
                        ind+=off;
                    }
                };
            }
        }
    }
    public function __curve(u:Vec2,v:Vec2,q:Vec2,col:Int){
        var mqx:Float=0.25*(u.x+2*v.x+q.x);
        var mqy:Float=0.25*(u.y+2*v.y+q.y);
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((mqx!=mqx));
            };
            if(!res)throw "assert("+"!assert_isNaN(mqx)"+") :: "+("vec_new(in n: "+"mq"+",in x: "+"0.25*(u.x+2*v.x+q.x)"+",in y: "+"0.25*(u.y+2*v.y+q.y)"+")");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((mqy!=mqy));
            };
            if(!res)throw "assert("+"!assert_isNaN(mqy)"+") :: "+("vec_new(in n: "+"mq"+",in x: "+"0.25*(u.x+2*v.x+q.x)"+",in y: "+"0.25*(u.y+2*v.y+q.y)"+")");
            #end
        };
        var mlx:Float=0.5*(u.x+q.x);
        var mly:Float=0.5*(u.y+q.y);
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((mlx!=mlx));
            };
            if(!res)throw "assert("+"!assert_isNaN(mlx)"+") :: "+("vec_new(in n: "+"ml"+",in x: "+"0.5*(u.x+q.x)"+",in y: "+"0.5*(u.y+q.y)"+")");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((mly!=mly));
            };
            if(!res)throw "assert("+"!assert_isNaN(mly)"+") :: "+("vec_new(in n: "+"ml"+",in x: "+"0.5*(u.x+q.x)"+",in y: "+"0.5*(u.y+q.y)"+")");
            #end
        };
        if(ZPP_VecMath.vec_dsq(mqx,mqy,mlx,mly)<0.3)__line((#if flash9 untyped __int__(u.x+0.5)#else Std.int(u.x+0.5)#end),(#if flash9 untyped __int__(u.y+0.5)#else Std.int(u.y+0.5)#end),(#if flash9 untyped __int__(q.x+0.5)#else Std.int(q.x+0.5)#end),(#if flash9 untyped __int__(q.y+0.5)#else Std.int(q.y+0.5)#end),col);
        else{
            var m1=Vec2.get(0.5*(u.x+v.x),0.5*(u.y+v.y));
            var m2=Vec2.get(0.5*(q.x+v.x),0.5*(q.y+v.y));
            var mm=Vec2.get(0.5*(m1.x+m2.x),0.5*(m1.y+m2.y));
            __curve(u,m1,mm,col);
            m1.dispose();
            __curve(q,m2,mm,col);
            m2.dispose();
            mm.dispose();
        }
    }
    public function __tri(p0:Vec2,p1:Vec2,p2:Vec2,col:Int){
        if(!xnull){
            p0=xform.outer.transform(p0);
            p1=xform.outer.transform(p1);
            p2=xform.outer.transform(p2);
        }
        var p0x=__round(p0.x);
        var p0y=__round(p0.y);
        var p1x=__round(p1.x);
        var p1y=__round(p1.y);
        var p2x=__round(p2.x);
        var p2y=__round(p2.y);
        if(!xnull){
            p0.dispose();
            p1.dispose();
            p2.dispose();
        }
        if(p1y<p0y){
            {
                var t=p0x;
                p0x=p1x;
                p1x=t;
            };
            {
                var t=p0y;
                p0y=p1y;
                p1y=t;
            };
        }
        if(p2y<p0y){
            var t=p0x;
            var s=p1x;
            p0x=p2x;
            p1x=t;
            p2x=s;
            var t=p0y;
            var s=p1y;
            p0y=p2y;
            p1y=t;
            p2y=s;
        }
        else if(p2y<p1y){
            {
                var t=p1x;
                p1x=p2x;
                p2x=t;
            };
            {
                var t=p1y;
                p1y=p2y;
                p2y=t;
            };
        }
        if(p0y!=p2y&&!(p0x==p1x&&p1x==p2x)){
            if((p0x-p2x)*(p1y-p2y)-(p0y-p2y)*(p1x-p2x)<0){
                var ml=(p2x-p0x)/(p2y-p0y);
                if(p0y!=p1y){
                    var mm=(p1x-p0x)/(p1y-p0y);
                    var fy=p0y;
                    var sy=p1y;
                    if(fy<0)fy=0;
                    if(sy>=height)sy=height-1;
                    for(y in fy...sy){
                        var dy=y-p0y;
                        var xm=__round(p0x+dy*mm);
                        var xl=__round(p0x+dy*ml);
                        if(true){
                            if(xm<0)xm=0;
                            if(xl>=width)xl=width-1;
                            for(x in xm...(xl+1))setpixel(x,y,col);
                        };
                        else{
                            if(xl<0)xl=0;
                            if(xm>=width)xm=width-1;
                            for(x in xl...(xm+1))setpixel(x,y,col);
                        };
                    }
                }
                if(p1y!=p2y){
                    var mm=(p2x-p1x)/(p2y-p1y);
                    var fy=p1y;
                    var sy=p2y;
                    if(fy<0)fy=0;
                    if(sy>=height)sy=height-1;
                    for(y in fy...(sy+1)){
                        var xm=__round(p1x+(y-p1y)*mm);
                        var xl=__round(p0x+(y-p0y)*ml);
                        if(true){
                            if(xm<0)xm=0;
                            if(xl>=width)xl=width-1;
                            for(x in xm...(xl+1))setpixel(x,y,col);
                        };
                        else{
                            if(xl<0)xl=0;
                            if(xm>=width)xm=width-1;
                            for(x in xl...(xm+1))setpixel(x,y,col);
                        };
                    }
                }
            };
            else{
                var ml=(p2x-p0x)/(p2y-p0y);
                if(p0y!=p1y){
                    var mm=(p1x-p0x)/(p1y-p0y);
                    var fy=p0y;
                    var sy=p1y;
                    if(fy<0)fy=0;
                    if(sy>=height)sy=height-1;
                    for(y in fy...sy){
                        var dy=y-p0y;
                        var xm=__round(p0x+dy*mm);
                        var xl=__round(p0x+dy*ml);
                        if(false){
                            if(xm<0)xm=0;
                            if(xl>=width)xl=width-1;
                            for(x in xm...(xl+1))setpixel(x,y,col);
                        };
                        else{
                            if(xl<0)xl=0;
                            if(xm>=width)xm=width-1;
                            for(x in xl...(xm+1))setpixel(x,y,col);
                        };
                    }
                }
                if(p1y!=p2y){
                    var mm=(p2x-p1x)/(p2y-p1y);
                    var fy=p1y;
                    var sy=p2y;
                    if(fy<0)fy=0;
                    if(sy>=height)sy=height-1;
                    for(y in fy...(sy+1)){
                        var xm=__round(p1x+(y-p1y)*mm);
                        var xl=__round(p0x+(y-p0y)*ml);
                        if(false){
                            if(xm<0)xm=0;
                            if(xl>=width)xl=width-1;
                            for(x in xm...(xl+1))setpixel(x,y,col);
                        };
                        else{
                            if(xl<0)xl=0;
                            if(xm>=width)xm=width-1;
                            for(x in xl...(xm+1))setpixel(x,y,col);
                        };
                    }
                }
            };
        }
    }
    public function __circle(x0:Int,y0:Int,radius:Int,col:Int){
        if(radius==0)setpixel(x0,y0,col);
        else{
            if(radius==1){
                setpixel(x0,y0+1,col);
                setpixel(x0,y0-1,col);
                setpixel(x0+1,y0,col);
                setpixel(x0-1,y0,col);
                setpixel(x0-1,y0-1,col);
                setpixel(x0-1,y0+1,col);
                setpixel(x0+1,y0-1,col);
                setpixel(x0+1,y0+1,col);
            }
            else{
                var x=0;
                var y=radius;
                var p=3-2*radius;
                while(y>=x){
                    setpixel(x0+x,y0+y,col);
                    setpixel(x0+x,y0-y,col);
                    setpixel(x0-x,y0+y,col);
                    setpixel(x0-x,y0-y,col);
                    setpixel(x0+y,y0+x,col);
                    setpixel(x0+y,y0-x,col);
                    setpixel(x0-y,y0+x,col);
                    setpixel(x0-y,y0-x,col);
                    if(p<0)p+=6+((x++)<<2);
                    else p+=10+((x++-y--)<<2);
                }
            }
        }
    }
    public function __fcircle(x0:Int,y0:Int,radius:Int,col:Int){
        if(radius==0)setpixel(x0,y0,col);
        else{
            if(radius==1){
                setpixel(x0,y0+1,col);
                setpixel(x0,y0-1,col);
                setpixel(x0+1,y0,col);
                setpixel(x0-1,y0,col);
                setpixel(x0-1,y0-1,col);
                setpixel(x0-1,y0+1,col);
                setpixel(x0+1,y0-1,col);
                setpixel(x0+1,y0+1,col);
            }
            else{
                var x=0;
                var y=radius;
                var p=3-2*radius;
                while(y>=x){
                    for(ix in x0-x...x0+x+1){
                        setpixel(ix,y0-y,col);
                        setpixel(ix,y0+y,col);
                    }
                    for(ix in x0-y...x0+y+1){
                        setpixel(ix,y0+x,col);
                        setpixel(ix,y0-x,col);
                    }
                    if(p<0)p+=6+((x++)<<2);
                    else p+=10+((x++-y--)<<2);
                }
            }
        }
    }
    public var penx:Int=0;
    public var peny:Int=0;
    public var colour:Int=0;
    public#if NAPE_NO_INLINE#else inline #end
    function __round(x:Float):Int{
        return(#if flash9 untyped __int__(x+0.5)#else Std.int(x+0.5)#end);
    }
    public var compoundstack:ZNPList_ZPP_Compound=null;
    public function draw_compound(compound:ZPP_Compound,xform:ZPP_Mat23,xdet:Float,xnull:Bool){
        {
            var cx_ite=compound.compounds.begin();
            while(cx_ite!=null){
                var c=cx_ite.elem();
                draw_compound(c,xform,xdet,xnull);
                cx_ite=cx_ite.next;
            }
        };
        {
            var cx_ite=compound.bodies.begin();
            while(cx_ite!=null){
                var b=cx_ite.elem();
                if(b.outer.debugDraw)draw_body(b,xform,xdet,xnull);
                cx_ite=cx_ite.next;
            }
        };
        {
            var cx_ite=compound.constraints.begin();
            while(cx_ite!=null){
                var c=cx_ite.elem();
                if(c.active&&c.outer.debugDraw)c.draw(outer);
                cx_ite=cx_ite.next;
            }
        };
    }
    var shapeList:ShapeList=null;
    var bodyList:BodyList=null;
    public function draw_space(space:ZPP_Space,xform:ZPP_Mat23,xdet:Float,xnull:Bool){
        if(outer.cullingEnabled){
            if(outer.drawBodies){
                if(outer.drawBodyDetail){
                    var bods=bodyList=space.bphase.bodiesInAABB(iport,false,false,null,bodyList);
                    while(!bods.empty()){
                        var b=bods.shift();
                        if(b.debugDraw)draw_body(b.zpp_inner,xform,xdet,xnull);
                    }
                }
                else{
                    var shapes=shapeList=space.bphase.shapesInAABB(iport,false,false,null,shapeList);
                    while(!shapes.empty()){
                        var s=shapes.shift();
                        if(s.body.debugDraw)draw_shape(s.zpp_inner,xform,xdet,xnull);
                    }
                }
            }
        }
        else{
            if(outer.drawBodies){
                if(compoundstack==null)compoundstack=new ZNPList_ZPP_Compound();
                {
                    var cx_ite=space.bodies.begin();
                    while(cx_ite!=null){
                        var b=cx_ite.elem();
                        if(b.outer.debugDraw)draw_body(b,xform,xdet,xnull);
                        cx_ite=cx_ite.next;
                    }
                };
                {
                    var cx_ite=space.compounds.begin();
                    while(cx_ite!=null){
                        var c=cx_ite.elem();
                        compoundstack.add(c);
                        cx_ite=cx_ite.next;
                    }
                };
                while(!compoundstack.empty()){
                    var x=compoundstack.pop_unsafe();
                    {
                        var cx_ite=x.bodies.begin();
                        while(cx_ite!=null){
                            var b=cx_ite.elem();
                            if(b.outer.debugDraw)draw_body(b,xform,xdet,xnull);
                            cx_ite=cx_ite.next;
                        }
                    };
                    {
                        var cx_ite=x.compounds.begin();
                        while(cx_ite!=null){
                            var c=cx_ite.elem();
                            compoundstack.add(c);
                            cx_ite=cx_ite.next;
                        }
                    };
                }
            }
        }
        if(outer.drawCollisionArbiters||outer.drawFluidArbiters||outer.drawSensorArbiters)for(arb in space.outer.arbiters)draw_arbiter(arb.zpp_inner,xform,xdet,xnull);
        if(outer.drawConstraints){
            if(compoundstack==null)compoundstack=new ZNPList_ZPP_Compound();
            {
                var cx_ite=space.constraints.begin();
                while(cx_ite!=null){
                    var c=cx_ite.elem();
                    if(c.active&&c.outer.debugDraw)c.draw(outer);
                    cx_ite=cx_ite.next;
                }
            };
            {
                var cx_ite=space.compounds.begin();
                while(cx_ite!=null){
                    var c=cx_ite.elem();
                    compoundstack.add(c);
                    cx_ite=cx_ite.next;
                }
            };
            while(!compoundstack.empty()){
                var x=compoundstack.pop_unsafe();
                {
                    var cx_ite=x.constraints.begin();
                    while(cx_ite!=null){
                        var c=cx_ite.elem();
                        if(c.active&&c.outer.debugDraw)c.draw(outer);
                        cx_ite=cx_ite.next;
                    }
                };
                {
                    var cx_ite=x.compounds.begin();
                    while(cx_ite!=null){
                        var c=cx_ite.elem();
                        compoundstack.add(c);
                        cx_ite=cx_ite.next;
                    }
                };
            }
        }
    }
    public function draw_body(body:ZPP_Body,xform:ZPP_Mat23,xdet:Float,xnull:Bool){
        {
            var cx_ite=body.shapes.begin();
            while(cx_ite!=null){
                var s=cx_ite.elem();
                draw_shape(s,xform,xdet,xnull);
                cx_ite=cx_ite.next;
            }
        };
        if(outer.drawBodyDetail){
            var col={
                var idc:Int;
                if(outer.colour==null)idc=(#if flash9 untyped __int__(0xffffff*Math.exp(-(body.id%500)/1500))#else Std.int(0xffffff*Math.exp(-(body.id%500)/1500))#end);
                else idc=outer.colour(body.id);
                var _r=(((idc&0xff0000)>>16))*0.7;
                var _g=(((idc&0xff00)>>8))*0.7;
                var _b=(idc&0xff)*0.7;
                if(body.space!=null&&body.outer.isSleeping){
                    _r=0.4*_r+0.6*bg_r;
                    _g=0.4*_g+0.6*bg_g;
                    _b=0.4*_b+0.6*bg_b;
                }
                0xff000000|(((#if flash9 untyped __int__(_r)#else Std.int(_r)#end))<<16)|(((#if flash9 untyped __int__(_g)#else Std.int(_g)#end))<<8)|((#if flash9 untyped __int__(_b)#else Std.int(_b)#end));
            };
            {
                colour={
                    var col=col;
                    var ncol=0xff0000;
                    var f=0.8;
                    var _r:Int=(#if flash9 untyped __int__(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#else Std.int(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#end);
                    var _g:Int=(#if flash9 untyped __int__(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#else Std.int(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#end);
                    var _b:Int=(#if flash9 untyped __int__(((col)&0xff)*f+((ncol)&0xff)*(1-f))#else Std.int(((col)&0xff)*f+((ncol)&0xff)*(1-f))#end);
                    0xff000000|(_r<<16)|(_g<<8)|(_b);
                };
            };
            var px:Float=0.0;
            var py:Float=0.0;
            var qx:Float=0.0;
            var qy:Float=0.0;
            if(!body.shapes.empty()){
                body.validate_worldCOM();
                if(xnull){
                    px=body.worldCOMx;
                    py=body.worldCOMy;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((px!=px));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(px)"+") :: "+("vec_set(in n: "+"p"+",in x: "+"body.worldCOMx"+",in y: "+"body.worldCOMy"+")");
                        #end
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((py!=py));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(py)"+") :: "+("vec_set(in n: "+"p"+",in x: "+"body.worldCOMx"+",in y: "+"body.worldCOMy"+")");
                        #end
                    };
                }
                else{
                    px=xform.a*body.worldCOMx+xform.b*body.worldCOMy+xform.tx;
                    py=xform.c*body.worldCOMx+xform.d*body.worldCOMy+xform.ty;
                };
                {
                    penx=__round(px);
                    peny=__round(py);
                    __circle(penx,peny,__round(1),colour);
                };
                body.validate_aabb();
                {
                    if(xnull){
                        __aabb(body.aabb,colour);
                    };
                    else{
                        var ox:Float=0.0;
                        var oy:Float=0.0;
                        if(false){
                            ox=body.aabb.minx;
                            oy=body.aabb.miny;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((ox!=ox));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(ox)"+") :: "+("vec_set(in n: "+"o"+",in x: "+"body.aabb.minx"+",in y: "+"body.aabb.miny"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((oy!=oy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(oy)"+") :: "+("vec_set(in n: "+"o"+",in x: "+"body.aabb.minx"+",in y: "+"body.aabb.miny"+")");
                                #end
                            };
                        }
                        else{
                            ox=xform.a*body.aabb.minx+xform.b*body.aabb.miny+xform.tx;
                            oy=xform.c*body.aabb.minx+xform.d*body.aabb.miny+xform.ty;
                        };
                        var wx:Float=body.aabb.width();
                        var wy:Float=0;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((wx!=wx));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(wx)"+") :: "+("vec_new(in n: "+"w"+",in x: "+"body.aabb.width()"+",in y: "+"0"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((wy!=wy));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(wy)"+") :: "+("vec_new(in n: "+"w"+",in x: "+"body.aabb.width()"+",in y: "+"0"+")");
                            #end
                        };
                        if(!false){
                            var t=xform.a*wx+xform.b*wy;
                            wy=xform.c*wx+xform.d*wy;
                            wx=t;
                        };
                        var hx:Float=0;
                        var hy:Float=body.aabb.height();
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((hx!=hx));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(hx)"+") :: "+("vec_new(in n: "+"h"+",in x: "+"0"+",in y: "+"body.aabb.height()"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((hy!=hy));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(hy)"+") :: "+("vec_new(in n: "+"h"+",in x: "+"0"+",in y: "+"body.aabb.height()"+")");
                            #end
                        };
                        if(!false){
                            var t=xform.a*hx+xform.b*hy;
                            hy=xform.c*hx+xform.d*hy;
                            hx=t;
                        };
                        {
                            penx=__round(ox);
                            peny=__round(oy);
                        };
                        {
                            var nx=__round(ox+wx);
                            var ny=__round(oy+wy);
                            __line(penx,peny,nx,ny,colour);
                            penx=nx;
                            peny=ny;
                        };
                        {
                            var nx=__round(ox+wx+hx);
                            var ny=__round(oy+wy+hy);
                            __line(penx,peny,nx,ny,colour);
                            penx=nx;
                            peny=ny;
                        };
                        {
                            var nx=__round(ox+hx);
                            var ny=__round(oy+hy);
                            __line(penx,peny,nx,ny,colour);
                            penx=nx;
                            peny=ny;
                        };
                        {
                            var nx=__round(ox);
                            var ny=__round(oy);
                            __line(penx,peny,nx,ny,colour);
                            penx=nx;
                            peny=ny;
                        };
                    }
                };
            }
            if(xnull){
                qx=body.pre_posx;
                qy=body.pre_posy;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((qx!=qx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(qx)"+") :: "+("vec_set(in n: "+"q"+",in x: "+"body.pre_posx"+",in y: "+"body.pre_posy"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((qy!=qy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(qy)"+") :: "+("vec_set(in n: "+"q"+",in x: "+"body.pre_posx"+",in y: "+"body.pre_posy"+")");
                    #end
                };
            }
            else{
                qx=xform.a*body.pre_posx+xform.b*body.pre_posy+xform.tx;
                qy=xform.c*body.pre_posx+xform.d*body.pre_posy+xform.ty;
            };
            if(xnull){
                px=body.posx;
                py=body.posy;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((px!=px));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(px)"+") :: "+("vec_set(in n: "+"p"+",in x: "+"body.posx"+",in y: "+"body.posy"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((py!=py));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(py)"+") :: "+("vec_set(in n: "+"p"+",in x: "+"body.posx"+",in y: "+"body.posy"+")");
                    #end
                };
            }
            else{
                px=xform.a*body.posx+xform.b*body.posy+xform.tx;
                py=xform.c*body.posx+xform.d*body.posy+xform.ty;
            };
            {
                penx=__round(qx);
                peny=__round(qy);
            };
            {
                var nx=__round(px);
                var ny=__round(py);
                __line(penx,peny,nx,ny,colour);
                penx=nx;
                peny=ny;
            };
            {
                penx=__round(px);
                peny=__round(py);
                __circle(penx,peny,__round(1),colour);
            };
        }
    }
    public function draw_shape(shape:ZPP_Shape,xform:ZPP_Mat23,xdet:Float,xnull:Bool){
        var col={
            var idc:Int;
            if(outer.colour==null)idc=(#if flash9 untyped __int__(0xffffff*Math.exp(-(shape.id%500)/1500))#else Std.int(0xffffff*Math.exp(-(shape.id%500)/1500))#end);
            else idc=outer.colour(shape.id);
            var _r=(((idc&0xff0000)>>16))*0.7;
            var _g=(((idc&0xff00)>>8))*0.7;
            var _b=(idc&0xff)*0.7;
            if(false){
                _r=0.4*_r+0.6*bg_r;
                _g=0.4*_g+0.6*bg_g;
                _b=0.4*_b+0.6*bg_b;
            }
            0xff000000|(((#if flash9 untyped __int__(_r)#else Std.int(_r)#end))<<16)|(((#if flash9 untyped __int__(_g)#else Std.int(_g)#end))<<8)|((#if flash9 untyped __int__(_b)#else Std.int(_b)#end));
        };
        var body=shape.body;
        if(body!=null){
            var bcol={
                var idc:Int;
                if(outer.colour==null)idc=(#if flash9 untyped __int__(0xffffff*Math.exp(-(body.id%500)/1500))#else Std.int(0xffffff*Math.exp(-(body.id%500)/1500))#end);
                else idc=outer.colour(body.id);
                var _r=(((idc&0xff0000)>>16))*0.7;
                var _g=(((idc&0xff00)>>8))*0.7;
                var _b=(idc&0xff)*0.7;
                if(body.space!=null&&body.outer.isSleeping){
                    _r=0.4*_r+0.6*bg_r;
                    _g=0.4*_g+0.6*bg_g;
                    _b=0.4*_b+0.6*bg_b;
                }
                0xff000000|(((#if flash9 untyped __int__(_r)#else Std.int(_r)#end))<<16)|(((#if flash9 untyped __int__(_g)#else Std.int(_g)#end))<<8)|((#if flash9 untyped __int__(_b)#else Std.int(_b)#end));
            };
            col={
                var col=col;
                var ncol=bcol;
                var f=0.2;
                var _r:Int=(#if flash9 untyped __int__(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#else Std.int(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#end);
                var _g:Int=(#if flash9 untyped __int__(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#else Std.int(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#end);
                var _b:Int=(#if flash9 untyped __int__(((col)&0xff)*f+((ncol)&0xff)*(1-f))#else Std.int(((col)&0xff)*f+((ncol)&0xff)*(1-f))#end);
                0xff000000|(_r<<16)|(_g<<8)|(_b);
            };
            {
                colour=col;
            };
            if(shape.isCircle()){
                var circ=shape.circle;
                circ.validate_worldCOM();
                var vx=circ.worldCOMx;
                var vy=circ.worldCOMy;
                if(!xnull){
                    var t=xform.a*vx+xform.b*vy+xform.tx;
                    vy=xform.c*vx+xform.d*vy+xform.ty;
                    vx=t;
                };
                {
                    penx=__round(vx);
                    peny=__round(vy);
                    __circle(penx,peny,__round(circ.radius*xdet),colour);
                };
                if(outer.drawShapeAngleIndicators){
                    var v0x=circ.worldCOMx+0.3*circ.radius*body.axisy;
                    var v0y=circ.worldCOMy+0.3*circ.radius*body.axisx;
                    var v1x=circ.worldCOMx+circ.radius*body.axisy;
                    var v1y=circ.worldCOMy+circ.radius*body.axisx;
                    if(!xnull){
                        var t=xform.a*v0x+xform.b*v0y+xform.tx;
                        v0y=xform.c*v0x+xform.d*v0y+xform.ty;
                        v0x=t;
                    };
                    if(!xnull){
                        var t=xform.a*v1x+xform.b*v1y+xform.tx;
                        v1y=xform.c*v1x+xform.d*v1y+xform.ty;
                        v1x=t;
                    };
                    {
                        penx=__round(v0x);
                        peny=__round(v0y);
                    };
                    {
                        var nx=__round(v1x);
                        var ny=__round(v1y);
                        __line(penx,peny,nx,ny,colour);
                        penx=nx;
                        peny=ny;
                    };
                }
            }
            else{
                var poly=shape.polygon;
                poly.validate_gverts();
                var u=poly.gverts.front();
                var vx=u.x;
                var vy=u.y;
                if(!xnull){
                    var t=xform.a*vx+xform.b*vy+xform.tx;
                    vy=xform.c*vx+xform.d*vy+xform.ty;
                    vx=t;
                };
                {
                    penx=__round(vx);
                    peny=__round(vy);
                };
                var vox=vx;
                var voy=vy;
                {
                    var cx_ite=poly.gverts.begin().next;
                    while(cx_ite!=null){
                        var u=cx_ite.elem();
                        {
                            vx=u.x;
                            vy=u.y;
                            if(!xnull){
                                var t=xform.a*vx+xform.b*vy+xform.tx;
                                vy=xform.c*vx+xform.d*vy+xform.ty;
                                vx=t;
                            };
                            {
                                var nx=__round(vx);
                                var ny=__round(vy);
                                __line(penx,peny,nx,ny,colour);
                                penx=nx;
                                peny=ny;
                            };
                        };
                        cx_ite=cx_ite.next;
                    }
                };
                {
                    var nx=__round(vox);
                    var ny=__round(voy);
                    __line(penx,peny,nx,ny,colour);
                    penx=nx;
                    peny=ny;
                };
                if(outer.drawShapeAngleIndicators){
                    poly.validate_worldCOM();
                    if(xnull){
                        vx=poly.worldCOMx;
                        vy=poly.worldCOMy;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((vx!=vx));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(vx)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"poly.worldCOMx"+",in y: "+"poly.worldCOMy"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((vy!=vy));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(vy)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"poly.worldCOMx"+",in y: "+"poly.worldCOMy"+")");
                            #end
                        };
                    }
                    else{
                        vx=xform.a*poly.worldCOMx+xform.b*poly.worldCOMy+xform.tx;
                        vy=xform.c*poly.worldCOMx+xform.d*poly.worldCOMy+xform.ty;
                    };
                    {
                        penx=__round(vx);
                        peny=__round(vy);
                    };
                    {
                        var nx=__round(vox);
                        var ny=__round(voy);
                        __line(penx,peny,nx,ny,colour);
                        penx=nx;
                        peny=ny;
                    };
                }
            }
            if(outer.drawShapeDetail){
                shape.validate_worldCOM();
                {
                    colour={
                        var col=col;
                        var ncol=0xff0000;
                        var f=0.8;
                        var _r:Int=(#if flash9 untyped __int__(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#else Std.int(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#end);
                        var _g:Int=(#if flash9 untyped __int__(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#else Std.int(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#end);
                        var _b:Int=(#if flash9 untyped __int__(((col)&0xff)*f+((ncol)&0xff)*(1-f))#else Std.int(((col)&0xff)*f+((ncol)&0xff)*(1-f))#end);
                        0xff000000|(_r<<16)|(_g<<8)|(_b);
                    };
                };
                var vx:Float=0.0;
                var vy:Float=0.0;
                if(xnull){
                    vx=shape.worldCOMx;
                    vy=shape.worldCOMy;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((vx!=vx));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(vx)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"shape.worldCOMx"+",in y: "+"shape.worldCOMy"+")");
                        #end
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((vy!=vy));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(vy)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"shape.worldCOMx"+",in y: "+"shape.worldCOMy"+")");
                        #end
                    };
                }
                else{
                    vx=xform.a*shape.worldCOMx+xform.b*shape.worldCOMy+xform.tx;
                    vy=xform.c*shape.worldCOMx+xform.d*shape.worldCOMy+xform.ty;
                };
                {
                    penx=__round(vx);
                    peny=__round(vy);
                    __circle(penx,peny,__round(1),colour);
                };
                shape.validate_aabb();
                {
                    if(xnull){
                        __aabb(shape.aabb,colour);
                    };
                    else{
                        var ox:Float=0.0;
                        var oy:Float=0.0;
                        if(false){
                            ox=shape.aabb.minx;
                            oy=shape.aabb.miny;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((ox!=ox));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(ox)"+") :: "+("vec_set(in n: "+"o"+",in x: "+"shape.aabb.minx"+",in y: "+"shape.aabb.miny"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((oy!=oy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(oy)"+") :: "+("vec_set(in n: "+"o"+",in x: "+"shape.aabb.minx"+",in y: "+"shape.aabb.miny"+")");
                                #end
                            };
                        }
                        else{
                            ox=xform.a*shape.aabb.minx+xform.b*shape.aabb.miny+xform.tx;
                            oy=xform.c*shape.aabb.minx+xform.d*shape.aabb.miny+xform.ty;
                        };
                        var wx:Float=shape.aabb.width();
                        var wy:Float=0;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((wx!=wx));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(wx)"+") :: "+("vec_new(in n: "+"w"+",in x: "+"shape.aabb.width()"+",in y: "+"0"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((wy!=wy));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(wy)"+") :: "+("vec_new(in n: "+"w"+",in x: "+"shape.aabb.width()"+",in y: "+"0"+")");
                            #end
                        };
                        if(!false){
                            var t=xform.a*wx+xform.b*wy;
                            wy=xform.c*wx+xform.d*wy;
                            wx=t;
                        };
                        var hx:Float=0;
                        var hy:Float=shape.aabb.height();
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((hx!=hx));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(hx)"+") :: "+("vec_new(in n: "+"h"+",in x: "+"0"+",in y: "+"shape.aabb.height()"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((hy!=hy));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(hy)"+") :: "+("vec_new(in n: "+"h"+",in x: "+"0"+",in y: "+"shape.aabb.height()"+")");
                            #end
                        };
                        if(!false){
                            var t=xform.a*hx+xform.b*hy;
                            hy=xform.c*hx+xform.d*hy;
                            hx=t;
                        };
                        {
                            penx=__round(ox);
                            peny=__round(oy);
                        };
                        {
                            var nx=__round(ox+wx);
                            var ny=__round(oy+wy);
                            __line(penx,peny,nx,ny,colour);
                            penx=nx;
                            peny=ny;
                        };
                        {
                            var nx=__round(ox+wx+hx);
                            var ny=__round(oy+wy+hy);
                            __line(penx,peny,nx,ny,colour);
                            penx=nx;
                            peny=ny;
                        };
                        {
                            var nx=__round(ox+hx);
                            var ny=__round(oy+hy);
                            __line(penx,peny,nx,ny,colour);
                            penx=nx;
                            peny=ny;
                        };
                        {
                            var nx=__round(ox);
                            var ny=__round(oy);
                            __line(penx,peny,nx,ny,colour);
                            penx=nx;
                            peny=ny;
                        };
                    }
                };
            }
        }
    }
    public function draw_arbiter(arb:ZPP_Arbiter,xform:ZPP_Mat23,xdet:Float,xnull:Bool){
        var vx:Float=0.0;
        var vy:Float=0.0;
        if(arb.outer.isSensorArbiter()){
            if(outer.drawSensorArbiters){
                var sarb=arb.outer;
                {
                    colour={
                        var col=0xff00;
                        var ncol=~bg_col;
                        var f=0.7;
                        var _r:Int=(#if flash9 untyped __int__(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#else Std.int(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#end);
                        var _g:Int=(#if flash9 untyped __int__(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#else Std.int(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#end);
                        var _b:Int=(#if flash9 untyped __int__(((col)&0xff)*f+((ncol)&0xff)*(1-f))#else Std.int(((col)&0xff)*f+((ncol)&0xff)*(1-f))#end);
                        0xff000000|(_r<<16)|(_g<<8)|(_b);
                    };
                };
                if(xnull){
                    vx=sarb.shape1.worldCOM.x;
                    vy=sarb.shape1.worldCOM.y;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((vx!=vx));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(vx)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"sarb.shape1.worldCOM.x"+",in y: "+"sarb.shape1.worldCOM.y"+")");
                        #end
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((vy!=vy));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(vy)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"sarb.shape1.worldCOM.x"+",in y: "+"sarb.shape1.worldCOM.y"+")");
                        #end
                    };
                }
                else{
                    vx=xform.a*sarb.shape1.worldCOM.x+xform.b*sarb.shape1.worldCOM.y+xform.tx;
                    vy=xform.c*sarb.shape1.worldCOM.x+xform.d*sarb.shape1.worldCOM.y+xform.ty;
                };
                {
                    penx=__round(vx);
                    peny=__round(vy);
                };
                if(xnull){
                    vx=sarb.shape2.worldCOM.x;
                    vy=sarb.shape2.worldCOM.y;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((vx!=vx));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(vx)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"sarb.shape2.worldCOM.x"+",in y: "+"sarb.shape2.worldCOM.y"+")");
                        #end
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((vy!=vy));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(vy)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"sarb.shape2.worldCOM.x"+",in y: "+"sarb.shape2.worldCOM.y"+")");
                        #end
                    };
                }
                else{
                    vx=xform.a*sarb.shape2.worldCOM.x+xform.b*sarb.shape2.worldCOM.y+xform.tx;
                    vy=xform.c*sarb.shape2.worldCOM.x+xform.d*sarb.shape2.worldCOM.y+xform.ty;
                };
                {
                    var nx=__round(vx);
                    var ny=__round(vy);
                    __line(penx,peny,nx,ny,colour);
                    penx=nx;
                    peny=ny;
                };
            }
        }
        else if(arb.outer.isFluidArbiter()){
            if(outer.drawFluidArbiters){
                var farb=arb.outer.fluidArbiter;
                {
                    colour={
                        var col=0xff;
                        var ncol=~bg_col;
                        var f=0.7;
                        var _r:Int=(#if flash9 untyped __int__(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#else Std.int(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#end);
                        var _g:Int=(#if flash9 untyped __int__(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#else Std.int(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#end);
                        var _b:Int=(#if flash9 untyped __int__(((col)&0xff)*f+((ncol)&0xff)*(1-f))#else Std.int(((col)&0xff)*f+((ncol)&0xff)*(1-f))#end);
                        0xff000000|(_r<<16)|(_g<<8)|(_b);
                    };
                };
                if(xnull){
                    vx=farb.position.x;
                    vy=farb.position.y;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((vx!=vx));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(vx)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"farb.position.x"+",in y: "+"farb.position.y"+")");
                        #end
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((vy!=vy));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(vy)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"farb.position.x"+",in y: "+"farb.position.y"+")");
                        #end
                    };
                }
                else{
                    vx=xform.a*farb.position.x+xform.b*farb.position.y+xform.tx;
                    vy=xform.c*farb.position.x+xform.d*farb.position.y+xform.ty;
                };
                {
                    penx=__round(vx);
                    peny=__round(vy);
                    __circle(penx,peny,__round(0.75),colour);
                };
            }
        }
        else{
            if(outer.drawCollisionArbiters){
                var carb=arb.outer.collisionArbiter;
                if(!carb.contacts.empty()){
                    var px:Float=0.0;
                    var py:Float=0.0;
                    if(carb.contacts.length==2){
                        var c1=carb.contacts.at(0).position;
                        var c2=carb.contacts.at(1).position;
                        var n=carb.normal;
                        var x=0.661437828;
                        var y=0.75;
                        if((n.y*c1.x-n.x*c1.y)<(n.y*c2.x-n.x*c2.y)){
                            x=-x;
                            y=-y;
                        }
                        {
                            colour={
                                var col=0xff;
                                var ncol=~bg_col;
                                var f=0.7;
                                var _r:Int=(#if flash9 untyped __int__(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#else Std.int(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#end);
                                var _g:Int=(#if flash9 untyped __int__(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#else Std.int(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#end);
                                var _b:Int=(#if flash9 untyped __int__(((col)&0xff)*f+((ncol)&0xff)*(1-f))#else Std.int(((col)&0xff)*f+((ncol)&0xff)*(1-f))#end);
                                0xff000000|(_r<<16)|(_g<<8)|(_b);
                            };
                        };
                        {
                            vx=c1.x+n.x*y-n.y*x;
                            vy=c1.y+n.y*y+n.x*x;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((vx!=vx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(vx)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"c1.x+n.x*y-n.y*x"+",in y: "+"c1.y+n.y*y+n.x*x"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((vy!=vy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(vy)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"c1.x+n.x*y-n.y*x"+",in y: "+"c1.y+n.y*y+n.x*x"+")");
                                #end
                            };
                        };
                        if(!xnull){
                            var t=xform.a*vx+xform.b*vy+xform.tx;
                            vy=xform.c*vx+xform.d*vy+xform.ty;
                            vx=t;
                        };
                        {
                            penx=__round(vx);
                            peny=__round(vy);
                        };
                        {
                            vx=c2.x+n.x*y+n.y*x;
                            vy=c2.y+n.y*y-n.x*x;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((vx!=vx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(vx)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"c2.x+n.x*y+n.y*x"+",in y: "+"c2.y+n.y*y-n.x*x"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((vy!=vy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(vy)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"c2.x+n.x*y+n.y*x"+",in y: "+"c2.y+n.y*y-n.x*x"+")");
                                #end
                            };
                        };
                        if(!xnull){
                            var t=xform.a*vx+xform.b*vy+xform.tx;
                            vy=xform.c*vx+xform.d*vy+xform.ty;
                            vx=t;
                        };
                        {
                            var nx=__round(vx);
                            var ny=__round(vy);
                            __line(penx,peny,nx,ny,colour);
                            penx=nx;
                            peny=ny;
                        };
                        {
                            colour={
                                var col=0xff0000;
                                var ncol=~bg_col;
                                var f=0.7;
                                var _r:Int=(#if flash9 untyped __int__(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#else Std.int(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#end);
                                var _g:Int=(#if flash9 untyped __int__(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#else Std.int(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#end);
                                var _b:Int=(#if flash9 untyped __int__(((col)&0xff)*f+((ncol)&0xff)*(1-f))#else Std.int(((col)&0xff)*f+((ncol)&0xff)*(1-f))#end);
                                0xff000000|(_r<<16)|(_g<<8)|(_b);
                            };
                        };
                        {
                            vx=c1.x-n.x*y-n.y*x;
                            vy=c1.y-n.y*y+n.x*x;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((vx!=vx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(vx)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"c1.x-n.x*y-n.y*x"+",in y: "+"c1.y-n.y*y+n.x*x"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((vy!=vy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(vy)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"c1.x-n.x*y-n.y*x"+",in y: "+"c1.y-n.y*y+n.x*x"+")");
                                #end
                            };
                        };
                        if(!xnull){
                            var t=xform.a*vx+xform.b*vy+xform.tx;
                            vy=xform.c*vx+xform.d*vy+xform.ty;
                            vx=t;
                        };
                        {
                            penx=__round(vx);
                            peny=__round(vy);
                        };
                        {
                            vx=c2.x-n.x*y+n.y*x;
                            vy=c2.y-n.y*y-n.x*x;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((vx!=vx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(vx)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"c2.x-n.x*y+n.y*x"+",in y: "+"c2.y-n.y*y-n.x*x"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((vy!=vy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(vy)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"c2.x-n.x*y+n.y*x"+",in y: "+"c2.y-n.y*y-n.x*x"+")");
                                #end
                            };
                        };
                        if(!xnull){
                            var t=xform.a*vx+xform.b*vy+xform.tx;
                            vy=xform.c*vx+xform.d*vy+xform.ty;
                            vx=t;
                        };
                        {
                            var nx=__round(vx);
                            var ny=__round(vy);
                            __line(penx,peny,nx,ny,colour);
                            penx=nx;
                            peny=ny;
                        };
                        {
                            px=0.5*(c1.x+c2.x);
                            py=0.5*(c1.y+c2.y);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((px!=px));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(px)"+") :: "+("vec_set(in n: "+"p"+",in x: "+"0.5*(c1.x+c2.x)"+",in y: "+"0.5*(c1.y+c2.y)"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((py!=py));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(py)"+") :: "+("vec_set(in n: "+"p"+",in x: "+"0.5*(c1.x+c2.x)"+",in y: "+"0.5*(c1.y+c2.y)"+")");
                                #end
                            };
                        };
                        if(!xnull){
                            var t=xform.a*px+xform.b*py+xform.tx;
                            py=xform.c*px+xform.d*py+xform.ty;
                            px=t;
                        };
                    }
                    else{
                        {
                            px=carb.contacts.at(0).position.x;
                            py=carb.contacts.at(0).position.y;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((px!=px));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(px)"+") :: "+("vec_set(in n: "+"p"+",in x: "+"carb.contacts.at(0).position.x"+",in y: "+"carb.contacts.at(0).position.y"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((py!=py));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(py)"+") :: "+("vec_set(in n: "+"p"+",in x: "+"carb.contacts.at(0).position.x"+",in y: "+"carb.contacts.at(0).position.y"+")");
                                #end
                            };
                        };
                        if(!xnull){
                            var t=xform.a*px+xform.b*py+xform.tx;
                            py=xform.c*px+xform.d*py+xform.ty;
                            px=t;
                        };
                        {
                            colour={
                                var col=0xff00ff;
                                var ncol=~bg_col;
                                var f=0.7;
                                var _r:Int=(#if flash9 untyped __int__(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#else Std.int(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#end);
                                var _g:Int=(#if flash9 untyped __int__(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#else Std.int(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#end);
                                var _b:Int=(#if flash9 untyped __int__(((col)&0xff)*f+((ncol)&0xff)*(1-f))#else Std.int(((col)&0xff)*f+((ncol)&0xff)*(1-f))#end);
                                0xff000000|(_r<<16)|(_g<<8)|(_b);
                            };
                        };
                        {
                            penx=__round(px);
                            peny=__round(py);
                            __circle(penx,peny,__round(1),colour);
                        };
                    }
                    {
                        colour={
                            var col=~bg_col;
                            var ncol=bg_col;
                            var f=0.7;
                            var _r:Int=(#if flash9 untyped __int__(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#else Std.int(((col>>16)&0xff)*f+((ncol>>16)&0xff)*(1-f))#end);
                            var _g:Int=(#if flash9 untyped __int__(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#else Std.int(((col>>8)&0xff)*f+((ncol>>8)&0xff)*(1-f))#end);
                            var _b:Int=(#if flash9 untyped __int__(((col)&0xff)*f+((ncol)&0xff)*(1-f))#else Std.int(((col)&0xff)*f+((ncol)&0xff)*(1-f))#end);
                            0xff000000|(_r<<16)|(_g<<8)|(_b);
                        };
                    };
                    {
                        penx=__round(px);
                        peny=__round(py);
                    };
                    {
                        vx=carb.normal.x*5;
                        vy=carb.normal.y*5;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((vx!=vx));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(vx)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"carb.normal.x*5"+",in y: "+"carb.normal.y*5"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((vy!=vy));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(vy)"+") :: "+("vec_set(in n: "+"v"+",in x: "+"carb.normal.x*5"+",in y: "+"carb.normal.y*5"+")");
                            #end
                        };
                    };
                    if(!xnull){
                        var t=xform.a*vx+xform.b*vy;
                        vy=xform.c*vx+xform.d*vy;
                        vx=t;
                    };
                    {
                        var nx=__round(px+vx);
                        var ny=__round(py+vy);
                        __line(penx,peny,nx,ny,colour);
                        penx=nx;
                        peny=ny;
                    };
                }
            }
        }
    }
    public function __box(x0:Int,y0:Int,x1:Int,y1:Int,col:Int){
        col|=0xff000000;
        __line(x0,y0,x0,y1,col);
        __line(x0,y1,x1,y1,col);
        __line(x1,y1,x1,y0,col);
        __line(x1,y0,x0,y0,col);
    }
    public function __aabb(aabb:ZPP_AABB,col:Int){
        col|=0xff000000;
        var u0=aabb.minx>ZPP_Const.NEGINF();
        var u1=aabb.maxx<ZPP_Const.POSINF();
        var v0=aabb.miny>ZPP_Const.NEGINF();
        var v1=aabb.maxy<ZPP_Const.POSINF();
        var x0=u0?__round(aabb.minx):0;
        var x1=u1?__round(aabb.maxx):width;
        var y0=v0?__round(aabb.miny):0;
        var y1=v1?__round(aabb.maxy):height;
        if(u0)__line(x0,y0,x0,y1,col);
        if(u1)__line(x1,y0,x1,y1,col);
        if(v0)__line(x0,y0,x1,y0,col);
        if(v1)__line(x0,y1,x1,y1,col);
    }
}
#end
