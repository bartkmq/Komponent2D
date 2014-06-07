package zpp_nape.geom;
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
import nape.util.ShapeDebug;
#if nape_swc@:keep #end
class ZPP_AABB{
    public var _invalidate:Null<ZPP_AABB->Void>=null;
    public var _validate:Null<Void->Void>=null;
    public var _immutable:Bool=false;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function validate():Void{
        if(_validate!=null){
            _validate();
        }
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function invalidate():Void{
        if(_invalidate!=null){
            _invalidate(this);
        }
    }
    public var outer:Null<AABB>=null;
    public function wrapper():AABB{
        if(outer==null){
            outer=new AABB();
            {
                var o=outer.zpp_inner;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_AABB"+", in obj: "+"outer.zpp_inner"+")");
                    #end
                };
                o.free();
                o.next=ZPP_AABB.zpp_pool;
                ZPP_AABB.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_AABB.POOL_CNT++;
                ZPP_AABB.POOL_SUB++;
                #end
            };
            outer.zpp_inner=this;
        }
        return outer;
    }
    public var next:ZPP_AABB=null;
    static public var zpp_pool:ZPP_AABB=null;
    #if NAPE_POOL_STATS 
    /**
     * @private
     */
    static public var POOL_CNT:Int=0;
    /**
     * @private
     */
    static public var POOL_TOT:Int=0;
    /**
     * @private
     */
    static public var POOL_ADD:Int=0;
    /**
     * @private
     */
    static public var POOL_ADDNEW:Int=0;
    /**
     * @private
     */
    static public var POOL_SUB:Int=0;
    #end
    
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        if(outer!=null){
            outer.zpp_inner=null;
            outer=null;
        }
        wrap_min=wrap_max=null;
        _invalidate=null;
        _validate=null;
    }
    public function new(){}
    public static#if NAPE_NO_INLINE#else inline #end
    function get(minx:Float,miny:Float,maxx:Float,maxy:Float):ZPP_AABB{
        var ret;
        {
            if(ZPP_AABB.zpp_pool==null){
                ret=new ZPP_AABB();
                #if NAPE_POOL_STATS ZPP_AABB.POOL_TOT++;
                ZPP_AABB.POOL_ADDNEW++;
                #end
            }
            else{
                ret=ZPP_AABB.zpp_pool;
                ZPP_AABB.zpp_pool=ret.next;
                ret.next=null;
                #if NAPE_POOL_STATS ZPP_AABB.POOL_CNT--;
                ZPP_AABB.POOL_ADD++;
                #end
            }
            ret.alloc();
        };
        {
            ret.minx=minx;
            ret.miny=miny;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.minx!=ret.minx));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.minx)"+") :: "+("vec_set(in n: "+"ret.min"+",in x: "+"minx"+",in y: "+"miny"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.miny!=ret.miny));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.miny)"+") :: "+("vec_set(in n: "+"ret.min"+",in x: "+"minx"+",in y: "+"miny"+")");
                #end
            };
        };
        {
            ret.maxx=maxx;
            ret.maxy=maxy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.maxx!=ret.maxx));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.maxx)"+") :: "+("vec_set(in n: "+"ret.max"+",in x: "+"maxx"+",in y: "+"maxy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.maxy!=ret.maxy));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.maxy)"+") :: "+("vec_set(in n: "+"ret.max"+",in x: "+"maxx"+",in y: "+"maxy"+")");
                #end
            };
        };
        return ret;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function copy():ZPP_AABB{
        return ZPP_AABB.get(minx,miny,maxx,maxy);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function width():Float{
        return maxx-minx;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function height():Float{
        return maxy-miny;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function perimeter():Float{
        return(width()+height())*2;
    }
    public var minx:Float=0.0;
    public var miny:Float=0.0;
    public var wrap_min:Null<Vec2>=null;
    public function getmin():Vec2{
        if(wrap_min==null){
            wrap_min=Vec2.get(minx,miny);
            wrap_min.zpp_inner._inuse=true;
            if(_immutable){
                wrap_min.zpp_inner._immutable=true;
            }
            else{
                wrap_min.zpp_inner._invalidate=mod_min;
            }
            wrap_min.zpp_inner._validate=dom_min;
        }
        return wrap_min;
    }
    public function dom_min():Void{
        validate();
        {
            wrap_min.zpp_inner.x=minx;
            wrap_min.zpp_inner.y=miny;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_min.zpp_inner.x!=wrap_min.zpp_inner.x));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_min.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap_min.zpp_inner."+",in x: "+"minx"+",in y: "+"miny"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_min.zpp_inner.y!=wrap_min.zpp_inner.y));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_min.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap_min.zpp_inner."+",in x: "+"minx"+",in y: "+"miny"+")");
                #end
            };
        };
    }
    public function mod_min(min:ZPP_Vec2):Void{
        if(min.x!=minx||min.y!=miny){
            {
                minx=min.x;
                miny=min.y;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((minx!=minx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(minx)"+") :: "+("vec_set(in n: "+"min"+",in x: "+"min.x"+",in y: "+"min.y"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((miny!=miny));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(miny)"+") :: "+("vec_set(in n: "+"min"+",in x: "+"min.x"+",in y: "+"min.y"+")");
                    #end
                };
            };
            invalidate();
        }
    }
    public var maxx:Float=0.0;
    public var maxy:Float=0.0;
    public var wrap_max:Null<Vec2>=null;
    public function getmax():Vec2{
        if(wrap_max==null){
            wrap_max=Vec2.get(maxx,maxy);
            wrap_max.zpp_inner._inuse=true;
            if(_immutable){
                wrap_max.zpp_inner._immutable=true;
            }
            else{
                wrap_max.zpp_inner._invalidate=mod_max;
            }
            wrap_max.zpp_inner._validate=dom_max;
        }
        return wrap_max;
    }
    public function dom_max():Void{
        validate();
        {
            wrap_max.zpp_inner.x=maxx;
            wrap_max.zpp_inner.y=maxy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_max.zpp_inner.x!=wrap_max.zpp_inner.x));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_max.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap_max.zpp_inner."+",in x: "+"maxx"+",in y: "+"maxy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_max.zpp_inner.y!=wrap_max.zpp_inner.y));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_max.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap_max.zpp_inner."+",in x: "+"maxx"+",in y: "+"maxy"+")");
                #end
            };
        };
    }
    public function mod_max(max:ZPP_Vec2):Void{
        if(max.x!=maxx||max.y!=maxy){
            {
                maxx=max.x;
                maxy=max.y;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((maxx!=maxx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(maxx)"+") :: "+("vec_set(in n: "+"max"+",in x: "+"max.x"+",in y: "+"max.y"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((maxy!=maxy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(maxy)"+") :: "+("vec_set(in n: "+"max"+",in x: "+"max.x"+",in y: "+"max.y"+")");
                    #end
                };
            };
            invalidate();
        }
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function intersectX(x:ZPP_AABB):Bool{
        return!(x.minx>maxx||minx>x.maxx);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function intersectY(x:ZPP_AABB):Bool{
        return!(x.miny>maxy||miny>x.maxy);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function intersect(x:ZPP_AABB):Bool{
        return x.miny<=maxy&&miny<=x.maxy&&x.minx<=maxx&&minx<=x.maxx;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function combine(x:ZPP_AABB):Void{
        if(x.minx<minx)minx=x.minx;
        if(x.maxx>maxx)maxx=x.maxx;
        if(x.miny<miny)miny=x.miny;
        if(x.maxy>maxy)maxy=x.maxy;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function contains(x:ZPP_AABB):Bool{
        return x.minx>=minx&&x.miny>=miny&&x.maxx<=maxx&&x.maxy<=maxy;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function containsPoint(v:ZPP_Vec2):Bool{
        return v.x>=minx&&v.x<=maxx&&v.y>=miny&&v.y<=maxy;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setCombine(a:ZPP_AABB,b:ZPP_AABB):Void{
        minx=if(a.minx<b.minx)a.minx else b.minx;
        miny=if(a.miny<b.miny)a.miny else b.miny;
        maxx=if(a.maxx>b.maxx)a.maxx else b.maxx;
        maxy=if(a.maxy>b.maxy)a.maxy else b.maxy;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setExpand(a:ZPP_AABB,fatten:Float):Void{
        minx=a.minx-fatten;
        miny=a.miny-fatten;
        maxx=a.maxx+fatten;
        maxy=a.maxy+fatten;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setExpandPoint(x:Float,y:Float):Void{
        if(x<minx)minx=x;
        if(x>maxx)maxx=x;
        if(y<miny)miny=y;
        if(y>maxy)maxy=y;
    }
    public function toString(){
        return "{ x: "+minx+" y: "+miny+" w: "+width()+" h: "+height()+" }";
    }
}
