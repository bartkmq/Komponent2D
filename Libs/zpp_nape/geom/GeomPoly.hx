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
import nape.util.ShapeDebug;
#if nape_swc@:keep #end
class ZPP_GeomVert{
    public var x:Float=0.0;
    public var y:Float=0.0;
    public var prev:ZPP_GeomVert=null;
    public var next:ZPP_GeomVert=null;
    public var wrap:Null<Vec2>=null;
    public var forced:Bool=false;
    static public var zpp_pool:ZPP_GeomVert=null;
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
    
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        if(wrap!=null){
            wrap.zpp_inner._inuse=false;
            wrap.dispose();
            wrap=null;
        }
        prev=next=null;
    }
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{
        forced=false;
    }
    public#if NAPE_NO_INLINE#else inline #end
    function wrapper():Vec2{
        if(wrap==null){
            wrap=Vec2.get(x,y);
            wrap.zpp_inner._inuse=true;
            wrap.zpp_inner._invalidate=modwrap;
            wrap.zpp_inner._validate=getwrap;
        }
        return wrap;
    }
    public#if NAPE_NO_INLINE#else inline #end
    function modwrap(n:ZPP_Vec2):Void{
        {
            this.x=n.x;
            this.y=n.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((this.x!=this.x));
                };
                if(!res)throw "assert("+"!assert_isNaN(this.x)"+") :: "+("vec_set(in n: "+"this."+",in x: "+"n.x"+",in y: "+"n.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((this.y!=this.y));
                };
                if(!res)throw "assert("+"!assert_isNaN(this.y)"+") :: "+("vec_set(in n: "+"this."+",in x: "+"n.x"+",in y: "+"n.y"+")");
                #end
            };
        };
    }
    public#if NAPE_NO_INLINE#else inline #end
    function getwrap():Void{
        {
            wrap.zpp_inner.x=this.x;
            wrap.zpp_inner.y=this.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap.zpp_inner.x!=wrap.zpp_inner.x));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap.zpp_inner."+",in x: "+"this.x"+",in y: "+"this.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap.zpp_inner.y!=wrap.zpp_inner.y));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap.zpp_inner."+",in x: "+"this.x"+",in y: "+"this.y"+")");
                #end
            };
        };
    }
    public static#if NAPE_NO_INLINE#else inline #end
    function get(x:Float,y:Float):ZPP_GeomVert{
        var ret;
        {
            if(ZPP_GeomVert.zpp_pool==null){
                ret=new ZPP_GeomVert();
                #if NAPE_POOL_STATS ZPP_GeomVert.POOL_TOT++;
                ZPP_GeomVert.POOL_ADDNEW++;
                #end
            }
            else{
                ret=ZPP_GeomVert.zpp_pool;
                ZPP_GeomVert.zpp_pool=ret.next;
                ret.next=null;
                #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT--;
                ZPP_GeomVert.POOL_ADD++;
                #end
            }
            ret.alloc();
        };
        {
            ret.x=x;
            ret.y=y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.x!=ret.x));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.x)"+") :: "+("vec_set(in n: "+"ret."+",in x: "+"x"+",in y: "+"y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.y!=ret.y));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.y)"+") :: "+("vec_set(in n: "+"ret."+",in x: "+"x"+",in y: "+"y"+")");
                #end
            };
        };
        return ret;
    }
    public function new(){}
}
#if nape_swc@:keep #end
class ZPP_GeomPoly{
    public var outer:GeomPoly=null;
    public var vertices:Null<ZPP_GeomVert>=null;
    public function new(outer:GeomPoly){
        this.outer=outer;
    }
}
#if nape_swc@:keep #end
class ZPP_GeomVertexIterator{
    public var ptr:ZPP_GeomVert=null;
    public var start:ZPP_GeomVert=null;
    public var first:Bool=false;
    public var forward:Bool=false;
    public var outer:GeomVertexIterator=null;
    public var next:ZPP_GeomVertexIterator=null;
    static public var zpp_pool:ZPP_GeomVertexIterator=null;
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
    function free():Void{
        outer.zpp_inner=null;
        ptr=start=null;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc(){}
    #if(!NAPE_RELEASE_BUILD)
    public static var internal=false;
    #end
    function new(){
        #if(!NAPE_RELEASE_BUILD)
        internal=true;
        #end
        outer=new GeomVertexIterator();
        #if(!NAPE_RELEASE_BUILD)
        internal=false;
        #end
    }
    public static function get(poly:ZPP_GeomVert,forward:Bool){
        var ret;
        {
            if(ZPP_GeomVertexIterator.zpp_pool==null){
                ret=new ZPP_GeomVertexIterator();
                #if NAPE_POOL_STATS ZPP_GeomVertexIterator.POOL_TOT++;
                ZPP_GeomVertexIterator.POOL_ADDNEW++;
                #end
            }
            else{
                ret=ZPP_GeomVertexIterator.zpp_pool;
                ZPP_GeomVertexIterator.zpp_pool=ret.next;
                ret.next=null;
                #if NAPE_POOL_STATS ZPP_GeomVertexIterator.POOL_CNT--;
                ZPP_GeomVertexIterator.POOL_ADD++;
                #end
            }
            ret.alloc();
        };
        ret.outer.zpp_inner=ret;
        ret.ptr=poly;
        ret.forward=forward;
        ret.start=poly;
        ret.first=poly!=null;
        return ret.outer;
    }
}
