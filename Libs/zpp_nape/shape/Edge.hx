package zpp_nape.shape;
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
class ZPP_Edge{
    public var next:ZPP_Edge=null;
    static public var zpp_pool:ZPP_Edge=null;
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
    function free(){
        polygon=null;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc(){}
    public var polygon:ZPP_Polygon=null;
    static public var internal:Bool=false;
    public var outer:Edge=null;
    public function wrapper(){
        if(outer==null){
            internal=true;
            outer=new Edge();
            internal=false;
            outer.zpp_inner=this;
        }
        return outer;
    }
    public var lnormx:Float=0.0;
    public var lnormy:Float=0.0;
    public var wrap_lnorm:Vec2=null;
    public var gnormx:Float=0.0;
    public var gnormy:Float=0.0;
    public var wrap_gnorm:Vec2=null;
    public var length:Float=0.0;
    public var lprojection:Float=0.0;
    public var gprojection:Float=0.0;
    public var lp0:ZPP_Vec2=null;
    public var gp0:ZPP_Vec2=null;
    public var lp1:ZPP_Vec2=null;
    public var gp1:ZPP_Vec2=null;
    public var tp0:Float=0.0;
    public var tp1:Float=0.0;
    private function lnorm_validate(){
        #if(!NAPE_RELEASE_BUILD)
        if(polygon==null)throw "Error: Edge not currently in use";
        #end
        polygon.validate_laxi();
        {
            wrap_lnorm.zpp_inner.x=lnormx;
            wrap_lnorm.zpp_inner.y=lnormy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_lnorm.zpp_inner.x!=wrap_lnorm.zpp_inner.x));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_lnorm.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap_lnorm.zpp_inner."+",in x: "+"lnormx"+",in y: "+"lnormy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_lnorm.zpp_inner.y!=wrap_lnorm.zpp_inner.y));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_lnorm.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap_lnorm.zpp_inner."+",in x: "+"lnormx"+",in y: "+"lnormy"+")");
                #end
            };
        };
    }
    private function gnorm_validate(){
        #if(!NAPE_RELEASE_BUILD)
        if(polygon==null)throw "Error: Edge not currently in use";
        if(polygon.body==null)throw "Error: Edge worldNormal only makes sense if the parent Polygon is contained within a rigid body";
        #end
        polygon.validate_gaxi();
        {
            wrap_gnorm.zpp_inner.x=gnormx;
            wrap_gnorm.zpp_inner.y=gnormy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_gnorm.zpp_inner.x!=wrap_gnorm.zpp_inner.x));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_gnorm.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap_gnorm.zpp_inner."+",in x: "+"gnormx"+",in y: "+"gnormy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_gnorm.zpp_inner.y!=wrap_gnorm.zpp_inner.y));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_gnorm.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap_gnorm.zpp_inner."+",in x: "+"gnormx"+",in y: "+"gnormy"+")");
                #end
            };
        };
    }
    public function getlnorm(){
        wrap_lnorm=Vec2.get(lnormx,lnormy);
        wrap_lnorm.zpp_inner._immutable=true;
        wrap_lnorm.zpp_inner._validate=lnorm_validate;
    }
    public function getgnorm(){
        wrap_gnorm=Vec2.get(gnormx,gnormy);
        wrap_gnorm.zpp_inner._immutable=true;
        wrap_gnorm.zpp_inner._validate=gnorm_validate;
    }
    public function new(){
        {
            lnormx=0;
            lnormy=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((lnormx!=lnormx));
                };
                if(!res)throw "assert("+"!assert_isNaN(lnormx)"+") :: "+("vec_set(in n: "+"lnorm"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((lnormy!=lnormy));
                };
                if(!res)throw "assert("+"!assert_isNaN(lnormy)"+") :: "+("vec_set(in n: "+"lnorm"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        {
            gnormx=0;
            gnormy=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((gnormx!=gnormx));
                };
                if(!res)throw "assert("+"!assert_isNaN(gnormx)"+") :: "+("vec_set(in n: "+"gnorm"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((gnormy!=gnormy));
                };
                if(!res)throw "assert("+"!assert_isNaN(gnormy)"+") :: "+("vec_set(in n: "+"gnorm"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        length=0;
        lprojection=0;
        gprojection=0;
    }
}
