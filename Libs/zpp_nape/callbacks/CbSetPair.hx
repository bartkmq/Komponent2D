package zpp_nape.callbacks;
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
import nape.util.ShapeDebug;
#if nape_swc@:keep #end
class ZPP_CbSetPair{
    public var a:ZPP_CbSet=null;
    public var b:ZPP_CbSet=null;
    public var next:ZPP_CbSetPair=null;
    static public var zpp_pool:ZPP_CbSetPair=null;
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
        a=b=null;
        listeners.clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{
        zip_listeners=true;
    }
    public function new(){
        listeners=new ZNPList_ZPP_InteractionListener();
    }
    public static#if NAPE_NO_INLINE#else inline #end
    function get(a:ZPP_CbSet,b:ZPP_CbSet):ZPP_CbSetPair{
        var ret;
        {
            if(ZPP_CbSetPair.zpp_pool==null){
                ret=new ZPP_CbSetPair();
                #if NAPE_POOL_STATS ZPP_CbSetPair.POOL_TOT++;
                ZPP_CbSetPair.POOL_ADDNEW++;
                #end
            }
            else{
                ret=ZPP_CbSetPair.zpp_pool;
                ZPP_CbSetPair.zpp_pool=ret.next;
                ret.next=null;
                #if NAPE_POOL_STATS ZPP_CbSetPair.POOL_CNT--;
                ZPP_CbSetPair.POOL_ADD++;
                #end
            }
            ret.alloc();
        };
        if(ZPP_CbSet.setlt(a,b)){
            ret.a=a;
            ret.b=b;
        }
        else{
            ret.a=b;
            ret.b=a;
        }
        return ret;
    }
    public static#if NAPE_NO_INLINE#else inline #end
    function setlt(x:ZPP_CbSetPair,y:ZPP_CbSetPair):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                (x.a==y.a)==(!ZPP_CbSet.setlt(x.a,y.a)&&!ZPP_CbSet.setlt(y.a,x.a));
            };
            if(!res)throw "assert("+"(x.a==y.a)==(!ZPP_CbSet.setlt(x.a,y.a)&&!ZPP_CbSet.setlt(y.a,x.a))"+") :: "+("Assumption that CbSet's are unique!! Aka we can compare for 'equal' CbSet with == is wrong?? :(");
            #end
        };
        return ZPP_CbSet.setlt(x.a,y.a)||(x.a==y.a&&ZPP_CbSet.setlt(x.b,y.b));
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    #if NAPE_NO_INLINE#else inline #end
    function compatible(i:ZPP_InteractionListener):Bool{
        return(i.options1.compatible(a.cbTypes)&&i.options2.compatible(b.cbTypes))||(i.options2.compatible(a.cbTypes)&&i.options1.compatible(b.cbTypes));
    }
    public var zip_listeners:Bool=false;
    public var listeners:ZNPList_ZPP_InteractionListener=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function invalidate():Void{
        zip_listeners=true;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function validate():Void{
        if(zip_listeners){
            zip_listeners=false;
            __validate();
        }
    }
    public function __validate():Void{
        listeners.clear();
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !a.zip_listeners;
            };
            if(!res)throw "assert("+"!a.zip_listeners"+") :: "+("a.listeners not validated??");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !b.zip_listeners;
            };
            if(!res)throw "assert("+"!b.zip_listeners"+") :: "+("b.listeners not validated??");
            #end
        };
        var aite=a.listeners.begin();
        var bite=b.listeners.begin();
        while(aite!=null&&bite!=null){
            var ax=aite.elem();
            var bx=bite.elem();
            if(ax==bx){
                if(compatible(ax)){
                    listeners.add(ax);
                }
                aite=aite.next;
                bite=bite.next;
            }
            else if(ZPP_Listener.setlt(ax,bx))aite=aite.next;
            else bite=bite.next;
        }
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function empty_intersection():Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !zip_listeners;
            };
            if(!res)throw "assert("+"!zip_listeners"+") :: "+("not validated before empty_intersection");
            #end
        };
        return listeners.empty();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function single_intersection(i:ZPP_InteractionListener):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !zip_listeners;
            };
            if(!res)throw "assert("+"!zip_listeners"+") :: "+("not validated before single_intersection");
            #end
        };
        var ite=listeners.begin();
        return ite!=null&&ite.elem()==i&&ite.next==null;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function forall(event:Int,cb:ZPP_InteractionListener->Void):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !zip_listeners;
            };
            if(!res)throw "assert("+"!zip_listeners"+") :: "+("not validated before forall");
            #end
        };
        {
            var cx_ite=listeners.begin();
            while(cx_ite!=null){
                var x=cx_ite.elem();
                {
                    if(x.event==event)cb(x);
                };
                cx_ite=cx_ite.next;
            }
        };
    }
}
