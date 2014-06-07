package zpp_nape.dynamics;
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
class ZPP_Arbiter{
    public var outer:Arbiter=null;
    #if NAPE_POOL_STATS public var arbid:Int=0;
    static var nextarbid:Int=0;
    #end
    public static var internal=false;
    public function wrapper(){
        if(outer==null){
            internal=true;
            if(type==COL){
                colarb.outer_zn=new CollisionArbiter();
                outer=colarb.outer_zn;
            }
            else if(type==FLUID){
                fluidarb.outer_zn=new FluidArbiter();
                outer=fluidarb.outer_zn;
            }
            else outer=new Arbiter();
            outer.zpp_inner=this;
            internal=false;
        }
        return outer;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inactiveme(){
        return!active;
    }
    public var hnext:ZPP_Arbiter=null;
    public function new(){
        #if NAPE_POOL_STATS arbid=nextarbid++;
        #end
    }
    public var id:Int=0;
    public var di:Int=0;
    public var stamp:Int=0;
    public var up_stamp:Int=0;
    public var sleep_stamp:Int=0;
    public var endGenerated:Int=0;
    public var active:Bool=false;
    public var cleared:Bool=false;
    public var sleeping:Bool=false;
    public var present:Int=0;
    public var intchange:Bool=false;
    public var presentable:Bool=false;
    public var continuous:Bool=false;
    public var fresh:Bool=false;
    public var immState:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function acting(){
        return active&&((immState&ZPP_Flags.id_ImmState_ACCEPT)!=0);
    }
    public var invalidated:Bool=false;
    public var b1:ZPP_Body=null;
    public var b2:ZPP_Body=null;
    public var ws1:ZPP_Shape=null;
    public var ws2:ZPP_Shape=null;
    public var pair:ZPP_AABBPair=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function swap_features(){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                type==COL;
            };
            if(!res)throw "assert("+"type==COL"+") :: "+("Arbiter::swap_features");
            #end
        };
        {
            var t=b1;
            b1=b2;
            b2=t;
        };
        {
            var t=ws1;
            ws1=ws2;
            ws2=t;
        };
        {
            var t=colarb.s1;
            colarb.s1=colarb.s2;
            colarb.s2=t;
        };
    }
    public var type:Int=0;
     public static var COL=1;
     public static var FLUID=4;
     public static var SENSOR=2;
    static public var types:Array<ArbiterType>=[null,ArbiterType.COLLISION,ArbiterType.SENSOR,null,ArbiterType.FLUID];
    public var colarb:ZPP_ColArbiter=null;
    public var fluidarb:ZPP_FluidArbiter=null;
    public var sensorarb:ZPP_SensorArbiter=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function lazyRetire(s:ZPP_Space,b:ZPP_Body=null){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !cleared;
            };
            if(!res)throw "assert("+"!cleared"+") :: "+("Arbiter::lazyRetire");
            #end
        };
        cleared=true;
        if(b==null||(b2==b))b1.arbiters.inlined_remove(this);
        if(b==null||(b1==b))b2.arbiters.inlined_remove(this);
        if(pair!=null){
            pair.arb=null;
            pair=null;
        }
        active=false;
        s.f_arbiters.modified=true;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function sup_assign(s1:ZPP_Shape,s2:ZPP_Shape,id:Int,di:Int){
        b1=s1.body;
        ws1=s1;
        b2=s2.body;
        ws2=s2;
        this.id=id;
        this.di=di;
        b1.arbiters.inlined_add(this);
        b2.arbiters.inlined_add(this);
        active=true;
        present=0;
        cleared=false;
        sleeping=false;
        fresh=false;
        presentable=false;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function sup_retire(){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                b1!=null;
            };
            if(!res)throw "assert("+"b1!=null"+") :: "+("Arbiter::sup_retire");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                b2!=null;
            };
            if(!res)throw "assert("+"b2!=null"+") :: "+("Arbiter::sup_retire");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                present==0;
            };
            if(!res)throw "assert("+"present==0"+") :: "+("Arbiter::sup_retire cbsets present");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !intchange;
            };
            if(!res)throw "assert("+"!intchange"+") :: "+("Arbiter::sup_retire intchange");
            #end
        };
        if(!cleared){
            b1.arbiters.inlined_remove(this);
            b2.arbiters.inlined_remove(this);
            if(pair!=null){
                pair.arb=null;
                pair=null;
            }
        }
        b1=b2=null;
        active=false;
        intchange=false;
    }
}
#if nape_swc@:keep #end
class ZPP_SensorArbiter extends ZPP_Arbiter{
    public var next:ZPP_SensorArbiter=null;
    static public var zpp_pool:ZPP_SensorArbiter=null;
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
    function alloc(){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                pair==null;
            };
            if(!res)throw "assert("+"pair==null"+") :: "+("AABBNode pair exists on arb going out of pool? (sensor)");
            #end
        };
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free(){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                pair==null;
            };
            if(!res)throw "assert("+"pair==null"+") :: "+("AABBNode pair exists on arb going into pool? (sensor)");
            #end
        };
    }
    public function new(){
        super();
        type=ZPP_Arbiter.SENSOR;
        sensorarb=this;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function assign(s1:ZPP_Shape,s2:ZPP_Shape,id:Int,di:Int){
        sup_assign(s1,s2,id,di);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function retire(){
        sup_retire();
        {
            var o=this;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SensorArbiter"+", in obj: "+"this"+")");
                #end
            };
            o.free();
            o.next=ZPP_SensorArbiter.zpp_pool;
            ZPP_SensorArbiter.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_SensorArbiter.POOL_CNT++;
            ZPP_SensorArbiter.POOL_SUB++;
            #end
        };
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function makemutable(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function makeimmutable(){}
}
#if nape_swc@:keep #end
class ZPP_FluidArbiter extends ZPP_Arbiter{
    public var outer_zn:FluidArbiter=null;
    public var next:ZPP_FluidArbiter=null;
    static public var zpp_pool:ZPP_FluidArbiter=null;
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
    function alloc(){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                pair==null;
            };
            if(!res)throw "assert("+"pair==null"+") :: "+("AABBNode pair exists on arb going out of pool? (fluid)");
            #end
        };
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free(){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                pair==null;
            };
            if(!res)throw "assert("+"pair==null"+") :: "+("AABBNode pair exists on arb going into pool? (fluid)");
            #end
        };
    }
    public var centroidx:Float=0.0;
    public var centroidy:Float=0.0;
    public var overlap:Float=0.0;
    public var r1x:Float=0.0;
    public var r1y:Float=0.0;
    public var r2x:Float=0.0;
    public var r2y:Float=0.0;
    public var nodrag:Bool=false;
    public var wMass:Float=0.0;
    public var adamp:Float=0.0;
    public var agamma:Float=0.0;
    public var vMassa:Float=0.0;
    public var vMassb:Float=0.0;
    public var vMassc:Float=0.0;
    public var dampx:Float=0.0;
    public var dampy:Float=0.0;
    public var lgamma:Float=0.0;
    public var nx:Float=0.0;
    public var ny:Float=0.0;
    public var buoyx:Float=0.0;
    public var buoyy:Float=0.0;
    private function position_validate(){
        #if(!NAPE_RELEASE_BUILD)
        if(inactiveme())throw "Error: Arbiter not currently in use";
        #end
        {
            wrap_position.zpp_inner.x=centroidx;
            wrap_position.zpp_inner.y=centroidy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_position.zpp_inner.x!=wrap_position.zpp_inner.x));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_position.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap_position.zpp_inner."+",in x: "+"centroidx"+",in y: "+"centroidy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_position.zpp_inner.y!=wrap_position.zpp_inner.y));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_position.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap_position.zpp_inner."+",in x: "+"centroidx"+",in y: "+"centroidy"+")");
                #end
            };
        };
    }
    private function position_invalidate(x:ZPP_Vec2){
        {
            centroidx=x.x;
            centroidy=x.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((centroidx!=centroidx));
                };
                if(!res)throw "assert("+"!assert_isNaN(centroidx)"+") :: "+("vec_set(in n: "+"centroid"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((centroidy!=centroidy));
                };
                if(!res)throw "assert("+"!assert_isNaN(centroidy)"+") :: "+("vec_set(in n: "+"centroid"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
        };
    }
    public var wrap_position:Vec2=null;
    public function getposition(){
        wrap_position=Vec2.get();
        wrap_position.zpp_inner._inuse=true;
        wrap_position.zpp_inner._immutable=!mutable;
        wrap_position.zpp_inner._validate=position_validate;
        wrap_position.zpp_inner._invalidate=position_invalidate;
    }
    public function new(){
        super();
        type=ZPP_Arbiter.FLUID;
        fluidarb=this;
        {
            buoyx=0;
            buoyy=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((buoyx!=buoyx));
                };
                if(!res)throw "assert("+"!assert_isNaN(buoyx)"+") :: "+("vec_set(in n: "+"buoy"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((buoyy!=buoyy));
                };
                if(!res)throw "assert("+"!assert_isNaN(buoyy)"+") :: "+("vec_set(in n: "+"buoy"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        pre_dt=-1.0;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function assign(s1:ZPP_Shape,s2:ZPP_Shape,id:Int,di:Int){
        sup_assign(s1,s2,id,di);
        {
            nx=0;
            ny=1;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((nx!=nx));
                };
                if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"0"+",in y: "+"1"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ny!=ny));
                };
                if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"0"+",in y: "+"1"+")");
                #end
            };
        };
        {
            dampx=0;
            dampy=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((dampx!=dampx));
                };
                if(!res)throw "assert("+"!assert_isNaN(dampx)"+") :: "+("vec_set(in n: "+"damp"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((dampy!=dampy));
                };
                if(!res)throw "assert("+"!assert_isNaN(dampy)"+") :: "+("vec_set(in n: "+"damp"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        adamp=0.0;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function retire(){
        sup_retire();
        {
            var o=this;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_FluidArbiter"+", in obj: "+"this"+")");
                #end
            };
            o.free();
            o.next=ZPP_FluidArbiter.zpp_pool;
            ZPP_FluidArbiter.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_FluidArbiter.POOL_CNT++;
            ZPP_FluidArbiter.POOL_SUB++;
            #end
        };
        pre_dt=-1.0;
    }
    public var mutable:Bool=false;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function makemutable(){
        mutable=true;
        if(wrap_position!=null)wrap_position.zpp_inner._immutable=false;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function makeimmutable(){
        mutable=false;
        if(wrap_position!=null)wrap_position.zpp_inner._immutable=true;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inject(area:Float,cx:Float,cy:Float){
        overlap=area;
        {
            centroidx=cx;
            centroidy=cy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((centroidx!=centroidx));
                };
                if(!res)throw "assert("+"!assert_isNaN(centroidx)"+") :: "+("vec_set(in n: "+"centroid"+",in x: "+"cx"+",in y: "+"cy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((centroidy!=centroidy));
                };
                if(!res)throw "assert("+"!assert_isNaN(centroidy)"+") :: "+("vec_set(in n: "+"centroid"+",in x: "+"cx"+",in y: "+"cy"+")");
                #end
            };
        };
    }
    public var pre_dt:Float=0.0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function preStep(s:ZPP_Space,dt:Float){
        if(pre_dt==-1.0)pre_dt=dt;
        var dtratio=dt/pre_dt;
        pre_dt=dt;
        {
            r1x=centroidx-b1.posx;
            r1y=centroidy-b1.posy;
        };
        {
            r2x=centroidx-b2.posx;
            r2y=centroidy-b2.posy;
        };
        var g1x:Float=0.0;
        var g1y:Float=0.0;
        if(ws1.fluidEnabled&&ws1.fluidProperties.wrap_gravity!=null){
            g1x=ws1.fluidProperties.gravityx;
            g1y=ws1.fluidProperties.gravityy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((g1x!=g1x));
                };
                if(!res)throw "assert("+"!assert_isNaN(g1x)"+") :: "+("vec_set(in n: "+"g1"+",in x: "+"ws1.fluidProperties.gravityx"+",in y: "+"ws1.fluidProperties.gravityy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((g1y!=g1y));
                };
                if(!res)throw "assert("+"!assert_isNaN(g1y)"+") :: "+("vec_set(in n: "+"g1"+",in x: "+"ws1.fluidProperties.gravityx"+",in y: "+"ws1.fluidProperties.gravityy"+")");
                #end
            };
        }
        else{
            g1x=s.gravityx;
            g1y=s.gravityy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((g1x!=g1x));
                };
                if(!res)throw "assert("+"!assert_isNaN(g1x)"+") :: "+("vec_set(in n: "+"g1"+",in x: "+"s.gravityx"+",in y: "+"s.gravityy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((g1y!=g1y));
                };
                if(!res)throw "assert("+"!assert_isNaN(g1y)"+") :: "+("vec_set(in n: "+"g1"+",in x: "+"s.gravityx"+",in y: "+"s.gravityy"+")");
                #end
            };
        };
        var g2x:Float=0.0;
        var g2y:Float=0.0;
        if(ws2.fluidEnabled&&ws2.fluidProperties.wrap_gravity!=null){
            g2x=ws2.fluidProperties.gravityx;
            g2y=ws2.fluidProperties.gravityy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((g2x!=g2x));
                };
                if(!res)throw "assert("+"!assert_isNaN(g2x)"+") :: "+("vec_set(in n: "+"g2"+",in x: "+"ws2.fluidProperties.gravityx"+",in y: "+"ws2.fluidProperties.gravityy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((g2y!=g2y));
                };
                if(!res)throw "assert("+"!assert_isNaN(g2y)"+") :: "+("vec_set(in n: "+"g2"+",in x: "+"ws2.fluidProperties.gravityx"+",in y: "+"ws2.fluidProperties.gravityy"+")");
                #end
            };
        }
        else{
            g2x=s.gravityx;
            g2y=s.gravityy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((g2x!=g2x));
                };
                if(!res)throw "assert("+"!assert_isNaN(g2x)"+") :: "+("vec_set(in n: "+"g2"+",in x: "+"s.gravityx"+",in y: "+"s.gravityy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((g2y!=g2y));
                };
                if(!res)throw "assert("+"!assert_isNaN(g2y)"+") :: "+("vec_set(in n: "+"g2"+",in x: "+"s.gravityx"+",in y: "+"s.gravityy"+")");
                #end
            };
        };
        var buoyx:Float=0;
        var buoyy:Float=0;
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((buoyx!=buoyx));
            };
            if(!res)throw "assert("+"!assert_isNaN(buoyx)"+") :: "+("vec_new(in n: "+"buoy"+",in x: "+"0"+",in y: "+"0"+")");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((buoyy!=buoyy));
            };
            if(!res)throw "assert("+"!assert_isNaN(buoyy)"+") :: "+("vec_new(in n: "+"buoy"+",in x: "+"0"+",in y: "+"0"+")");
            #end
        };
        if(ws1.fluidEnabled&&ws2.fluidEnabled){
            var mass1=overlap*ws1.fluidProperties.density;
            var mass2=overlap*ws2.fluidProperties.density;
            if(mass1>mass2){
                var t=(mass1+mass2);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"buoy"+",in b: "+"g1"+",in s: "+"mass1+mass2"+")");
                    #end
                };
                buoyx-=g1x*t;
                buoyy-=g1y*t;
            };
            else if(mass1<mass2){
                var t=(mass1+mass2);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"buoy"+",in b: "+"g2"+",in s: "+"mass1+mass2"+")");
                    #end
                };
                buoyx+=g2x*t;
                buoyy+=g2y*t;
            };
            else{
                var gx:Float=0.0;
                var gy:Float=0.0;
                {
                    gx=g1x+g2x;
                    gy=g1y+g2y;
                };
                {
                    var t=(0.5);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"g"+",in s: "+"0.5"+")");
                        #end
                    };
                    gx*=t;
                    gy*=t;
                };
                if((ws1.worldCOMx*gx+ws1.worldCOMy*gy)>(ws2.worldCOMx*gx+ws2.worldCOMy*gy)){
                    var t=(mass1+mass2);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"buoy"+",in b: "+"g"+",in s: "+"mass1+mass2"+")");
                        #end
                    };
                    buoyx-=gx*t;
                    buoyy-=gy*t;
                };
                else{
                    var t=(mass1+mass2);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"buoy"+",in b: "+"g"+",in s: "+"mass1+mass2"+")");
                        #end
                    };
                    buoyx+=gx*t;
                    buoyy+=gy*t;
                };
            }
        }
        else if(ws1.fluidEnabled){
            var mass=overlap*ws1.fluidProperties.density;
            {
                var t=(mass);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"buoy"+",in b: "+"g1"+",in s: "+"mass"+")");
                    #end
                };
                buoyx-=g1x*t;
                buoyy-=g1y*t;
            };
        }
        else if(ws2.fluidEnabled){
            var mass=overlap*ws2.fluidProperties.density;
            {
                var t=(mass);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"buoy"+",in b: "+"g2"+",in s: "+"mass"+")");
                    #end
                };
                buoyx+=g2x*t;
                buoyy+=g2y*t;
            };
        }
        {
            var t=(dt);
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((t!=t));
                };
                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"buoy"+",in s: "+"dt"+")");
                #end
            };
            buoyx*=t;
            buoyy*=t;
        };
        {
            this.buoyx=buoyx;
            this.buoyy=buoyy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((this.buoyx!=this.buoyx));
                };
                if(!res)throw "assert("+"!assert_isNaN(this.buoyx)"+") :: "+("vec_set(in n: "+"this.buoy"+",in x: "+"buoyx"+",in y: "+"buoyy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((this.buoyy!=this.buoyy));
                };
                if(!res)throw "assert("+"!assert_isNaN(this.buoyy)"+") :: "+("vec_set(in n: "+"this.buoy"+",in x: "+"buoyx"+",in y: "+"buoyy"+")");
                #end
            };
        };
        if(b1.isDynamic()){
            {
                var t=(b1.imass);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.vel"+",in b: "+"buoy"+",in s: "+"b1.imass"+")");
                    #end
                };
                b1.velx-=buoyx*t;
                b1.vely-=buoyy*t;
            };
            b1.angvel-=(buoyy*r1x-buoyx*r1y)*b1.iinertia;
        }
        if(b2.isDynamic()){
            {
                var t=(b2.imass);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.vel"+",in b: "+"buoy"+",in s: "+"b2.imass"+")");
                    #end
                };
                b2.velx+=buoyx*t;
                b2.vely+=buoyy*t;
            };
            b2.angvel+=(buoyy*r2x-buoyx*r2y)*b2.iinertia;
        }
        if((!ws1.fluidEnabled||ws1.fluidProperties.viscosity==0)&&(!ws2.fluidEnabled||ws2.fluidProperties.viscosity==0)){
            nodrag=true;
            {
                dampx=0;
                dampy=0;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((dampx!=dampx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(dampx)"+") :: "+("vec_set(in n: "+"damp"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((dampy!=dampy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(dampy)"+") :: "+("vec_set(in n: "+"damp"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
            };
            adamp=0;
        }
        else{
            nodrag=false;
            var tViscosity=0.0;
            if(ws1.fluidEnabled){
                ws2.validate_angDrag();
                tViscosity+=ws1.fluidProperties.viscosity*ws2.angDrag*overlap/ws2.area;
            }
            if(ws2.fluidEnabled){
                ws1.validate_angDrag();
                tViscosity+=ws2.fluidProperties.viscosity*ws1.angDrag*overlap/ws1.area;
            }
            if(tViscosity!=0){
                var iSum=b1.sinertia+b2.sinertia;
                if(iSum!=0)wMass=1/iSum;
                else wMass=0.0;
                var biasCoef;
                tViscosity*=0.0004;
                wMass*={
                    var omega=2*Math.PI*tViscosity;
                    agamma=1/(dt*omega*(2*1+omega*dt));
                    var ig=1/(1+agamma);
                    biasCoef=dt*omega*omega*agamma;
                    agamma*=ig;
                    ig;
                };
            }
            else{
                wMass=0.0;
                agamma=0.0;
            }
            var vrnx:Float=(b2.velx+b2.kinvelx-r2y*(b2.angvel+b2.kinangvel))-(b1.velx+b1.kinvelx-r1y*(b2.angvel+b2.kinangvel));
            var vrny:Float=(b2.vely+b2.kinvely+r2x*(b2.angvel+b2.kinangvel))-(b1.vely+b1.kinvely+r1x*(b1.angvel+b1.kinangvel));
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((vrnx!=vrnx));
                };
                if(!res)throw "assert("+"!assert_isNaN(vrnx)"+") :: "+("vec_new(in n: "+"vrn"+",in x: "+"(b2.velx+b2.kinvelx-r2y*(b2.angvel+b2.kinangvel))-(b1.velx+b1.kinvelx-r1y*(b2.angvel+b2.kinangvel))"+",in y: "+"(b2.vely+b2.kinvely+r2x*(b2.angvel+b2.kinangvel))-(b1.vely+b1.kinvely+r1x*(b1.angvel+b1.kinangvel))"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((vrny!=vrny));
                };
                if(!res)throw "assert("+"!assert_isNaN(vrny)"+") :: "+("vec_new(in n: "+"vrn"+",in x: "+"(b2.velx+b2.kinvelx-r2y*(b2.angvel+b2.kinangvel))-(b1.velx+b1.kinvelx-r1y*(b2.angvel+b2.kinangvel))"+",in y: "+"(b2.vely+b2.kinvely+r2x*(b2.angvel+b2.kinangvel))-(b1.vely+b1.kinvely+r1x*(b1.angvel+b1.kinangvel))"+")");
                #end
            };
            if((vrnx*vrnx+vrny*vrny)<(Config.epsilon*Config.epsilon)){}
            else{
                {
                    var d=(vrnx*vrnx+vrny*vrny);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            d!=0.0;
                        };
                        if(!res)throw "assert("+"d!=0.0"+") :: "+("vec_normalise(in n: "+"vrn"+")");
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
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"vrn"+",in s: "+"imag"+")");
                            #end
                        };
                        vrnx*=t;
                        vrny*=t;
                    };
                };
                {
                    nx=vrnx;
                    ny=vrny;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((nx!=nx));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"vrnx"+",in y: "+"vrny"+")");
                        #end
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((ny!=ny));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"vrnx"+",in y: "+"vrny"+")");
                        #end
                    };
                };
            }
            var tViscosity=0.0;
            if(ws1.fluidEnabled){
                var f=-ws1.fluidProperties.viscosity*overlap/ws2.area;
                if(ws2.type==ZPP_Flags.id_ShapeType_CIRCLE)tViscosity-=f*ws2.circle.radius*Config.fluidLinearDrag/(2*ws2.circle.radius*Math.PI);
                else{
                    var poly=ws2.polygon;
                    var bord=0.0;
                    var acc=0.0;
                    {
                        var cx_ite=poly.edges.begin();
                        while(cx_ite!=null){
                            var ex=cx_ite.elem();
                            {
                                bord+=ex.length;
                                var fact=f*ex.length*(ex.gnormx*nx+ex.gnormy*ny);
                                if(fact>0)fact=fact*=-Config.fluidVacuumDrag;
                                acc-=fact*0.5*Config.fluidLinearDrag;
                            };
                            cx_ite=cx_ite.next;
                        }
                    };
                    tViscosity+=acc/bord;
                }
            }
            if(ws2.fluidEnabled){
                var f=-ws2.fluidProperties.viscosity*overlap/ws1.area;
                if(ws1.type==ZPP_Flags.id_ShapeType_CIRCLE)tViscosity-=f*ws1.circle.radius*Config.fluidLinearDrag/(2*ws1.circle.radius*Math.PI);
                else{
                    var poly=ws1.polygon;
                    var bord=0.0;
                    var acc=0.0;
                    {
                        var cx_ite=poly.edges.begin();
                        while(cx_ite!=null){
                            var ex=cx_ite.elem();
                            {
                                bord+=ex.length;
                                var fact=f*ex.length*(ex.gnormx*nx+ex.gnormy*ny);
                                if(fact>0)fact=fact*=-Config.fluidVacuumDrag;
                                acc-=fact*0.5*Config.fluidLinearDrag;
                            };
                            cx_ite=cx_ite.next;
                        }
                    };
                    tViscosity+=acc/bord;
                }
            }
            if(tViscosity!=0){
                var m=b1.smass+b2.smass;
                var Ka:Float=0.0;
                var Kb:Float=0.0;
                var Kc:Float=0.0;
                {
                    Ka=m;
                    Kb=0;
                    Kc=m;
                };
                if(b1.sinertia!=0){
                    var X=r1x*b1.sinertia;
                    var Y=r1y*b1.sinertia;
                    {
                        Ka+=Y*r1y;
                        Kb+=-Y*r1x;
                        Kc+=X*r1x;
                    };
                };
                if(b2.sinertia!=0){
                    var X=r2x*b2.sinertia;
                    var Y=r2y*b2.sinertia;
                    {
                        Ka+=Y*r2y;
                        Kb+=-Y*r2x;
                        Kc+=X*r2x;
                    };
                };
                {
                    var det=(Ka*Kc-Kb*Kb);
                    if((det!=det)){
                        Ka=Kb=Kc=0;
                        3;
                    }
                    else if(det==0){
                        var flag=0;
                        if(Ka!=0)Ka=1/Ka;
                        else{
                            Ka=0;
                            flag|=1;
                        }
                        if(Kc!=0)Kc=1/Kc;
                        else{
                            Kc=0;
                            flag|=2;
                        }
                        Kb=0;
                        flag;
                    }
                    else{
                        det=1/det;
                        var t=Kc*det;
                        Kc=Ka*det;
                        Ka=t;
                        Kb*=-det;
                        0;
                    }
                };
                {
                    vMassa=Ka;
                    vMassb=Kb;
                    vMassc=Kc;
                };
                var biasCoef;
                {
                    var X=({
                        var omega=2*Math.PI*tViscosity;
                        lgamma=1/(dt*omega*(2*1+omega*dt));
                        var ig=1/(1+lgamma);
                        biasCoef=dt*omega*omega*lgamma;
                        lgamma*=ig;
                        ig;
                    });
                    vMassa*=X;
                    vMassb*=X;
                    vMassc*=X;
                };
            }
            else{
                {
                    vMassa=0;
                    vMassb=0;
                    vMassc=0;
                };
                lgamma=0.0;
            }
        }
        {
            var t=(dtratio);
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((t!=t));
                };
                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"damp"+",in s: "+"dtratio"+")");
                #end
            };
            dampx*=t;
            dampy*=t;
        };
        adamp*=dtratio;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function warmStart(){
        {
            var t=(b1.imass);
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((t!=t));
                };
                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.vel"+",in b: "+"damp"+",in s: "+"b1.imass"+")");
                #end
            };
            b1.velx-=dampx*t;
            b1.vely-=dampy*t;
        };
        {
            var t=(b2.imass);
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((t!=t));
                };
                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.vel"+",in b: "+"damp"+",in s: "+"b2.imass"+")");
                #end
            };
            b2.velx+=dampx*t;
            b2.vely+=dampy*t;
        };
        b1.angvel-=b1.iinertia*(dampy*r1x-dampx*r1y);
        b2.angvel+=b2.iinertia*(dampy*r2x-dampx*r2y);
        b1.angvel-=adamp*b1.iinertia;
        b2.angvel+=adamp*b2.iinertia;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function applyImpulseVel(){
        if(!nodrag){
            var w1=b1.angvel+b1.kinangvel;
            var w2=b2.angvel+b2.kinangvel;
            var jx:Float=(b1.velx+b1.kinvelx-r1y*w1)-(b2.velx+b2.kinvelx-r2y*w2);
            var jy:Float=(b1.vely+b1.kinvely+r1x*w1)-(b2.vely+b2.kinvely+r2x*w2);
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((jx!=jx));
                };
                if(!res)throw "assert("+"!assert_isNaN(jx)"+") :: "+("vec_new(in n: "+"j"+",in x: "+"(b1.velx+b1.kinvelx-r1y*w1)-(b2.velx+b2.kinvelx-r2y*w2)"+",in y: "+"(b1.vely+b1.kinvely+r1x*w1)-(b2.vely+b2.kinvely+r2x*w2)"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((jy!=jy));
                };
                if(!res)throw "assert("+"!assert_isNaN(jy)"+") :: "+("vec_new(in n: "+"j"+",in x: "+"(b1.velx+b1.kinvelx-r1y*w1)-(b2.velx+b2.kinvelx-r2y*w2)"+",in y: "+"(b1.vely+b1.kinvely+r1x*w1)-(b2.vely+b2.kinvely+r2x*w2)"+")");
                #end
            };
            {
                var t=vMassa*jx+vMassb*jy;
                jy=vMassb*jx+vMassc*jy;
                jx=t;
            };
            {
                var t=(lgamma);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"j"+",in b: "+"damp"+",in s: "+"lgamma"+")");
                    #end
                };
                jx-=dampx*t;
                jy-=dampy*t;
            };
            {
                var t=(1.0);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"damp"+",in b: "+"j"+",in s: "+"1.0"+")");
                    #end
                };
                dampx+=jx*t;
                dampy+=jy*t;
            };
            {
                var t=(b1.imass);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.vel"+",in b: "+"j"+",in s: "+"b1.imass"+")");
                    #end
                };
                b1.velx-=jx*t;
                b1.vely-=jy*t;
            };
            {
                var t=(b2.imass);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.vel"+",in b: "+"j"+",in s: "+"b2.imass"+")");
                    #end
                };
                b2.velx+=jx*t;
                b2.vely+=jy*t;
            };
            b1.angvel-=b1.iinertia*(jy*r1x-jx*r1y);
            b2.angvel+=b2.iinertia*(jy*r2x-jx*r2y);
            var j_damp=(w1-w2)*wMass-adamp*agamma;
            adamp+=j_damp;
            b1.angvel-=j_damp*b1.iinertia;
            b2.angvel+=j_damp*b2.iinertia;
        }
    }
}
#if nape_swc@:keep #end
class ZPP_ColArbiter extends ZPP_Arbiter{
    public var outer_zn:CollisionArbiter=null;
    public var dyn_fric:Float=0.0;
    public var stat_fric:Float=0.0;
    public var restitution:Float=0.0;
    public var rfric:Float=0.0;
    public var userdef_dyn_fric:Bool=false;
    public var userdef_stat_fric:Bool=false;
    public var userdef_restitution:Bool=false;
    public var userdef_rfric:Bool=false;
    public var s1:ZPP_Shape=null;
    public var s2:ZPP_Shape=null;
    public var contacts:ZPP_Contact=null;
    public var wrap_contacts:ContactList=null;
    public var innards:ZPP_IContact=null;
    public var nx:Float=0.0;
    public var ny:Float=0.0;
    private function normal_validate(){
        if(cleared)throw "Error: Arbiter not currently in use";
        {
            wrap_normal.zpp_inner.x=nx;
            wrap_normal.zpp_inner.y=ny;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_normal.zpp_inner.x!=wrap_normal.zpp_inner.x));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_normal.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap_normal.zpp_inner."+",in x: "+"nx"+",in y: "+"ny"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_normal.zpp_inner.y!=wrap_normal.zpp_inner.y));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_normal.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap_normal.zpp_inner."+",in x: "+"nx"+",in y: "+"ny"+")");
                #end
            };
        };
        if(ws1.id>ws2.id){
            {
                wrap_normal.zpp_inner.x=-wrap_normal.zpp_inner.x;
                wrap_normal.zpp_inner.y=-wrap_normal.zpp_inner.y;
            };
        }
    }
    public var wrap_normal:Vec2=null;
    public function getnormal(){
        wrap_normal=Vec2.get(0,0);
        wrap_normal.zpp_inner._immutable=true;
        wrap_normal.zpp_inner._inuse=true;
        wrap_normal.zpp_inner._validate=normal_validate;
    }
    var kMassa:Float=0.0;
    var kMassb:Float=0.0;
    var kMassc:Float=0.0;
    var Ka:Float=0.0;
    var Kb:Float=0.0;
    var Kc:Float=0.0;
    public var rMass:Float=0.0;
    public var jrAcc:Float=0.0;
    var rn1a:Float=0.0;
    var rt1a:Float=0.0;
    var rn1b:Float=0.0;
    var rt1b:Float=0.0;
    var rn2a:Float=0.0;
    var rt2a:Float=0.0;
    var rn2b:Float=0.0;
    var rt2b:Float=0.0;
    var k1x:Float=0.0;
    var k1y:Float=0.0;
    var k2x:Float=0.0;
    var k2y:Float=0.0;
    public var surfacex:Float=0.0;
    public var surfacey:Float=0.0;
    public static inline var FACE1=0;
    public static inline var FACE2=1;
    public static inline var CIRCLE=2;
    public var ptype:Int;
    public var lnormx:Float=0.0;
    public var lnormy:Float=0.0;
    public var lproj:Float=0.0;
    public var radius:Float=0.0;
    public var rev:Bool=false;
    var biasCoef:Float=0.0;
    public var __ref_edge1:ZPP_Edge=null;
    public var __ref_edge2:ZPP_Edge=null;
    public var __ref_vertex:Int=0;
    public var c1:ZPP_IContact=null;
    public var oc1:ZPP_Contact=null;
    public var c2:ZPP_IContact=null;
    public var oc2:ZPP_Contact=null;
    public var hc2:Bool=false;
    public var hpc2:Bool=false;
    public var next:ZPP_ColArbiter=null;
    static public var zpp_pool:ZPP_ColArbiter=null;
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
    
    public function new(){
        super();
        pre_dt=-1.0;
        contacts=new ZPP_Contact();
        innards=new ZPP_IContact();
        type=ZPP_Arbiter.COL;
        colarb=this;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc(){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                pair==null;
            };
            if(!res)throw "assert("+"pair==null"+") :: "+("AABBNode exists on col arbiter going out of pool?");
            #end
        };
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free(){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                contacts.empty();
            };
            if(!res)throw "assert("+"contacts.empty()"+") :: "+("still has contacts on free?");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                pair==null;
            };
            if(!res)throw "assert("+"pair==null"+") :: "+("AABBNode exists on col arbiter going into pool?");
            #end
        };
        userdef_dyn_fric=false;
        userdef_stat_fric=false;
        userdef_restitution=false;
        userdef_rfric=false;
        __ref_edge1=__ref_edge2=null;
    }
    public var stat:Bool=false;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function injectContact(px:Float,py:Float,nx:Float,ny:Float,dist:Float,hash:Int,posOnly=false){
        var c:ZPP_Contact=null;
        {
            var cx_ite=contacts.begin();
            while(cx_ite!=null){
                var cur=cx_ite.elem();
                if(hash==cur.hash){
                    c=cur;
                    break;
                };
                cx_ite=cx_ite.next;
            }
        };
        if(c==null){
            {
                if(ZPP_Contact.zpp_pool==null){
                    c=new ZPP_Contact();
                    #if NAPE_POOL_STATS ZPP_Contact.POOL_TOT++;
                    ZPP_Contact.POOL_ADDNEW++;
                    #end
                }
                else{
                    c=ZPP_Contact.zpp_pool;
                    ZPP_Contact.zpp_pool=c.next;
                    c.next=null;
                    #if NAPE_POOL_STATS ZPP_Contact.POOL_CNT--;
                    ZPP_Contact.POOL_ADD++;
                    #end
                }
                c.alloc();
            };
            var ci=c.inner;
            ci.jnAcc=ci.jtAcc=0;
            c.hash=hash;
            c.fresh=true;
            c.arbiter=this;
            jrAcc=0;
            contacts.inlined_add(c);
            innards.add(ci);
        }
        else c.fresh=false;
        {
            c.px=px;
            c.py=py;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((c.px!=c.px));
                };
                if(!res)throw "assert("+"!assert_isNaN(c.px)"+") :: "+("vec_set(in n: "+"c.p"+",in x: "+"px"+",in y: "+"py"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((c.py!=c.py));
                };
                if(!res)throw "assert("+"!assert_isNaN(c.py)"+") :: "+("vec_set(in n: "+"c.p"+",in x: "+"px"+",in y: "+"py"+")");
                #end
            };
        };
        {
            this.nx=nx;
            this.ny=ny;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((this.nx!=this.nx));
                };
                if(!res)throw "assert("+"!assert_isNaN(this.nx)"+") :: "+("vec_set(in n: "+"this.n"+",in x: "+"nx"+",in y: "+"ny"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((this.ny!=this.ny));
                };
                if(!res)throw "assert("+"!assert_isNaN(this.ny)"+") :: "+("vec_set(in n: "+"this.n"+",in x: "+"nx"+",in y: "+"ny"+")");
                #end
            };
        };
        c.dist=dist;
        c.stamp=stamp;
        c.posOnly=posOnly;
        return c;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function assign(s1:ZPP_Shape,s2:ZPP_Shape,id:Int,di:Int){
        sup_assign(s1,s2,id,di);
        this.s1=s1;
        this.s2=s2;
        calcProperties();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function calcProperties(){
        if(!userdef_restitution){
            if(s1.material.elasticity<=ZPP_Const.NEGINF()||s2.material.elasticity<=ZPP_Const.NEGINF())restitution=0;
            else if(s1.material.elasticity>=ZPP_Const.POSINF()||s2.material.elasticity>=ZPP_Const.POSINF())restitution=1;
            else restitution=(s1.material.elasticity+s2.material.elasticity)/2;
            if(restitution<0)restitution=0;
            if(restitution>1)restitution=1;
        }
        if(!userdef_dyn_fric){
            dyn_fric=ZPP_Math.sqrt(s1.material.dynamicFriction*s2.material.dynamicFriction);
        }
        if(!userdef_stat_fric){
            stat_fric=ZPP_Math.sqrt(s1.material.staticFriction*s2.material.staticFriction);
        }
        if(!userdef_rfric){
            rfric=ZPP_Math.sqrt(s1.material.rollingFriction*s2.material.rollingFriction);
        }
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function validate_props(){
        if(invalidated){
            invalidated=false;
            calcProperties();
        }
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function retire(){
        sup_retire();
        while(!contacts.empty()){
            {
                var o=contacts.inlined_pop_unsafe();
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Contact"+", in obj: "+"contacts.inlined_pop_unsafe()"+")");
                    #end
                };
                o.free();
                o.next=ZPP_Contact.zpp_pool;
                ZPP_Contact.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_Contact.POOL_CNT++;
                ZPP_Contact.POOL_SUB++;
                #end
            };
            innards.inlined_pop();
        }
        {
            var o=this;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_ColArbiter"+", in obj: "+"this"+")");
                #end
            };
            o.free();
            o.next=ZPP_ColArbiter.zpp_pool;
            ZPP_ColArbiter.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_ColArbiter.POOL_CNT++;
            ZPP_ColArbiter.POOL_SUB++;
            #end
        };
        pre_dt=-1.0;
    }
    public var mutable:Bool=false;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function makemutable(){
        mutable=true;
        if(wrap_normal!=null)wrap_normal.zpp_inner._immutable=false;
        if(wrap_contacts!=null)wrap_contacts.zpp_inner.immutable=false;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function makeimmutable(){
        mutable=false;
        if(wrap_normal!=null)wrap_normal.zpp_inner._immutable=true;
        if(wrap_contacts!=null)wrap_contacts.zpp_inner.immutable=true;
    }
    private function contacts_adder(x:Contact){
        #if(!NAPE_RELEASE_BUILD)
        throw "Error: Cannot add new contacts, information required is far too specific and detailed :)";
        #end
        return false;
    }
    private function contacts_subber(x:Contact){
        var pre=null;
        var prei=null;
        var cx_itei=innards.begin();
        {
            var cx_ite=contacts.begin();
            while(cx_ite!=null){
                var c=cx_ite.elem();
                {
                    if(c==x.zpp_inner){
                        contacts.erase(pre);
                        innards.erase(prei);
                        {
                            var o=c;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    o!=null;
                                };
                                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Contact"+", in obj: "+"c"+")");
                                #end
                            };
                            o.free();
                            o.next=ZPP_Contact.zpp_pool;
                            ZPP_Contact.zpp_pool=o;
                            #if NAPE_POOL_STATS ZPP_Contact.POOL_CNT++;
                            ZPP_Contact.POOL_SUB++;
                            #end
                        };
                        break;
                    }
                    pre=cx_ite;
                    prei=cx_itei;
                    cx_itei=cx_itei.next;
                };
                cx_ite=cx_ite.next;
            }
        };
    }
    public function setupcontacts(){
        wrap_contacts=ZPP_ContactList.get(contacts,true);
        wrap_contacts.zpp_inner.immutable=!mutable;
        wrap_contacts.zpp_inner.adder=contacts_adder;
        wrap_contacts.zpp_inner.dontremove=true;
        wrap_contacts.zpp_inner.subber=contacts_subber;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function cleanupContacts(){
        var fst=true;
        var pre=null;
        var prei=null;
        var cx_itei=innards.begin();
        hc2=false;
        {
            var cx_ite=contacts.begin();
            while(cx_ite!=null){
                var c=cx_ite.elem();
                {
                    if(c.stamp+Config.arbiterExpirationDelay<stamp){
                        cx_ite=contacts.inlined_erase(pre);
                        cx_itei=innards.inlined_erase(prei);
                        {
                            var o=c;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    o!=null;
                                };
                                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Contact"+", in obj: "+"c"+")");
                                #end
                            };
                            o.free();
                            o.next=ZPP_Contact.zpp_pool;
                            ZPP_Contact.zpp_pool=o;
                            #if NAPE_POOL_STATS ZPP_Contact.POOL_CNT++;
                            ZPP_Contact.POOL_SUB++;
                            #end
                        };
                        continue;
                    }
                    var ci=c.inner;
                    var pact=c.active;
                    c.active=c.stamp==stamp;
                    if(c.active){
                        if(fst){
                            fst=false;
                            c1=ci;
                            oc1=c;
                        }
                        else{
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !hc2;
                                };
                                if(!res)throw "assert("+"!hc2"+") :: "+("arbiter has +2 contacts??");
                                #end
                            };
                            hc2=true;
                            c2=ci;
                            oc2=c;
                        }
                    }
                    if(pact!=c.active)contacts.modified=true;
                    pre=cx_ite;
                    prei=cx_itei;
                    cx_itei=cx_itei.next;
                };
                cx_ite=cx_ite.next;
            }
        };
        if(hc2){
            hpc2=true;
            if(oc1.posOnly){
                var tmp=c1;
                c1=c2;
                c2=tmp;
                var tmp2=oc1;
                oc1=oc2;
                oc2=tmp2;
                hc2=false;
            }
            else if(oc2.posOnly){
                hc2=false;
            }
            if(oc1.posOnly){
                fst=true;
            }
        }
        else{
            hpc2=false;
        }
        return fst;
    }
    public var pre_dt:Float=0.0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function preStep(dt:Float){
        validate_props();
        #if NAPE_TIMES Debug.AACNT++;
        #end
        if(pre_dt==-1.0)pre_dt=dt;
        var dtratio=dt/pre_dt;
        pre_dt=dt;
        var mass_sum=b1.smass+b2.smass;
        hc2=false;
        var fst=true;
        var statType=(!b1.isDynamic()||!b2.isDynamic());
        var bias=(statType?(continuous?Config.contactContinuousStaticBiasCoef:Config.contactStaticBiasCoef):(continuous?Config.contactContinuousBiasCoef:Config.contactBiasCoef));
        biasCoef=bias;
        continuous=false;
        var pre=null;
        var prei=null;
        var cx_itei=innards.begin();
        {
            var cx_ite=contacts.begin();
            while(cx_ite!=null){
                var c=cx_ite.elem();
                {
                    if(c.stamp+Config.arbiterExpirationDelay<stamp){
                        cx_ite=contacts.inlined_erase(pre);
                        cx_itei=innards.inlined_erase(prei);
                        {
                            var o=c;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    o!=null;
                                };
                                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Contact"+", in obj: "+"c"+")");
                                #end
                            };
                            o.free();
                            o.next=ZPP_Contact.zpp_pool;
                            ZPP_Contact.zpp_pool=o;
                            #if NAPE_POOL_STATS ZPP_Contact.POOL_CNT++;
                            ZPP_Contact.POOL_SUB++;
                            #end
                        };
                        continue;
                    }
                    #if NAPE_TIMES Debug.CCNT++;
                    #end
                    var ci=c.inner;
                    var pact=c.active;
                    c.active=c.stamp==stamp;
                    if(c.active){
                        #if NAPE_TIMES Debug.ACCNT++;
                        #end
                        if(fst){
                            fst=false;
                            c1=ci;
                            oc1=c;
                        }
                        else{
                            hc2=true;
                            c2=ci;
                            oc2=c;
                        }
                        {
                            ci.r2x=c.px-b2.posx;
                            ci.r2y=c.py-b2.posy;
                        };
                        {
                            ci.r1x=c.px-b1.posx;
                            ci.r1y=c.py-b1.posy;
                        };
                        var kt=mass_sum+b2.sinertia*ZPP_Math.sqr((ci.r2x*nx+ci.r2y*ny));
                        kt+=b1.sinertia*ZPP_Math.sqr((ci.r1x*nx+ci.r1y*ny));
                        ci.tMass=if(kt<Config.epsilon*Config.epsilon)0 else 1.0/kt;
                        var nt=mass_sum+b2.sinertia*ZPP_Math.sqr((ny*ci.r2x-nx*ci.r2y));
                        nt+=b1.sinertia*ZPP_Math.sqr((ny*ci.r1x-nx*ci.r1y));
                        ci.nMass=if(nt<Config.epsilon*Config.epsilon)0 else 1.0/nt;
                        var vrx:Float=0.0;
                        var vry:Float=0.0;
                        {
                            var ang=b2.angvel+b2.kinangvel;
                            vrx=(b2.velx+b2.kinvelx-ci.r2y*ang);
                            vry=(b2.vely+b2.kinvely+ci.r2x*ang);
                            ang=b1.angvel+b1.kinangvel;
                            vrx-=(b1.velx+b1.kinvelx-ci.r1y*ang);
                            vry-=(b1.vely+b1.kinvely+ci.r1x*ang);
                        };
                        var vdot=(nx*vrx+ny*vry);
                        c.elasticity=restitution;
                        ci.bounce=vdot*c.elasticity;
                        if(ci.bounce>-Config.elasticThreshold){
                            ci.bounce=0;
                        }
                        vdot=(vry*nx-vrx*ny);
                        var thr=Config.staticFrictionThreshold;
                        if(vdot*vdot>thr*thr){
                            ci.friction=dyn_fric;
                        }
                        else{
                            ci.friction=stat_fric;
                        }
                        ci.jnAcc*=dtratio;
                        ci.jtAcc*=dtratio;
                    }
                    if(pact!=c.active)contacts.modified=true;
                    pre=cx_ite;
                    prei=cx_itei;
                    cx_itei=cx_itei.next;
                };
                cx_ite=cx_ite.next;
            }
        };
        if(hc2){
            hpc2=true;
            if(oc1.posOnly){
                var tmp=c1;
                c1=c2;
                c2=tmp;
                var tmp2=oc1;
                oc1=oc2;
                oc2=tmp2;
                hc2=false;
            }
            else if(oc2.posOnly){
                hc2=false;
            }
            if(oc1.posOnly){
                fst=true;
            }
        }
        else{
            hpc2=false;
        }
        jrAcc*=dtratio;
        if(!fst){
            rn1a=(ny*c1.r1x-nx*c1.r1y);
            rt1a=(c1.r1x*nx+c1.r1y*ny);
            rn1b=(ny*c1.r2x-nx*c1.r2y);
            rt1b=(c1.r2x*nx+c1.r2y*ny);
            k1x=b2.kinvelx-c1.r2y*b2.kinangvel-(b1.kinvelx-c1.r1y*b1.kinangvel);
            k1y=b2.kinvely+c1.r2x*b2.kinangvel-(b1.kinvely+c1.r1x*b1.kinangvel);
        }
        if(hc2){
            rn2a=(ny*c2.r1x-nx*c2.r1y);
            rt2a=(c2.r1x*nx+c2.r1y*ny);
            rn2b=(ny*c2.r2x-nx*c2.r2y);
            rt2b=(c2.r2x*nx+c2.r2y*ny);
            k2x=b2.kinvelx-c2.r2y*b2.kinangvel-(b1.kinvelx-c2.r1y*b1.kinangvel);
            k2y=b2.kinvely+c2.r2x*b2.kinangvel-(b1.kinvely+c2.r1x*b1.kinangvel);
            {
                kMassa=mass_sum+b1.sinertia*rn1a*rn1a+b2.sinertia*rn1b*rn1b;
                kMassb=mass_sum+b1.sinertia*rn1a*rn2a+b2.sinertia*rn1b*rn2b;
                kMassc=mass_sum+b1.sinertia*rn2a*rn2a+b2.sinertia*rn2b*rn2b;
            };
            var norm=(kMassa*kMassa+2*kMassb*kMassb+kMassc*kMassc);
            if(norm<Config.illConditionedThreshold*(kMassa*kMassc-kMassb*kMassb)){
                {
                    Ka=kMassa;
                    Kb=kMassb;
                    Kc=kMassc;
                };
                {
                    var det=(kMassa*kMassc-kMassb*kMassb);
                    if((det!=det)){
                        kMassa=kMassb=kMassc=0;
                        3;
                    }
                    else if(det==0){
                        var flag=0;
                        if(kMassa!=0)kMassa=1/kMassa;
                        else{
                            kMassa=0;
                            flag|=1;
                        }
                        if(kMassc!=0)kMassc=1/kMassc;
                        else{
                            kMassc=0;
                            flag|=2;
                        }
                        kMassb=0;
                        flag;
                    }
                    else{
                        det=1/det;
                        var t=kMassc*det;
                        kMassc=kMassa*det;
                        kMassa=t;
                        kMassb*=-det;
                        0;
                    }
                };
            }
            else{
                hc2=false;
                if(oc2.dist<oc1.dist){
                    var t=c1;
                    c1=c2;
                    c2=t;
                };
                oc2.active=false;
                contacts.modified=true;
            }
        }
        {
            surfacex=b2.svelx;
            surfacey=b2.svely;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((surfacex!=surfacex));
                };
                if(!res)throw "assert("+"!assert_isNaN(surfacex)"+") :: "+("vec_set(in n: "+"surface"+",in x: "+"b2.svelx"+",in y: "+"b2.svely"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((surfacey!=surfacey));
                };
                if(!res)throw "assert("+"!assert_isNaN(surfacey)"+") :: "+("vec_set(in n: "+"surface"+",in x: "+"b2.svelx"+",in y: "+"b2.svely"+")");
                #end
            };
        };
        {
            var t=(1.0);
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((t!=t));
                };
                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"surface"+",in b: "+"b1.svel"+",in s: "+"1.0"+")");
                #end
            };
            surfacex+=b1.svelx*t;
            surfacey+=b1.svely*t;
        };
        {
            surfacex=-surfacex;
            surfacey=-surfacey;
        };
        rMass=b1.sinertia+b2.sinertia;
        if(rMass!=0)rMass=1/rMass;
        return fst;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function warmStart(){
        {
            var jx=nx*c1.jnAcc-ny*c1.jtAcc;
            var jy=ny*c1.jnAcc+nx*c1.jtAcc;
            {
                var t=(b1.imass);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.vel"+",in b: "+"j"+",in s: "+"b1.imass"+")");
                    #end
                };
                b1.velx-=jx*t;
                b1.vely-=jy*t;
            };
            b1.angvel-=b1.iinertia*(jy*c1.r1x-jx*c1.r1y);
            {
                var t=(b2.imass);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.vel"+",in b: "+"j"+",in s: "+"b2.imass"+")");
                    #end
                };
                b2.velx+=jx*t;
                b2.vely+=jy*t;
            };
            b2.angvel+=b2.iinertia*(jy*c1.r2x-jx*c1.r2y);
        };
        if(hc2){
            var jx=nx*c2.jnAcc-ny*c2.jtAcc;
            var jy=ny*c2.jnAcc+nx*c2.jtAcc;
            {
                var t=(b1.imass);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.vel"+",in b: "+"j"+",in s: "+"b1.imass"+")");
                    #end
                };
                b1.velx-=jx*t;
                b1.vely-=jy*t;
            };
            b1.angvel-=b1.iinertia*(jy*c2.r1x-jx*c2.r1y);
            {
                var t=(b2.imass);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.vel"+",in b: "+"j"+",in s: "+"b2.imass"+")");
                    #end
                };
                b2.velx+=jx*t;
                b2.vely+=jy*t;
            };
            b2.angvel+=b2.iinertia*(jy*c2.r2x-jx*c2.r2y);
        };
        b2.angvel+=jrAcc*b2.iinertia;
        b1.angvel-=jrAcc*b1.iinertia;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function applyImpulseVel(){
        var jx:Float;
        var jy:Float;
        var j:Float;
        var jMax:Float;
        var jOld:Float;
        var cjAcc:Float;
        var v1x=k1x+b2.velx-c1.r2y*b2.angvel-(b1.velx-c1.r1y*b1.angvel);
        var v1y=k1y+b2.vely+c1.r2x*b2.angvel-(b1.vely+c1.r1x*b1.angvel);
        j=((v1y*nx-v1x*ny)+surfacex)*c1.tMass;
        jMax=c1.friction*c1.jnAcc;
        jOld=c1.jtAcc;
        cjAcc=jOld-j;
        if(cjAcc>jMax)cjAcc=jMax else if(cjAcc<-jMax)cjAcc=-jMax;
        j=cjAcc-jOld;
        c1.jtAcc=cjAcc;
        jx=-ny*j;
        jy=nx*j;
        b2.velx+=jx*b2.imass;
        b2.vely+=jy*b2.imass;
        b1.velx-=jx*b1.imass;
        b1.vely-=jy*b1.imass;
        b2.angvel+=rt1b*j*b2.iinertia;
        b1.angvel-=rt1a*j*b1.iinertia;
        if(hc2){
            var v2x=k2x+b2.velx-c2.r2y*b2.angvel-(b1.velx-c2.r1y*b1.angvel);
            var v2y=k2y+b2.vely+c2.r2x*b2.angvel-(b1.vely+c2.r1x*b1.angvel);
            j=((v2y*nx-v2x*ny)+surfacex)*c2.tMass;
            jMax=c2.friction*c2.jnAcc;
            jOld=c2.jtAcc;
            cjAcc=jOld-j;
            if(cjAcc>jMax)cjAcc=jMax else if(cjAcc<-jMax)cjAcc=-jMax;
            j=cjAcc-jOld;
            c2.jtAcc=cjAcc;
            jx=-ny*j;
            jy=nx*j;
            b2.velx+=jx*b2.imass;
            b2.vely+=jy*b2.imass;
            b1.velx-=jx*b1.imass;
            b1.vely-=jy*b1.imass;
            b2.angvel+=rt2b*j*b2.iinertia;
            b1.angvel-=rt2a*j*b1.iinertia;
            v1x=k1x+b2.velx-c1.r2y*b2.angvel-(b1.velx-c1.r1y*b1.angvel);
            v1y=k1y+b2.vely+c1.r2x*b2.angvel-(b1.vely+c1.r1x*b1.angvel);
            v2x=k2x+b2.velx-c2.r2y*b2.angvel-(b1.velx-c2.r1y*b1.angvel);
            v2y=k2y+b2.vely+c2.r2x*b2.angvel-(b1.vely+c2.r1x*b1.angvel);
            var ax:Float=c1.jnAcc;
            var ay:Float=c2.jnAcc;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ax!=ax));
                };
                if(!res)throw "assert("+"!assert_isNaN(ax)"+") :: "+("vec_new(in n: "+"a"+",in x: "+"c1.jnAcc"+",in y: "+"c2.jnAcc"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ay!=ay));
                };
                if(!res)throw "assert("+"!assert_isNaN(ay)"+") :: "+("vec_new(in n: "+"a"+",in x: "+"c1.jnAcc"+",in y: "+"c2.jnAcc"+")");
                #end
            };
            var jnx=(v1x*nx+v1y*ny)+surfacey+(c1.bounce)-(Ka*ax+Kb*ay);
            var jny=(v2x*nx+v2y*ny)+surfacey+(c2.bounce)-(Kb*ax+Kc*ay);
            var xx=-(kMassa*jnx+kMassb*jny);
            var xy=-(kMassb*jnx+kMassc*jny);
            if(xx>=0&&xy>=0){
                {
                    jnx=xx-ax;
                    jny=xy-ay;
                };
                c1.jnAcc=xx;
                c2.jnAcc=xy;
            }
            else{
                xx=-c1.nMass*jnx;
                if(xx>=0&&(Kb*xx+jny)>=0){
                    jnx=xx-ax;
                    jny=-ay;
                    c1.jnAcc=xx;
                    c2.jnAcc=0;
                }
                else{
                    xy=-c2.nMass*jny;
                    if(xy>=0&&(Kb*xy+jnx)>=0){
                        jnx=-ax;
                        jny=xy-ay;
                        c1.jnAcc=0;
                        c2.jnAcc=xy;
                    }
                    else if(jnx>=0&&jny>=0){
                        jnx=-ax;
                        jny=-ay;
                        c1.jnAcc=c2.jnAcc=0;
                    }
                    else{
                        jnx=0;
                        jny=0;
                    }
                }
            }
            j=jnx+jny;
            jx=nx*j;
            jy=ny*j;
            b2.velx+=jx*b2.imass;
            b2.vely+=jy*b2.imass;
            b1.velx-=jx*b1.imass;
            b1.vely-=jy*b1.imass;
            b2.angvel+=(rn1b*jnx+rn2b*jny)*b2.iinertia;
            b1.angvel-=(rn1a*jnx+rn2a*jny)*b1.iinertia;
        }
        else{
            if(radius!=0.0){
                var dw=b2.angvel-b1.angvel;
                j=dw*rMass;
                jMax=rfric*c1.jnAcc;
                jOld=jrAcc;
                jrAcc-=j;
                if(jrAcc>jMax)jrAcc=jMax else if(jrAcc<-jMax)jrAcc=-jMax;
                j=jrAcc-jOld;
                b2.angvel+=j*b2.iinertia;
                b1.angvel-=j*b1.iinertia;
            }
            v1x=k1x+b2.velx-c1.r2y*b2.angvel-(b1.velx-c1.r1y*b1.angvel);
            v1y=k1y+b2.vely+c1.r2x*b2.angvel-(b1.vely+c1.r1x*b1.angvel);
            j=(c1.bounce+(nx*v1x+ny*v1y)+surfacey)*c1.nMass;
            jOld=c1.jnAcc;
            cjAcc=jOld-j;
            if(cjAcc<0.0)cjAcc=0.0;
            j=cjAcc-jOld;
            c1.jnAcc=cjAcc;
            jx=nx*j;
            jy=ny*j;
            b2.velx+=jx*b2.imass;
            b2.vely+=jy*b2.imass;
            b1.velx-=jx*b1.imass;
            b1.vely-=jy*b1.imass;
            b2.angvel+=rn1b*j*b2.iinertia;
            b1.angvel-=rn1a*j*b1.iinertia;
        }
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function applyImpulsePos(){
        if(ptype==ZPP_ColArbiter.CIRCLE){
            var c=c1;
            var dx:Float=0.0;
            var dy:Float=0.0;
            var r2x:Float=0.0;
            var r2y:Float=0.0;
            {
                r2x=(b2.axisy*c.lr2x-b2.axisx*c.lr2y);
                r2y=(c.lr2x*b2.axisx+c.lr2y*b2.axisy);
            };
            {
                var t=(1.0);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"r2"+",in b: "+"b2.pos"+",in s: "+"1.0"+")");
                    #end
                };
                r2x+=b2.posx*t;
                r2y+=b2.posy*t;
            };
            var r1x:Float=0.0;
            var r1y:Float=0.0;
            {
                r1x=(b1.axisy*c.lr1x-b1.axisx*c.lr1y);
                r1y=(c.lr1x*b1.axisx+c.lr1y*b1.axisy);
            };
            {
                var t=(1.0);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"r1"+",in b: "+"b1.pos"+",in s: "+"1.0"+")");
                    #end
                };
                r1x+=b1.posx*t;
                r1y+=b1.posy*t;
            };
            var dx:Float=0.0;
            var dy:Float=0.0;
            {
                dx=r2x-r1x;
                dy=r2y-r1y;
            };
            var dl=ZPP_Math.sqrt((dx*dx+dy*dy));
            var r=radius-Config.collisionSlop;
            var err=(dl-r);
            if((dx*nx+dy*ny)<0){
                {
                    dx=-dx;
                    dy=-dy;
                };
                err-=radius;
            }
            if(err<0){
                if(dl<Config.epsilon){
                    if(b1.smass!=0.0)b1.posx+=Config.epsilon*10;
                    else b2.posx+=Config.epsilon*10;
                }
                else{
                    {
                        var t=(1.0/(dl));
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"d"+",in s: "+"1.0/(dl)"+")");
                            #end
                        };
                        dx*=t;
                        dy*=t;
                    };
                    var px:Float=0.5*(r1x+r2x);
                    var py:Float=0.5*(r1y+r2y);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((px!=px));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(px)"+") :: "+("vec_new(in n: "+"p"+",in x: "+"0.5*(r1x+r2x)"+",in y: "+"0.5*(r1y+r2y)"+")");
                        #end
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((py!=py));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(py)"+") :: "+("vec_new(in n: "+"p"+",in x: "+"0.5*(r1x+r2x)"+",in y: "+"0.5*(r1y+r2y)"+")");
                        #end
                    };
                    var pen=dl-r;
                    {
                        r1x=px-b1.posx;
                        r1y=py-b1.posy;
                    };
                    {
                        r2x=px-b2.posx;
                        r2y=py-b2.posy;
                    };
                    var rn1=(dy*r1x-dx*r1y);
                    var rn2=(dy*r2x-dx*r2y);
                    var K=b2.smass+rn2*rn2*b2.sinertia+b1.smass+rn1*rn1*b1.sinertia;
                    if(K!=0){
                        var jn=-biasCoef*pen/K;
                        var Jx:Float=0.0;
                        var Jy:Float=0.0;
                        {
                            var t=(jn);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_mul(in a: "+"d"+",in s: "+"jn"+",out r: "+"J"+")");
                                #end
                            };
                            Jx=dx*t;
                            Jy=dy*t;
                        };
                        {
                            var t=(b1.imass);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.pos"+",in b: "+"J"+",in s: "+"b1.imass"+")");
                                #end
                            };
                            b1.posx-=Jx*t;
                            b1.posy-=Jy*t;
                        };
                        b1.delta_rot(-rn1*b1.iinertia*jn);
                        {
                            var t=(b2.imass);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.pos"+",in b: "+"J"+",in s: "+"b2.imass"+")");
                                #end
                            };
                            b2.posx+=Jx*t;
                            b2.posy+=Jy*t;
                        };
                        b2.delta_rot(rn2*b2.iinertia*jn);
                    }
                }
            }
        }
        else{
            var gnormx:Float=0.0;
            var gnormy:Float=0.0;
            var gproj;
            var clip1x:Float=0.0;
            var clip1y:Float=0.0;
            var clip2x:Float=0;
            var clip2y:Float=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((clip2x!=clip2x));
                };
                if(!res)throw "assert("+"!assert_isNaN(clip2x)"+") :: "+("vec_new(in n: "+"clip2"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((clip2y!=clip2y));
                };
                if(!res)throw "assert("+"!assert_isNaN(clip2y)"+") :: "+("vec_new(in n: "+"clip2"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            if(ptype==ZPP_ColArbiter.FACE1){
                {
                    gnormx=(b1.axisy*lnormx-b1.axisx*lnormy);
                    gnormy=(lnormx*b1.axisx+lnormy*b1.axisy);
                };
                gproj=lproj+(gnormx*b1.posx+gnormy*b1.posy);
                {
                    clip1x=(b2.axisy*c1.lr1x-b2.axisx*c1.lr1y);
                    clip1y=(c1.lr1x*b2.axisx+c1.lr1y*b2.axisy);
                };
                {
                    var t=(1.0);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"clip1"+",in b: "+"b2.pos"+",in s: "+"1.0"+")");
                        #end
                    };
                    clip1x+=b2.posx*t;
                    clip1y+=b2.posy*t;
                };
                if(hpc2){
                    {
                        clip2x=(b2.axisy*c2.lr1x-b2.axisx*c2.lr1y);
                        clip2y=(c2.lr1x*b2.axisx+c2.lr1y*b2.axisy);
                    };
                    {
                        var t=(1.0);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"clip2"+",in b: "+"b2.pos"+",in s: "+"1.0"+")");
                            #end
                        };
                        clip2x+=b2.posx*t;
                        clip2y+=b2.posy*t;
                    };
                }
            }
            else{
                {
                    gnormx=(b2.axisy*lnormx-b2.axisx*lnormy);
                    gnormy=(lnormx*b2.axisx+lnormy*b2.axisy);
                };
                gproj=lproj+(gnormx*b2.posx+gnormy*b2.posy);
                {
                    clip1x=(b1.axisy*c1.lr1x-b1.axisx*c1.lr1y);
                    clip1y=(c1.lr1x*b1.axisx+c1.lr1y*b1.axisy);
                };
                {
                    var t=(1.0);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"clip1"+",in b: "+"b1.pos"+",in s: "+"1.0"+")");
                        #end
                    };
                    clip1x+=b1.posx*t;
                    clip1y+=b1.posy*t;
                };
                if(hpc2){
                    {
                        clip2x=(b1.axisy*c2.lr1x-b1.axisx*c2.lr1y);
                        clip2y=(c2.lr1x*b1.axisx+c2.lr1y*b1.axisy);
                    };
                    {
                        var t=(1.0);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"clip2"+",in b: "+"b1.pos"+",in s: "+"1.0"+")");
                            #end
                        };
                        clip2x+=b1.posx*t;
                        clip2y+=b1.posy*t;
                    };
                }
            }
            var err1=(clip1x*gnormx+clip1y*gnormy)-gproj-radius;
            err1+=Config.collisionSlop;
            var err2=0.0;
            if(hpc2){
                err2=(clip2x*gnormx+clip2y*gnormy)-gproj-radius;
                err2+=Config.collisionSlop;
            }
            if(err1<0||err2<0){
                if(rev){
                    gnormx=-gnormx;
                    gnormy=-gnormy;
                };
                var c1r1x:Float=0.0;
                var c1r1y:Float=0.0;
                {
                    c1r1x=clip1x-b1.posx;
                    c1r1y=clip1y-b1.posy;
                };
                var c1r2x:Float=0.0;
                var c1r2y:Float=0.0;
                {
                    c1r2x=clip1x-b2.posx;
                    c1r2y=clip1y-b2.posy;
                };
                var c2r1x:Float=0;
                var c2r1y:Float=0;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((c2r1x!=c2r1x));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(c2r1x)"+") :: "+("vec_new(in n: "+"c2r1"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((c2r1y!=c2r1y));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(c2r1y)"+") :: "+("vec_new(in n: "+"c2r1"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
                var c2r2x:Float=0;
                var c2r2y:Float=0;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((c2r2x!=c2r2x));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(c2r2x)"+") :: "+("vec_new(in n: "+"c2r2"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((c2r2y!=c2r2y));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(c2r2y)"+") :: "+("vec_new(in n: "+"c2r2"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
                if(hpc2){
                    {
                        c2r1x=clip2x-b1.posx;
                        c2r1y=clip2y-b1.posy;
                    };
                    {
                        c2r2x=clip2x-b2.posx;
                        c2r2y=clip2y-b2.posy;
                    };
                    var rn1a=(gnormy*c1r1x-gnormx*c1r1y);
                    var rn1b=(gnormy*c1r2x-gnormx*c1r2y);
                    var rn2a=(gnormy*c2r1x-gnormx*c2r1y);
                    var rn2b=(gnormy*c2r2x-gnormx*c2r2y);
                    var mass_sum=b1.smass+b2.smass;
                    {
                        kMassa=mass_sum+(b1.sinertia*rn1a*rn1a)+b2.sinertia*rn1b*rn1b;
                        kMassb=mass_sum+(b1.sinertia*rn1a*rn2a)+b2.sinertia*rn1b*rn2b;
                        kMassc=mass_sum+(b1.sinertia*rn2a*rn2a)+b2.sinertia*rn2b*rn2b;
                    };
                    var Ka:Float=0.0;
                    var Kb:Float=0.0;
                    var Kc:Float=0.0;
                    {
                        Ka=kMassa;
                        Kb=kMassb;
                        Kc=kMassc;
                    };
                    var bx:Float=err1*biasCoef;
                    var by:Float=err2*biasCoef;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((bx!=bx));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(bx)"+") :: "+("vec_new(in n: "+"b"+",in x: "+"err1*biasCoef"+",in y: "+"err2*biasCoef"+")");
                        #end
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((by!=by));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(by)"+") :: "+("vec_new(in n: "+"b"+",in x: "+"err1*biasCoef"+",in y: "+"err2*biasCoef"+")");
                        #end
                    };
                    do{
                        var xx:Float=0.0;
                        var xy:Float=0.0;
                        {
                            xx=bx;
                            xy=by;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((xx!=xx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(xx)"+") :: "+("vec_set(in n: "+"x"+",in x: "+"bx"+",in y: "+"by"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((xy!=xy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(xy)"+") :: "+("vec_set(in n: "+"x"+",in x: "+"bx"+",in y: "+"by"+")");
                                #end
                            };
                        };
                        {
                            xx=-xx;
                            xy=-xy;
                        };
                        {
                            var det=(kMassa*kMassc-kMassb*kMassb);
                            if((det!=det))xx=xy=0;
                            else if(det==0){
                                if(kMassa!=0)xx/=kMassa;
                                else xx=0;
                                if(kMassc!=0)xy/=kMassc;
                                else xy=0;
                            }
                            else{
                                det=1/det;
                                var t=det*(kMassc*xx-kMassb*xy);
                                xy=det*(kMassa*xy-kMassb*xx);
                                xx=t;
                            }
                        };
                        if(xx>=0&&xy>=0){
                            {
                                var t=((xx+xy)*b1.imass);
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((t!=t));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.pos"+",in b: "+"gnorm"+",in s: "+"(xx+xy)*b1.imass"+")");
                                    #end
                                };
                                b1.posx-=gnormx*t;
                                b1.posy-=gnormy*t;
                            };
                            b1.delta_rot(-b1.iinertia*(rn1a*xx+rn2a*xy));
                            {
                                var t=((xx+xy)*b2.imass);
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((t!=t));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.pos"+",in b: "+"gnorm"+",in s: "+"(xx+xy)*b2.imass"+")");
                                    #end
                                };
                                b2.posx+=gnormx*t;
                                b2.posy+=gnormy*t;
                            };
                            b2.delta_rot(b2.iinertia*(rn1b*xx+rn2b*xy));
                            break;
                        };
                        {
                            xx=-bx/Ka;
                            xy=0;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((xx!=xx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(xx)"+") :: "+("vec_set(in n: "+"x"+",in x: "+"-bx/Ka"+",in y: "+"0"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((xy!=xy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(xy)"+") :: "+("vec_set(in n: "+"x"+",in x: "+"-bx/Ka"+",in y: "+"0"+")");
                                #end
                            };
                        };
                        var vn2=Kb*xx+by;
                        if(xx>=0&&vn2>=0){
                            {
                                var t=((xx+xy)*b1.imass);
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((t!=t));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.pos"+",in b: "+"gnorm"+",in s: "+"(xx+xy)*b1.imass"+")");
                                    #end
                                };
                                b1.posx-=gnormx*t;
                                b1.posy-=gnormy*t;
                            };
                            b1.delta_rot(-b1.iinertia*(rn1a*xx+rn2a*xy));
                            {
                                var t=((xx+xy)*b2.imass);
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((t!=t));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.pos"+",in b: "+"gnorm"+",in s: "+"(xx+xy)*b2.imass"+")");
                                    #end
                                };
                                b2.posx+=gnormx*t;
                                b2.posy+=gnormy*t;
                            };
                            b2.delta_rot(b2.iinertia*(rn1b*xx+rn2b*xy));
                            break;
                        };
                        {
                            xx=0;
                            xy=-by/Kc;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((xx!=xx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(xx)"+") :: "+("vec_set(in n: "+"x"+",in x: "+"0"+",in y: "+"-by/Kc"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((xy!=xy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(xy)"+") :: "+("vec_set(in n: "+"x"+",in x: "+"0"+",in y: "+"-by/Kc"+")");
                                #end
                            };
                        };
                        var vn1=Kb*xy+bx;
                        if(xy>=0&&vn1>=0){
                            {
                                var t=((xx+xy)*b1.imass);
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((t!=t));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.pos"+",in b: "+"gnorm"+",in s: "+"(xx+xy)*b1.imass"+")");
                                    #end
                                };
                                b1.posx-=gnormx*t;
                                b1.posy-=gnormy*t;
                            };
                            b1.delta_rot(-b1.iinertia*(rn1a*xx+rn2a*xy));
                            {
                                var t=((xx+xy)*b2.imass);
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((t!=t));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.pos"+",in b: "+"gnorm"+",in s: "+"(xx+xy)*b2.imass"+")");
                                    #end
                                };
                                b2.posx+=gnormx*t;
                                b2.posy+=gnormy*t;
                            };
                            b2.delta_rot(b2.iinertia*(rn1b*xx+rn2b*xy));
                            break;
                        };
                    }
                    while(false);
                }
                else{
                    var rn1=(gnormy*c1r1x-gnormx*c1r1y);
                    var rn2=(gnormy*c1r2x-gnormx*c1r2y);
                    var K=b2.smass+rn2*rn2*b2.sinertia+b1.smass+rn1*rn1*b1.sinertia;
                    if(K!=0){
                        var jn=-biasCoef*err1/K;
                        var Jx:Float=0.0;
                        var Jy:Float=0.0;
                        {
                            var t=(jn);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_mul(in a: "+"gnorm"+",in s: "+"jn"+",out r: "+"J"+")");
                                #end
                            };
                            Jx=gnormx*t;
                            Jy=gnormy*t;
                        };
                        {
                            var t=(b1.imass);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.pos"+",in b: "+"J"+",in s: "+"b1.imass"+")");
                                #end
                            };
                            b1.posx-=Jx*t;
                            b1.posy-=Jy*t;
                        };
                        b1.delta_rot(-rn1*b1.iinertia*jn);
                        {
                            var t=(b2.imass);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.pos"+",in b: "+"J"+",in s: "+"b2.imass"+")");
                                #end
                            };
                            b2.posx+=Jx*t;
                            b2.posy+=Jy*t;
                        };
                        b2.delta_rot(rn2*b2.iinertia*jn);
                    }
                }
            }
        }
    }
}
