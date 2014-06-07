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
import zpp_nape.util.Debug;
import zpp_nape.util.RBTree;
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
#if nape_swc@:keep #end
class Hashable2_Boolfalse{
    public var value:Bool=false;
    public var next:Hashable2_Boolfalse=null;
    static public var zpp_pool:Hashable2_Boolfalse=null;
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
    
    public var hnext:Hashable2_Boolfalse=null;
    public var id:Int=0;
    public var di:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc(){}
    function new(){}
    public static#if NAPE_NO_INLINE#else inline #end
    function get(id:Int,di:Int,val:Bool){
        var ret=getpersist(id,di);
        ret.value=val;
        return ret;
    }
    public static#if NAPE_NO_INLINE#else inline #end
    function getpersist(id:Int,di:Int){
        var ret;
        {
            if(Hashable2_Boolfalse.zpp_pool==null){
                ret=new Hashable2_Boolfalse();
                #if NAPE_POOL_STATS Hashable2_Boolfalse.POOL_TOT++;
                Hashable2_Boolfalse.POOL_ADDNEW++;
                #end
            }
            else{
                ret=Hashable2_Boolfalse.zpp_pool;
                Hashable2_Boolfalse.zpp_pool=ret.next;
                ret.next=null;
                #if NAPE_POOL_STATS Hashable2_Boolfalse.POOL_CNT--;
                Hashable2_Boolfalse.POOL_ADD++;
                #end
            }
            ret.alloc();
        };
        ret.id=id;
        ret.di=di;
        return ret;
    }
    public static#if NAPE_NO_INLINE#else inline #end
    function ordered_get(id:Int,di:Int,val:Bool){
        return if(id<=di)get(id,di,val)else get(di,id,val);
    }
    public static#if NAPE_NO_INLINE#else inline #end
    function ordered_get_persist(id:Int,di:Int){
        return if(id<=di)getpersist(id,di);
        else getpersist(di,id);
    }
}

#if nape_swc@:keep #end
#if nape_swc@:keep #end
class FastHash2_Hashable2_Boolfalse{
    public var table:TArray<Hashable2_Boolfalse>=null;
    public var cnt:Int=0;
    public function new(){
        cnt=0;
        #if flash10 table=new flash.Vector<Hashable2_Boolfalse>(0x100000,true);
        #else table=new Array<Hashable2_Boolfalse>();
        for(i in 0...(0x100000))table.push(null);
        #end
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function empty(){
        return cnt==0;
    }
    public function clear(){
        {
            for(i in 0...this.table.length){
                var n=this.table[i];
                if(n==null)continue;
                while(n!=null){
                    var t=n.hnext;
                    n.hnext=null;
                    (n);
                    n=t;
                }
                this.table[i]=null;
            }
        };
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function get(id:Int,di:Int):Hashable2_Boolfalse{
        var n=table[hash(id,di)];
        if(n==null)return null;
        else if(n.id==id&&n.di==di)return n;
        else{
            do n=n.hnext while(n!=null&&(n.id!=id||n.di!=di));
            return n;
        }
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function ordered_get(id:Int,di:Int){
        if(id>di){
            var t=id;
            id=di;
            di=t;
        }
        return get(id,di);
    }
    public function has(id:Int,di:Int):Bool{
        var n=table[hash(id,di)];
        if(n==null)return false;
        else if(n.id==id&&n.di==di)return true;
        else{
            do n=n.hnext while(n!=null&&(n.id!=id||n.di!=di));
            return n!=null;
        }
    }
    public function maybeAdd(arb:Hashable2_Boolfalse){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                arb!=null;
            };
            if(!res)throw "assert("+"arb!=null"+") :: "+("cannot add null to hash2");
            #end
        };
        var h=hash(arb.id,arb.di);
        var n=table[h];
        var cont=true;
        if(n==null){
            table[h]=arb;
            arb.hnext=null;
        }
        else{
            #if NAPE_ASSERT var nor=n;
            while(n!=null){
                if(n.id==arb.id&&n.di==arb.di){
                    cont=false;
                    break;
                }
                n=n.hnext;
            }
            n=nor;
            #end
            if(cont){
                arb.hnext=n.hnext;
                n.hnext=arb;
            }
        }
        if(cont)cnt++;
    }
    public function add(arb:Hashable2_Boolfalse){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                arb!=null;
            };
            if(!res)throw "assert("+"arb!=null"+") :: "+("cannot add null to hash2");
            #end
        };
        var h=hash(arb.id,arb.di);
        var n=table[h];
        if(n==null){
            table[h]=arb;
            arb.hnext=null;
        }
        else{
            #if NAPE_ASSERT var nor=n;
            while(n!=null){
                if(n.id==arb.id&&n.di==arb.di)throw "ASSERTION: FastHash2("+"Hashable2_Boolfalse"+") already cotnains object with ids";
                n=n.hnext;
            }
            n=nor;
            #end
            arb.hnext=n.hnext;
            n.hnext=arb;
            #if NAPE_TIMES Debug.HASH++;
            #end
        }
        #if NAPE_TIMES Debug.HASHT++;
        #end
        cnt++;
    }
    public function remove(arb:Hashable2_Boolfalse){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                arb!=null;
            };
            if(!res)throw "assert("+"arb!=null"+") :: "+("cannot remove null from hash2");
            #end
        };
        var h=hash(arb.id,arb.di);
        var n=table[h];
        if(n==arb)table[h]=n.hnext;
        else if(n!=null){
            var pre:Hashable2_Boolfalse;
            do{
                pre=n;
                n=n.hnext;
            }
            while(n!=null&&n!=arb);
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    n!=null;
                };
                if(!res)throw "assert("+"n!=null"+") :: "+("object doesn't exist in hash2");
                #end
            };
            pre.hnext=n.hnext;
        }
        arb.hnext=null;
        cnt--;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function hash(id:Int,di:Int){
        return((id*106039)+di)&0xfffff;
    }
}
