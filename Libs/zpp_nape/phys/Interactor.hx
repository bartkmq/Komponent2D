package zpp_nape.phys;
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
import nape.util.ShapeDebug;
#if nape_swc@:keep #end
class ZPP_Interactor{
    public var outer_i:Interactor=null;
    public var id:Int=0;
    public var userData:Dynamic<Dynamic>=null;
    public var ishape:ZPP_Shape=null;
    public var ibody:ZPP_Body=null;
    public var icompound:ZPP_Compound=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function isShape(){
        return ishape!=null;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function isBody(){
        return ibody!=null;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function isCompound(){
        return icompound!=null;
    }
    public function __iaddedToSpace(){
        if(group!=null)group.addInteractor(this);
        {
            var cx_ite=cbTypes.begin();
            while(cx_ite!=null){
                var cb=cx_ite.elem();
                cb.addInteractor(this);
                cx_ite=cx_ite.next;
            }
        };
        alloc_cbSet();
    }
    public function __iremovedFromSpace(){
        if(group!=null)group.remInteractor(this);
        {
            var cx_ite=cbTypes.begin();
            while(cx_ite!=null){
                var cb=cx_ite.elem();
                cb.remInteractor(this);
                cx_ite=cx_ite.next;
            }
        };
        dealloc_cbSet();
    }
    public function wake(){
        if(isShape()){
            var body=ishape.body;
            if(body!=null&&body.space!=null)body.space.non_inlined_wake(body);
            true;
        }
        else if(isBody()){
            if(ibody.space!=null)ibody.space.non_inlined_wake(ibody)else false;
        }
        else{
            if(icompound.space!=null)icompound.space.wakeCompound(icompound);
            true;
        }
    }
    public var cbsets:ZNPList_ZPP_CallbackSet=null;
    public static function get(i1:ZPP_Interactor,i2:ZPP_Interactor){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                i1!=i2;
            };
            if(!res)throw "assert("+"i1!=i2"+") :: "+("trying to get cbset between interactor and itself?");
            #end
        };
        var id=if(i1.id<i2.id)i1.id else i2.id;
        var di=if(i1.id<i2.id)i2.id else i1.id;
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                id<di;
            };
            if(!res)throw "assert("+"id<di"+") :: "+("interactor id's not ordered well when getting cbset");
            #end
        };
        var xs=if(i1.cbsets.length<i2.cbsets.length)i1.cbsets else i2.cbsets;
        var ret:ZPP_CallbackSet=null;
        {
            var cx_ite=xs.begin();
            while(cx_ite!=null){
                var x=cx_ite.elem();
                {
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !(x.id==di&&x.di==id);
                        };
                        if(!res)throw "assert("+"!(x.id==di&&x.di==id)"+") :: "+("cbset order doesn't match interactor order getting cbset?");
                        #end
                    };
                    if(x.id==id&&x.di==di){
                        ret=x;
                        break;
                    }
                };
                cx_ite=cx_ite.next;
            }
        };
        return ret;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function getSpace(){
        return if(isShape())ishape.body==null?null:ishape.body.space else if(isBody())ibody.space else icompound.space;
    }
    public var group:ZPP_InteractionGroup=null;
    public var cbTypes:ZNPList_ZPP_CbType=null;
    public var cbSet:ZPP_CbSet=null;
    public var wrap_cbTypes:CbTypeList=null;
    public function setupcbTypes(){
        wrap_cbTypes=ZPP_CbTypeList.get(cbTypes);
        wrap_cbTypes.zpp_inner.adder=wrap_cbTypes_adder;
        wrap_cbTypes.zpp_inner.subber=wrap_cbTypes_subber;
        wrap_cbTypes.zpp_inner.dontremove=true;
        #if(!NAPE_RELEASE_BUILD)
        wrap_cbTypes.zpp_inner._modifiable=immutable_cbTypes;
        #end
    }
    #if(!NAPE_RELEASE_BUILD)
    function immutable_cbTypes(){
        immutable_midstep("Interactor::cbTypes");
    }
    #end
    function wrap_cbTypes_subber(pcb:CbType):Void{
        var cb=pcb.zpp_inner;
        if(cbTypes.has(cb)){
            var space=getSpace();
            if(space!=null){
                dealloc_cbSet();
                cb.remInteractor(this);
            }
            cbTypes.remove(cb);
            if(space!=null){
                alloc_cbSet();
                wake();
            }
        }
    }
    function wrap_cbTypes_adder(cb:CbType):Bool{
        insert_cbtype(cb.zpp_inner);
        return false;
    }
    public function insert_cbtype(cb:ZPP_CbType){
        if(!cbTypes.has(cb)){
            var space=getSpace();
            if(space!=null){
                dealloc_cbSet();
                cb.addInteractor(this);
            }
            {
                var pre=null;
                {
                    var cx_ite=cbTypes.begin();
                    while(cx_ite!=null){
                        var j=cx_ite.elem();
                        {
                            if(ZPP_CbType.setlt(cb,j))break;
                            pre=cx_ite;
                        };
                        cx_ite=cx_ite.next;
                    }
                };
                cbTypes.inlined_insert(pre,cb);
            };
            if(space!=null){
                alloc_cbSet();
                wake();
            }
        }
    }
    public function alloc_cbSet(){
        var space=getSpace();
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                space!=null;
            };
            if(!res)throw "assert("+"space!=null"+") :: "+("space null in alloc_cbSet");
            #end
        };
        if((cbSet=space.cbsets.get(cbTypes))!=null){
            cbSet.increment();
            cbSet.addInteractor(this);
            cbSet.validate();
            space.freshInteractorType(this);
        }
    }
    public function dealloc_cbSet(){
        var space=getSpace();
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                space!=null;
            };
            if(!res)throw "assert("+"space!=null"+") :: "+("space null in dealloc_cbSet");
            #end
        };
        if(cbSet!=null){
            cbSet.remInteractor(this);
            space.nullInteractorType(this);
            if(cbSet.decrement()){
                space.cbsets.remove(cbSet);
                {
                    var o=cbSet;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            o!=null;
                        };
                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_CbSet"+", in obj: "+"cbSet"+")");
                        #end
                    };
                    o.free();
                    o.next=ZPP_CbSet.zpp_pool;
                    ZPP_CbSet.zpp_pool=o;
                    #if NAPE_POOL_STATS ZPP_CbSet.POOL_CNT++;
                    ZPP_CbSet.POOL_SUB++;
                    #end
                };
            }
            cbSet=null;
        }
    }
    public function setGroup(group:ZPP_InteractionGroup){
        if(this.group!=group){
            var inspace=getSpace()!=null;
            if(inspace&&this.group!=null)this.group.remInteractor(this);
            this.group=group;
            if(inspace&&group!=null)group.addInteractor(this);
            if(inspace){
                if(isShape())ishape.body.wake();
                else if(isBody())ibody.wake();
                else icompound.wake();
            }
        }
    }
    public function immutable_midstep(n:String){
        if(isBody())ibody.__immutable_midstep(n);
        else if(isShape())ishape.__immutable_midstep(n);
        else icompound.__imutable_midstep(n);
    }
    public function new(){
        id=ZPP_ID.Interactor();
        cbsets=new ZNPList_ZPP_CallbackSet();
        cbTypes=new ZNPList_ZPP_CbType();
    }
    public#if NAPE_NO_INLINE#else inline #end
    static function int_callback(set:ZPP_CallbackSet,x:ZPP_InteractionListener,cb:ZPP_Callback){
        var o1=set.int1;
        var o2=set.int2;
        if(x.options1.compatible(o1.cbTypes)&&x.options2.compatible(o2.cbTypes)){
            cb.int1=o1;
            cb.int2=o2;
        }
        else{
            cb.int1=o2;
            cb.int2=o1;
        }
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function lookup_group(){
        var cur:ZPP_Interactor=this;
        while(cur!=null&&cur.group==null){
            if(cur.isShape())cur=cur.ishape.body;
            else if(cur.isCompound())cur=cur.icompound.compound;
            else cur=cur.ibody.compound;
        }
        return if(cur==null)null else cur.group;
    }
    public function copyto(ret:Interactor){
        ret.zpp_inner_i.group=group;
        for(cb in outer_i.cbTypes)ret.cbTypes.add(cb);
        if(userData!=null)ret.zpp_inner_i.userData=Reflect.copy(userData);
    }
}
