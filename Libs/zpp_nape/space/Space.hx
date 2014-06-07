package zpp_nape.space;
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
class ZPP_Island{
    public var next:ZPP_Island=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_Island{
        return this;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZPP_Island{
        return next;
    }
    public var _inuse:Bool=false;
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZPP_Island):Void{
        next=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_Island):ZPP_Island{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_Island):ZPP_Island{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Island"+"] add -> o="+o);
            #end
        };
        var temp={
            o._inuse=true;
            o;
        };
        temp.next=begin();
        next=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZPP_Island):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_Island"+"] addAll -> "+x);
            #end
        };
        {
            var cx_ite=x.begin();
            while(cx_ite!=null){
                var i=cx_ite.elem();
                add(i);
                cx_ite=cx_ite.next;
            }
        };
    }
    public function insert(cur:ZPP_Island,o:ZPP_Island):ZPP_Island{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZPP_Island,o:ZPP_Island):ZPP_Island{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Island"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            o._inuse=true;
            o;
        };
        if(cur==null){
            temp.next=begin();
            next=temp;
        }
        else{
            temp.next=cur.next;
            cur.next=temp;
        }
        pushmod=modified=true;
        length++;
        return temp;
    }
    public function pop():Void{
        inlined_pop();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop():Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Island"+"] pop");
            #end
        };
        var ret=begin();
        next=ret.next;
        {
            ret.elem()._inuse=false;
        };
        {};
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_Island{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_Island{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Island"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_Island):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                has(obj);
            };
            if(!res)throw "assert("+"has(obj)"+") :: "+("removed but didn't exist");
            #end
        };
        inlined_try_remove(obj);
    }
    public function try_remove(obj:ZPP_Island):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Island"+"] remove -> "+obj);
            #end
        };
        var pre=null;
        var cur=begin();
        var ret=false;
        while(cur!=null){
            if(cur.elem()==obj){
                erase(pre);
                ret=true;
                break;
            }
            pre=cur;
            cur=cur.next;
        }
        return ret;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_remove(obj:ZPP_Island):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                has(obj);
            };
            if(!res)throw "assert("+"has(obj)"+") :: "+("removed but didn't exist");
            #end
        };
        inlined_try_remove(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_try_remove(obj:ZPP_Island):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Island"+"] remove -> "+obj);
            #end
        };
        var pre=null;
        var cur=begin();
        var ret=false;
        while(cur!=null){
            if(cur.elem()==obj){
                inlined_erase(pre);
                ret=true;
                break;
            }
            pre=cur;
            cur=cur.next;
        }
        return ret;
    }
    public function erase(pre:ZPP_Island):ZPP_Island{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZPP_Island):ZPP_Island{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Island"+"] erase -> "+pre);
            #end
        };
        var old:ZPP_Island;
        var ret:ZPP_Island;
        if(pre==null){
            old=begin();
            ret=old.next;
            next=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {
            old.elem()._inuse=false;
        };
        {};
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZPP_Island,n:Int):ZPP_Island{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(false){
            while(!empty())inlined_pop();
            pushmod=true;
        }
    }
    public function reverse():Void{
        var cur=begin();
        var pre=null;
        while(cur!=null){
            var nx=cur.next;
            cur.next=pre;
            next=cur;
            pre=cur;
            cur=nx;
        }
        modified=true;
        pushmod=true;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function empty():Bool{
        return begin()==null;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function size():Int{
        return length;
    }
    public function has(obj:ZPP_Island):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_Island):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Island"+"] has -> "+obj);
            #end
        };
        var ret;
        {
            ret=false;
            {
                var cx_ite=this.begin();
                while(cx_ite!=null){
                    var npite=cx_ite.elem();
                    {
                        if(npite==obj){
                            ret=true;
                            break;
                        }
                    };
                    cx_ite=cx_ite.next;
                }
            };
        };
        return ret;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function front():ZPP_Island{
        return begin().elem();
    }
    public function back():ZPP_Island{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZPP_Island{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Island"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_Island{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Island"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
    public var comps:ZNPList_ZPP_Component=null;
    public var sleep:Bool=false;
    public var waket:Int=0;
    static public var zpp_pool:ZPP_Island=null;
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
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                comps.empty();
            };
            if(!res)throw "assert("+"comps.empty()"+") :: "+("Island freed with components in island");
            #end
        };
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc(){
        waket=0;
    }
    public function new(){
        comps=new ZNPList_ZPP_Component();
    }
}
#if nape_swc@:keep #end
class ZPP_Component{
    public var next:ZPP_Component=null;
    static public var zpp_pool:ZPP_Component=null;
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
    
    public var parent:ZPP_Component=null;
    public var rank:Int=0;
    public var isBody:Bool=false;
    public var body:ZPP_Body=null;
    public var constraint:ZPP_Constraint=null;
    public var island:ZPP_Island=null;
    public var sleeping:Bool=false;
    public var waket:Int=0;
    public var woken:Bool=false;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free(){
        body=null;
        constraint=null;
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                island==null;
            };
            if(!res)throw "assert("+"island==null"+") :: "+("component freeed whilst connected to an island?");
            #end
        };
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc(){}
    public function new(){
        reset();
        woken=false;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function reset(){
        sleeping=false;
        island=null;
        parent=this;
        rank=0;
    }
}
#if nape_swc@:keep #end
class ZPP_CallbackSet{
    public var id:Int=0;
    public var di:Int=0;
    public var int1:ZPP_Interactor=null;
    public var int2:ZPP_Interactor=null;
    public static function get(i1:ZPP_Interactor,i2:ZPP_Interactor){
        var ret;
        {
            if(ZPP_CallbackSet.zpp_pool==null){
                ret=new ZPP_CallbackSet();
                #if NAPE_POOL_STATS ZPP_CallbackSet.POOL_TOT++;
                ZPP_CallbackSet.POOL_ADDNEW++;
                #end
            }
            else{
                ret=ZPP_CallbackSet.zpp_pool;
                ZPP_CallbackSet.zpp_pool=ret.next;
                ret.next=null;
                #if NAPE_POOL_STATS ZPP_CallbackSet.POOL_CNT--;
                ZPP_CallbackSet.POOL_ADD++;
                #end
            }
            ret.alloc();
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                i1!=i2;
            };
            if(!res)throw "assert("+"i1!=i2"+") :: "+("CallbackSet between the same interactor?");
            #end
        };
        if(i1.id<i2.id){
            ret.int1=i1;
            ret.int2=i2;
        }
        else{
            ret.int1=i2;
            ret.int2=i1;
        }
        ret.id=ret.int1.id;
        ret.di=ret.int2.id;
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ret.id<ret.di;
            };
            if(!res)throw "assert("+"ret.id<ret.di"+") :: "+("badly ordered id's in callback set?");
            #end
        };
        return ret;
    }
    public var arbiters:ZNPList_ZPP_Arbiter;
    public var COLLISIONstate:Int;
    public var COLLISIONstamp:Int;
    public var SENSORstate:Int;
    public var SENSORstamp:Int;
    public var FLUIDstate:Int;
    public var FLUIDstamp:Int;
    public var next:ZPP_CallbackSet=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_CallbackSet{
        return this;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZPP_CallbackSet{
        return next;
    }
    public var _inuse:Bool=false;
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZPP_CallbackSet):Void{
        next=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_CallbackSet):ZPP_CallbackSet{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_CallbackSet):ZPP_CallbackSet{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_CallbackSet"+"] add -> o="+o);
            #end
        };
        var temp={
            o._inuse=true;
            o;
        };
        temp.next=begin();
        next=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZPP_CallbackSet):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_CallbackSet"+"] addAll -> "+x);
            #end
        };
        {
            var cx_ite=x.begin();
            while(cx_ite!=null){
                var i=cx_ite.elem();
                add(i);
                cx_ite=cx_ite.next;
            }
        };
    }
    public function insert(cur:ZPP_CallbackSet,o:ZPP_CallbackSet):ZPP_CallbackSet{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZPP_CallbackSet,o:ZPP_CallbackSet):ZPP_CallbackSet{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_CallbackSet"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            o._inuse=true;
            o;
        };
        if(cur==null){
            temp.next=begin();
            next=temp;
        }
        else{
            temp.next=cur.next;
            cur.next=temp;
        }
        pushmod=modified=true;
        length++;
        return temp;
    }
    public function pop():Void{
        inlined_pop();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop():Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_CallbackSet"+"] pop");
            #end
        };
        var ret=begin();
        next=ret.next;
        {
            ret.elem()._inuse=false;
        };
        {};
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_CallbackSet{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_CallbackSet{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_CallbackSet"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_CallbackSet):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                has(obj);
            };
            if(!res)throw "assert("+"has(obj)"+") :: "+("removed but didn't exist");
            #end
        };
        inlined_try_remove(obj);
    }
    public function try_remove(obj:ZPP_CallbackSet):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_CallbackSet"+"] remove -> "+obj);
            #end
        };
        var pre=null;
        var cur=begin();
        var ret=false;
        while(cur!=null){
            if(cur.elem()==obj){
                erase(pre);
                ret=true;
                break;
            }
            pre=cur;
            cur=cur.next;
        }
        return ret;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_remove(obj:ZPP_CallbackSet):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                has(obj);
            };
            if(!res)throw "assert("+"has(obj)"+") :: "+("removed but didn't exist");
            #end
        };
        inlined_try_remove(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_try_remove(obj:ZPP_CallbackSet):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_CallbackSet"+"] remove -> "+obj);
            #end
        };
        var pre=null;
        var cur=begin();
        var ret=false;
        while(cur!=null){
            if(cur.elem()==obj){
                inlined_erase(pre);
                ret=true;
                break;
            }
            pre=cur;
            cur=cur.next;
        }
        return ret;
    }
    public function erase(pre:ZPP_CallbackSet):ZPP_CallbackSet{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZPP_CallbackSet):ZPP_CallbackSet{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_CallbackSet"+"] erase -> "+pre);
            #end
        };
        var old:ZPP_CallbackSet;
        var ret:ZPP_CallbackSet;
        if(pre==null){
            old=begin();
            ret=old.next;
            next=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {
            old.elem()._inuse=false;
        };
        {};
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZPP_CallbackSet,n:Int):ZPP_CallbackSet{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(false){
            while(!empty())inlined_pop();
            pushmod=true;
        }
    }
    public function reverse():Void{
        var cur=begin();
        var pre=null;
        while(cur!=null){
            var nx=cur.next;
            cur.next=pre;
            next=cur;
            pre=cur;
            cur=nx;
        }
        modified=true;
        pushmod=true;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function empty():Bool{
        return begin()==null;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function size():Int{
        return length;
    }
    public function has(obj:ZPP_CallbackSet):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_CallbackSet):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_CallbackSet"+"] has -> "+obj);
            #end
        };
        var ret;
        {
            ret=false;
            {
                var cx_ite=this.begin();
                while(cx_ite!=null){
                    var npite=cx_ite.elem();
                    {
                        if(npite==obj){
                            ret=true;
                            break;
                        }
                    };
                    cx_ite=cx_ite.next;
                }
            };
        };
        return ret;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function front():ZPP_CallbackSet{
        return begin().elem();
    }
    public function back():ZPP_CallbackSet{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZPP_CallbackSet{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_CallbackSet"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_CallbackSet{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_CallbackSet"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
    static public var zpp_pool:ZPP_CallbackSet=null;
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
    
    public var freed:Bool=false;
    public var lazydel:Bool=false;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free(){
        int1=int2=null;
        id=di=-1;
        freed=true;
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                arbiters.empty();
            };
            if(!res)throw "assert("+"arbiters.empty()"+") :: "+("callbackset released with lingering arbs");
            #end
        };
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc(){
        freed=false;
        lazydel=false;
        
        COLLISIONstate=ZPP_Flags.id_PreFlag_ACCEPT;
        COLLISIONstamp=0;
        SENSORstate=ZPP_Flags.id_PreFlag_ACCEPT;
        SENSORstamp=0;
        FLUIDstate=ZPP_Flags.id_PreFlag_ACCEPT;
        FLUIDstamp=0;
    }
    public function new(){
        arbiters=new ZNPList_ZPP_Arbiter();
    }
    public#if NAPE_NO_INLINE#else inline #end
    function add_arb(x:ZPP_Arbiter){
        if(!arbiters.inlined_has(x)){
            arbiters.inlined_add(x);
            return true;
        }
        else return false;
    }
    public function try_remove_arb(x:ZPP_Arbiter){
        return arbiters.inlined_try_remove(x);
    }
    public function remove_arb(x:ZPP_Arbiter){
        arbiters.inlined_remove(x);
    }
    public function empty_arb(type:Int){
        var retvar;
        {
            retvar=true;
            {
                var cx_ite=arbiters.begin();
                while(cx_ite!=null){
                    var x=cx_ite.elem();
                    {
                        if((x.type&type)==0){
                            {
                                cx_ite=cx_ite.next;
                                continue;
                            };
                        }
                        else{
                            retvar=false;
                            break;
                        }
                    };
                    cx_ite=cx_ite.next;
                }
            };
        };
        return retvar;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function really_empty(){
        return arbiters.empty();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function sleeping(){
        var ret;
        {
            ret=true;
            {
                var cx_ite=arbiters.begin();
                while(cx_ite!=null){
                    var x=cx_ite.elem();
                    {
                        if(x.sleeping){
                            {
                                cx_ite=cx_ite.next;
                                continue;
                            };
                        }
                        else{
                            ret=false;
                            break;
                        }
                    };
                    cx_ite=cx_ite.next;
                }
            };
        };
        return ret;
    }
}
#if nape_swc@:keep #end
class ZPP_CbSetManager{
    public var cbsets:ZPP_Set_ZPP_CbSet=null;
    public var space:ZPP_Space=null;
    public function get(cbTypes:ZNPList_ZPP_CbType){
        if(cbTypes.empty())return null;
        #if NAPE_ASSERT ZPP_CbSet.assert_cbTypes(cbTypes);
        #end
        var fake;
        {
            if(ZPP_CbSet.zpp_pool==null){
                fake=new ZPP_CbSet();
                #if NAPE_POOL_STATS ZPP_CbSet.POOL_TOT++;
                ZPP_CbSet.POOL_ADDNEW++;
                #end
            }
            else{
                fake=ZPP_CbSet.zpp_pool;
                ZPP_CbSet.zpp_pool=fake.next;
                fake.next=null;
                #if NAPE_POOL_STATS ZPP_CbSet.POOL_CNT--;
                ZPP_CbSet.POOL_ADD++;
                #end
            }
            fake.alloc();
        };
        var faketypes=fake.cbTypes;
        fake.cbTypes=cbTypes;
        var res=cbsets.find_weak(fake);
        var ret=if(res!=null)res.data else{
            var set=ZPP_CbSet.get(cbTypes);
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !cbsets.has_weak(set);
                };
                if(!res)throw "assert("+"!cbsets.has_weak(set)"+") :: "+("CbSet exists in Set already, but not found??");
                #end
            };
            cbsets.insert(set);
            set.manager=this;
            set;
        }
        fake.cbTypes=faketypes;
        {
            var o=fake;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_CbSet"+", in obj: "+"fake"+")");
                #end
            };
            o.free();
            o.next=ZPP_CbSet.zpp_pool;
            ZPP_CbSet.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_CbSet.POOL_CNT++;
            ZPP_CbSet.POOL_SUB++;
            #end
        };
        return ret;
    }
    public function remove(set:ZPP_CbSet){
        cbsets.remove(set);
        {
            while(!set.cbpairs.empty()){
                var pair=set.cbpairs.pop_unsafe();
                {
                    if(pair.a!=pair.b){
                        if(set==pair.a)pair.b.cbpairs.remove(pair);
                        else pair.a.cbpairs.remove(pair);
                    }
                    {
                        var o=pair;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                o!=null;
                            };
                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_CbSetPair"+", in obj: "+"pair"+")");
                            #end
                        };
                        o.free();
                        o.next=ZPP_CbSetPair.zpp_pool;
                        ZPP_CbSetPair.zpp_pool=o;
                        #if NAPE_POOL_STATS ZPP_CbSetPair.POOL_CNT++;
                        ZPP_CbSetPair.POOL_SUB++;
                        #end
                    };
                };
            }
        };
        set.manager=null;
    }
    public function clear(){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                cbsets.empty();
            };
            if(!res)throw "assert("+"cbsets.empty()"+") :: "+("space cleared, called setmanager clear(), and tree was non-empty. wuh");
            #end
        };
    }
    public function new(space:ZPP_Space){
        {
            if(ZPP_Set_ZPP_CbSet.zpp_pool==null){
                cbsets=new ZPP_Set_ZPP_CbSet();
                #if NAPE_POOL_STATS ZPP_Set_ZPP_CbSet.POOL_TOT++;
                ZPP_Set_ZPP_CbSet.POOL_ADDNEW++;
                #end
            }
            else{
                cbsets=ZPP_Set_ZPP_CbSet.zpp_pool;
                ZPP_Set_ZPP_CbSet.zpp_pool=cbsets.next;
                cbsets.next=null;
                #if NAPE_POOL_STATS ZPP_Set_ZPP_CbSet.POOL_CNT--;
                ZPP_Set_ZPP_CbSet.POOL_ADD++;
                #end
            }
            cbsets.alloc();
        };
        cbsets.lt=ZPP_CbSet.setlt;
        this.space=space;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function validate(){
        {
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    cbsets!=null;
                };
                if(!res)throw "assert("+"cbsets!=null"+") :: "+("Iterate  null set?");
                #end
            };
            if(!cbsets.empty()){
                var set_ite=cbsets.parent;
                while(set_ite.prev!=null)set_ite=set_ite.prev;
                while(set_ite!=null){
                    var cb=set_ite.data;
                    cb.validate();
                    if(set_ite.next!=null){
                        set_ite=set_ite.next;
                        while(set_ite.prev!=null)set_ite=set_ite.prev;
                    }
                    else{
                        while(set_ite.parent!=null&&set_ite==set_ite.parent.next)set_ite=set_ite.parent;
                        set_ite=set_ite.parent;
                    }
                }
            }
        };
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function pair(a:ZPP_CbSet,b:ZPP_CbSet){
        var ret:ZPP_CbSetPair=null;
        var pairs=if(a.cbpairs.length<b.cbpairs.length)a.cbpairs else b.cbpairs;
        {
            var cx_ite=pairs.begin();
            while(cx_ite!=null){
                var p=cx_ite.elem();
                {
                    if((p.a==a&&p.b==b)||(p.a==b&&p.b==a)){
                        ret=p;
                        break;
                    }
                };
                cx_ite=cx_ite.next;
            }
        };
        if(ret==null){
            ret=ZPP_CbSetPair.get(a,b);
            a.cbpairs.add(ret);
            if(b!=a)b.cbpairs.add(ret);
        }
        ret.validate();
        return ret;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function valid_listener(i:ZPP_Listener){
        return i.space==this.space;
    }
}
#if nape_swc@:keep #end
class ZPP_Space{
    public var outer:Space=null;
    public var userData:Dynamic<Dynamic>=null;
    public var gravityx:Float=0.0;
    public var gravityy:Float=0.0;
    public var wrap_gravity:Vec2=null;
    public function getgravity(){
        wrap_gravity=Vec2.get(gravityx,gravityy);
        wrap_gravity.zpp_inner._inuse=true;
        wrap_gravity.zpp_inner._invalidate=gravity_invalidate;
        wrap_gravity.zpp_inner._validate=gravity_validate;
    }
    private function gravity_invalidate(x:ZPP_Vec2){
        #if(!NAPE_RELEASE_BUILD)
        if(midstep)throw "Error: Space::gravity cannot be set during space step";
        #end
        {
            gravityx=x.x;
            gravityy=x.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((gravityx!=gravityx));
                };
                if(!res)throw "assert("+"!assert_isNaN(gravityx)"+") :: "+("vec_set(in n: "+"gravity"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((gravityy!=gravityy));
                };
                if(!res)throw "assert("+"!assert_isNaN(gravityy)"+") :: "+("vec_set(in n: "+"gravity"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
        };
        var stack=new ZNPList_ZPP_Compound();
        {
            var cx_ite=bodies.begin();
            while(cx_ite!=null){
                var x=cx_ite.elem();
                {
                    var o=x;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            o.space==this;
                        };
                        if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                        #end
                    };
                    if(!o.world){
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                o.component!=null;
                            };
                            if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                            #end
                        };
                        o.component.waket=stamp+(midstep?0:1);
                        if(o.isKinematic())o.kinematicDelaySleep=true;
                        if(o.component.sleeping){
                            really_wake(o,false);
                        }
                    }
                };
                cx_ite=cx_ite.next;
            }
        };
        {
            var cx_ite=compounds.begin();
            while(cx_ite!=null){
                var i=cx_ite.elem();
                stack.add(i);
                cx_ite=cx_ite.next;
            }
        };
        while(!stack.empty()){
            var s=stack.pop_unsafe();
            {
                var cx_ite=s.bodies.begin();
                while(cx_ite!=null){
                    var x=cx_ite.elem();
                    {
                        var o=x;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                o.space==this;
                            };
                            if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                            #end
                        };
                        if(!o.world){
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    o.component!=null;
                                };
                                if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                #end
                            };
                            o.component.waket=stamp+(midstep?0:1);
                            if(o.isKinematic())o.kinematicDelaySleep=true;
                            if(o.component.sleeping){
                                really_wake(o,false);
                            }
                        }
                    };
                    cx_ite=cx_ite.next;
                }
            };
            {
                var cx_ite=s.compounds.begin();
                while(cx_ite!=null){
                    var i=cx_ite.elem();
                    stack.add(i);
                    cx_ite=cx_ite.next;
                }
            };
        }
    }
    private function gravity_validate(){
        {
            wrap_gravity.zpp_inner.x=gravityx;
            wrap_gravity.zpp_inner.y=gravityy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_gravity.zpp_inner.x!=wrap_gravity.zpp_inner.x));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_gravity.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap_gravity.zpp_inner."+",in x: "+"gravityx"+",in y: "+"gravityy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_gravity.zpp_inner.y!=wrap_gravity.zpp_inner.y));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_gravity.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap_gravity.zpp_inner."+",in x: "+"gravityx"+",in y: "+"gravityy"+")");
                #end
            };
        };
    }
    public var bodies:ZNPList_ZPP_Body=null;
    public var wrap_bodies:BodyList=null;
    public var compounds:ZNPList_ZPP_Compound=null;
    public var wrap_compounds:CompoundList=null;
    public var constraints:ZNPList_ZPP_Constraint=null;
    public var wrap_constraints:ConstraintList=null;
    public var kinematics:ZNPList_ZPP_Body=null;
    public var bphase:ZPP_Broadphase=null;
    public var __static:Body=null;
    public var global_lin_drag:Float=0.0;
    public var global_ang_drag:Float=0.0;
    public var stamp:Int=0;
    public var midstep:Bool=false;
    public var time:Float=0.0;
    public var sortcontacts:Bool=false;
    
    public var c_arbiters_true:ZNPList_ZPP_ColArbiter=null;
    public var c_arbiters_false:ZNPList_ZPP_ColArbiter=null;
    public var f_arbiters:ZNPList_ZPP_FluidArbiter=null;
    public var s_arbiters:ZNPList_ZPP_SensorArbiter=null;
    public var wrap_arbiters:ArbiterList=null;
    public var live:ZNPList_ZPP_Body=null;
    public var wrap_live:BodyList=null;
    public var live_constraints:ZNPList_ZPP_Constraint=null;
    public var wrap_livecon:ConstraintList=null;
    public var staticsleep:ZNPList_ZPP_Body=null;
    public var islands:ZPP_Island=null;
    public var listeners:ZNPList_ZPP_Listener=null;
    public var wrap_listeners:ListenerList=null;
    public var callbacks:ZPP_Callback=null;
    public var callbackset_list:ZPP_CallbackSet=null;
    public var cbsets:ZPP_CbSetManager=null;
    public function clear(){
        {
            while(!listeners.empty()){
                var c=listeners.pop_unsafe();
                remListener(c);
            }
        };
        {
            while(!callbackset_list.empty()){
                var c=callbackset_list.pop_unsafe();
                {
                    c.arbiters.clear();
                    {
                        var o=c;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                o!=null;
                            };
                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_CallbackSet"+", in obj: "+"c"+")");
                            #end
                        };
                        o.free();
                        o.next=ZPP_CallbackSet.zpp_pool;
                        ZPP_CallbackSet.zpp_pool=o;
                        #if NAPE_POOL_STATS ZPP_CallbackSet.POOL_CNT++;
                        ZPP_CallbackSet.POOL_SUB++;
                        #end
                    };
                };
            }
        };
        {
            while(!c_arbiters_true.empty()){
                var arb=c_arbiters_true.pop_unsafe();
                arb.retire();
            }
        };
        {
            while(!c_arbiters_false.empty()){
                var arb=c_arbiters_false.pop_unsafe();
                arb.retire();
            }
        };
        {
            while(!s_arbiters.empty()){
                var arb=s_arbiters.pop_unsafe();
                arb.retire();
            }
        };
        {
            while(!f_arbiters.empty()){
                var arb=f_arbiters.pop_unsafe();
                arb.retire();
            }
        };
        bphase.clear();
        {
            while(!bodies.empty()){
                var b=bodies.pop_unsafe();
                {
                    {
                        if(b.component!=null){
                            var i=b.component.island;
                            if(i!=null){
                                {
                                    while(!i.comps.empty()){
                                        var c=i.comps.pop_unsafe();
                                        c.reset();
                                    }
                                };
                                {
                                    var o=i;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o!=null;
                                        };
                                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Island"+", in obj: "+"i"+")");
                                        #end
                                    };
                                    o.free();
                                    o.next=ZPP_Island.zpp_pool;
                                    ZPP_Island.zpp_pool=o;
                                    #if NAPE_POOL_STATS ZPP_Island.POOL_CNT++;
                                    ZPP_Island.POOL_SUB++;
                                    #end
                                };
                            }
                        }
                    };
                    b.removedFromSpace();
                    b.space=null;
                };
            }
        };
        {
            while(!constraints.empty()){
                var c=constraints.pop_unsafe();
                {
                    {
                        if(c.component!=null){
                            var i=c.component.island;
                            if(i!=null){
                                {
                                    while(!i.comps.empty()){
                                        var c=i.comps.pop_unsafe();
                                        c.reset();
                                    }
                                };
                                {
                                    var o=i;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o!=null;
                                        };
                                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Island"+", in obj: "+"i"+")");
                                        #end
                                    };
                                    o.free();
                                    o.next=ZPP_Island.zpp_pool;
                                    ZPP_Island.zpp_pool=o;
                                    #if NAPE_POOL_STATS ZPP_Island.POOL_CNT++;
                                    ZPP_Island.POOL_SUB++;
                                    #end
                                };
                            }
                        }
                    };
                    c.removedFromSpace();
                    c.space=null;
                };
            }
        };
        kinematics.clear();
        var stack=new ZNPList_ZPP_Compound();
        {
            while(!compounds.empty()){
                var c=compounds.pop_unsafe();
                stack.add(c);
            }
        };
        while(!stack.empty()){
            var comp=stack.pop_unsafe();
            comp.removedFromSpace();
            comp.space=null;
            {
                var cx_ite=comp.bodies.begin();
                while(cx_ite!=null){
                    var b=cx_ite.elem();
                    {
                        {
                            if(b.component!=null){
                                var i=b.component.island;
                                if(i!=null){
                                    {
                                        while(!i.comps.empty()){
                                            var c=i.comps.pop_unsafe();
                                            c.reset();
                                        }
                                    };
                                    {
                                        var o=i;
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                o!=null;
                                            };
                                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Island"+", in obj: "+"i"+")");
                                            #end
                                        };
                                        o.free();
                                        o.next=ZPP_Island.zpp_pool;
                                        ZPP_Island.zpp_pool=o;
                                        #if NAPE_POOL_STATS ZPP_Island.POOL_CNT++;
                                        ZPP_Island.POOL_SUB++;
                                        #end
                                    };
                                }
                            }
                        };
                        b.removedFromSpace();
                        b.space=null;
                    };
                    cx_ite=cx_ite.next;
                }
            };
            {
                var cx_ite=comp.constraints.begin();
                while(cx_ite!=null){
                    var c=cx_ite.elem();
                    {
                        {
                            if(c.component!=null){
                                var i=c.component.island;
                                if(i!=null){
                                    {
                                        while(!i.comps.empty()){
                                            var c=i.comps.pop_unsafe();
                                            c.reset();
                                        }
                                    };
                                    {
                                        var o=i;
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                o!=null;
                                            };
                                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Island"+", in obj: "+"i"+")");
                                            #end
                                        };
                                        o.free();
                                        o.next=ZPP_Island.zpp_pool;
                                        ZPP_Island.zpp_pool=o;
                                        #if NAPE_POOL_STATS ZPP_Island.POOL_CNT++;
                                        ZPP_Island.POOL_SUB++;
                                        #end
                                    };
                                }
                            }
                        };
                        c.removedFromSpace();
                        c.space=null;
                    };
                    cx_ite=cx_ite.next;
                }
            };
            {
                var cx_ite=comp.compounds.begin();
                while(cx_ite!=null){
                    var i=cx_ite.elem();
                    stack.add(i);
                    cx_ite=cx_ite.next;
                }
            };
        }
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                islands.empty();
            };
            if(!res)throw "assert("+"islands.empty()"+") :: "+("islands exist... wuh?");
            #end
        };
        staticsleep.clear();
        live.clear();
        live_constraints.clear();
        stamp=0;
        time=0.0;
        #if NAPE_TIMES Debug.FOR=Debug.BROAD=Debug.PRE=Debug.POS=Debug.VEL=Debug.DRAW=Debug.SORT=0;
        Debug.ltime=0;
        #end
        mrca1.clear();
        mrca2.clear();
        prelisteners.clear();
        cbsets.clear();
    }
    private function bodies_adder(x:Body){
        #if(!NAPE_RELEASE_BUILD)
        if(x.zpp_inner.compound!=null)throw "Error: Cannot set the space of a Body belonging to a Compound, only the root Compound space can be set";
        #end
        if(x.zpp_inner.space!=this){
            if(x.zpp_inner.space!=null)x.zpp_inner.space.outer.bodies.remove(x);
            addBody(x.zpp_inner);
            return true;
        }
        else return false;
    }
    private function bodies_subber(x:Body){
        remBody(x.zpp_inner);
    }
    #if(!NAPE_RELEASE_BUILD)
    private function bodies_modifiable(){
        if(midstep)throw "Error: Space::bodies cannot be set during space step()";
    }
    #end
    private function compounds_adder(x:Compound){
        #if(!NAPE_RELEASE_BUILD)
        if(x.zpp_inner.compound!=null)throw "Error: Cannot set the space of an inner Compound, only the root Compound space can be set";
        #end
        if(x.zpp_inner.space!=this){
            if(x.zpp_inner.space!=null)x.zpp_inner.space.wrap_compounds.remove(x);
            addCompound(x.zpp_inner);
            return true;
        }
        else return false;
    }
    private function compounds_subber(x:Compound){
        remCompound(x.zpp_inner);
    }
    #if(!NAPE_RELEASE_BUILD)
    private function compounds_modifiable(){
        if(midstep)throw "Error: Space::compounds cannot be set during space step()";
    }
    #end
    private function constraints_adder(x:Constraint){
        #if(!NAPE_RELEASE_BUILD)
        if(x.zpp_inner.compound!=null)throw "Error: Cannot set the space of a Constraint belonging to a Compound, only the root Compound space can be set";
        #end
        if(x.zpp_inner.space!=this){
            if(x.zpp_inner.space!=null)x.zpp_inner.space.outer.constraints.remove(x);
            this.addConstraint(x.zpp_inner);
            return true;
        }
        else return false;
    }
    private function constraints_subber(x:Constraint){
        remConstraint(x.zpp_inner);
    }
    #if(!NAPE_RELEASE_BUILD)
    private function constraints_modifiable(){
        if(midstep)throw "Error: Space::constraints cannot be set during space step()";
    }
    #end
    private function listeners_adder(x:Listener){
        if(x.zpp_inner.space!=this){
            if(x.zpp_inner.space!=null)x.zpp_inner.space.outer.listeners.remove(x);
            addListener(x.zpp_inner);
            return true;
        }
        else return false;
    }
    private function listeners_subber(x:Listener){
        remListener(x.zpp_inner);
    }
    #if(!NAPE_RELEASE_BUILD)
    private function listeners_modifiable(){
        if(midstep)throw "Error: Space::listeners cannot be set during space step()";
    }
    #end
    public function new(gravity:ZPP_Vec2,broadphase:Broadphase){
        toiEvents=new ZNPList_ZPP_ToiEvent();
        global_lin_drag=0.015;
        global_ang_drag=0.015;
        #if(!NAPE_RELEASE_BUILD)
        ZPP_Callback.internal=true;
        #end
        precb=new PreCallback();
        precb.zpp_inner=new ZPP_Callback();
        #if(!NAPE_RELEASE_BUILD)
        ZPP_Callback.internal=false;
        #end
        sortcontacts=true;
        pre_dt=0.0;
        if(broadphase==null||broadphase==Broadphase.DYNAMIC_AABB_TREE)bphase=new ZPP_DynAABBPhase(this);
        else if(broadphase==Broadphase.SWEEP_AND_PRUNE)bphase=new ZPP_SweepPhase(this);
        time=0.0;
        var me=this;
        if(gravity!=null){
            this.gravityx=gravity.x;
            this.gravityy=gravity.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((this.gravityx!=this.gravityx));
                };
                if(!res)throw "assert("+"!assert_isNaN(this.gravityx)"+") :: "+("vec_set(in n: "+"this.gravity"+",in x: "+"gravity.x"+",in y: "+"gravity.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((this.gravityy!=this.gravityy));
                };
                if(!res)throw "assert("+"!assert_isNaN(this.gravityy)"+") :: "+("vec_set(in n: "+"this.gravity"+",in x: "+"gravity.x"+",in y: "+"gravity.y"+")");
                #end
            };
        };
        else{
            this.gravityx=0;
            this.gravityy=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((this.gravityx!=this.gravityx));
                };
                if(!res)throw "assert("+"!assert_isNaN(this.gravityx)"+") :: "+("vec_set(in n: "+"this.gravity"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((this.gravityy!=this.gravityy));
                };
                if(!res)throw "assert("+"!assert_isNaN(this.gravityy)"+") :: "+("vec_set(in n: "+"this.gravity"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        bodies=new ZNPList_ZPP_Body();
        wrap_bodies=ZPP_BodyList.get(bodies);
        wrap_bodies.zpp_inner.adder=bodies_adder;
        wrap_bodies.zpp_inner.subber=bodies_subber;
        #if(!NAPE_RELEASE_BUILD)
        wrap_bodies.zpp_inner._modifiable=bodies_modifiable;
        #end
        compounds=new ZNPList_ZPP_Compound();
        wrap_compounds=ZPP_CompoundList.get(compounds);
        wrap_compounds.zpp_inner.adder=compounds_adder;
        wrap_compounds.zpp_inner.subber=compounds_subber;
        #if(!NAPE_RELEASE_BUILD)
        wrap_compounds.zpp_inner._modifiable=compounds_modifiable;
        #end
        kinematics=new ZNPList_ZPP_Body();
        c_arbiters_true=new ZNPList_ZPP_ColArbiter();
        c_arbiters_false=new ZNPList_ZPP_ColArbiter();
        f_arbiters=new ZNPList_ZPP_FluidArbiter();
        s_arbiters=new ZNPList_ZPP_SensorArbiter();
        islands=new ZPP_Island();
        live=new ZNPList_ZPP_Body();
        wrap_live=ZPP_BodyList.get(live,true);
        staticsleep=new ZNPList_ZPP_Body();
        constraints=new ZNPList_ZPP_Constraint();
        wrap_constraints=ZPP_ConstraintList.get(constraints);
        wrap_constraints.zpp_inner.adder=constraints_adder;
        wrap_constraints.zpp_inner.subber=constraints_subber;
        #if(!NAPE_RELEASE_BUILD)
        wrap_constraints.zpp_inner._modifiable=constraints_modifiable;
        #end
        live_constraints=new ZNPList_ZPP_Constraint();
        wrap_livecon=ZPP_ConstraintList.get(live_constraints,true);
        __static=ZPP_Body.__static();
        __static.zpp_inner.space=this;
        callbacks=new ZPP_Callback();
        midstep=false;
        listeners=new ZNPList_ZPP_Listener();
        wrap_listeners=ZPP_ListenerList.get(listeners);
        wrap_listeners.zpp_inner.adder=listeners_adder;
        wrap_listeners.zpp_inner.subber=listeners_subber;
        #if(!NAPE_RELEASE_BUILD)
        wrap_listeners.zpp_inner._modifiable=listeners_modifiable;
        #end
        callbackset_list=new ZPP_CallbackSet();
        mrca1=new ZNPList_ZPP_Interactor();
        mrca2=new ZNPList_ZPP_Interactor();
        prelisteners=new ZNPList_ZPP_InteractionListener();
        cbsets=new ZPP_CbSetManager(this);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function revoke_listener(x:ZPP_InteractionListener){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function unrevoke_listener(x:ZPP_InteractionListener){}
    public function addListener(x:ZPP_Listener){
        x.space=this;
        x.addedToSpace();
        if(x.interaction!=null){
            unrevoke_listener(x.interaction);
        }
    }
    public function remListener(x:ZPP_Listener){
        if(x.interaction!=null){
            revoke_listener(x.interaction);
        }
        x.removedFromSpace();
        x.space=null;
    }
    public function add_callbackset(cb:ZPP_CallbackSet){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !(cb.freed||cb.lazydel);
            };
            if(!res)throw "assert("+"!(cb.freed||cb.lazydel)"+") :: "+("adding callbacket which is freed or lazydel??");
            #end
        };
        cb.int1.cbsets.inlined_add(cb);
        cb.int2.cbsets.inlined_add(cb);
        callbackset_list.inlined_add(cb);
    }
    public function remove_callbackset(cb:ZPP_CallbackSet){
        cb.lazydel=true;
        cb.int1.cbsets.inlined_remove(cb);
        cb.int2.cbsets.inlined_remove(cb);
    }
    public function transmitType(p:ZPP_Body,new_type:Int){
        {
            var o=p;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o.space==this;
                };
                if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                #end
            };
            if(!o.world){
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o.component!=null;
                    };
                    if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                    #end
                };
                o.component.waket=stamp+(midstep?0:1);
                if(o.isKinematic())o.kinematicDelaySleep=true;
                if(o.component.sleeping){
                    really_wake(o,false);
                }
            }
        };
        if(p.type==ZPP_Flags.id_BodyType_DYNAMIC){
            live.remove(p);
        }
        else if(p.type==ZPP_Flags.id_BodyType_KINEMATIC){
            kinematics.remove(p);
            staticsleep.remove(p);
        }
        else if(p.type==ZPP_Flags.id_BodyType_STATIC){
            staticsleep.remove(p);
        }
        p.type=new_type;
        if(p.type==ZPP_Flags.id_BodyType_KINEMATIC)kinematics.add(p);
        if(p.type==ZPP_Flags.id_BodyType_STATIC)static_validation(p);
        p.component.sleeping=true;
        {
            var o=p;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o.space==this;
                };
                if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                #end
            };
            if(!o.world){
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o.component!=null;
                    };
                    if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                    #end
                };
                o.component.waket=stamp+(midstep?0:1);
                if(o.isKinematic())o.kinematicDelaySleep=true;
                if(o.component.sleeping){
                    really_wake(o,true);
                }
            }
        };
    }
    public#if NAPE_NO_INLINE#else inline #end
    function added_shape(s:ZPP_Shape,dontwake:Bool=false){
        if(!dontwake){
            {
                var o=s.body;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o.space==this;
                    };
                    if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                    #end
                };
                if(!o.world){
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            o.component!=null;
                        };
                        if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                        #end
                    };
                    o.component.waket=stamp+(midstep?0:1);
                    if(o.isKinematic())o.kinematicDelaySleep=true;
                    if(o.component.sleeping){
                        really_wake(o,false);
                    }
                }
            };
        }
        bphase.insert(s);
        s.addedToSpace();
    }
    public function removed_shape(s:ZPP_Shape,deleting=false){
        var body=s.body;
        if(!deleting){
            body.wake();
        }
        var pre=null;
        {
            var cx_ite=body.arbiters.begin();
            while(cx_ite!=null){
                var xarb=cx_ite.elem();
                {
                    var rem=xarb.ws1==s||xarb.ws2==s;
                    if(rem){
                        if(xarb.present!=0){
                            MRCA_chains(xarb.ws1,xarb.ws2);
                            {
                                var cx_ite=mrca1.begin();
                                while(cx_ite!=null){
                                    var i1=cx_ite.elem();
                                    {
                                        {
                                            var cx_ite=mrca2.begin();
                                            while(cx_ite!=null){
                                                var i2=cx_ite.elem();
                                                {
                                                    var cb1=i1.cbSet;
                                                    var cb2=i2.cbSet;
                                                    {
                                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                        var res={
                                                            cb1!=null&&cb2!=null;
                                                        };
                                                        if(!res)throw "assert("+"cb1!=null&&cb2!=null"+") :: "+("null cbsets from MRCA_chains?");
                                                        #end
                                                    };
                                                    cb1.validate();
                                                    cb2.validate();
                                                    if(ZPP_CbSet.empty_intersection(cb1,cb2)){
                                                        cx_ite=cx_ite.next;
                                                        continue;
                                                    };
                                                    var callbackset=ZPP_Interactor.get(i1,i2);
                                                    {
                                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                        var res={
                                                            callbackset!=null;
                                                        };
                                                        if(!res)throw "assert("+"callbackset!=null"+") :: "+("null callbackset on arbiter deletion?");
                                                        #end
                                                    };
                                                    callbackset.remove_arb(xarb);
                                                    xarb.present--;
                                                    ZPP_CbSet.find_all(cb1,cb2,ZPP_Flags.id_CbEvent_END,function(listener:ZPP_InteractionListener){
                                                        if((listener.itype&xarb.type)!=0&&callbackset.empty_arb(listener.itype)){
                                                            var cb=push_callback(listener);
                                                            cb.event=ZPP_Flags.id_CbEvent_END;
                                                            ZPP_Interactor.int_callback(callbackset,listener,cb);
                                                            cb.set=callbackset;
                                                        }
                                                    });
                                                    if(callbackset.really_empty()){
                                                        remove_callbackset(callbackset);
                                                    }
                                                };
                                                cx_ite=cx_ite.next;
                                            }
                                        };
                                    };
                                    cx_ite=cx_ite.next;
                                }
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    xarb.present>=0;
                                };
                                if(!res)throw "assert("+"xarb.present>=0"+") :: "+("xarb present < 0?");
                                #end
                            };
                        }
                        if(xarb.b1!=body&&xarb.b1.isDynamic()){
                            var o=xarb.b1;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    o.space==this;
                                };
                                if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                #end
                            };
                            if(!o.world){
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        o.component!=null;
                                    };
                                    if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                    #end
                                };
                                o.component.waket=stamp+(midstep?0:1);
                                if(o.isKinematic())o.kinematicDelaySleep=true;
                                if(o.component.sleeping){
                                    really_wake(o,false);
                                }
                            }
                        };
                        if(xarb.b2!=body&&xarb.b2.isDynamic()){
                            var o=xarb.b2;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    o.space==this;
                                };
                                if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                #end
                            };
                            if(!o.world){
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        o.component!=null;
                                    };
                                    if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                    #end
                                };
                                o.component.waket=stamp+(midstep?0:1);
                                if(o.isKinematic())o.kinematicDelaySleep=true;
                                if(o.component.sleeping){
                                    really_wake(o,false);
                                }
                            }
                        };
                        xarb.lazyRetire(this,body);
                        cx_ite=body.arbiters.erase(pre);
                        continue;
                    }
                    pre=cx_ite;
                };
                cx_ite=cx_ite.next;
            }
        };
        bphase.remove(s);
        s.removedFromSpace();
    }
    public function addConstraint(con:ZPP_Constraint){
        con.space=this;
        con.addedToSpace();
        if(con.active){
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    con.component.island==null;
                };
                if(!res)throw "assert("+"con.component.island==null"+") :: "+("newly added constraint has an island??");
                #end
            };
            con.component.sleeping=true;
            wake_constraint(con,true);
        }
    }
    public function remConstraint(con:ZPP_Constraint){
        if(con.active){
            wake_constraint(con,true);
            live_constraints.remove(con);
        }
        con.removedFromSpace();
        con.space=null;
    }
    public function addCompound(x:ZPP_Compound){
        x.space=this;
        x.addedToSpace();
        {
            var cx_ite=x.bodies.begin();
            while(cx_ite!=null){
                var i=cx_ite.elem();
                addBody(i);
                cx_ite=cx_ite.next;
            }
        };
        {
            var cx_ite=x.constraints.begin();
            while(cx_ite!=null){
                var i=cx_ite.elem();
                addConstraint(i);
                cx_ite=cx_ite.next;
            }
        };
        {
            var cx_ite=x.compounds.begin();
            while(cx_ite!=null){
                var i=cx_ite.elem();
                addCompound(i);
                cx_ite=cx_ite.next;
            }
        };
    }
    public function remCompound(x:ZPP_Compound){
        {
            var cx_ite=x.bodies.begin();
            while(cx_ite!=null){
                var i=cx_ite.elem();
                remBody(i);
                cx_ite=cx_ite.next;
            }
        };
        {
            var cx_ite=x.constraints.begin();
            while(cx_ite!=null){
                var i=cx_ite.elem();
                remConstraint(i);
                cx_ite=cx_ite.next;
            }
        };
        {
            var cx_ite=x.compounds.begin();
            while(cx_ite!=null){
                var i=cx_ite.elem();
                remCompound(i);
                cx_ite=cx_ite.next;
            }
        };
        x.removedFromSpace();
        x.space=null;
    }
    public function addBody(body:ZPP_Body,flag:Int=-1){
        body.space=this;
        body.addedToSpace();
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                body.component.island==null;
            };
            if(!res)throw "assert("+"body.component.island==null"+") :: "+("newly added body has an island??");
            #end
        };
        body.component.sleeping=true;
        {
            var o=body;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o.space==this;
                };
                if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                #end
            };
            if(!o.world){
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o.component!=null;
                    };
                    if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                    #end
                };
                o.component.waket=stamp+(midstep?0:1);
                if(o.isKinematic())o.kinematicDelaySleep=true;
                if(o.component.sleeping){
                    really_wake(o,true);
                }
            }
        };
        {
            var cx_ite=body.shapes.begin();
            while(cx_ite!=null){
                var shape=cx_ite.elem();
                added_shape(shape,true);
                cx_ite=cx_ite.next;
            }
        };
        if(body.isStatic()){
            static_validation(body);
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    staticsleep.has(body);
                };
                if(!res)throw "assert("+"staticsleep.has(body)"+") :: "+("a2b f="+flag);
                #end
            };
        }
        else{
            if(body.isDynamic()){
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        live.has(body);
                    };
                    if(!res)throw "assert("+"live.has(body)"+") :: "+("a4b f="+flag);
                    #end
                };
            }
            else{
                if(flag!=ZPP_Flags.id_BodyType_KINEMATIC)kinematics.add(body);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        staticsleep.has(body);
                    };
                    if(!res)throw "assert("+"staticsleep.has(body)"+") :: "+("a6b f="+flag);
                    #end
                };
            }
        }
    }
    public function remBody(body:ZPP_Body,flag:Int=-1){
        if(body.isStatic()){
            {
                var o=body;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o.space==this;
                    };
                    if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                    #end
                };
                if(!o.world){
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            o.component!=null;
                        };
                        if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                        #end
                    };
                    o.component.waket=stamp+(midstep?0:1);
                    if(o.isKinematic())o.kinematicDelaySleep=true;
                    if(o.component.sleeping){
                        really_wake(o,true);
                    }
                }
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !body.component.sleeping;
                };
                if(!res)throw "assert("+"!body.component.sleeping"+") :: "+("as3");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    staticsleep.has(body);
                };
                if(!res)throw "assert("+"staticsleep.has(body)"+") :: "+("as4");
                #end
            };
            staticsleep.remove(body);
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !staticsleep.has(body);
                };
                if(!res)throw "assert("+"!staticsleep.has(body)"+") :: "+("e1");
                #end
            };
        }
        else{
            if(body.isDynamic()){
                {
                    var o=body;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            o.space==this;
                        };
                        if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                        #end
                    };
                    if(!o.world){
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                o.component!=null;
                            };
                            if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                            #end
                        };
                        o.component.waket=stamp+(midstep?0:1);
                        if(o.isKinematic())o.kinematicDelaySleep=true;
                        if(o.component.sleeping){
                            really_wake(o,true);
                        }
                    }
                };
                live.remove(body);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !live.has(body);
                    };
                    if(!res)throw "assert("+"!live.has(body)"+") :: "+("e3b");
                    #end
                };
            }
            else{
                if(flag!=ZPP_Flags.id_BodyType_KINEMATIC)kinematics.remove(body);
                {
                    var o=body;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            o.space==this;
                        };
                        if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                        #end
                    };
                    if(!o.world){
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                o.component!=null;
                            };
                            if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                            #end
                        };
                        o.component.waket=stamp+(midstep?0:1);
                        if(o.isKinematic())o.kinematicDelaySleep=true;
                        if(o.component.sleeping){
                            really_wake(o,true);
                        }
                    }
                };
                staticsleep.remove(body);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !staticsleep.has(body);
                    };
                    if(!res)throw "assert("+"!staticsleep.has(body)"+") :: "+("e4");
                    #end
                };
            }
        }
        {
            var cx_ite=body.shapes.begin();
            while(cx_ite!=null){
                var shape=cx_ite.elem();
                removed_shape(shape,true);
                cx_ite=cx_ite.next;
            }
        };
        body.removedFromSpace();
        body.space=null;
    }
    public function shapesUnderPoint(x:Float,y:Float,filter:ZPP_InteractionFilter,output:ShapeList){
        return bphase.shapesUnderPoint(x,y,filter,output);
    }
    public function bodiesUnderPoint(x:Float,y:Float,filter:ZPP_InteractionFilter,output:BodyList){
        return bphase.bodiesUnderPoint(x,y,filter,output);
    }
    public function shapesInAABB(aabb:AABB,strict:Bool,cont:Bool,filter:ZPP_InteractionFilter,output:ShapeList){
        return bphase.shapesInAABB(aabb.zpp_inner,strict,cont,filter,output);
    }
    public function bodiesInAABB(aabb:AABB,strict:Bool,cont:Bool,filter:ZPP_InteractionFilter,output:BodyList){
        return bphase.bodiesInAABB(aabb.zpp_inner,strict,cont,filter,output);
    }
    public function shapesInCircle(pos:Vec2,rad:Float,cont:Bool,filter:ZPP_InteractionFilter,output:ShapeList){
        return bphase.shapesInCircle(pos.x,pos.y,rad,cont,filter,output);
    }
    public function bodiesInCircle(pos:Vec2,rad:Float,cont:Bool,filter:ZPP_InteractionFilter,output:BodyList){
        return bphase.bodiesInCircle(pos.x,pos.y,rad,cont,filter,output);
    }
    public function shapesInShape(shape:ZPP_Shape,cont:Bool,filter:ZPP_InteractionFilter,output:ShapeList){
        return bphase.shapesInShape(shape,cont,filter,output);
    }
    public function bodiesInShape(shape:ZPP_Shape,cont:Bool,filter:ZPP_InteractionFilter,output:BodyList){
        return bphase.bodiesInShape(shape,cont,filter,output);
    }
    public function rayCast(ray:Ray,inner:Bool,filter:InteractionFilter){
        return bphase.rayCast(ray.zpp_inner,inner,filter==null?null:filter.zpp_inner);
    }
    public function rayMultiCast(ray:Ray,inner:Bool,filter:InteractionFilter,output:RayResultList){
        return bphase.rayMultiCast(ray.zpp_inner,inner,filter==null?null:filter.zpp_inner,output);
    }
    var convexShapeList:ShapeList=null;
    public function convexCast(shape:ZPP_Shape,deltaTime:Float,filter:InteractionFilter,dynamics:Bool){
        var toi;
        {
            if(ZPP_ToiEvent.zpp_pool==null){
                toi=new ZPP_ToiEvent();
                #if NAPE_POOL_STATS ZPP_ToiEvent.POOL_TOT++;
                ZPP_ToiEvent.POOL_ADDNEW++;
                #end
            }
            else{
                toi=ZPP_ToiEvent.zpp_pool;
                ZPP_ToiEvent.zpp_pool=toi.next;
                toi.next=null;
                #if NAPE_POOL_STATS ZPP_ToiEvent.POOL_CNT--;
                ZPP_ToiEvent.POOL_ADD++;
                #end
            }
            toi.alloc();
        };
        prepareCast(shape);
        var body=shape.body;
        var prex=body.posx;
        var prey=body.posy;
        body.sweepTime=0;
        body.sweep_angvel=body.angvel;
        body.sweepIntegrate(deltaTime);
        var postx=body.posx;
        var posty=body.posy;
        shape.validate_sweepRadius();
        var rad=shape.sweepRadius;
        var aabb;
        {
            if(ZPP_AABB.zpp_pool==null){
                aabb=new ZPP_AABB();
                #if NAPE_POOL_STATS ZPP_AABB.POOL_TOT++;
                ZPP_AABB.POOL_ADDNEW++;
                #end
            }
            else{
                aabb=ZPP_AABB.zpp_pool;
                ZPP_AABB.zpp_pool=aabb.next;
                aabb.next=null;
                #if NAPE_POOL_STATS ZPP_AABB.POOL_CNT--;
                ZPP_AABB.POOL_ADD++;
                #end
            }
            aabb.alloc();
        };
        aabb.minx=({
            var x=prex;
            var y=postx;
            x<y?x:y;
        })-rad;
        aabb.maxx=({
            var x=prex;
            var y=postx;
            x>y?x:y;
        })+rad;
        aabb.miny=({
            var x=prey;
            var y=posty;
            x<y?x:y;
        })-rad;
        aabb.maxy=({
            var x=prey;
            var y=posty;
            x>y?x:y;
        })+rad;
        var list=convexShapeList=bphase.shapesInAABB(aabb,false,false,filter==null?null:filter.zpp_inner,convexShapeList);
        {
            var o=aabb;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_AABB"+", in obj: "+"aabb"+")");
                #end
            };
            o.free();
            o.next=ZPP_AABB.zpp_pool;
            ZPP_AABB.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_AABB.POOL_CNT++;
            ZPP_AABB.POOL_SUB++;
            #end
        };
        var minAxisx:Float=0.0;
        var minAxisy:Float=0.0;
        {
            minAxisx=0;
            minAxisy=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((minAxisx!=minAxisx));
                };
                if(!res)throw "assert("+"!assert_isNaN(minAxisx)"+") :: "+("vec_set(in n: "+"minAxis"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((minAxisy!=minAxisy));
                };
                if(!res)throw "assert("+"!assert_isNaN(minAxisy)"+") :: "+("vec_set(in n: "+"minAxis"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        var minPosx:Float=0.0;
        var minPosy:Float=0.0;
        {
            minPosx=0;
            minPosy=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((minPosx!=minPosx));
                };
                if(!res)throw "assert("+"!assert_isNaN(minPosx)"+") :: "+("vec_set(in n: "+"minPos"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((minPosy!=minPosy));
                };
                if(!res)throw "assert("+"!assert_isNaN(minPosy)"+") :: "+("vec_set(in n: "+"minPos"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        var mins=null;
        var mint=deltaTime+1;
        for(s in list){
            if(s!=shape.outer&&s.body!=body.outer){
                toi.s1=shape;
                toi.s2=s.zpp_inner;
                if(dynamics){
                    s.zpp_inner.validate_sweepRadius();
                    s.body.zpp_inner.sweep_angvel=s.body.zpp_inner.angvel;
                    s.body.zpp_inner.sweepTime=0;
                    ZPP_SweepDistance.dynamicSweep(toi,deltaTime,0,0,true);
                    s.body.zpp_inner.sweepIntegrate(0);
                    s.body.zpp_inner.sweepValidate(s.zpp_inner);
                }
                else{
                    ZPP_SweepDistance.staticSweep(toi,deltaTime,0,0);
                }
                toi.toi*=deltaTime;
                if(toi.toi>0&&toi.toi<mint){
                    mint=toi.toi;
                    {
                        minAxisx=toi.axis.x;
                        minAxisy=toi.axis.y;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((minAxisx!=minAxisx));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(minAxisx)"+") :: "+("vec_set(in n: "+"minAxis"+",in x: "+"toi.axis.x"+",in y: "+"toi.axis.y"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((minAxisy!=minAxisy));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(minAxisy)"+") :: "+("vec_set(in n: "+"minAxis"+",in x: "+"toi.axis.x"+",in y: "+"toi.axis.y"+")");
                            #end
                        };
                    };
                    {
                        minPosx=toi.c2.x;
                        minPosy=toi.c2.y;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((minPosx!=minPosx));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(minPosx)"+") :: "+("vec_set(in n: "+"minPos"+",in x: "+"toi.c2.x"+",in y: "+"toi.c2.y"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((minPosy!=minPosy));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(minPosy)"+") :: "+("vec_set(in n: "+"minPos"+",in x: "+"toi.c2.x"+",in y: "+"toi.c2.y"+")");
                            #end
                        };
                    };
                    mins=s;
                }
            }
        }
        list.clear();
        {
            var o=toi;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_ToiEvent"+", in obj: "+"toi"+")");
                #end
            };
            o.free();
            o.next=ZPP_ToiEvent.zpp_pool;
            ZPP_ToiEvent.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_ToiEvent.POOL_CNT++;
            ZPP_ToiEvent.POOL_SUB++;
            #end
        };
        body.sweepIntegrate(0);
        body.sweepValidate(shape);
        if(mint<=deltaTime){
            return ZPP_ConvexRayResult.getConvex(Vec2.get(-minAxisx,-minAxisy),Vec2.get(minPosx,minPosy),mint,mins);
        }
        else return null;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    #if NAPE_NO_INLINE#else inline #end
    function prepareCast(s:ZPP_Shape){
        if(s.isCircle())s.circle.validate_worldCOM();
        else s.polygon.validate_gaxi();
    }
    public function convexMultiCast(shape:ZPP_Shape,deltaTime:Float,filter:InteractionFilter,dynamics:Bool,output:ConvexResultList){
        var toi;
        {
            if(ZPP_ToiEvent.zpp_pool==null){
                toi=new ZPP_ToiEvent();
                #if NAPE_POOL_STATS ZPP_ToiEvent.POOL_TOT++;
                ZPP_ToiEvent.POOL_ADDNEW++;
                #end
            }
            else{
                toi=ZPP_ToiEvent.zpp_pool;
                ZPP_ToiEvent.zpp_pool=toi.next;
                toi.next=null;
                #if NAPE_POOL_STATS ZPP_ToiEvent.POOL_CNT--;
                ZPP_ToiEvent.POOL_ADD++;
                #end
            }
            toi.alloc();
        };
        prepareCast(shape);
        var body=shape.body;
        var prex=body.posx;
        var prey=body.posy;
        body.sweepTime=0;
        body.sweep_angvel=body.angvel;
        body.sweepIntegrate(deltaTime);
        var postx=body.posx;
        var posty=body.posy;
        shape.validate_sweepRadius();
        var rad=shape.sweepRadius;
        var aabb;
        {
            if(ZPP_AABB.zpp_pool==null){
                aabb=new ZPP_AABB();
                #if NAPE_POOL_STATS ZPP_AABB.POOL_TOT++;
                ZPP_AABB.POOL_ADDNEW++;
                #end
            }
            else{
                aabb=ZPP_AABB.zpp_pool;
                ZPP_AABB.zpp_pool=aabb.next;
                aabb.next=null;
                #if NAPE_POOL_STATS ZPP_AABB.POOL_CNT--;
                ZPP_AABB.POOL_ADD++;
                #end
            }
            aabb.alloc();
        };
        aabb.minx=({
            var x=prex;
            var y=postx;
            x<y?x:y;
        })-rad;
        aabb.maxx=({
            var x=prex;
            var y=postx;
            x>y?x:y;
        })+rad;
        aabb.miny=({
            var x=prey;
            var y=posty;
            x<y?x:y;
        })-rad;
        aabb.maxy=({
            var x=prey;
            var y=posty;
            x>y?x:y;
        })+rad;
        var list=convexShapeList=bphase.shapesInAABB(aabb,false,false,filter==null?null:filter.zpp_inner,convexShapeList);
        {
            var o=aabb;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_AABB"+", in obj: "+"aabb"+")");
                #end
            };
            o.free();
            o.next=ZPP_AABB.zpp_pool;
            ZPP_AABB.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_AABB.POOL_CNT++;
            ZPP_AABB.POOL_SUB++;
            #end
        };
        var ret=(output==null?new ConvexResultList():output);
        for(s in list){
            if(s!=shape.outer&&s.body!=body.outer){
                toi.s1=shape;
                toi.s2=s.zpp_inner;
                if(dynamics){
                    s.zpp_inner.validate_sweepRadius();
                    s.body.zpp_inner.sweep_angvel=s.body.zpp_inner.angvel;
                    s.body.zpp_inner.sweepTime=0;
                    ZPP_SweepDistance.dynamicSweep(toi,deltaTime,0,0,true);
                    s.body.zpp_inner.sweepIntegrate(0);
                    s.body.zpp_inner.sweepValidate(s.zpp_inner);
                }
                else{
                    ZPP_SweepDistance.staticSweep(toi,deltaTime,0,0);
                }
                toi.toi*=deltaTime;
                if(toi.toi>0){
                    var res=ZPP_ConvexRayResult.getConvex(Vec2.get(-toi.axis.x,-toi.axis.y),Vec2.get(toi.c2.x,toi.c2.y),toi.toi,s);
                    {
                        var pre=null;
                        {
                            var cx_ite=ret.zpp_inner.inner.begin();
                            while(cx_ite!=null){
                                var j=cx_ite.elem();
                                {
                                    if((res.toi<j.toi))break;
                                    pre=cx_ite;
                                };
                                cx_ite=cx_ite.next;
                            }
                        };
                        ret.zpp_inner.inner.inlined_insert(pre,res);
                    };
                }
            }
        }
        list.clear();
        {
            var o=toi;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_ToiEvent"+", in obj: "+"toi"+")");
                #end
            };
            o.free();
            o.next=ZPP_ToiEvent.zpp_pool;
            ZPP_ToiEvent.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_ToiEvent.POOL_CNT++;
            ZPP_ToiEvent.POOL_SUB++;
            #end
        };
        body.sweepIntegrate(0);
        body.sweepValidate(shape);
        return ret;
    }
    public function push_callback(i:ZPP_Listener){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                i!=null;
            };
            if(!res)throw "assert("+"i!=null"+") :: "+("null listenere for push_callback?");
            #end
        };
        var cb;
        {
            if(ZPP_Callback.zpp_pool==null){
                cb=new ZPP_Callback();
                #if NAPE_POOL_STATS ZPP_Callback.POOL_TOT++;
                ZPP_Callback.POOL_ADDNEW++;
                #end
            }
            else{
                cb=ZPP_Callback.zpp_pool;
                ZPP_Callback.zpp_pool=cb.next;
                cb.next=null;
                #if NAPE_POOL_STATS ZPP_Callback.POOL_CNT--;
                ZPP_Callback.POOL_ADD++;
                #end
            }
            cb.alloc();
        };
        callbacks.push(cb);
        cb.listener=i;
        return cb;
    }
    public var pre_dt:Float=0.0;
    public function step(deltaTime:Float,velocityIterations:Int,positionIterations:Int){
        #if(!NAPE_RELEASE_BUILD)
        if(midstep)throw "Error: ... REALLY?? you're going to call space.step() inside of space.step()? COME ON!!";
        #end
        time+=deltaTime;
        pre_dt=deltaTime;
        midstep=true;
        stamp++;
        #if NAPE_TIMES var pt=flash.Lib.getTimer();
        #end
        validation();
        #if NAPE_TIMES Debug.VALID+=flash.Lib.getTimer()-pt;
        #end
        #if NAPE_TIMES var pt=flash.Lib.getTimer();
        #end
        bphase.broadphase(this,true);
        #if NAPE_TIMES Debug.BROAD+=flash.Lib.getTimer()-pt;
        #end
        #if NAPE_TIMES Debug.ACNT=0;
        Debug.AACNT=0;
        Debug.CCNT=0;
        Debug.ACCNT=0;
        #end
        #if NAPE_TIMES var pt=flash.Lib.getTimer();
        #end
        prestep(deltaTime);
        #if NAPE_TIMES Debug.PRE+=flash.Lib.getTimer()-pt;
        #end
        #if NAPE_TIMES var pt=flash.Lib.getTimer();
        #end
        if(sortcontacts){
            {
                var xxlist=c_arbiters_false;
                if(!xxlist.empty()&&xxlist.begin().next!=null){
                    var head:ZNPNode_ZPP_ColArbiter=xxlist.begin();
                    var tail:ZNPNode_ZPP_ColArbiter=null;
                    var left:ZNPNode_ZPP_ColArbiter=null;
                    var right:ZNPNode_ZPP_ColArbiter=null;
                    var nxt:ZNPNode_ZPP_ColArbiter=null;
                    var listSize=1;
                    var numMerges:Int,leftSize:Int,rightSize:Int;
                    do{
                        numMerges=0;
                        left=head;
                        tail=head=null;
                        while(left!=null){
                            numMerges++;
                            right=left;
                            leftSize=0;
                            rightSize=listSize;
                            while(right!=null&&leftSize<listSize){
                                leftSize++;
                                right=right.next;
                            }
                            while(leftSize>0||(rightSize>0&&right!=null)){
                                if(leftSize==0){
                                    nxt=right;
                                    right=right.next;
                                    rightSize--;
                                }
                                else if(rightSize==0||right==null){
                                    nxt=left;
                                    left=left.next;
                                    leftSize--;
                                }
                                else if({
                                    if(left.elem().active&&right.elem().active)left.elem().oc1.dist<right.elem().oc1.dist else true;
                                }){
                                    nxt=left;
                                    left=left.next;
                                    leftSize--;
                                }
                                else{
                                    nxt=right;
                                    right=right.next;
                                    rightSize--;
                                }
                                if(tail!=null)tail.next=nxt;
                                else head=nxt;
                                tail=nxt;
                            }
                            left=right;
                        }
                        tail.next=null;
                        listSize<<=1;
                    }
                    while(numMerges>1);
                    xxlist.setbegin(head);
                }
            };
        }
        #if NAPE_TIMES Debug.SORT+=flash.Lib.getTimer()-pt;
        #end
        updateVel(deltaTime);
        warmStart();
        #if NAPE_TIMES var pt=flash.Lib.getTimer();
        #end
        iterateVel(velocityIterations);
        #if NAPE_TIMES Debug.VEL+=flash.Lib.getTimer()-pt;
        #end
        {
            {
                var cx_ite=kinematics.begin();
                while(cx_ite!=null){
                    var cur=cx_ite.elem();
                    {
                        {
                            cur.pre_posx=cur.posx;
                            cur.pre_posy=cur.posy;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((cur.pre_posx!=cur.pre_posx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(cur.pre_posx)"+") :: "+("vec_set(in n: "+"cur.pre_pos"+",in x: "+"cur.posx"+",in y: "+"cur.posy"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((cur.pre_posy!=cur.pre_posy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(cur.pre_posy)"+") :: "+("vec_set(in n: "+"cur.pre_pos"+",in x: "+"cur.posx"+",in y: "+"cur.posy"+")");
                                #end
                            };
                        };
                        cur.pre_rot=cur.rot;
                    };
                    cx_ite=cx_ite.next;
                }
            };
        };
        {
            {
                var cx_ite=live.begin();
                while(cx_ite!=null){
                    var cur=cx_ite.elem();
                    {
                        {
                            cur.pre_posx=cur.posx;
                            cur.pre_posy=cur.posy;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((cur.pre_posx!=cur.pre_posx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(cur.pre_posx)"+") :: "+("vec_set(in n: "+"cur.pre_pos"+",in x: "+"cur.posx"+",in y: "+"cur.posy"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((cur.pre_posy!=cur.pre_posy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(cur.pre_posy)"+") :: "+("vec_set(in n: "+"cur.pre_pos"+",in x: "+"cur.posx"+",in y: "+"cur.posy"+")");
                                #end
                            };
                        };
                        cur.pre_rot=cur.rot;
                    };
                    cx_ite=cx_ite.next;
                }
            };
        };
        updatePos(deltaTime);
        continuous=true;
        continuousCollisions(deltaTime);
        continuous=false;
        #if NAPE_TIMES var pt=flash.Lib.getTimer();
        #end
        iteratePos(positionIterations);
        #if NAPE_TIMES Debug.POS+=flash.Lib.getTimer()-pt;
        #end
        {
            {
                var cx_ite=kinematics.begin();
                while(cx_ite!=null){
                    var cur=cx_ite.elem();
                    {
                        var upos=!(cur.posx==cur.pre_posx&&cur.posy==cur.pre_posy);
                        var urot=cur.pre_rot!=cur.rot;
                        if(upos)cur.invalidate_pos();
                        if(urot)cur.invalidate_rot();
                    };
                    cx_ite=cx_ite.next;
                }
            };
        };
        {
            {
                var cx_ite=live.begin();
                while(cx_ite!=null){
                    var cur=cx_ite.elem();
                    {
                        var upos=!(cur.posx==cur.pre_posx&&cur.posy==cur.pre_posy);
                        var urot=cur.pre_rot!=cur.rot;
                        if(upos)cur.invalidate_pos();
                        if(urot)cur.invalidate_rot();
                    };
                    cx_ite=cx_ite.next;
                }
            };
        };
        var pre=null;
        {
            var cx_ite=staticsleep.begin();
            while(cx_ite!=null){
                var b=cx_ite.elem();
                {
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !b.isDynamic();
                        };
                        if(!res)throw "assert("+"!b.isDynamic()"+") :: "+("step :: swef static/kinematics");
                        #end
                    };
                    if(!b.isKinematic()||(b.velx==0&&b.vely==0&&b.angvel==0)){
                        if(b.kinematicDelaySleep){
                            b.kinematicDelaySleep=false;
                            {
                                cx_ite=cx_ite.next;
                                continue;
                            };
                        }
                        b.component.sleeping=true;
                        cx_ite=staticsleep.inlined_erase(pre);
                        continue;
                    }
                    pre=cx_ite;
                };
                cx_ite=cx_ite.next;
            }
        };
        #if NAPE_TIMES var pt=flash.Lib.getTimer();
        #end
        doForests(deltaTime);
        sleepArbiters();
        #if NAPE_TIMES Debug.FOR+=flash.Lib.getTimer()-pt;
        #end
        midstep=false;
        var pre=null;
        {
            var cx_ite=callbackset_list.begin();
            while(cx_ite!=null){
                var set=cx_ite.elem();
                {
                    if(set.really_empty()){
                        cx_ite=callbackset_list.inlined_erase(pre);
                        var inf=set.int1.id+" "+set.int2.id;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !set.int1.cbsets.has(set);
                            };
                            if(!res)throw "assert("+"!set.int1.cbsets.has(set)"+") :: "+("freeing set that wasn't lazy del'ed "+inf);
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !set.int2.cbsets.has(set);
                            };
                            if(!res)throw "assert("+"!set.int2.cbsets.has(set)"+") :: "+("freeing set that wasn't lazy del'ed "+inf);
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                set.lazydel;
                            };
                            if(!res)throw "assert("+"set.lazydel"+") :: "+("freeing set not with lazydel true");
                            #end
                        };
                        {
                            var o=set;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    o!=null;
                                };
                                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_CallbackSet"+", in obj: "+"set"+")");
                                #end
                            };
                            o.free();
                            o.next=ZPP_CallbackSet.zpp_pool;
                            ZPP_CallbackSet.zpp_pool=o;
                            #if NAPE_POOL_STATS ZPP_CallbackSet.POOL_CNT++;
                            ZPP_CallbackSet.POOL_SUB++;
                            #end
                        };
                        continue;
                    }
                    var sleeping=set.sleeping();
                    ZPP_CbSet.find_all(set.int1.cbSet,set.int2.cbSet,ZPP_Flags.id_CbEvent_ONGOING,function(x:ZPP_InteractionListener){
                        if((!sleeping||x.allowSleepingCallbacks)&&!set.empty_arb(x.itype)){
                            var cb=push_callback(x);
                            cb.event=ZPP_Flags.id_CbEvent_ONGOING;
                            ZPP_Interactor.int_callback(set,x,cb);
                            cb.set=set;
                        }
                    });
                    pre=cx_ite;
                };
                cx_ite=cx_ite.next;
            }
        };
        while(!callbacks.empty()){
            var cb=callbacks.pop();
            if(cb.listener.type==ZPP_Flags.id_ListenerType_BODY){
                var o=cb.listener.body;
                o.handler(cb.wrapper_body());
            }
            else if(cb.listener.type==ZPP_Flags.id_ListenerType_CONSTRAINT){
                var o=cb.listener.constraint;
                o.handler(cb.wrapper_con());
            }
            else if(cb.listener.type==ZPP_Flags.id_ListenerType_INTERACTION){
                var o=cb.listener.interaction;
                o.handleri(cb.wrapper_int());
            }
            {
                var o=cb;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Callback"+", in obj: "+"cb"+")");
                    #end
                };
                o.free();
                o.next=ZPP_Callback.zpp_pool;
                ZPP_Callback.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_Callback.POOL_CNT++;
                ZPP_Callback.POOL_SUB++;
                #end
            };
        }
    }
    public var toiEvents:ZNPList_ZPP_ToiEvent=null;
    public function continuousCollisions(deltaTime:Float){
        var MAX_VEL=2*Math.PI/deltaTime;
        bphase.broadphase(this,false);
        var curTimeAlpha=0.0;
        while(curTimeAlpha<1&&!toiEvents.empty()){
            var minTOI:ZPP_ToiEvent=null;
            var minTime=2.0;
            var minKinematic=false;
            var preMin:ZNPNode_ZPP_ToiEvent=null;
            var pre:ZNPNode_ZPP_ToiEvent=null;
            {
                var cx_ite=toiEvents.begin();
                while(cx_ite!=null){
                    var toi=cx_ite.elem();
                    {
                        var b1=toi.s1.body;
                        var b2=toi.s2.body;
                        if(b1.sweepFrozen&&b2.sweepFrozen){
                            if(toi.toi!=0&&ZPP_Collide.testCollide_safe(toi.s1,toi.s2)){
                                toi.toi=0;
                            }
                            else{
                                cx_ite=toiEvents.erase(pre);
                                {
                                    var o=toi;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o!=null;
                                        };
                                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_ToiEvent"+", in obj: "+"toi"+")");
                                        #end
                                    };
                                    o.free();
                                    o.next=ZPP_ToiEvent.zpp_pool;
                                    ZPP_ToiEvent.zpp_pool=o;
                                    #if NAPE_POOL_STATS ZPP_ToiEvent.POOL_CNT++;
                                    ZPP_ToiEvent.POOL_SUB++;
                                    #end
                                };
                                continue;
                            }
                        }
                        if((toi.frozen1!=b1.sweepFrozen||toi.frozen2!=b2.sweepFrozen)){
                            if(!toi.kinematic){
                                toi.frozen1=b1.sweepFrozen;
                                toi.frozen2=b2.sweepFrozen;
                                if(toi.frozen1){
                                    var tmp=toi.s1;
                                    toi.s1=toi.s2;
                                    toi.s2=tmp;
                                    toi.frozen1=false;
                                    toi.frozen2=true;
                                }
                                ZPP_SweepDistance.staticSweep(toi,deltaTime,0,Config.collisionSlopCCD);
                                if(toi.toi<0){
                                    cx_ite=toiEvents.erase(pre);
                                    {
                                        var o=toi;
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                o!=null;
                                            };
                                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_ToiEvent"+", in obj: "+"toi"+")");
                                            #end
                                        };
                                        o.free();
                                        o.next=ZPP_ToiEvent.zpp_pool;
                                        ZPP_ToiEvent.zpp_pool=o;
                                        #if NAPE_POOL_STATS ZPP_ToiEvent.POOL_CNT++;
                                        ZPP_ToiEvent.POOL_SUB++;
                                        #end
                                    };
                                    continue;
                                }
                            }
                            else{
                                cx_ite=toiEvents.erase(pre);
                                {
                                    var o=toi;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o!=null;
                                        };
                                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_ToiEvent"+", in obj: "+"toi"+")");
                                        #end
                                    };
                                    o.free();
                                    o.next=ZPP_ToiEvent.zpp_pool;
                                    ZPP_ToiEvent.zpp_pool=o;
                                    #if NAPE_POOL_STATS ZPP_ToiEvent.POOL_CNT++;
                                    ZPP_ToiEvent.POOL_SUB++;
                                    #end
                                };
                                continue;
                            }
                        }
                        if(toi.toi>=0&&(toi.toi<minTime||(!minKinematic&&toi.kinematic))){
                            minTOI=toi;
                            minTime=toi.toi;
                            minKinematic=toi.kinematic;
                            preMin=pre;
                        }
                        pre=cx_ite;
                    };
                    cx_ite=cx_ite.next;
                }
            };
            if(minTOI==null){
                break;
            }
            toiEvents.erase(preMin);
            curTimeAlpha=minTOI.toi;
            var b1=minTOI.s1.body;
            var b2=minTOI.s2.body;
            if(!b1.sweepFrozen){
                b1.sweepIntegrate(curTimeAlpha*deltaTime);
                b1.sweepValidate(minTOI.s1);
            }
            if(!b2.sweepFrozen){
                b2.sweepIntegrate(curTimeAlpha*deltaTime);
                b2.sweepValidate(minTOI.s2);
            }
            var wasnull=minTOI.arbiter==null;
            var arb=narrowPhase(minTOI.s1,minTOI.s2,true,minTOI.arbiter,true);
            if(arb==null){
                if(minTOI.arbiter!=null&&minTOI.arbiter.pair!=null){
                    minTOI.arbiter.pair.arb=null;
                    minTOI.arbiter.pair=null;
                }
            }
            else{
                if(!presteparb(arb,deltaTime,true)){
                    if(arb.type==ZPP_Arbiter.COL&&arb.acting()){
                        arb.colarb.warmStart();
                        arb.colarb.applyImpulseVel();
                        arb.colarb.applyImpulseVel();
                        arb.colarb.applyImpulseVel();
                        arb.colarb.applyImpulseVel();
                        b1.sweep_angvel=(b1.angvel)%MAX_VEL;
                        b2.sweep_angvel=(b2.angvel)%MAX_VEL;
                    }
                }
            }
            if(arb!=null&&arb.acting()&&arb.type==ZPP_Arbiter.COL){
                if(!b1.sweepFrozen&&!b1.isKinematic()){
                    b1.sweepFrozen=true;
                    if(minTOI.failed)b1.angvel=b1.sweep_angvel=0;
                    else if(minTOI.slipped)b1.angvel=(b1.sweep_angvel*=Config.angularCCDSlipScale);
                    else b1.angvel=b1.sweep_angvel;
                }
                if(!b2.sweepFrozen&&!b2.isKinematic()){
                    b2.sweepFrozen=true;
                    if(minTOI.failed)b2.angvel=b2.sweep_angvel=0;
                    else if(minTOI.slipped)b2.angvel=(b2.sweep_angvel*=Config.angularCCDSlipScale);
                    else b2.angvel=b2.sweep_angvel;
                }
            }
        }
        {
            while(!toiEvents.empty()){
                var toi=toiEvents.pop_unsafe();
                {
                    {
                        var o=toi;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                o!=null;
                            };
                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_ToiEvent"+", in obj: "+"toi"+")");
                            #end
                        };
                        o.free();
                        o.next=ZPP_ToiEvent.zpp_pool;
                        ZPP_ToiEvent.zpp_pool=o;
                        #if NAPE_POOL_STATS ZPP_ToiEvent.POOL_CNT++;
                        ZPP_ToiEvent.POOL_SUB++;
                        #end
                    };
                };
            }
        };
        {
            var cx_ite=kinematics.begin();
            while(cx_ite!=null){
                var cur=cx_ite.elem();
                {
                    cur.sweepIntegrate(deltaTime);
                    cur.sweepTime=0;
                };
                cx_ite=cx_ite.next;
            }
        };
        {
            var cx_ite=live.begin();
            while(cx_ite!=null){
                var cur=cx_ite.elem();
                {
                    if(!cur.sweepFrozen){
                        cur.sweepIntegrate(deltaTime);
                    }
                    cur.sweepTime=0;
                };
                cx_ite=cx_ite.next;
            }
        };
    }
    public function continuousEvent(s1:ZPP_Shape,s2:ZPP_Shape,stat:Bool,in_arb:ZPP_Arbiter,_:Bool){
        if(s1.body.sweepFrozen&&s2.body.sweepFrozen)return in_arb;
        if(s1.body.disableCCD||s2.body.disableCCD)return in_arb;
        if((in_arb!=null&&in_arb.colarb==null)||interactionType(s1,s2,s1.body,s2.body)<=0){
            return in_arb;
        }
        var b1=s1.body;
        var b2=s2.body;
        if(stat||b1.bullet||b2.bullet){
            var toi;
            {
                if(ZPP_ToiEvent.zpp_pool==null){
                    toi=new ZPP_ToiEvent();
                    #if NAPE_POOL_STATS ZPP_ToiEvent.POOL_TOT++;
                    ZPP_ToiEvent.POOL_ADDNEW++;
                    #end
                }
                else{
                    toi=ZPP_ToiEvent.zpp_pool;
                    ZPP_ToiEvent.zpp_pool=toi.next;
                    toi.next=null;
                    #if NAPE_POOL_STATS ZPP_ToiEvent.POOL_CNT--;
                    ZPP_ToiEvent.POOL_ADD++;
                    #end
                }
                toi.alloc();
            };
            var kin=(b1.isKinematic()||b2.isKinematic());
            if(stat&&!kin){
                if(!s1.body.isDynamic()){
                    toi.s2=s1;
                    toi.s1=s2;
                }
                else{
                    toi.s1=s1;
                    toi.s2=s2;
                }
                toi.kinematic=false;
                ZPP_SweepDistance.staticSweep(toi,pre_dt,0,Config.collisionSlopCCD);
            }
            else{
                toi.s1=s1;
                toi.s2=s2;
                toi.kinematic=kin;
                if(toi.s1.body.sweepFrozen||toi.s2.body.sweepFrozen){
                    if(toi.s1.body.sweepFrozen){
                        var tmp=toi.s1;
                        toi.s1=toi.s2;
                        toi.s2=tmp;
                        toi.frozen1=false;
                        toi.frozen2=true;
                    }
                    ZPP_SweepDistance.staticSweep(toi,pre_dt,0,Config.collisionSlopCCD);
                }
                else{
                    ZPP_SweepDistance.dynamicSweep(toi,pre_dt,0,Config.collisionSlopCCD);
                }
            }
            if((stat&&toi.toi<0)||toi.failed){
                {
                    var o=toi;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            o!=null;
                        };
                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_ToiEvent"+", in obj: "+"toi"+")");
                        #end
                    };
                    o.free();
                    o.next=ZPP_ToiEvent.zpp_pool;
                    ZPP_ToiEvent.zpp_pool=o;
                    #if NAPE_POOL_STATS ZPP_ToiEvent.POOL_CNT++;
                    ZPP_ToiEvent.POOL_SUB++;
                    #end
                };
            }
            else{
                toiEvents.add(toi);
                toi.frozen1=toi.s1.body.sweepFrozen;
                toi.frozen2=toi.s2.body.sweepFrozen;
                toi.arbiter=(in_arb!=null)?in_arb.colarb:null;
            }
        }
        return in_arb;
    }
    public function bodyCbWake(b:ZPP_Body){
        if(b.isDynamic()&&b.cbSet!=null){
            if(midstep){
                {
                    var cx_ite=b.cbSet.bodylisteners.begin();
                    while(cx_ite!=null){
                        var i=cx_ite.elem();
                        {
                            if(i.event!=ZPP_Flags.id_CbEvent_WAKE){
                                cx_ite=cx_ite.next;
                                continue;
                            };
                            var cb=push_callback(i);
                            cb.event=ZPP_Flags.id_CbEvent_WAKE;
                            cb.body=b;
                        };
                        cx_ite=cx_ite.next;
                    }
                };
            }
            else b.component.woken=true;
        }
    }
    public function bodyCbSleep(b:ZPP_Body){
        if(b.isDynamic()&&b.cbSet!=null){
            {
                var cx_ite=b.cbSet.bodylisteners.begin();
                while(cx_ite!=null){
                    var i=cx_ite.elem();
                    {
                        if(i.event!=ZPP_Flags.id_CbEvent_SLEEP){
                            cx_ite=cx_ite.next;
                            continue;
                        };
                        var cb=push_callback(i);
                        cb.event=ZPP_Flags.id_CbEvent_SLEEP;
                        cb.body=b;
                    };
                    cx_ite=cx_ite.next;
                }
            };
        }
    }
    public function constraintCbWake(con:ZPP_Constraint){
        if(con.cbSet!=null){
            if(midstep){
                {
                    var cx_ite=con.cbSet.conlisteners.begin();
                    while(cx_ite!=null){
                        var i=cx_ite.elem();
                        {
                            if(i.event!=ZPP_Flags.id_CbEvent_WAKE){
                                cx_ite=cx_ite.next;
                                continue;
                            };
                            var cb=push_callback(i);
                            cb.event=ZPP_Flags.id_CbEvent_WAKE;
                            cb.constraint=con;
                        };
                        cx_ite=cx_ite.next;
                    }
                };
            }
            else con.component.woken=true;
        }
    }
    public function constraintCbSleep(con:ZPP_Constraint){
        if(con.cbSet!=null){
            {
                var cx_ite=con.cbSet.conlisteners.begin();
                while(cx_ite!=null){
                    var i=cx_ite.elem();
                    {
                        if(i.event!=ZPP_Flags.id_CbEvent_SLEEP){
                            cx_ite=cx_ite.next;
                            continue;
                        };
                        var cb=push_callback(i);
                        cb.event=ZPP_Flags.id_CbEvent_SLEEP;
                        cb.constraint=con;
                    };
                    cx_ite=cx_ite.next;
                }
            };
        }
    }
    public function constraintCbBreak(con:ZPP_Constraint){
        if(con.cbSet!=null){
            {
                var cx_ite=con.cbSet.conlisteners.begin();
                while(cx_ite!=null){
                    var i=cx_ite.elem();
                    {
                        if(i.event!=ZPP_Flags.id_CbEvent_BREAK){
                            cx_ite=cx_ite.next;
                            continue;
                        };
                        var cb=push_callback(i);
                        cb.event=ZPP_Flags.id_CbEvent_BREAK;
                        cb.constraint=con;
                    };
                    cx_ite=cx_ite.next;
                }
            };
        }
    }
    public function nullListenerType(cb1:ZPP_CbSet,cb2:ZPP_CbSet){
        var stack=new ZNPList_ZPP_Interactor();
        {
            var cx_ite=cb1.interactors.begin();
            while(cx_ite!=null){
                var i=cx_ite.elem();
                stack.add(i);
                cx_ite=cx_ite.next;
            }
        };
        if(cb1!=cb2){
            var cx_ite=cb2.interactors.begin();
            while(cx_ite!=null){
                var i=cx_ite.elem();
                stack.add(i);
                cx_ite=cx_ite.next;
            }
        };
        while(!stack.empty()){
            var intx=stack.pop_unsafe();
            if(intx.isCompound()){
                var comp=intx.icompound;
                {
                    var cx_ite=comp.bodies.begin();
                    while(cx_ite!=null){
                        var i=cx_ite.elem();
                        stack.add(i);
                        cx_ite=cx_ite.next;
                    }
                };
                {
                    var cx_ite=comp.compounds.begin();
                    while(cx_ite!=null){
                        var i=cx_ite.elem();
                        stack.add(i);
                        cx_ite=cx_ite.next;
                    }
                };
            }
            else{
                var xbody=if(intx.isBody())intx.ibody else intx.ishape.body;
                var xshp=if(intx.isShape())intx.ishape else null;
                {
                    var cx_ite=xbody.arbiters.begin();
                    while(cx_ite!=null){
                        var xarb=cx_ite.elem();
                        {
                            if(xarb.present==0){
                                cx_ite=cx_ite.next;
                                continue;
                            };
                            if(xshp!=null&&!(xarb.ws1==xshp||xarb.ws2==xshp)){
                                cx_ite=cx_ite.next;
                                continue;
                            };
                            MRCA_chains(xarb.ws1,xarb.ws2);
                            {
                                var cx_ite=mrca1.begin();
                                while(cx_ite!=null){
                                    var i1=cx_ite.elem();
                                    {
                                        if(i1.cbSet!=cb1&&i1.cbSet!=cb2){
                                            cx_ite=cx_ite.next;
                                            continue;
                                        };
                                        {
                                            var cx_ite=mrca2.begin();
                                            while(cx_ite!=null){
                                                var i2=cx_ite.elem();
                                                {
                                                    if((i1.cbSet==cb1&&i2.cbSet!=cb2)||(i1.cbSet==cb2&&i2.cbSet!=cb1)){
                                                        cx_ite=cx_ite.next;
                                                        continue;
                                                    };
                                                    var callbackset=ZPP_Interactor.get(i1,i2);
                                                    if(callbackset!=null){
                                                        {
                                                            while(!callbackset.arbiters.empty()){
                                                                var arb=callbackset.arbiters.pop_unsafe();
                                                                {
                                                                    arb.present--;
                                                                    {
                                                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                                        var res={
                                                                            arb.present>=0;
                                                                        };
                                                                        if(!res)throw "assert("+"arb.present>=0"+") :: "+("xarb present <0 ?");
                                                                        #end
                                                                    };
                                                                };
                                                            }
                                                        };
                                                        remove_callbackset(callbackset);
                                                    }
                                                };
                                                cx_ite=cx_ite.next;
                                            }
                                        };
                                    };
                                    cx_ite=cx_ite.next;
                                }
                            };
                        };
                        cx_ite=cx_ite.next;
                    }
                };
            }
        }
    }
    public function nullInteractorType(intx:ZPP_Interactor,me:ZPP_Interactor=null){
        if(me==null)me=intx;
        if(intx.isCompound()){
            var comp=intx.icompound;
            {
                var cx_ite=comp.bodies.begin();
                while(cx_ite!=null){
                    var body=cx_ite.elem();
                    nullInteractorType(body,me);
                    cx_ite=cx_ite.next;
                }
            };
            {
                var cx_ite=comp.compounds.begin();
                while(cx_ite!=null){
                    var comp=cx_ite.elem();
                    nullInteractorType(comp,me);
                    cx_ite=cx_ite.next;
                }
            };
        }
        else{
            var xbody=if(intx.isBody())intx.ibody else intx.ishape.body;
            var xshp=if(intx.isShape())intx.ishape else null;
            {
                var cx_ite=xbody.arbiters.begin();
                while(cx_ite!=null){
                    var xarb=cx_ite.elem();
                    {
                        if(xarb.present==0){
                            cx_ite=cx_ite.next;
                            continue;
                        };
                        if(xshp!=null&&!(xarb.ws1==xshp||xarb.ws2==xshp)){
                            cx_ite=cx_ite.next;
                            continue;
                        };
                        MRCA_chains(xarb.ws1,xarb.ws2);
                        {
                            var cx_ite=mrca1.begin();
                            while(cx_ite!=null){
                                var i1=cx_ite.elem();
                                {
                                    {
                                        var cx_ite=mrca2.begin();
                                        while(cx_ite!=null){
                                            var i2=cx_ite.elem();
                                            {
                                                if(i1!=me&&i2!=me){
                                                    cx_ite=cx_ite.next;
                                                    continue;
                                                };
                                                var callbackset=ZPP_Interactor.get(i1,i2);
                                                if(callbackset!=null){
                                                    xarb.present--;
                                                    {
                                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                        var res={
                                                            xarb.present>=0;
                                                        };
                                                        if(!res)throw "assert("+"xarb.present>=0"+") :: "+("xarb present <0?");
                                                        #end
                                                    };
                                                    callbackset.remove_arb(xarb);
                                                    if(callbackset.really_empty()){
                                                        remove_callbackset(callbackset);
                                                    }
                                                }
                                            };
                                            cx_ite=cx_ite.next;
                                        }
                                    };
                                };
                                cx_ite=cx_ite.next;
                            }
                        };
                    };
                    cx_ite=cx_ite.next;
                }
            };
        }
    }
    public function freshListenerType(cb1:ZPP_CbSet,cb2:ZPP_CbSet){
        var stack=new ZNPList_ZPP_Interactor();
        {
            var cx_ite=cb1.interactors.begin();
            while(cx_ite!=null){
                var i=cx_ite.elem();
                stack.add(i);
                cx_ite=cx_ite.next;
            }
        };
        if(cb1!=cb2){
            var cx_ite=cb2.interactors.begin();
            while(cx_ite!=null){
                var i=cx_ite.elem();
                stack.add(i);
                cx_ite=cx_ite.next;
            }
        };
        while(!stack.empty()){
            var intx=stack.pop_unsafe();
            if(intx.isCompound()){
                var comp=intx.icompound;
                {
                    var cx_ite=comp.bodies.begin();
                    while(cx_ite!=null){
                        var i=cx_ite.elem();
                        stack.add(i);
                        cx_ite=cx_ite.next;
                    }
                };
                {
                    var cx_ite=comp.compounds.begin();
                    while(cx_ite!=null){
                        var i=cx_ite.elem();
                        stack.add(i);
                        cx_ite=cx_ite.next;
                    }
                };
            }
            else{
                var xbody=if(intx.isBody())intx.ibody else intx.ishape.body;
                var xshp=if(intx.isShape())intx.ishape else null;
                {
                    var cx_ite=xbody.arbiters.begin();
                    while(cx_ite!=null){
                        var xarb=cx_ite.elem();
                        {
                            if(!xarb.presentable){
                                cx_ite=cx_ite.next;
                                continue;
                            };
                            if(xshp!=null&&!(xarb.ws1==xshp||xarb.ws2==xshp)){
                                cx_ite=cx_ite.next;
                                continue;
                            };
                            MRCA_chains(xarb.ws1,xarb.ws2);
                            {
                                var cx_ite=mrca1.begin();
                                while(cx_ite!=null){
                                    var i1=cx_ite.elem();
                                    {
                                        if(i1.cbSet!=cb1&&i1.cbSet!=cb2){
                                            cx_ite=cx_ite.next;
                                            continue;
                                        };
                                        {
                                            var cx_ite=mrca2.begin();
                                            while(cx_ite!=null){
                                                var i2=cx_ite.elem();
                                                {
                                                    if((i1.cbSet==cb1&&i2.cbSet!=cb2)||(i1.cbSet==cb2&&i2.cbSet!=cb1)){
                                                        cx_ite=cx_ite.next;
                                                        continue;
                                                    };
                                                    var callbackset=ZPP_Interactor.get(i1,i2);
                                                    if(callbackset==null){
                                                        callbackset=ZPP_CallbackSet.get(i1,i2);
                                                        add_callbackset(callbackset);
                                                    }
                                                    if(callbackset.add_arb(xarb)){
                                                        xarb.present++;
                                                    }
                                                };
                                                cx_ite=cx_ite.next;
                                            }
                                        };
                                    };
                                    cx_ite=cx_ite.next;
                                }
                            };
                        };
                        cx_ite=cx_ite.next;
                    }
                };
            }
        }
    }
    public function freshInteractorType(intx:ZPP_Interactor,me:ZPP_Interactor=null){
        if(me==null)me=intx;
        if(intx.isCompound()){
            var comp=intx.icompound;
            {
                var cx_ite=comp.bodies.begin();
                while(cx_ite!=null){
                    var body=cx_ite.elem();
                    freshInteractorType(body,me);
                    cx_ite=cx_ite.next;
                }
            };
            {
                var cx_ite=comp.compounds.begin();
                while(cx_ite!=null){
                    var comp=cx_ite.elem();
                    freshInteractorType(comp,me);
                    cx_ite=cx_ite.next;
                }
            };
        }
        else{
            var xbody=if(intx.isBody())intx.ibody else intx.ishape.body;
            var xshp=if(intx.isShape())intx.ishape else null;
            {
                var cx_ite=xbody.arbiters.begin();
                while(cx_ite!=null){
                    var xarb=cx_ite.elem();
                    {
                        if(!xarb.presentable){
                            cx_ite=cx_ite.next;
                            continue;
                        };
                        if(xshp!=null&&!(xarb.ws1==xshp||xarb.ws2==xshp)){
                            cx_ite=cx_ite.next;
                            continue;
                        };
                        MRCA_chains(xarb.ws1,xarb.ws2);
                        {
                            var cx_ite=mrca1.begin();
                            while(cx_ite!=null){
                                var i1=cx_ite.elem();
                                {
                                    {
                                        var cx_ite=mrca2.begin();
                                        while(cx_ite!=null){
                                            var i2=cx_ite.elem();
                                            {
                                                if(i1!=me&&i2!=me){
                                                    cx_ite=cx_ite.next;
                                                    continue;
                                                };
                                                var cb1=i1.cbSet;
                                                var cb2=i2.cbSet;
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        cb1!=null&&cb2!=null;
                                                    };
                                                    if(!res)throw "assert("+"cb1!=null&&cb2!=null"+") :: "+("null cbset from MRCA_chain?");
                                                    #end
                                                };
                                                cb1.validate();
                                                cb2.validate();
                                                if(!ZPP_CbSet.empty_intersection(cb1,cb2)){
                                                    var callbackset=ZPP_Interactor.get(i1,i2);
                                                    if(callbackset==null){
                                                        callbackset=ZPP_CallbackSet.get(i1,i2);
                                                        add_callbackset(callbackset);
                                                    }
                                                    if(callbackset.add_arb(xarb)){
                                                        xarb.present++;
                                                    }
                                                }
                                            };
                                            cx_ite=cx_ite.next;
                                        }
                                    };
                                };
                                cx_ite=cx_ite.next;
                            }
                        };
                    };
                    cx_ite=cx_ite.next;
                }
            };
        }
    }
    public function wakeCompound(x:ZPP_Compound){
        {
            var cx_ite=x.bodies.begin();
            while(cx_ite!=null){
                var y=cx_ite.elem();
                {
                    var o=y;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            o.space==this;
                        };
                        if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                        #end
                    };
                    if(!o.world){
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                o.component!=null;
                            };
                            if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                            #end
                        };
                        o.component.waket=stamp+(midstep?0:1);
                        if(o.isKinematic())o.kinematicDelaySleep=true;
                        if(o.component.sleeping){
                            really_wake(o,false);
                        }
                    }
                };
                cx_ite=cx_ite.next;
            }
        };
        {
            var cx_ite=x.constraints.begin();
            while(cx_ite!=null){
                var i=cx_ite.elem();
                wake_constraint(i);
                cx_ite=cx_ite.next;
            }
        };
        {
            var cx_ite=x.compounds.begin();
            while(cx_ite!=null){
                var i=cx_ite.elem();
                wakeCompound(i);
                cx_ite=cx_ite.next;
            }
        };
    }
    public function wakeIsland(i:ZPP_Island){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                i.sleep;
            };
            if(!res)throw "assert("+"i.sleep"+") :: "+("this island is not sleeping but referenced? wtf");
            #end
        };
        while(!i.comps.empty()){
            var c=i.comps.pop_unsafe();
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    c.sleeping;
                };
                if(!res)throw "assert("+"c.sleeping"+") :: "+("???");
                #end
            };
            c.waket=stamp+(midstep?0:1);
            if(c.isBody){
                var b=c.body;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        b.space==this;
                    };
                    if(!res)throw "assert("+"b.space==this"+") :: "+("wakeIsland:: body is not actually IN this space??");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        b.isDynamic();
                    };
                    if(!res)throw "assert("+"b.isDynamic()"+") :: "+("din din");
                    #end
                };
                live.add(b);
                {
                    var cx_ite=b.arbiters.begin();
                    while(cx_ite!=null){
                        var arb=cx_ite.elem();
                        {
                            if(arb.sleeping){
                                arb.sleeping=false;
                                arb.up_stamp+=stamp-arb.sleep_stamp;
                                if(arb.type==ZPP_Arbiter.COL){
                                    var carb=arb.colarb;
                                    if(carb.stat)c_arbiters_true.inlined_add(carb);
                                    else c_arbiters_false.inlined_add(carb);
                                }
                                else if(arb.type==ZPP_Arbiter.FLUID)f_arbiters.inlined_add(arb.fluidarb);
                                else s_arbiters.inlined_add(arb.sensorarb);
                            }
                        };
                        cx_ite=cx_ite.next;
                    }
                };
                bodyCbWake(b);
                c.reset();
                if(!b.isStatic()){
                    var cx_ite=b.shapes.begin();
                    while(cx_ite!=null){
                        var shape=cx_ite.elem();
                        if(shape.node!=null)bphase.sync(shape);
                        cx_ite=cx_ite.next;
                    }
                };
            }
            else{
                var con=c.constraint;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        con.space==this;
                    };
                    if(!res)throw "assert("+"con.space==this"+") :: "+("wakeIsland:: constraint is not actually IN this space??");
                    #end
                };
                live_constraints.inlined_add(con);
                constraintCbWake(con);
                c.reset();
            }
        }
        {
            var o=i;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Island"+", in obj: "+"i"+")");
                #end
            };
            o.free();
            o.next=ZPP_Island.zpp_pool;
            ZPP_Island.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_Island.POOL_CNT++;
            ZPP_Island.POOL_SUB++;
            #end
        };
    }
    public function non_inlined_wake(o:ZPP_Body,fst=false){
        {
            var o=o;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o.space==this;
                };
                if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                #end
            };
            if(!o.world){
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o.component!=null;
                    };
                    if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                    #end
                };
                o.component.waket=stamp+(midstep?0:1);
                if(o.isKinematic())o.kinematicDelaySleep=true;
                if(o.component.sleeping){
                    really_wake(o,fst);
                }
            }
        };
    }
    public function really_wake(o:ZPP_Body,fst=false){
        if(o.component.island==null){
            o.component.sleeping=false;
            if(o.isKinematic()||o.isStatic())staticsleep.inlined_add(o);
            else live.inlined_add(o);
            {
                var cx_ite=o.constraints.begin();
                while(cx_ite!=null){
                    var con=cx_ite.elem();
                    if(con.space==this)wake_constraint(con);
                    cx_ite=cx_ite.next;
                }
            };
            {
                var cx_ite=o.arbiters.begin();
                while(cx_ite!=null){
                    var arb=cx_ite.elem();
                    {
                        if(arb.sleeping){
                            arb.sleeping=false;
                            arb.up_stamp+=stamp+(midstep?0:1)-arb.sleep_stamp;
                            if(arb.type==ZPP_Arbiter.COL){
                                var carb=arb.colarb;
                                if(carb.stat)c_arbiters_true.inlined_add(carb);
                                else c_arbiters_false.inlined_add(carb);
                            }
                            else if(arb.type==ZPP_Arbiter.FLUID)f_arbiters.inlined_add(arb.fluidarb);
                            else s_arbiters.inlined_add(arb.sensorarb);
                        }
                        if(arb.type!=ZPP_Arbiter.SENSOR&&!arb.cleared&&arb.up_stamp>=stamp&&((arb.immState&ZPP_Flags.id_ImmState_ACCEPT)!=0)){
                            if(arb.b1.isDynamic()&&arb.b1.component.sleeping){
                                var o=arb.b1;
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        o.space==this;
                                    };
                                    if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                    #end
                                };
                                if(!o.world){
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o.component!=null;
                                        };
                                        if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                        #end
                                    };
                                    o.component.waket=stamp+(midstep?0:1);
                                    if(o.isKinematic())o.kinematicDelaySleep=true;
                                    if(o.component.sleeping){
                                        really_wake(o,false);
                                    }
                                }
                            };
                            if(arb.b2.isDynamic()&&arb.b2.component.sleeping){
                                var o=arb.b2;
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        o.space==this;
                                    };
                                    if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                    #end
                                };
                                if(!o.world){
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o.component!=null;
                                        };
                                        if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                        #end
                                    };
                                    o.component.waket=stamp+(midstep?0:1);
                                    if(o.isKinematic())o.kinematicDelaySleep=true;
                                    if(o.component.sleeping){
                                        really_wake(o,false);
                                    }
                                }
                            };
                        }
                    };
                    cx_ite=cx_ite.next;
                }
            };
            if(!fst&&o.isDynamic())bodyCbWake(o);
            if(!fst&&!bphase.is_sweep&&!o.isStatic()){
                var cx_ite=o.shapes.begin();
                while(cx_ite!=null){
                    var shape=cx_ite.elem();
                    if(shape.node!=null)bphase.sync(shape);
                    cx_ite=cx_ite.next;
                }
            };
        }
        else{
            wakeIsland(o.component.island);
        }
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o.component.island==null;
            };
            if(!res)throw "assert("+"o.component.island==null"+") :: "+("woken, but island non-null?");
            #end
        };
    }
    public function wake_constraint(con:ZPP_Constraint,fst=false){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                con.space==this;
            };
            if(!res)throw "assert("+"con.space==this"+") :: "+("constraint woken, but not actually IN the space!");
            #end
        };
        if(con.active){
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    con.component!=null;
                };
                if(!res)throw "assert("+"con.component!=null"+") :: "+("constraint woken but no component exists?");
                #end
            };
            con.component.waket=stamp+(midstep?0:1);
            if(con.component.sleeping){
                if(con.component.island==null){
                    con.component.sleeping=false;
                    live_constraints.inlined_add(con);
                    con.wake_connected();
                    if(!fst)constraintCbWake(con);
                }
                else{
                    wakeIsland(con.component.island);
                }
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        con.component.island==null;
                    };
                    if(!res)throw "assert("+"con.component.island==null"+") :: "+("woken (con), but island non-null?");
                    #end
                };
                return true;
            }
            else return false;
        }
        else return false;
    }
    public function doForests(dt:Float){
        {
            var cx_ite=c_arbiters_false.begin();
            while(cx_ite!=null){
                var arb=cx_ite.elem();
                {
                    if(!arb.cleared&&arb.up_stamp==stamp&&((arb.immState&ZPP_Flags.id_ImmState_ACCEPT)!=0)){
                        if(arb.b1.isDynamic()&&arb.b2.isDynamic()){
                            var xr=({
                                if(arb.b1.component==arb.b1.component.parent)arb.b1.component;
                                else{
                                    var obj=arb.b1.component;
                                    var stack=null;
                                    while(obj!=obj.parent){
                                        var nxt=obj.parent;
                                        obj.parent=stack;
                                        stack=obj;
                                        obj=nxt;
                                    }
                                    while(stack!=null){
                                        var nxt=stack.parent;
                                        stack.parent=obj;
                                        stack=nxt;
                                    }
                                    obj;
                                }
                            });
                            var yr=({
                                if(arb.b2.component==arb.b2.component.parent)arb.b2.component;
                                else{
                                    var obj=arb.b2.component;
                                    var stack=null;
                                    while(obj!=obj.parent){
                                        var nxt=obj.parent;
                                        obj.parent=stack;
                                        stack=obj;
                                        obj=nxt;
                                    }
                                    while(stack!=null){
                                        var nxt=stack.parent;
                                        stack.parent=obj;
                                        stack=nxt;
                                    }
                                    obj;
                                }
                            });
                            if(xr!=yr){
                                if(xr.rank<yr.rank)xr.parent=yr;
                                else if(xr.rank>yr.rank)yr.parent=xr;
                                else{
                                    yr.parent=xr;
                                    xr.rank++;
                                }
                            }
                        }
                    }
                };
                cx_ite=cx_ite.next;
            }
        };
        {
            var cx_ite=f_arbiters.begin();
            while(cx_ite!=null){
                var arb=cx_ite.elem();
                {
                    if(!arb.cleared&&arb.up_stamp==stamp&&((arb.immState&ZPP_Flags.id_ImmState_ACCEPT)!=0)){
                        if(arb.b1.isDynamic()&&arb.b2.isDynamic()){
                            var xr=({
                                if(arb.b1.component==arb.b1.component.parent)arb.b1.component;
                                else{
                                    var obj=arb.b1.component;
                                    var stack=null;
                                    while(obj!=obj.parent){
                                        var nxt=obj.parent;
                                        obj.parent=stack;
                                        stack=obj;
                                        obj=nxt;
                                    }
                                    while(stack!=null){
                                        var nxt=stack.parent;
                                        stack.parent=obj;
                                        stack=nxt;
                                    }
                                    obj;
                                }
                            });
                            var yr=({
                                if(arb.b2.component==arb.b2.component.parent)arb.b2.component;
                                else{
                                    var obj=arb.b2.component;
                                    var stack=null;
                                    while(obj!=obj.parent){
                                        var nxt=obj.parent;
                                        obj.parent=stack;
                                        stack=obj;
                                        obj=nxt;
                                    }
                                    while(stack!=null){
                                        var nxt=stack.parent;
                                        stack.parent=obj;
                                        stack=nxt;
                                    }
                                    obj;
                                }
                            });
                            if(xr!=yr){
                                if(xr.rank<yr.rank)xr.parent=yr;
                                else if(xr.rank>yr.rank)yr.parent=xr;
                                else{
                                    yr.parent=xr;
                                    xr.rank++;
                                }
                            }
                        }
                    }
                };
                cx_ite=cx_ite.next;
            }
        };
        {
            var cx_ite=live_constraints.begin();
            while(cx_ite!=null){
                var con=cx_ite.elem();
                con.forest();
                cx_ite=cx_ite.next;
            }
        };
        while(!live.empty()){
            var o=live.inlined_pop_unsafe();
            var oc=o.component;
            var root=({
                if(oc==oc.parent)oc;
                else{
                    var obj=oc;
                    var stack=null;
                    while(obj!=obj.parent){
                        var nxt=obj.parent;
                        obj.parent=stack;
                        stack=obj;
                        obj=nxt;
                    }
                    while(stack!=null){
                        var nxt=stack.parent;
                        stack.parent=obj;
                        stack=nxt;
                    }
                    obj;
                }
            });
            if(root.island==null){
                {
                    if(ZPP_Island.zpp_pool==null){
                        root.island=new ZPP_Island();
                        #if NAPE_POOL_STATS ZPP_Island.POOL_TOT++;
                        ZPP_Island.POOL_ADDNEW++;
                        #end
                    }
                    else{
                        root.island=ZPP_Island.zpp_pool;
                        ZPP_Island.zpp_pool=root.island.next;
                        root.island.next=null;
                        #if NAPE_POOL_STATS ZPP_Island.POOL_CNT--;
                        ZPP_Island.POOL_ADD++;
                        #end
                    }
                    root.island.alloc();
                };
                islands.inlined_add(root.island);
                root.island.sleep=true;
            }
            oc.island=root.island;
            oc.island.comps.inlined_add(oc);
            var rest=o.atRest(dt);
            oc.island.sleep=oc.island.sleep&&rest;
            if(oc.waket>oc.island.waket)oc.island.waket=oc.waket;
        }
        while(!live_constraints.empty()){
            var o=live_constraints.inlined_pop_unsafe();
            var oc=o.component;
            var root=({
                if(oc==oc.parent)oc;
                else{
                    var obj=oc;
                    var stack=null;
                    while(obj!=obj.parent){
                        var nxt=obj.parent;
                        obj.parent=stack;
                        stack=obj;
                        obj=nxt;
                    }
                    while(stack!=null){
                        var nxt=stack.parent;
                        stack.parent=obj;
                        stack=nxt;
                    }
                    obj;
                }
            });
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    root.island!=null;
                };
                if(!res)throw "assert("+"root.island!=null"+") :: "+("constraint without bodies?");
                #end
            };
            oc.island=root.island;
            oc.island.comps.inlined_add(oc);
            if(oc.waket>oc.island.waket)oc.island.waket=oc.waket;
        }
        while(!islands.empty()){
            var i=islands.inlined_pop_unsafe();
            if(i.sleep){
                {
                    var cx_ite=i.comps.begin();
                    while(cx_ite!=null){
                        var c=cx_ite.elem();
                        {
                            if(c.isBody){
                                var b=c.body;
                                {
                                    b.velx=0;
                                    b.vely=0;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((b.velx!=b.velx));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(b.velx)"+") :: "+("vec_set(in n: "+"b.vel"+",in x: "+"0"+",in y: "+"0"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((b.vely!=b.vely));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(b.vely)"+") :: "+("vec_set(in n: "+"b.vel"+",in x: "+"0"+",in y: "+"0"+")");
                                        #end
                                    };
                                };
                                b.angvel=0;
                                c.sleeping=true;
                                {
                                    var cx_ite=b.shapes.begin();
                                    while(cx_ite!=null){
                                        var shape=cx_ite.elem();
                                        bphase.sync(shape);
                                        cx_ite=cx_ite.next;
                                    }
                                }
                                bodyCbSleep(b);
                            }
                            else{
                                var con=c.constraint;
                                constraintCbSleep(con);
                                c.sleeping=true;
                            }
                        };
                        cx_ite=cx_ite.next;
                    }
                };
            }
            else{
                while(!i.comps.empty()){
                    var c=i.comps.inlined_pop_unsafe();
                    c.waket=i.waket;
                    if(c.isBody)live.inlined_add(c.body);
                    else live_constraints.inlined_add(c.constraint);
                    c.reset();
                }
                {
                    var o=i;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            o!=null;
                        };
                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Island"+", in obj: "+"i"+")");
                        #end
                    };
                    o.free();
                    o.next=ZPP_Island.zpp_pool;
                    ZPP_Island.zpp_pool=o;
                    #if NAPE_POOL_STATS ZPP_Island.POOL_CNT++;
                    ZPP_Island.POOL_SUB++;
                    #end
                };
            }
        }
    }
    public function sleepArbiters(){
        {
            var pre=null;
            var arbs=c_arbiters_true;
            var arbite=arbs.begin();
            var fst=c_arbiters_false!=null;
            {
                if(fst&&arbite==null){
                    fst=false;
                    arbite=c_arbiters_false.begin();
                    arbs=c_arbiters_false;
                    pre=null;
                }
            };
            while(arbite!=null){
                var arb=arbite.elem();
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !arb.cleared;
                    };
                    if(!res)throw "assert("+"!arb.cleared"+") :: "+("arb cleared in sleepArbiters");
                    #end
                };
                if(arb.b1.component.sleeping&&arb.b2.component.sleeping){
                    arb.sleep_stamp=stamp;
                    arb.sleeping=true;
                    arbite=arbs.inlined_erase(pre);
                    {
                        if(fst&&arbite==null){
                            fst=false;
                            arbite=c_arbiters_false.begin();
                            arbs=c_arbiters_false;
                            pre=null;
                        }
                    };
                    continue;
                }
                pre=arbite;
                arbite=arbite.next;
                {
                    if(fst&&arbite==null){
                        fst=false;
                        arbite=c_arbiters_false.begin();
                        arbs=c_arbiters_false;
                        pre=null;
                    }
                };
            }
        };
        {
            var pre=null;
            var arbs=f_arbiters;
            var arbite=arbs.begin();
            var fst=null!=null;
            {
                if(fst&&arbite==null){
                    fst=false;
                    arbite=null.begin();
                    arbs=null;
                    pre=null;
                }
            };
            while(arbite!=null){
                var arb=arbite.elem();
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !arb.cleared;
                    };
                    if(!res)throw "assert("+"!arb.cleared"+") :: "+("arb cleared in sleepArbiters");
                    #end
                };
                if(arb.b1.component.sleeping&&arb.b2.component.sleeping){
                    arb.sleep_stamp=stamp;
                    arb.sleeping=true;
                    arbite=arbs.inlined_erase(pre);
                    {
                        if(fst&&arbite==null){
                            fst=false;
                            arbite=null.begin();
                            arbs=null;
                            pre=null;
                        }
                    };
                    continue;
                }
                pre=arbite;
                arbite=arbite.next;
                {
                    if(fst&&arbite==null){
                        fst=false;
                        arbite=null.begin();
                        arbs=null;
                        pre=null;
                    }
                };
            }
        };
        {
            var pre=null;
            var arbs=s_arbiters;
            var arbite=arbs.begin();
            var fst=null!=null;
            {
                if(fst&&arbite==null){
                    fst=false;
                    arbite=null.begin();
                    arbs=null;
                    pre=null;
                }
            };
            while(arbite!=null){
                var arb=arbite.elem();
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !arb.cleared;
                    };
                    if(!res)throw "assert("+"!arb.cleared"+") :: "+("arb cleared in sleepArbiters");
                    #end
                };
                if(arb.b1.component.sleeping&&arb.b2.component.sleeping){
                    arb.sleep_stamp=stamp;
                    arb.sleeping=true;
                    arbite=arbs.inlined_erase(pre);
                    {
                        if(fst&&arbite==null){
                            fst=false;
                            arbite=null.begin();
                            arbs=null;
                            pre=null;
                        }
                    };
                    continue;
                }
                pre=arbite;
                arbite=arbite.next;
                {
                    if(fst&&arbite==null){
                        fst=false;
                        arbite=null.begin();
                        arbs=null;
                        pre=null;
                    }
                };
            }
        };
    }
    public function static_validation(body:ZPP_Body){
        if(!body.shapes.empty()){
            body.validate_aabb();
        }
        body.validate_mass();
        body.validate_inertia();
        #if(!NAPE_RELEASE_BUILD)
        if(body.velx!=0||body.vely!=0||body.angvel!=0)throw "Error: Static body cannot have any real velocity, only kinematic or surface velocities";
        #end
        {
            var cx_ite=body.shapes.begin();
            while(cx_ite!=null){
                var s=cx_ite.elem();
                {
                    if(s.isPolygon()){
                        s.polygon.splice_collinear();
                        #if(!NAPE_RELEASE_BUILD)
                        var res=s.polygon.valid();
                        if(res!=ValidationResult.VALID){
                            throw "Error: Cannot simulate with an invalid Polygon : "+s.polygon.outer.toString()+" is invalid : "+res.toString();
                        }
                        #end
                        s.polygon.validate_gaxi();
                    }
                };
                cx_ite=cx_ite.next;
            }
        };
        body.sweepFrozen=true;
    }
    public function validation(){
        cbsets.validate();
        {
            {
                var cx_ite=live.begin();
                while(cx_ite!=null){
                    var cur=cx_ite.elem();
                    {
                        cur.sweepRadius=0;
                        {
                            var cx_ite=cur.shapes.begin();
                            while(cx_ite!=null){
                                var s=cx_ite.elem();
                                {
                                    if(s.isPolygon()){
                                        s.polygon.splice_collinear();
                                        #if(!NAPE_RELEASE_BUILD)
                                        var res=s.polygon.valid();
                                        if(res!=ValidationResult.VALID){
                                            throw "Error: Cannot simulate with an invalid Polygon : "+s.polygon.outer.toString()+" is invalid : "+res.toString();
                                        }
                                        #end
                                        s.polygon.validate_gaxi();
                                    }
                                    s.validate_sweepRadius();
                                    if(s.sweepRadius>cur.sweepRadius)cur.sweepRadius=s.sweepRadius;
                                };
                                cx_ite=cx_ite.next;
                            }
                        };
                        cur.validate_mass();
                        cur.validate_inertia();
                        if(!cur.shapes.empty()){
                            cur.validate_aabb();
                            cur.validate_worldCOM();
                        }
                        cur.validate_gravMass();
                        cur.validate_axis();
                        #if(!NAPE_RELEASE_BUILD)
                        if(!cur.nomove&&cur.isDynamic()&&cur.mass==0){
                            throw "Error: Dynamic Body cannot be simulated with 0 mass unless allowMovement is false";
                        }
                        if(!cur.norotate&&cur.isDynamic()&&cur.inertia==0){
                            throw "Error: Dynamic Body cannot be simulated with 0 inertia unless allowRotation is false";
                        }
                        #end
                        if(!false){
                            if(cur.component.woken&&cur.cbSet!=null){
                                {
                                    var cx_ite=cur.cbSet.bodylisteners.begin();
                                    while(cx_ite!=null){
                                        var i=cx_ite.elem();
                                        {
                                            if(i.event!=ZPP_Flags.id_CbEvent_WAKE){
                                                cx_ite=cx_ite.next;
                                                continue;
                                            };
                                            var cb=push_callback(i);
                                            cb.event=ZPP_Flags.id_CbEvent_WAKE;
                                            cb.body=cur;
                                        };
                                        cx_ite=cx_ite.next;
                                    }
                                }
                            }
                            cur.component.woken=false;
                        }
                        {
                            var cx_ite=cur.shapes.begin();
                            while(cx_ite!=null){
                                var shape=cx_ite.elem();
                                bphase.sync(shape);
                                cx_ite=cx_ite.next;
                            }
                        };
                    };
                    cx_ite=cx_ite.next;
                }
            };
        };
        {
            {
                var cx_ite=kinematics.begin();
                while(cx_ite!=null){
                    var cur=cx_ite.elem();
                    {
                        cur.sweepRadius=0;
                        {
                            var cx_ite=cur.shapes.begin();
                            while(cx_ite!=null){
                                var s=cx_ite.elem();
                                {
                                    if(s.isPolygon()){
                                        s.polygon.splice_collinear();
                                        #if(!NAPE_RELEASE_BUILD)
                                        var res=s.polygon.valid();
                                        if(res!=ValidationResult.VALID){
                                            throw "Error: Cannot simulate with an invalid Polygon : "+s.polygon.outer.toString()+" is invalid : "+res.toString();
                                        }
                                        #end
                                        s.polygon.validate_gaxi();
                                    }
                                    s.validate_sweepRadius();
                                    if(s.sweepRadius>cur.sweepRadius)cur.sweepRadius=s.sweepRadius;
                                };
                                cx_ite=cx_ite.next;
                            }
                        };
                        cur.validate_mass();
                        cur.validate_inertia();
                        if(!cur.shapes.empty()){
                            cur.validate_aabb();
                            cur.validate_worldCOM();
                        }
                        cur.validate_gravMass();
                        cur.validate_axis();
                        #if(!NAPE_RELEASE_BUILD)
                        if(!cur.nomove&&cur.isDynamic()&&cur.mass==0){
                            throw "Error: Dynamic Body cannot be simulated with 0 mass unless allowMovement is false";
                        }
                        if(!cur.norotate&&cur.isDynamic()&&cur.inertia==0){
                            throw "Error: Dynamic Body cannot be simulated with 0 inertia unless allowRotation is false";
                        }
                        #end
                        if(!true){
                            if(cur.component.woken&&cur.cbSet!=null){
                                {
                                    var cx_ite=cur.cbSet.bodylisteners.begin();
                                    while(cx_ite!=null){
                                        var i=cx_ite.elem();
                                        {
                                            if(i.event!=ZPP_Flags.id_CbEvent_WAKE){
                                                cx_ite=cx_ite.next;
                                                continue;
                                            };
                                            var cb=push_callback(i);
                                            cb.event=ZPP_Flags.id_CbEvent_WAKE;
                                            cb.body=cur;
                                        };
                                        cx_ite=cx_ite.next;
                                    }
                                }
                            }
                            cur.component.woken=false;
                        }
                        {
                            var cx_ite=cur.shapes.begin();
                            while(cx_ite!=null){
                                var shape=cx_ite.elem();
                                bphase.sync(shape);
                                cx_ite=cx_ite.next;
                            }
                        };
                    };
                    cx_ite=cx_ite.next;
                }
            };
        };
        {
            var cx_ite=live_constraints.begin();
            while(cx_ite!=null){
                var con=cx_ite.elem();
                {
                    if(con.active){
                        #if(!NAPE_RELEASE_BUILD)
                        con.validate();
                        #end
                        if(con.component.woken&&con.cbSet!=null){
                            {
                                var cx_ite=con.cbSet.conlisteners.begin();
                                while(cx_ite!=null){
                                    var i=cx_ite.elem();
                                    {
                                        if(i.event!=ZPP_Flags.id_CbEvent_WAKE){
                                            cx_ite=cx_ite.next;
                                            continue;
                                        };
                                        var cb=push_callback(i);
                                        cb.event=ZPP_Flags.id_CbEvent_WAKE;
                                        cb.constraint=con;
                                    };
                                    cx_ite=cx_ite.next;
                                }
                            };
                        }
                        con.component.woken=false;
                    }
                };
                cx_ite=cx_ite.next;
            }
        };
    }
    public function updateVel(dt:Float){
        var pre=null;
        var linDrag=1-(dt*global_lin_drag);
        var angDrag=1-(dt*global_ang_drag);
        {
            var cx_ite=live.begin();
            while(cx_ite!=null){
                var cur=cx_ite.elem();
                {
                    if(cur.smass!=0.0){
                        var time=dt*cur.imass;
                        cur.velx=(linDrag*cur.velx)+(cur.forcex+gravityx*cur.gravMass)*time;
                        cur.vely=(linDrag*cur.vely)+(cur.forcey+gravityy*cur.gravMass)*time;
                    }
                    if(cur.sinertia!=0.0){
                        var dpx:Float=0.0;
                        var dpy:Float=0.0;
                        {
                            dpx=cur.worldCOMx-cur.posx;
                            dpy=cur.worldCOMy-cur.posy;
                        };
                        var torque=cur.torque+((gravityy*dpx-gravityx*dpy)*cur.gravMass);
                        cur.angvel=(angDrag*cur.angvel)+(torque*dt*cur.iinertia);
                    }
                    pre=cx_ite;
                };
                cx_ite=cx_ite.next;
            }
        };
    }
    public function updatePos(dt:Float){
        var MAX_VEL=2*Math.PI/dt;
        {
            {
                var cx_ite=live.begin();
                while(cx_ite!=null){
                    var cur=cx_ite.elem();
                    {
                        {
                            cur.pre_posx=cur.posx;
                            cur.pre_posy=cur.posy;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((cur.pre_posx!=cur.pre_posx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(cur.pre_posx)"+") :: "+("vec_set(in n: "+"cur.pre_pos"+",in x: "+"cur.posx"+",in y: "+"cur.posy"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((cur.pre_posy!=cur.pre_posy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(cur.pre_posy)"+") :: "+("vec_set(in n: "+"cur.pre_pos"+",in x: "+"cur.posx"+",in y: "+"cur.posy"+")");
                                #end
                            };
                        };
                        cur.pre_rot=cur.rot;
                        cur.sweepTime=0;
                        cur.sweep_angvel=(cur.angvel)%MAX_VEL;
                        cur.sweepIntegrate(dt);
                        if(!cur.disableCCD){
                            var linThreshold=Config.staticCCDLinearThreshold*cur.sweepRadius;
                            var angThreshold=Config.staticCCDAngularThreshold;
                            if((((cur.velx*cur.velx+cur.vely*cur.vely)*dt*dt)>(linThreshold*linThreshold)||(cur.angvel*cur.angvel*dt*dt)>(angThreshold*angThreshold))||cur.isKinematic()){
                                var angvel=cur.sweep_angvel;
                                if(angvel<0)angvel=-angvel;
                                var iangvel=1/angvel;
                                {
                                    var cx_ite=cur.shapes.begin();
                                    while(cx_ite!=null){
                                        var s=cx_ite.elem();
                                        {
                                            var aabb=s.aabb;
                                            var minx=aabb.minx;
                                            var miny=aabb.miny;
                                            var maxx=aabb.maxx;
                                            var maxy=aabb.maxy;
                                            var count:Int=(#if flash9 untyped __int__(angvel*dt*s.sweepCoef*(1/120))#else Std.int(angvel*dt*s.sweepCoef*(1/120))#end);
                                            if(count>8)count=8;
                                            var anginc=(angvel*dt)/count;
                                            cur.sweepIntegrate(dt);
                                            s.force_validate_aabb();
                                            if(minx<aabb.minx)aabb.minx=minx else minx=aabb.minx;
                                            if(miny<aabb.miny)aabb.miny=miny else miny=aabb.miny;
                                            if(maxx>aabb.maxx)aabb.maxx=maxx else maxx=aabb.maxx;
                                            if(maxy>aabb.maxy)aabb.maxy=maxy else maxy=aabb.maxy;
                                            for(i in 1...count){
                                                cur.sweepIntegrate(anginc*i*iangvel);
                                                s.force_validate_aabb();
                                                if(minx<aabb.minx)aabb.minx=minx else minx=aabb.minx;
                                                if(miny<aabb.miny)aabb.miny=miny else miny=aabb.miny;
                                                if(maxx>aabb.maxx)aabb.maxx=maxx else maxx=aabb.maxx;
                                                if(maxy>aabb.maxy)aabb.maxy=maxy else maxy=aabb.maxy;
                                            }
                                            bphase.sync(s);
                                        };
                                        cx_ite=cx_ite.next;
                                    }
                                };
                                cur.sweepFrozen=false;
                                if(cur.isDynamic()&&cur.bulletEnabled){
                                    var linThreshold2=Config.bulletCCDLinearThreshold*cur.sweepRadius;
                                    var angThreshold2=Config.bulletCCDAngularThreshold;
                                    if((((cur.velx*cur.velx+cur.vely*cur.vely)*dt*dt)>(linThreshold2*linThreshold2)||(cur.angvel*cur.angvel*dt*dt)>(angThreshold2*angThreshold2))){
                                        cur.bullet=true;
                                    }
                                }
                            }
                            else{
                                cur.sweepFrozen=true;
                                cur.bullet=false;
                            }
                        }
                        else{
                            cur.sweepFrozen=true;
                            cur.bullet=false;
                        }
                    };
                    cx_ite=cx_ite.next;
                }
            };
        };
        {
            {
                var cx_ite=kinematics.begin();
                while(cx_ite!=null){
                    var cur=cx_ite.elem();
                    {
                        {
                            cur.pre_posx=cur.posx;
                            cur.pre_posy=cur.posy;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((cur.pre_posx!=cur.pre_posx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(cur.pre_posx)"+") :: "+("vec_set(in n: "+"cur.pre_pos"+",in x: "+"cur.posx"+",in y: "+"cur.posy"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((cur.pre_posy!=cur.pre_posy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(cur.pre_posy)"+") :: "+("vec_set(in n: "+"cur.pre_pos"+",in x: "+"cur.posx"+",in y: "+"cur.posy"+")");
                                #end
                            };
                        };
                        cur.pre_rot=cur.rot;
                        cur.sweepTime=0;
                        cur.sweep_angvel=(cur.angvel)%MAX_VEL;
                        cur.sweepIntegrate(dt);
                        if(!cur.disableCCD){
                            var linThreshold=Config.staticCCDLinearThreshold*cur.sweepRadius;
                            var angThreshold=Config.staticCCDAngularThreshold;
                            if((((cur.velx*cur.velx+cur.vely*cur.vely)*dt*dt)>(linThreshold*linThreshold)||(cur.angvel*cur.angvel*dt*dt)>(angThreshold*angThreshold))||cur.isKinematic()){
                                var angvel=cur.sweep_angvel;
                                if(angvel<0)angvel=-angvel;
                                var iangvel=1/angvel;
                                {
                                    var cx_ite=cur.shapes.begin();
                                    while(cx_ite!=null){
                                        var s=cx_ite.elem();
                                        {
                                            var aabb=s.aabb;
                                            var minx=aabb.minx;
                                            var miny=aabb.miny;
                                            var maxx=aabb.maxx;
                                            var maxy=aabb.maxy;
                                            var count:Int=(#if flash9 untyped __int__(angvel*dt*s.sweepCoef*(1/120))#else Std.int(angvel*dt*s.sweepCoef*(1/120))#end);
                                            if(count>8)count=8;
                                            var anginc=(angvel*dt)/count;
                                            cur.sweepIntegrate(dt);
                                            s.force_validate_aabb();
                                            if(minx<aabb.minx)aabb.minx=minx else minx=aabb.minx;
                                            if(miny<aabb.miny)aabb.miny=miny else miny=aabb.miny;
                                            if(maxx>aabb.maxx)aabb.maxx=maxx else maxx=aabb.maxx;
                                            if(maxy>aabb.maxy)aabb.maxy=maxy else maxy=aabb.maxy;
                                            for(i in 1...count){
                                                cur.sweepIntegrate(anginc*i*iangvel);
                                                s.force_validate_aabb();
                                                if(minx<aabb.minx)aabb.minx=minx else minx=aabb.minx;
                                                if(miny<aabb.miny)aabb.miny=miny else miny=aabb.miny;
                                                if(maxx>aabb.maxx)aabb.maxx=maxx else maxx=aabb.maxx;
                                                if(maxy>aabb.maxy)aabb.maxy=maxy else maxy=aabb.maxy;
                                            }
                                            bphase.sync(s);
                                        };
                                        cx_ite=cx_ite.next;
                                    }
                                };
                                cur.sweepFrozen=false;
                                if(cur.isDynamic()&&cur.bulletEnabled){
                                    var linThreshold2=Config.bulletCCDLinearThreshold*cur.sweepRadius;
                                    var angThreshold2=Config.bulletCCDAngularThreshold;
                                    if((((cur.velx*cur.velx+cur.vely*cur.vely)*dt*dt)>(linThreshold2*linThreshold2)||(cur.angvel*cur.angvel*dt*dt)>(angThreshold2*angThreshold2))){
                                        cur.bullet=true;
                                    }
                                }
                            }
                            else{
                                cur.sweepFrozen=true;
                                cur.bullet=false;
                            }
                        }
                        else{
                            cur.sweepFrozen=true;
                            cur.bullet=false;
                        }
                    };
                    cx_ite=cx_ite.next;
                }
            };
        };
    }
    public var continuous:Bool=false;
    public function presteparb(arb:ZPP_Arbiter,dt:Float,?cont=false){
        if(!arb.cleared&&(arb.b1.component.sleeping&&arb.b2.component.sleeping)){
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    false;
                };
                if(!res)throw "assert("+"false"+") :: "+("sleep in prestep (no longer should occur)");
                #end
            };
            arb.sleep_stamp=stamp;
            arb.sleeping=true;
            return true;
        }
        #if NAPE_TIMES Debug.ACNT++;
        #end
        if(!arb.cleared||arb.present!=0||arb.intchange){
            var endcb=(!cont&&arb.up_stamp==stamp-1)&&!arb.cleared&&!arb.intchange;
            var begcb=(arb.fresh)&&!arb.cleared&&!arb.intchange;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !begcb||arb.up_stamp==stamp;
                };
                if(!res)throw "assert("+"!begcb||arb.up_stamp==stamp"+") :: "+(" fresh -> active=true hasn't held :("+begcb+" "+arb.up_stamp+" "+stamp);
                #end
            };
            if(endcb){
                arb.endGenerated=stamp;
            }
            if(begcb||endcb||arb.cleared||arb.intchange){
                inlined_MRCA_chains(arb.ws1,arb.ws2);
                {
                    var cx_ite=mrca1.begin();
                    while(cx_ite!=null){
                        var i1=cx_ite.elem();
                        {
                            {
                                var cx_ite=mrca2.begin();
                                while(cx_ite!=null){
                                    var i2=cx_ite.elem();
                                    {
                                        var cb1=i1.cbSet;
                                        var cb2=i2.cbSet;
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                cb1!=null&&cb2!=null;
                                            };
                                            if(!res)throw "assert("+"cb1!=null&&cb2!=null"+") :: "+("null cb from MRCA_chains?");
                                            #end
                                        };
                                        if(ZPP_CbSet.empty_intersection(cb1,cb2)){
                                            cx_ite=cx_ite.next;
                                            continue;
                                        };
                                        var callbackset=ZPP_Interactor.get(i1,i2);
                                        if(begcb||arb.intchange){
                                            if(callbackset==null){
                                                callbackset=ZPP_CallbackSet.get(i1,i2);
                                                add_callbackset(callbackset);
                                            }
                                            ZPP_CbSet.find_all(cb1,cb2,ZPP_Flags.id_CbEvent_BEGIN,function(listener:ZPP_InteractionListener){
                                                if((listener.itype&arb.type)!=0&&callbackset.empty_arb(listener.itype)){
                                                    var cb=push_callback(listener);
                                                    cb.event=ZPP_Flags.id_CbEvent_BEGIN;
                                                    ZPP_Interactor.int_callback(callbackset,listener,cb);
                                                    cb.set=callbackset;
                                                }
                                            });
                                            if(callbackset.add_arb(arb))arb.present++;
                                        }
                                        else{
                                            arb.present--;
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    arb.present>=0;
                                                };
                                                if(!res)throw "assert("+"arb.present>=0"+") :: "+("arb present < 0?");
                                                #end
                                            };
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    callbackset!=null;
                                                };
                                                if(!res)throw "assert("+"callbackset!=null"+") :: "+("end arbiter with no callbackset");
                                                #end
                                            };
                                            callbackset.remove_arb(arb);
                                            ZPP_CbSet.find_all(cb1,cb2,ZPP_Flags.id_CbEvent_END,function(listener:ZPP_InteractionListener){
                                                if((listener.itype&arb.type)!=0&&callbackset.empty_arb(listener.itype)){
                                                    var cb=push_callback(listener);
                                                    cb.event=ZPP_Flags.id_CbEvent_END;
                                                    ZPP_Interactor.int_callback(callbackset,listener,cb);
                                                    cb.set=callbackset;
                                                }
                                            });
                                            if(callbackset.really_empty()){
                                                remove_callbackset(callbackset);
                                            }
                                        }
                                    };
                                    cx_ite=cx_ite.next;
                                }
                            };
                        };
                        cx_ite=cx_ite.next;
                    }
                };
            }
            arb.fresh=false;
            arb.intchange=false;
        }
        if(arb.cleared||arb.up_stamp+(arb.type==ZPP_Arbiter.COL?Config.arbiterExpirationDelay:0)<stamp){
            if(arb.type==ZPP_Arbiter.SENSOR)arb.sensorarb.retire();
            else if(arb.type==ZPP_Arbiter.FLUID)arb.fluidarb.retire();
            else arb.colarb.retire();
            return true;
        }
        var pact=arb.active;
        arb.active=arb.presentable=arb.up_stamp==stamp;
        if((arb.immState&ZPP_Flags.id_ImmState_ACCEPT)!=0){
            if(arb.active&&arb.type!=ZPP_Arbiter.SENSOR){
                if(arb.colarb!=null){
                    if(arb.colarb.preStep(dt)){
                        arb.active=false;
                    }
                }
                else arb.fluidarb.preStep(this,dt);
            }
        }
        else if(arb.colarb!=null){
            if(arb.colarb.cleanupContacts())arb.active=false;
        }
        if(pact!=arb.active){
            arb.b1.arbiters.modified=true;
            arb.b2.arbiters.modified=true;
            c_arbiters_true.modified=c_arbiters_false.modified=true;
            s_arbiters.modified=f_arbiters.modified=true;
        }
        return false;
    }
    public function prestep(dt:Float){
        var pre=null;
        {
            var cx_ite=live_constraints.begin();
            while(cx_ite!=null){
                var con=cx_ite.elem();
                {
                    if(con.preStep(dt)){
                        cx_ite=live_constraints.erase(pre);
                        con.broken();
                        constraintCbBreak(con);
                        if(con.removeOnBreak){
                            con.component.sleeping=true;
                            midstep=false;
                            if(con.compound!=null)con.compound.wrap_constraints.remove(con.outer);
                            else wrap_constraints.remove(con.outer);
                            midstep=true;
                        }
                        else con.active=false;
                        con.clearcache();
                        continue;
                    }
                    pre=cx_ite;
                };
                cx_ite=cx_ite.next;
            }
        };
        {
            var pre=null;
            var arbs=c_arbiters_true;
            var arbite=arbs.begin();
            var fst=c_arbiters_false!=null;
            {
                if(fst&&arbite==null){
                    fst=false;
                    arbite=c_arbiters_false.begin();
                    arbs=c_arbiters_false;
                    pre=null;
                }
            };
            while(arbite!=null){
                var arb=arbite.elem();
                if(presteparb(arb,dt)){
                    arbite=arbs.inlined_erase(pre);
                    {
                        if(fst&&arbite==null){
                            fst=false;
                            arbite=c_arbiters_false.begin();
                            arbs=c_arbiters_false;
                            pre=null;
                        }
                    };
                    continue;
                }
                pre=arbite;
                arbite=arbite.next;
                {
                    if(fst&&arbite==null){
                        fst=false;
                        arbite=c_arbiters_false.begin();
                        arbs=c_arbiters_false;
                        pre=null;
                    }
                };
            }
        };
        {
            var pre=null;
            var arbs=f_arbiters;
            var arbite=arbs.begin();
            var fst=null!=null;
            {
                if(fst&&arbite==null){
                    fst=false;
                    arbite=null.begin();
                    arbs=null;
                    pre=null;
                }
            };
            while(arbite!=null){
                var arb=arbite.elem();
                if(presteparb(arb,dt)){
                    arbite=arbs.inlined_erase(pre);
                    {
                        if(fst&&arbite==null){
                            fst=false;
                            arbite=null.begin();
                            arbs=null;
                            pre=null;
                        }
                    };
                    continue;
                }
                pre=arbite;
                arbite=arbite.next;
                {
                    if(fst&&arbite==null){
                        fst=false;
                        arbite=null.begin();
                        arbs=null;
                        pre=null;
                    }
                };
            }
        };
        {
            var pre=null;
            var arbs=s_arbiters;
            var arbite=arbs.begin();
            var fst=null!=null;
            {
                if(fst&&arbite==null){
                    fst=false;
                    arbite=null.begin();
                    arbs=null;
                    pre=null;
                }
            };
            while(arbite!=null){
                var arb=arbite.elem();
                if(presteparb(arb,dt)){
                    arbite=arbs.inlined_erase(pre);
                    {
                        if(fst&&arbite==null){
                            fst=false;
                            arbite=null.begin();
                            arbs=null;
                            pre=null;
                        }
                    };
                    continue;
                }
                pre=arbite;
                arbite=arbite.next;
                {
                    if(fst&&arbite==null){
                        fst=false;
                        arbite=null.begin();
                        arbs=null;
                        pre=null;
                    }
                };
            }
        };
    }
    public function warmStart(){
        {
            var cx_ite=f_arbiters.begin();
            while(cx_ite!=null){
                var arb=cx_ite.elem();
                if(arb.acting())arb.warmStart();
                cx_ite=cx_ite.next;
            }
        };
        {
            var arbi=c_arbiters_false.begin();
            var fst=true;
            if(arbi==null){
                arbi=c_arbiters_true.begin();
                fst=false;
            }
            while(arbi!=null){
                var arb=arbi.elem();
                {
                    if(arb.acting())arb.warmStart();
                }
                arbi=arbi.next;
                if(fst&&arbi==null){
                    arbi=c_arbiters_true.begin();
                    fst=false;
                }
            }
        };
        {
            var cx_ite=live_constraints.begin();
            while(cx_ite!=null){
                var con=cx_ite.elem();
                con.warmStart();
                cx_ite=cx_ite.next;
            }
        };
    }
    public function iterateVel(times:Int){
        for(i in 0...times){
            {
                var cx_ite=f_arbiters.begin();
                while(cx_ite!=null){
                    var arb=cx_ite.elem();
                    if(arb.acting())arb.applyImpulseVel();
                    cx_ite=cx_ite.next;
                }
            };
            var pre=null;
            {
                var cx_ite=live_constraints.begin();
                while(cx_ite!=null){
                    var con=cx_ite.elem();
                    {
                        if(con.applyImpulseVel()){
                            cx_ite=live_constraints.erase(pre);
                            con.broken();
                            constraintCbBreak(con);
                            if(con.removeOnBreak){
                                con.component.sleeping=true;
                                midstep=false;
                                if(con.compound!=null)con.compound.wrap_constraints.remove(con.outer);
                                else wrap_constraints.remove(con.outer);
                                midstep=true;
                            }
                            else con.active=false;
                            con.clearcache();
                            continue;
                        }
                        pre=cx_ite;
                    };
                    cx_ite=cx_ite.next;
                }
            };
            {
                var arbi=c_arbiters_false.begin();
                var fst=true;
                if(arbi==null){
                    arbi=c_arbiters_true.begin();
                    fst=false;
                }
                while(arbi!=null){
                    var arb=arbi.elem();
                    {
                        if(arb.acting())arb.applyImpulseVel();
                    }
                    arbi=arbi.next;
                    if(fst&&arbi==null){
                        arbi=c_arbiters_true.begin();
                        fst=false;
                    }
                }
            };
        }
    }
    public function iteratePos(times:Int){
        for(i in 0...times){
            var pre=null;
            {
                var cx_ite=live_constraints.begin();
                while(cx_ite!=null){
                    var con=cx_ite.elem();
                    {
                        if(!con.__velocity&&con.stiff){
                            if(con.applyImpulsePos()){
                                cx_ite=live_constraints.erase(pre);
                                con.broken();
                                constraintCbBreak(con);
                                if(con.removeOnBreak){
                                    con.component.sleeping=true;
                                    midstep=false;
                                    if(con.compound!=null)con.compound.wrap_constraints.remove(con.outer);
                                    else wrap_constraints.remove(con.outer);
                                    midstep=true;
                                }
                                else con.active=false;
                                con.clearcache();
                                continue;
                            }
                        }
                        pre=cx_ite;
                    };
                    cx_ite=cx_ite.next;
                }
            };
            {
                var arbi=c_arbiters_false.begin();
                var fst=true;
                if(arbi==null){
                    arbi=c_arbiters_true.begin();
                    fst=false;
                }
                while(arbi!=null){
                    var arb=arbi.elem();
                    {
                        {
                            if(arb.acting()){
                                arb.applyImpulsePos();
                            }
                        };
                    }
                    arbi=arbi.next;
                    if(fst&&arbi==null){
                        arbi=c_arbiters_true.begin();
                        fst=false;
                    }
                }
            };
        }
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function group_ignore(s1:ZPP_Shape,s2:ZPP_Shape){
        var g1=s1.lookup_group();
        if(g1==null)return false;
        else{
            var g2=s2.lookup_group();
            if(g2==null)return false;
            else{
                var ret=false;
                while(g1!=null&&g2!=null){
                    if(g1==g2){
                        ret=g1.ignore;
                        break;
                    }
                    if(g1.depth<g2.depth)g2=g2.group;
                    else g1=g1.group;
                }
                return ret;
            }
        }
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function interactionType(s1:ZPP_Shape,s2:ZPP_Shape,b1:ZPP_Body,b2:ZPP_Body){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !(b1==b2);
            };
            if(!res)throw "assert("+"!(b1==b2)"+") :: "+("both shapes from same object?");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !(b1.isStatic()&&b2.isStatic());
            };
            if(!res)throw "assert("+"!(b1.isStatic()&&b2.isStatic())"+") :: "+("both objects static?");
            #end
        };
        var con_ignore;
        {
            con_ignore=false;
            {
                var cx_ite=b1.constraints.begin();
                while(cx_ite!=null){
                    var con=cx_ite.elem();
                    {
                        if(con.ignore&&con.pair_exists(b1.id,b2.id)){
                            con_ignore=true;
                            break;
                        }
                    };
                    cx_ite=cx_ite.next;
                }
            };
        };
        if(!con_ignore&&!group_ignore(s1,s2)){
            if((s1.sensorEnabled||s2.sensorEnabled)&&s1.filter.shouldSense(s2.filter)){
                return 2;
            }
            else if((s1.fluidEnabled||s2.fluidEnabled)&&s1.filter.shouldFlow(s2.filter)&&!(b1.imass==0&&b2.imass==0&&b1.iinertia==0&&b2.iinertia==0)){
                return 0;
            }
            else if(s1.filter.shouldCollide(s2.filter)&&!(b1.imass==0&&b2.imass==0&&b1.iinertia==0&&b2.iinertia==0)){
                return 1;
            }
            else return-1;
        }
        else return-1;
    }
    var precb:PreCallback=null;
    var prelisteners:ZNPList_ZPP_InteractionListener=null;
    public function narrowPhase(s1:ZPP_Shape,s2:ZPP_Shape,stat:Bool,in_arb:ZPP_Arbiter,continuous:Bool){
        #if NAPE_TIMES var pt=flash.Lib.getTimer();
        #end
        var ret:ZPP_Arbiter=null;
        var b1=s1.body;
        var b2=s2.body;
        var itype=interactionType(s1,s2,b1,b2);
        if(itype!=-1){
            var sa,sb;
            if(s1.type>s2.type){
                sa=s2;
                sb=s1;
            }
            else if(s1.type==s2.type){
                if(s1.id<s2.id){
                    sa=s1;
                    sb=s2;
                }
                else{
                    sb=s1;
                    sa=s2;
                }
            }
            else{
                sa=s1;
                sb=s2;
            }
            var reverse=sa==s2;
            if(itype==0){
                var xarb=if(in_arb==null)({
                    var ret:ZPP_Arbiter=null;
                    var b=if(b1.arbiters.length<b2.arbiters.length)b1 else b2;
                    {
                        var cx_ite=b.arbiters.begin();
                        while(cx_ite!=null){
                            var arb=cx_ite.elem();
                            {
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !(arb.id==sb.id&&arb.di==sa.id);
                                    };
                                    if(!res)throw "assert("+"!(arb.id==sb.id&&arb.di==sa.id)"+") :: "+("arbiter id order doesn't match s1/s2 order?>");
                                    #end
                                };
                                if(arb.id==sa.id&&arb.di==sb.id){
                                    ret=arb;
                                    break;
                                }
                            };
                            cx_ite=cx_ite.next;
                        }
                    };
                    ret;
                })else in_arb;
                var first=xarb==null;
                var arb;
                var swapped=false;
                if(first){
                    if(ZPP_FluidArbiter.zpp_pool==null){
                        arb=new ZPP_FluidArbiter();
                        #if NAPE_POOL_STATS ZPP_FluidArbiter.POOL_TOT++;
                        ZPP_FluidArbiter.POOL_ADDNEW++;
                        #end
                    }
                    else{
                        arb=ZPP_FluidArbiter.zpp_pool;
                        ZPP_FluidArbiter.zpp_pool=arb.next;
                        arb.next=null;
                        #if NAPE_POOL_STATS ZPP_FluidArbiter.POOL_CNT--;
                        ZPP_FluidArbiter.POOL_ADD++;
                        #end
                    }
                    arb.alloc();
                }
                else{
                    if(xarb.fluidarb==null){
                        xarb.lazyRetire(this,null);
                        {
                            if(ZPP_FluidArbiter.zpp_pool==null){
                                arb=new ZPP_FluidArbiter();
                                #if NAPE_POOL_STATS ZPP_FluidArbiter.POOL_TOT++;
                                ZPP_FluidArbiter.POOL_ADDNEW++;
                                #end
                            }
                            else{
                                arb=ZPP_FluidArbiter.zpp_pool;
                                ZPP_FluidArbiter.zpp_pool=arb.next;
                                arb.next=null;
                                #if NAPE_POOL_STATS ZPP_FluidArbiter.POOL_CNT--;
                                ZPP_FluidArbiter.POOL_ADD++;
                                #end
                            }
                            arb.alloc();
                        };
                        arb.intchange=true;
                        first=true;
                        swapped=true;
                    }
                    else arb=xarb.fluidarb;
                }
                ret={
                    var inttype=ZPP_Flags.id_InteractionType_FLUID;
                    if(first||arb.stamp!=stamp||continuous){
                        arb.stamp=stamp;
                        if(ZPP_Collide.flowCollide(sa,sb,arb)){
                            if(first){
                                arb.assign(s1,s2,sa.id,sb.id);
                                f_arbiters.inlined_add(arb);
                                arb.fresh=!swapped;
                            }
                            else{
                                arb.fresh=(arb.up_stamp<stamp-1||(arb.endGenerated==stamp&&continuous));
                            }
                            arb.up_stamp=arb.stamp;
                            if(arb.fresh||(arb.immState&ZPP_Flags.id_ImmState_ALWAYS)==0){
                                arb.immState=ZPP_Flags.id_ImmState_ACCEPT;
                                var anyimpure=false;
                                var arbs1=arb.ws1.id>arb.ws2.id?arb.ws2:arb.ws1;
                                var arbs2=arb.ws1.id>arb.ws2.id?arb.ws1:arb.ws2;
                                inlined_MRCA_chains(arbs1,arbs2);
                                {
                                    var cx_ite=mrca1.begin();
                                    while(cx_ite!=null){
                                        var i1=cx_ite.elem();
                                        {
                                            {
                                                var cx_ite=mrca2.begin();
                                                while(cx_ite!=null){
                                                    var i2=cx_ite.elem();
                                                    {
                                                        var cb1=i1.cbSet;
                                                        var cb2=i2.cbSet;
                                                        {
                                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                            var res={
                                                                cb1!=null&&cb2!=null;
                                                            };
                                                            if(!res)throw "assert("+"cb1!=null&&cb2!=null"+") :: "+("null cb from MRCA_chains?");
                                                            #end
                                                        };
                                                        if(ZPP_CbSet.empty_intersection(cb1,cb2)){
                                                            cx_ite=cx_ite.next;
                                                            continue;
                                                        };
                                                        var callbackset:ZPP_CallbackSet=null;
                                                        var ncallbackset:ZPP_CallbackSet=null;
                                                        prelisteners.inlined_clear();
                                                        var lite:ZNPNode_ZPP_InteractionListener=null;
                                                        ZPP_CbSet.find_all(cb1,cb2,ZPP_Flags.id_CbEvent_PRE,function(listener:ZPP_InteractionListener){
                                                            if((listener.itype&inttype)!=0){
                                                                lite=prelisteners.inlined_insert(lite,listener);
                                                                anyimpure=anyimpure||!listener.pure;
                                                            }
                                                        });
                                                        if(prelisteners.empty()){
                                                            cx_ite=cx_ite.next;
                                                            continue;
                                                        };
                                                        callbackset=ZPP_Interactor.get(i1,i2);
                                                        if(callbackset==null){
                                                            ncallbackset=ZPP_CallbackSet.get(i1,i2);
                                                            add_callbackset(ncallbackset);
                                                        }
                                                        if(callbackset==null||((callbackset.FLUIDstamp!=stamp||continuous)&&(callbackset.FLUIDstate&ZPP_Flags.id_ImmState_ALWAYS==0))){
                                                            if(ncallbackset!=null)callbackset=ncallbackset;
                                                            if(callbackset!=null){
                                                                {
                                                                    var cx_ite=prelisteners.begin();
                                                                    while(cx_ite!=null){
                                                                        var listener=cx_ite.elem();
                                                                        {
                                                                            if(listener.itype==ZPP_Flags.id_InteractionType_ANY){
                                                                                callbackset.COLLISIONstamp=stamp;
                                                                                callbackset.SENSORstamp=stamp;
                                                                                callbackset.FLUIDstamp=stamp;
                                                                            }
                                                                            else callbackset.FLUIDstamp=stamp;
                                                                        };
                                                                        cx_ite=cx_ite.next;
                                                                    }
                                                                };
                                                            }
                                                            #if(!NAPE_RELEASE_BUILD)
                                                            arb.makemutable();
                                                            #end
                                                            var pact=arb.active;
                                                            arb.active=true;
                                                            var emptycontacts=false;
                                                            #if false arb.cleanupContacts();
                                                            #end
                                                            precb.zpp_inner.pre_arbiter=arb;
                                                            precb.zpp_inner.set=callbackset;
                                                            {
                                                                var cx_ite=prelisteners.begin();
                                                                while(cx_ite!=null){
                                                                    var listener=cx_ite.elem();
                                                                    {
                                                                        precb.zpp_inner.listener=listener;
                                                                        #if NAPE_ASSERT if(callbackset.int1==null){
                                                                            var err="";
                                                                            err+="OKAY WTF IS HAPPENING HERE\n";
                                                                            err+="ncallbackset is null? "+(ncallbackset==null?"yes":"no")+"\n";
                                                                            err+="assuming yes, let's find callbacksets on interactor\n";
                                                                            err+="i1 (id="+i1.id+"):\n";
                                                                            {
                                                                                var cx_ite=i1.cbsets.begin();
                                                                                while(cx_ite!=null){
                                                                                    var cbset=cx_ite.elem();
                                                                                    err+="  ("+cbset.id+" "+(cbset.int1==null?"null":"")+","+cbset.di+" "+(cbset.int2==null?"null":"")+") arbs size="+cbset.arbiters.size()+" freed="+cbset.freed+" lazydel="+cbset.lazydel+"\n";
                                                                                    cx_ite=cx_ite.next;
                                                                                }
                                                                            };
                                                                            err+="i2 (id="+i2.id+"):\n";
                                                                            {
                                                                                var cx_ite=i2.cbsets.begin();
                                                                                while(cx_ite!=null){
                                                                                    var cbset=cx_ite.elem();
                                                                                    err+="  ("+cbset.id+" "+(cbset.int1==null?"null":"")+","+cbset.di+" "+(cbset.int2==null?"null":"")+") arbs size="+cbset.arbiters.size()+" freed="+cbset.freed+" lazydel="+cbset.lazydel+"\n";
                                                                                    cx_ite=cx_ite.next;
                                                                                }
                                                                            };
                                                                            throw err;
                                                                        }
                                                                        #end
                                                                        ZPP_Interactor.int_callback(callbackset,listener,precb.zpp_inner);
                                                                        precb.zpp_inner.pre_swapped=i1!=precb.zpp_inner.int1;
                                                                        var ret=listener.handlerp(precb);
                                                                        if(ret!=null){
                                                                            arb.immState=if(ret==PreFlag.ACCEPT)ZPP_Flags.id_ImmState_ACCEPT|ZPP_Flags.id_ImmState_ALWAYS else if(ret==PreFlag.ACCEPT_ONCE)ZPP_Flags.id_ImmState_ACCEPT else if(ret==PreFlag.IGNORE)ZPP_Flags.id_ImmState_IGNORE|ZPP_Flags.id_ImmState_ALWAYS else ZPP_Flags.id_ImmState_IGNORE;
                                                                        }
                                                                    };
                                                                    cx_ite=cx_ite.next;
                                                                }
                                                            };
                                                            #if(!NAPE_RELEASE_BUILD)
                                                            arb.makeimmutable();
                                                            #end
                                                            arb.active=pact;
                                                            if(callbackset!=null){
                                                                {
                                                                    var cx_ite=prelisteners.begin();
                                                                    while(cx_ite!=null){
                                                                        var listener=cx_ite.elem();
                                                                        {
                                                                            if(listener.itype==ZPP_Flags.id_InteractionType_ANY){
                                                                                callbackset.COLLISIONstate=arb.immState;
                                                                                callbackset.SENSORstate=arb.immState;
                                                                                callbackset.FLUIDstate=arb.immState;
                                                                            }
                                                                            else callbackset.FLUIDstate=arb.immState;
                                                                        };
                                                                        cx_ite=cx_ite.next;
                                                                    }
                                                                };
                                                            }
                                                        }
                                                        else if(callbackset==null){
                                                            if((arb.immState&ZPP_Flags.id_ImmState_ALWAYS)==0)arb.immState=ZPP_Flags.id_ImmState_ACCEPT;
                                                        }
                                                        else arb.immState=callbackset.FLUIDstate;
                                                    };
                                                    cx_ite=cx_ite.next;
                                                }
                                            };
                                        };
                                        cx_ite=cx_ite.next;
                                    }
                                };
                                if(anyimpure&&(arb.immState&ZPP_Flags.id_ImmState_ALWAYS)==0){
                                    if(true){
                                        if(arb.b1.isDynamic()){
                                            var o=arb.b1;
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    o.space==this;
                                                };
                                                if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                                #end
                                            };
                                            if(!o.world){
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        o.component!=null;
                                                    };
                                                    if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                                    #end
                                                };
                                                o.component.waket=stamp+(midstep?0:1);
                                                if(o.isKinematic())o.kinematicDelaySleep=true;
                                                if(o.component.sleeping){
                                                    really_wake(o,false);
                                                }
                                            }
                                        };
                                        if(arb.b1.isDynamic()){
                                            var o=arb.b2;
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    o.space==this;
                                                };
                                                if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                                #end
                                            };
                                            if(!o.world){
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        o.component!=null;
                                                    };
                                                    if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                                    #end
                                                };
                                                o.component.waket=stamp+(midstep?0:1);
                                                if(o.isKinematic())o.kinematicDelaySleep=true;
                                                if(o.component.sleeping){
                                                    really_wake(o,false);
                                                }
                                            }
                                        };
                                    }
                                    else{
                                        if(!arb.b1.isStatic()){
                                            var o=arb.b1;
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    o.space==this;
                                                };
                                                if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                                #end
                                            };
                                            if(!o.world){
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        o.component!=null;
                                                    };
                                                    if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                                    #end
                                                };
                                                o.component.waket=stamp+(midstep?0:1);
                                                if(o.isKinematic())o.kinematicDelaySleep=true;
                                                if(o.component.sleeping){
                                                    really_wake(o,false);
                                                }
                                            }
                                        };
                                        if(!arb.b2.isStatic()){
                                            var o=arb.b2;
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    o.space==this;
                                                };
                                                if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                                #end
                                            };
                                            if(!o.world){
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        o.component!=null;
                                                    };
                                                    if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                                    #end
                                                };
                                                o.component.waket=stamp+(midstep?0:1);
                                                if(o.isKinematic())o.kinematicDelaySleep=true;
                                                if(o.component.sleeping){
                                                    really_wake(o,false);
                                                }
                                            }
                                        };
                                    }
                                }
                            }
                            if(true&&(arb.immState&ZPP_Flags.id_ImmState_ACCEPT)!=0){
                                if(arb.b1.isDynamic()&&arb.b1.component.sleeping){
                                    var o=arb.b1;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o.space==this;
                                        };
                                        if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                        #end
                                    };
                                    if(!o.world){
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                o.component!=null;
                                            };
                                            if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                            #end
                                        };
                                        o.component.waket=stamp+(midstep?0:1);
                                        if(o.isKinematic())o.kinematicDelaySleep=true;
                                        if(o.component.sleeping){
                                            really_wake(o,false);
                                        }
                                    }
                                };
                                if(arb.b2.isDynamic()&&arb.b2.component.sleeping){
                                    var o=arb.b2;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o.space==this;
                                        };
                                        if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                        #end
                                    };
                                    if(!o.world){
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                o.component!=null;
                                            };
                                            if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                            #end
                                        };
                                        o.component.waket=stamp+(midstep?0:1);
                                        if(o.isKinematic())o.kinematicDelaySleep=true;
                                        if(o.component.sleeping){
                                            really_wake(o,false);
                                        }
                                    }
                                };
                            }
                            if(arb.sleeping){
                                arb.sleeping=false;
                                f_arbiters.inlined_add(arb);
                            }
                            arb;
                        }
                        else if(first){
                            {
                                var o=arb;
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        o!=null;
                                    };
                                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_FluidArbiter"+", in obj: "+"arb"+")");
                                    #end
                                };
                                o.free();
                                o.next=ZPP_FluidArbiter.zpp_pool;
                                ZPP_FluidArbiter.zpp_pool=o;
                                #if NAPE_POOL_STATS ZPP_FluidArbiter.POOL_CNT++;
                                ZPP_FluidArbiter.POOL_SUB++;
                                #end
                            };
                            null;
                        }
                        else arb;
                    }
                    else arb;
                };
            }
            else if(itype==1){
                var carbs=stat?c_arbiters_true:c_arbiters_false;
                var xarb=if(in_arb==null)({
                    var ret:ZPP_Arbiter=null;
                    var b=if(b1.arbiters.length<b2.arbiters.length)b1 else b2;
                    {
                        var cx_ite=b.arbiters.begin();
                        while(cx_ite!=null){
                            var arb=cx_ite.elem();
                            {
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !(arb.id==sb.id&&arb.di==sa.id);
                                    };
                                    if(!res)throw "assert("+"!(arb.id==sb.id&&arb.di==sa.id)"+") :: "+("arbiter id order doesn't match s1/s2 order?>");
                                    #end
                                };
                                if(arb.id==sa.id&&arb.di==sb.id){
                                    ret=arb;
                                    break;
                                }
                            };
                            cx_ite=cx_ite.next;
                        }
                    };
                    ret;
                })else in_arb;
                var first=xarb==null;
                var arb;
                var swapped=false;
                if(first){
                    {
                        if(ZPP_ColArbiter.zpp_pool==null){
                            arb=new ZPP_ColArbiter();
                            #if NAPE_POOL_STATS ZPP_ColArbiter.POOL_TOT++;
                            ZPP_ColArbiter.POOL_ADDNEW++;
                            #end
                        }
                        else{
                            arb=ZPP_ColArbiter.zpp_pool;
                            ZPP_ColArbiter.zpp_pool=arb.next;
                            arb.next=null;
                            #if NAPE_POOL_STATS ZPP_ColArbiter.POOL_CNT--;
                            ZPP_ColArbiter.POOL_ADD++;
                            #end
                        }
                        arb.alloc();
                    };
                    arb.stat=stat;
                }
                else{
                    if(xarb.colarb==null){
                        xarb.lazyRetire(this,null);
                        {
                            if(ZPP_ColArbiter.zpp_pool==null){
                                arb=new ZPP_ColArbiter();
                                #if NAPE_POOL_STATS ZPP_ColArbiter.POOL_TOT++;
                                ZPP_ColArbiter.POOL_ADDNEW++;
                                #end
                            }
                            else{
                                arb=ZPP_ColArbiter.zpp_pool;
                                ZPP_ColArbiter.zpp_pool=arb.next;
                                arb.next=null;
                                #if NAPE_POOL_STATS ZPP_ColArbiter.POOL_CNT--;
                                ZPP_ColArbiter.POOL_ADD++;
                                #end
                            }
                            arb.alloc();
                        };
                        arb.intchange=true;
                        arb.stat=stat;
                        first=true;
                        swapped=true;
                    }
                    else{
                        arb=xarb.colarb;
                        reverse=sa!=arb.s1;
                        if(arb.stat!=stat){
                            arb.stat=stat;
                            if(!arb.sleeping){
                                (if(stat)c_arbiters_false else c_arbiters_true).remove(arb);
                                carbs.add(arb);
                            }
                        }
                    }
                }
                ret={
                    var inttype=ZPP_Flags.id_InteractionType_COLLISION;
                    if(first||arb.stamp!=stamp||continuous){
                        arb.stamp=stamp;
                        if(ZPP_Collide.contactCollide(sa,sb,arb,reverse)){
                            if(first){
                                arb.assign(s1,s2,sa.id,sb.id);
                                carbs.inlined_add(arb);
                                arb.fresh=!swapped;
                            }
                            else{
                                arb.fresh=(arb.up_stamp<stamp-1||(arb.endGenerated==stamp&&continuous));
                            }
                            arb.up_stamp=arb.stamp;
                            if(arb.fresh||(arb.immState&ZPP_Flags.id_ImmState_ALWAYS)==0){
                                arb.immState=ZPP_Flags.id_ImmState_ACCEPT;
                                var anyimpure=false;
                                var arbs1=arb.ws1.id>arb.ws2.id?arb.ws2:arb.ws1;
                                var arbs2=arb.ws1.id>arb.ws2.id?arb.ws1:arb.ws2;
                                inlined_MRCA_chains(arbs1,arbs2);
                                {
                                    var cx_ite=mrca1.begin();
                                    while(cx_ite!=null){
                                        var i1=cx_ite.elem();
                                        {
                                            {
                                                var cx_ite=mrca2.begin();
                                                while(cx_ite!=null){
                                                    var i2=cx_ite.elem();
                                                    {
                                                        var cb1=i1.cbSet;
                                                        var cb2=i2.cbSet;
                                                        {
                                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                            var res={
                                                                cb1!=null&&cb2!=null;
                                                            };
                                                            if(!res)throw "assert("+"cb1!=null&&cb2!=null"+") :: "+("null cb from MRCA_chains?");
                                                            #end
                                                        };
                                                        if(ZPP_CbSet.empty_intersection(cb1,cb2)){
                                                            cx_ite=cx_ite.next;
                                                            continue;
                                                        };
                                                        var callbackset:ZPP_CallbackSet=null;
                                                        var ncallbackset:ZPP_CallbackSet=null;
                                                        prelisteners.inlined_clear();
                                                        var lite:ZNPNode_ZPP_InteractionListener=null;
                                                        ZPP_CbSet.find_all(cb1,cb2,ZPP_Flags.id_CbEvent_PRE,function(listener:ZPP_InteractionListener){
                                                            if((listener.itype&inttype)!=0){
                                                                lite=prelisteners.inlined_insert(lite,listener);
                                                                anyimpure=anyimpure||!listener.pure;
                                                            }
                                                        });
                                                        if(prelisteners.empty()){
                                                            cx_ite=cx_ite.next;
                                                            continue;
                                                        };
                                                        callbackset=ZPP_Interactor.get(i1,i2);
                                                        if(callbackset==null){
                                                            ncallbackset=ZPP_CallbackSet.get(i1,i2);
                                                            add_callbackset(ncallbackset);
                                                        }
                                                        if(callbackset==null||((callbackset.COLLISIONstamp!=stamp||continuous)&&(callbackset.COLLISIONstate&ZPP_Flags.id_ImmState_ALWAYS==0))){
                                                            if(ncallbackset!=null)callbackset=ncallbackset;
                                                            if(callbackset!=null){
                                                                {
                                                                    var cx_ite=prelisteners.begin();
                                                                    while(cx_ite!=null){
                                                                        var listener=cx_ite.elem();
                                                                        {
                                                                            if(listener.itype==ZPP_Flags.id_InteractionType_ANY){
                                                                                callbackset.COLLISIONstamp=stamp;
                                                                                callbackset.SENSORstamp=stamp;
                                                                                callbackset.FLUIDstamp=stamp;
                                                                            }
                                                                            else callbackset.COLLISIONstamp=stamp;
                                                                        };
                                                                        cx_ite=cx_ite.next;
                                                                    }
                                                                };
                                                            }
                                                            #if(!NAPE_RELEASE_BUILD)
                                                            arb.makemutable();
                                                            #end
                                                            var pact=arb.active;
                                                            arb.active=true;
                                                            var emptycontacts=false;
                                                            #if true arb.cleanupContacts();
                                                            #end
                                                            precb.zpp_inner.pre_arbiter=arb;
                                                            precb.zpp_inner.set=callbackset;
                                                            {
                                                                var cx_ite=prelisteners.begin();
                                                                while(cx_ite!=null){
                                                                    var listener=cx_ite.elem();
                                                                    {
                                                                        precb.zpp_inner.listener=listener;
                                                                        #if NAPE_ASSERT if(callbackset.int1==null){
                                                                            var err="";
                                                                            err+="OKAY WTF IS HAPPENING HERE\n";
                                                                            err+="ncallbackset is null? "+(ncallbackset==null?"yes":"no")+"\n";
                                                                            err+="assuming yes, let's find callbacksets on interactor\n";
                                                                            err+="i1 (id="+i1.id+"):\n";
                                                                            {
                                                                                var cx_ite=i1.cbsets.begin();
                                                                                while(cx_ite!=null){
                                                                                    var cbset=cx_ite.elem();
                                                                                    err+="  ("+cbset.id+" "+(cbset.int1==null?"null":"")+","+cbset.di+" "+(cbset.int2==null?"null":"")+") arbs size="+cbset.arbiters.size()+" freed="+cbset.freed+" lazydel="+cbset.lazydel+"\n";
                                                                                    cx_ite=cx_ite.next;
                                                                                }
                                                                            };
                                                                            err+="i2 (id="+i2.id+"):\n";
                                                                            {
                                                                                var cx_ite=i2.cbsets.begin();
                                                                                while(cx_ite!=null){
                                                                                    var cbset=cx_ite.elem();
                                                                                    err+="  ("+cbset.id+" "+(cbset.int1==null?"null":"")+","+cbset.di+" "+(cbset.int2==null?"null":"")+") arbs size="+cbset.arbiters.size()+" freed="+cbset.freed+" lazydel="+cbset.lazydel+"\n";
                                                                                    cx_ite=cx_ite.next;
                                                                                }
                                                                            };
                                                                            throw err;
                                                                        }
                                                                        #end
                                                                        ZPP_Interactor.int_callback(callbackset,listener,precb.zpp_inner);
                                                                        precb.zpp_inner.pre_swapped=i1!=precb.zpp_inner.int1;
                                                                        var ret=listener.handlerp(precb);
                                                                        if(ret!=null){
                                                                            arb.immState=if(ret==PreFlag.ACCEPT)ZPP_Flags.id_ImmState_ACCEPT|ZPP_Flags.id_ImmState_ALWAYS else if(ret==PreFlag.ACCEPT_ONCE)ZPP_Flags.id_ImmState_ACCEPT else if(ret==PreFlag.IGNORE)ZPP_Flags.id_ImmState_IGNORE|ZPP_Flags.id_ImmState_ALWAYS else ZPP_Flags.id_ImmState_IGNORE;
                                                                        }
                                                                    };
                                                                    cx_ite=cx_ite.next;
                                                                }
                                                            };
                                                            #if(!NAPE_RELEASE_BUILD)
                                                            arb.makeimmutable();
                                                            #end
                                                            arb.active=pact;
                                                            if(callbackset!=null){
                                                                {
                                                                    var cx_ite=prelisteners.begin();
                                                                    while(cx_ite!=null){
                                                                        var listener=cx_ite.elem();
                                                                        {
                                                                            if(listener.itype==ZPP_Flags.id_InteractionType_ANY){
                                                                                callbackset.COLLISIONstate=arb.immState;
                                                                                callbackset.SENSORstate=arb.immState;
                                                                                callbackset.FLUIDstate=arb.immState;
                                                                            }
                                                                            else callbackset.COLLISIONstate=arb.immState;
                                                                        };
                                                                        cx_ite=cx_ite.next;
                                                                    }
                                                                };
                                                            }
                                                        }
                                                        else if(callbackset==null){
                                                            if((arb.immState&ZPP_Flags.id_ImmState_ALWAYS)==0)arb.immState=ZPP_Flags.id_ImmState_ACCEPT;
                                                        }
                                                        else arb.immState=callbackset.COLLISIONstate;
                                                    };
                                                    cx_ite=cx_ite.next;
                                                }
                                            };
                                        };
                                        cx_ite=cx_ite.next;
                                    }
                                };
                                if(anyimpure&&(arb.immState&ZPP_Flags.id_ImmState_ALWAYS)==0){
                                    if(true){
                                        if(arb.b1.isDynamic()){
                                            var o=arb.b1;
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    o.space==this;
                                                };
                                                if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                                #end
                                            };
                                            if(!o.world){
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        o.component!=null;
                                                    };
                                                    if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                                    #end
                                                };
                                                o.component.waket=stamp+(midstep?0:1);
                                                if(o.isKinematic())o.kinematicDelaySleep=true;
                                                if(o.component.sleeping){
                                                    really_wake(o,false);
                                                }
                                            }
                                        };
                                        if(arb.b1.isDynamic()){
                                            var o=arb.b2;
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    o.space==this;
                                                };
                                                if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                                #end
                                            };
                                            if(!o.world){
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        o.component!=null;
                                                    };
                                                    if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                                    #end
                                                };
                                                o.component.waket=stamp+(midstep?0:1);
                                                if(o.isKinematic())o.kinematicDelaySleep=true;
                                                if(o.component.sleeping){
                                                    really_wake(o,false);
                                                }
                                            }
                                        };
                                    }
                                    else{
                                        if(!arb.b1.isStatic()){
                                            var o=arb.b1;
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    o.space==this;
                                                };
                                                if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                                #end
                                            };
                                            if(!o.world){
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        o.component!=null;
                                                    };
                                                    if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                                    #end
                                                };
                                                o.component.waket=stamp+(midstep?0:1);
                                                if(o.isKinematic())o.kinematicDelaySleep=true;
                                                if(o.component.sleeping){
                                                    really_wake(o,false);
                                                }
                                            }
                                        };
                                        if(!arb.b2.isStatic()){
                                            var o=arb.b2;
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    o.space==this;
                                                };
                                                if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                                #end
                                            };
                                            if(!o.world){
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        o.component!=null;
                                                    };
                                                    if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                                    #end
                                                };
                                                o.component.waket=stamp+(midstep?0:1);
                                                if(o.isKinematic())o.kinematicDelaySleep=true;
                                                if(o.component.sleeping){
                                                    really_wake(o,false);
                                                }
                                            }
                                        };
                                    }
                                }
                            }
                            if(true&&(arb.immState&ZPP_Flags.id_ImmState_ACCEPT)!=0){
                                if(arb.b1.isDynamic()&&arb.b1.component.sleeping){
                                    var o=arb.b1;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o.space==this;
                                        };
                                        if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                        #end
                                    };
                                    if(!o.world){
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                o.component!=null;
                                            };
                                            if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                            #end
                                        };
                                        o.component.waket=stamp+(midstep?0:1);
                                        if(o.isKinematic())o.kinematicDelaySleep=true;
                                        if(o.component.sleeping){
                                            really_wake(o,false);
                                        }
                                    }
                                };
                                if(arb.b2.isDynamic()&&arb.b2.component.sleeping){
                                    var o=arb.b2;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o.space==this;
                                        };
                                        if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                        #end
                                    };
                                    if(!o.world){
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                o.component!=null;
                                            };
                                            if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                            #end
                                        };
                                        o.component.waket=stamp+(midstep?0:1);
                                        if(o.isKinematic())o.kinematicDelaySleep=true;
                                        if(o.component.sleeping){
                                            really_wake(o,false);
                                        }
                                    }
                                };
                            }
                            if(arb.sleeping){
                                arb.sleeping=false;
                                carbs.inlined_add(arb);
                            }
                            arb;
                        }
                        else if(first){
                            {
                                var o=arb;
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        o!=null;
                                    };
                                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_ColArbiter"+", in obj: "+"arb"+")");
                                    #end
                                };
                                o.free();
                                o.next=ZPP_ColArbiter.zpp_pool;
                                ZPP_ColArbiter.zpp_pool=o;
                                #if NAPE_POOL_STATS ZPP_ColArbiter.POOL_CNT++;
                                ZPP_ColArbiter.POOL_SUB++;
                                #end
                            };
                            null;
                        }
                        else arb;
                    }
                    else arb;
                };
            }
            else{
                var xarb=if(in_arb==null)({
                    var ret:ZPP_Arbiter=null;
                    var b=if(b1.arbiters.length<b2.arbiters.length)b1 else b2;
                    {
                        var cx_ite=b.arbiters.begin();
                        while(cx_ite!=null){
                            var arb=cx_ite.elem();
                            {
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !(arb.id==sb.id&&arb.di==sa.id);
                                    };
                                    if(!res)throw "assert("+"!(arb.id==sb.id&&arb.di==sa.id)"+") :: "+("arbiter id order doesn't match s1/s2 order?>");
                                    #end
                                };
                                if(arb.id==sa.id&&arb.di==sb.id){
                                    ret=arb;
                                    break;
                                }
                            };
                            cx_ite=cx_ite.next;
                        }
                    };
                    ret;
                })else in_arb;
                var first=xarb==null;
                var arb;
                var swapped=false;
                if(first){
                    if(ZPP_SensorArbiter.zpp_pool==null){
                        arb=new ZPP_SensorArbiter();
                        #if NAPE_POOL_STATS ZPP_SensorArbiter.POOL_TOT++;
                        ZPP_SensorArbiter.POOL_ADDNEW++;
                        #end
                    }
                    else{
                        arb=ZPP_SensorArbiter.zpp_pool;
                        ZPP_SensorArbiter.zpp_pool=arb.next;
                        arb.next=null;
                        #if NAPE_POOL_STATS ZPP_SensorArbiter.POOL_CNT--;
                        ZPP_SensorArbiter.POOL_ADD++;
                        #end
                    }
                    arb.alloc();
                }
                else{
                    if(xarb.sensorarb==null){
                        xarb.lazyRetire(this,null);
                        {
                            if(ZPP_SensorArbiter.zpp_pool==null){
                                arb=new ZPP_SensorArbiter();
                                #if NAPE_POOL_STATS ZPP_SensorArbiter.POOL_TOT++;
                                ZPP_SensorArbiter.POOL_ADDNEW++;
                                #end
                            }
                            else{
                                arb=ZPP_SensorArbiter.zpp_pool;
                                ZPP_SensorArbiter.zpp_pool=arb.next;
                                arb.next=null;
                                #if NAPE_POOL_STATS ZPP_SensorArbiter.POOL_CNT--;
                                ZPP_SensorArbiter.POOL_ADD++;
                                #end
                            }
                            arb.alloc();
                        };
                        arb.intchange=true;
                        first=true;
                        swapped=true;
                    }
                    else arb=xarb.sensorarb;
                }
                ret={
                    var inttype=ZPP_Flags.id_InteractionType_SENSOR;
                    if(first||arb.stamp!=stamp||continuous){
                        arb.stamp=stamp;
                        if(ZPP_Collide.testCollide(sa,sb)){
                            if(first){
                                arb.assign(s1,s2,sa.id,sb.id);
                                s_arbiters.inlined_add(arb);
                                arb.fresh=!swapped;
                            }
                            else{
                                arb.fresh=(arb.up_stamp<stamp-1||(arb.endGenerated==stamp&&continuous));
                            }
                            arb.up_stamp=arb.stamp;
                            if(arb.fresh||(arb.immState&ZPP_Flags.id_ImmState_ALWAYS)==0){
                                arb.immState=ZPP_Flags.id_ImmState_ACCEPT;
                                var anyimpure=false;
                                var arbs1=arb.ws1.id>arb.ws2.id?arb.ws2:arb.ws1;
                                var arbs2=arb.ws1.id>arb.ws2.id?arb.ws1:arb.ws2;
                                inlined_MRCA_chains(arbs1,arbs2);
                                {
                                    var cx_ite=mrca1.begin();
                                    while(cx_ite!=null){
                                        var i1=cx_ite.elem();
                                        {
                                            {
                                                var cx_ite=mrca2.begin();
                                                while(cx_ite!=null){
                                                    var i2=cx_ite.elem();
                                                    {
                                                        var cb1=i1.cbSet;
                                                        var cb2=i2.cbSet;
                                                        {
                                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                            var res={
                                                                cb1!=null&&cb2!=null;
                                                            };
                                                            if(!res)throw "assert("+"cb1!=null&&cb2!=null"+") :: "+("null cb from MRCA_chains?");
                                                            #end
                                                        };
                                                        if(ZPP_CbSet.empty_intersection(cb1,cb2)){
                                                            cx_ite=cx_ite.next;
                                                            continue;
                                                        };
                                                        var callbackset:ZPP_CallbackSet=null;
                                                        var ncallbackset:ZPP_CallbackSet=null;
                                                        prelisteners.inlined_clear();
                                                        var lite:ZNPNode_ZPP_InteractionListener=null;
                                                        ZPP_CbSet.find_all(cb1,cb2,ZPP_Flags.id_CbEvent_PRE,function(listener:ZPP_InteractionListener){
                                                            if((listener.itype&inttype)!=0){
                                                                lite=prelisteners.inlined_insert(lite,listener);
                                                                anyimpure=anyimpure||!listener.pure;
                                                            }
                                                        });
                                                        if(prelisteners.empty()){
                                                            cx_ite=cx_ite.next;
                                                            continue;
                                                        };
                                                        callbackset=ZPP_Interactor.get(i1,i2);
                                                        if(callbackset==null){
                                                            ncallbackset=ZPP_CallbackSet.get(i1,i2);
                                                            add_callbackset(ncallbackset);
                                                        }
                                                        if(callbackset==null||((callbackset.SENSORstamp!=stamp||continuous)&&(callbackset.SENSORstate&ZPP_Flags.id_ImmState_ALWAYS==0))){
                                                            if(ncallbackset!=null)callbackset=ncallbackset;
                                                            if(callbackset!=null){
                                                                {
                                                                    var cx_ite=prelisteners.begin();
                                                                    while(cx_ite!=null){
                                                                        var listener=cx_ite.elem();
                                                                        {
                                                                            if(listener.itype==ZPP_Flags.id_InteractionType_ANY){
                                                                                callbackset.COLLISIONstamp=stamp;
                                                                                callbackset.SENSORstamp=stamp;
                                                                                callbackset.FLUIDstamp=stamp;
                                                                            }
                                                                            else callbackset.SENSORstamp=stamp;
                                                                        };
                                                                        cx_ite=cx_ite.next;
                                                                    }
                                                                };
                                                            }
                                                            #if(!NAPE_RELEASE_BUILD)
                                                            arb.makemutable();
                                                            #end
                                                            var pact=arb.active;
                                                            arb.active=true;
                                                            var emptycontacts=false;
                                                            #if false arb.cleanupContacts();
                                                            #end
                                                            precb.zpp_inner.pre_arbiter=arb;
                                                            precb.zpp_inner.set=callbackset;
                                                            {
                                                                var cx_ite=prelisteners.begin();
                                                                while(cx_ite!=null){
                                                                    var listener=cx_ite.elem();
                                                                    {
                                                                        precb.zpp_inner.listener=listener;
                                                                        #if NAPE_ASSERT if(callbackset.int1==null){
                                                                            var err="";
                                                                            err+="OKAY WTF IS HAPPENING HERE\n";
                                                                            err+="ncallbackset is null? "+(ncallbackset==null?"yes":"no")+"\n";
                                                                            err+="assuming yes, let's find callbacksets on interactor\n";
                                                                            err+="i1 (id="+i1.id+"):\n";
                                                                            {
                                                                                var cx_ite=i1.cbsets.begin();
                                                                                while(cx_ite!=null){
                                                                                    var cbset=cx_ite.elem();
                                                                                    err+="  ("+cbset.id+" "+(cbset.int1==null?"null":"")+","+cbset.di+" "+(cbset.int2==null?"null":"")+") arbs size="+cbset.arbiters.size()+" freed="+cbset.freed+" lazydel="+cbset.lazydel+"\n";
                                                                                    cx_ite=cx_ite.next;
                                                                                }
                                                                            };
                                                                            err+="i2 (id="+i2.id+"):\n";
                                                                            {
                                                                                var cx_ite=i2.cbsets.begin();
                                                                                while(cx_ite!=null){
                                                                                    var cbset=cx_ite.elem();
                                                                                    err+="  ("+cbset.id+" "+(cbset.int1==null?"null":"")+","+cbset.di+" "+(cbset.int2==null?"null":"")+") arbs size="+cbset.arbiters.size()+" freed="+cbset.freed+" lazydel="+cbset.lazydel+"\n";
                                                                                    cx_ite=cx_ite.next;
                                                                                }
                                                                            };
                                                                            throw err;
                                                                        }
                                                                        #end
                                                                        ZPP_Interactor.int_callback(callbackset,listener,precb.zpp_inner);
                                                                        precb.zpp_inner.pre_swapped=i1!=precb.zpp_inner.int1;
                                                                        var ret=listener.handlerp(precb);
                                                                        if(ret!=null){
                                                                            arb.immState=if(ret==PreFlag.ACCEPT)ZPP_Flags.id_ImmState_ACCEPT|ZPP_Flags.id_ImmState_ALWAYS else if(ret==PreFlag.ACCEPT_ONCE)ZPP_Flags.id_ImmState_ACCEPT else if(ret==PreFlag.IGNORE)ZPP_Flags.id_ImmState_IGNORE|ZPP_Flags.id_ImmState_ALWAYS else ZPP_Flags.id_ImmState_IGNORE;
                                                                        }
                                                                    };
                                                                    cx_ite=cx_ite.next;
                                                                }
                                                            };
                                                            #if(!NAPE_RELEASE_BUILD)
                                                            arb.makeimmutable();
                                                            #end
                                                            arb.active=pact;
                                                            if(callbackset!=null){
                                                                {
                                                                    var cx_ite=prelisteners.begin();
                                                                    while(cx_ite!=null){
                                                                        var listener=cx_ite.elem();
                                                                        {
                                                                            if(listener.itype==ZPP_Flags.id_InteractionType_ANY){
                                                                                callbackset.COLLISIONstate=arb.immState;
                                                                                callbackset.SENSORstate=arb.immState;
                                                                                callbackset.FLUIDstate=arb.immState;
                                                                            }
                                                                            else callbackset.SENSORstate=arb.immState;
                                                                        };
                                                                        cx_ite=cx_ite.next;
                                                                    }
                                                                };
                                                            }
                                                        }
                                                        else if(callbackset==null){
                                                            if((arb.immState&ZPP_Flags.id_ImmState_ALWAYS)==0)arb.immState=ZPP_Flags.id_ImmState_ACCEPT;
                                                        }
                                                        else arb.immState=callbackset.SENSORstate;
                                                    };
                                                    cx_ite=cx_ite.next;
                                                }
                                            };
                                        };
                                        cx_ite=cx_ite.next;
                                    }
                                };
                                if(anyimpure&&(arb.immState&ZPP_Flags.id_ImmState_ALWAYS)==0){
                                    if(false){
                                        if(arb.b1.isDynamic()){
                                            var o=arb.b1;
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    o.space==this;
                                                };
                                                if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                                #end
                                            };
                                            if(!o.world){
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        o.component!=null;
                                                    };
                                                    if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                                    #end
                                                };
                                                o.component.waket=stamp+(midstep?0:1);
                                                if(o.isKinematic())o.kinematicDelaySleep=true;
                                                if(o.component.sleeping){
                                                    really_wake(o,false);
                                                }
                                            }
                                        };
                                        if(arb.b1.isDynamic()){
                                            var o=arb.b2;
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    o.space==this;
                                                };
                                                if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                                #end
                                            };
                                            if(!o.world){
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        o.component!=null;
                                                    };
                                                    if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                                    #end
                                                };
                                                o.component.waket=stamp+(midstep?0:1);
                                                if(o.isKinematic())o.kinematicDelaySleep=true;
                                                if(o.component.sleeping){
                                                    really_wake(o,false);
                                                }
                                            }
                                        };
                                    }
                                    else{
                                        if(!arb.b1.isStatic()){
                                            var o=arb.b1;
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    o.space==this;
                                                };
                                                if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                                #end
                                            };
                                            if(!o.world){
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        o.component!=null;
                                                    };
                                                    if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                                    #end
                                                };
                                                o.component.waket=stamp+(midstep?0:1);
                                                if(o.isKinematic())o.kinematicDelaySleep=true;
                                                if(o.component.sleeping){
                                                    really_wake(o,false);
                                                }
                                            }
                                        };
                                        if(!arb.b2.isStatic()){
                                            var o=arb.b2;
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    o.space==this;
                                                };
                                                if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                                #end
                                            };
                                            if(!o.world){
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        o.component!=null;
                                                    };
                                                    if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                                    #end
                                                };
                                                o.component.waket=stamp+(midstep?0:1);
                                                if(o.isKinematic())o.kinematicDelaySleep=true;
                                                if(o.component.sleeping){
                                                    really_wake(o,false);
                                                }
                                            }
                                        };
                                    }
                                }
                            }
                            if(false&&(arb.immState&ZPP_Flags.id_ImmState_ACCEPT)!=0){
                                if(arb.b1.isDynamic()&&arb.b1.component.sleeping){
                                    var o=arb.b1;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o.space==this;
                                        };
                                        if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                        #end
                                    };
                                    if(!o.world){
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                o.component!=null;
                                            };
                                            if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                            #end
                                        };
                                        o.component.waket=stamp+(midstep?0:1);
                                        if(o.isKinematic())o.kinematicDelaySleep=true;
                                        if(o.component.sleeping){
                                            really_wake(o,false);
                                        }
                                    }
                                };
                                if(arb.b2.isDynamic()&&arb.b2.component.sleeping){
                                    var o=arb.b2;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o.space==this;
                                        };
                                        if(!res)throw "assert("+"o.space==this"+") :: "+("object being woken in the space... is not actually in the space!");
                                        #end
                                    };
                                    if(!o.world){
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                o.component!=null;
                                            };
                                            if(!res)throw "assert("+"o.component!=null"+") :: "+("body woken, but no component exists?");
                                            #end
                                        };
                                        o.component.waket=stamp+(midstep?0:1);
                                        if(o.isKinematic())o.kinematicDelaySleep=true;
                                        if(o.component.sleeping){
                                            really_wake(o,false);
                                        }
                                    }
                                };
                            }
                            if(arb.sleeping){
                                arb.sleeping=false;
                                s_arbiters.inlined_add(arb);
                            }
                            arb;
                        }
                        else if(first){
                            {
                                var o=arb;
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        o!=null;
                                    };
                                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SensorArbiter"+", in obj: "+"arb"+")");
                                    #end
                                };
                                o.free();
                                o.next=ZPP_SensorArbiter.zpp_pool;
                                ZPP_SensorArbiter.zpp_pool=o;
                                #if NAPE_POOL_STATS ZPP_SensorArbiter.POOL_CNT++;
                                ZPP_SensorArbiter.POOL_SUB++;
                                #end
                            };
                            null;
                        }
                        else arb;
                    }
                    else arb;
                };
            }
        }
        #if NAPE_TIMES Debug.NARROW+=flash.Lib.getTimer()-pt;
        #end
        return ret;
    }
    public var mrca1:ZNPList_ZPP_Interactor;
    public var mrca2:ZNPList_ZPP_Interactor;
    public function MRCA_chains(s1:ZPP_Shape,s2:ZPP_Shape){
        inlined_MRCA_chains(s1,s2);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_MRCA_chains(s1:ZPP_Shape,s2:ZPP_Shape){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                s1!=s2&&s1.body!=s2.body;
            };
            if(!res)throw "assert("+"s1!=s2&&s1.body!=s2.body"+") :: "+("MRCA chain for equal shapes, or shapes of the same body?");
            #end
        };
        mrca1.inlined_clear();
        mrca2.inlined_clear();
        if(s1.cbSet!=null)mrca1.inlined_add(s1);
        if(s1.body.cbSet!=null)mrca1.inlined_add(s1.body);
        if(s2.cbSet!=null)mrca2.inlined_add(s2);
        if(s2.body.cbSet!=null)mrca2.inlined_add(s2.body);
        var c1=s1.body.compound;
        var c2=s2.body.compound;
        while(c1!=c2){
            var d1=if(c1==null)0 else c1.depth;
            var d2=if(c2==null)0 else c2.depth;
            if(d1<d2){
                if(c2.cbSet!=null)mrca2.inlined_add(c2);
                c2=c2.compound;
            }
            else{
                if(c1.cbSet!=null)mrca1.inlined_add(c1);
                c1=c1.compound;
            }
        }
    }
}
