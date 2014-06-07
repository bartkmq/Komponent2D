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
class ZNPList_ZPP_CbType{
    public var head:ZNPNode_ZPP_CbType=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_CbType{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_CbType):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_CbType):ZPP_CbType{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_CbType):ZPP_CbType{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_CbType"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_CbType.zpp_pool==null){
                    ret=new ZNPNode_ZPP_CbType();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CbType.POOL_TOT++;
                    ZNPNode_ZPP_CbType.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_CbType.zpp_pool;
                    ZNPNode_ZPP_CbType.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CbType.POOL_CNT--;
                    ZNPNode_ZPP_CbType.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_CbType):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_CbType"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_CbType,o:ZPP_CbType):ZNPNode_ZPP_CbType{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_CbType,o:ZPP_CbType):ZNPNode_ZPP_CbType{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_CbType"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_CbType.zpp_pool==null){
                    ret=new ZNPNode_ZPP_CbType();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CbType.POOL_TOT++;
                    ZNPNode_ZPP_CbType.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_CbType.zpp_pool;
                    ZNPNode_ZPP_CbType.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CbType.POOL_CNT--;
                    ZNPNode_ZPP_CbType.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_CbType"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_CbType"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_CbType.zpp_pool;
            ZNPNode_ZPP_CbType.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_CbType.POOL_CNT++;
            ZNPNode_ZPP_CbType.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_CbType{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_CbType{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_CbType"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_CbType):Void{
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
    public function try_remove(obj:ZPP_CbType):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_CbType"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_CbType):Void{
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
    function inlined_try_remove(obj:ZPP_CbType):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_CbType"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_CbType):ZNPNode_ZPP_CbType{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_CbType):ZNPNode_ZPP_CbType{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_CbType"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_CbType;
        var ret:ZNPNode_ZPP_CbType;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_CbType"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_CbType.zpp_pool;
            ZNPNode_ZPP_CbType.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_CbType.POOL_CNT++;
            ZNPNode_ZPP_CbType.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_CbType,n:Int):ZNPNode_ZPP_CbType{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_CbType):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_CbType):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_CbType"+"] has -> "+obj);
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
    function front():ZPP_CbType{
        return begin().elem();
    }
    public function back():ZPP_CbType{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_CbType{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_CbType"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_CbType{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_CbType"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_CallbackSet{
    public var head:ZNPNode_ZPP_CallbackSet=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_CallbackSet{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_CallbackSet):Void{
        head=i;
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
            var ret;
            {
                if(ZNPNode_ZPP_CallbackSet.zpp_pool==null){
                    ret=new ZNPNode_ZPP_CallbackSet();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CallbackSet.POOL_TOT++;
                    ZNPNode_ZPP_CallbackSet.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_CallbackSet.zpp_pool;
                    ZNPNode_ZPP_CallbackSet.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CallbackSet.POOL_CNT--;
                    ZNPNode_ZPP_CallbackSet.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_CallbackSet):Void{
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
    public function insert(cur:ZNPNode_ZPP_CallbackSet,o:ZPP_CallbackSet):ZNPNode_ZPP_CallbackSet{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_CallbackSet,o:ZPP_CallbackSet):ZNPNode_ZPP_CallbackSet{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_CallbackSet"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_CallbackSet.zpp_pool==null){
                    ret=new ZNPNode_ZPP_CallbackSet();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CallbackSet.POOL_TOT++;
                    ZNPNode_ZPP_CallbackSet.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_CallbackSet.zpp_pool;
                    ZNPNode_ZPP_CallbackSet.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CallbackSet.POOL_CNT--;
                    ZNPNode_ZPP_CallbackSet.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_CallbackSet"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_CallbackSet.zpp_pool;
            ZNPNode_ZPP_CallbackSet.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_CallbackSet.POOL_CNT++;
            ZNPNode_ZPP_CallbackSet.POOL_SUB++;
            #end
        };
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
    public function erase(pre:ZNPNode_ZPP_CallbackSet):ZNPNode_ZPP_CallbackSet{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_CallbackSet):ZNPNode_ZPP_CallbackSet{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_CallbackSet"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_CallbackSet;
        var ret:ZNPNode_ZPP_CallbackSet;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_CallbackSet"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_CallbackSet.zpp_pool;
            ZNPNode_ZPP_CallbackSet.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_CallbackSet.POOL_CNT++;
            ZNPNode_ZPP_CallbackSet.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_CallbackSet,n:Int):ZNPNode_ZPP_CallbackSet{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function iterator_at(ind:Int):ZNPNode_ZPP_CallbackSet{
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
}
#if nape_swc@:keep #end
class ZNPList_ZPP_Shape{
    public var head:ZNPNode_ZPP_Shape=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_Shape{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_Shape):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_Shape):ZPP_Shape{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_Shape):ZPP_Shape{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Shape"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_Shape.zpp_pool==null){
                    ret=new ZNPNode_ZPP_Shape();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Shape.POOL_TOT++;
                    ZNPNode_ZPP_Shape.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_Shape.zpp_pool;
                    ZNPNode_ZPP_Shape.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Shape.POOL_CNT--;
                    ZNPNode_ZPP_Shape.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_Shape):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_Shape"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_Shape,o:ZPP_Shape):ZNPNode_ZPP_Shape{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_Shape,o:ZPP_Shape):ZNPNode_ZPP_Shape{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Shape"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_Shape.zpp_pool==null){
                    ret=new ZNPNode_ZPP_Shape();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Shape.POOL_TOT++;
                    ZNPNode_ZPP_Shape.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_Shape.zpp_pool;
                    ZNPNode_ZPP_Shape.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Shape.POOL_CNT--;
                    ZNPNode_ZPP_Shape.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Shape"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_Shape"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_Shape.zpp_pool;
            ZNPNode_ZPP_Shape.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_Shape.POOL_CNT++;
            ZNPNode_ZPP_Shape.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_Shape{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_Shape{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Shape"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_Shape):Void{
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
    public function try_remove(obj:ZPP_Shape):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Shape"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_Shape):Void{
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
    function inlined_try_remove(obj:ZPP_Shape):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Shape"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_Shape):ZNPNode_ZPP_Shape{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_Shape):ZNPNode_ZPP_Shape{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Shape"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_Shape;
        var ret:ZNPNode_ZPP_Shape;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_Shape"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_Shape.zpp_pool;
            ZNPNode_ZPP_Shape.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_Shape.POOL_CNT++;
            ZNPNode_ZPP_Shape.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_Shape,n:Int):ZNPNode_ZPP_Shape{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_Shape):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_Shape):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Shape"+"] has -> "+obj);
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
    function front():ZPP_Shape{
        return begin().elem();
    }
    public function back():ZPP_Shape{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_Shape{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Shape"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_Shape{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Shape"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_Body{
    public var head:ZNPNode_ZPP_Body=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_Body{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_Body):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_Body):ZPP_Body{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_Body):ZPP_Body{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Body"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_Body.zpp_pool==null){
                    ret=new ZNPNode_ZPP_Body();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Body.POOL_TOT++;
                    ZNPNode_ZPP_Body.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_Body.zpp_pool;
                    ZNPNode_ZPP_Body.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Body.POOL_CNT--;
                    ZNPNode_ZPP_Body.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_Body):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_Body"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_Body,o:ZPP_Body):ZNPNode_ZPP_Body{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_Body,o:ZPP_Body):ZNPNode_ZPP_Body{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Body"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_Body.zpp_pool==null){
                    ret=new ZNPNode_ZPP_Body();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Body.POOL_TOT++;
                    ZNPNode_ZPP_Body.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_Body.zpp_pool;
                    ZNPNode_ZPP_Body.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Body.POOL_CNT--;
                    ZNPNode_ZPP_Body.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Body"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_Body"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_Body.zpp_pool;
            ZNPNode_ZPP_Body.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_Body.POOL_CNT++;
            ZNPNode_ZPP_Body.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_Body{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_Body{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Body"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_Body):Void{
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
    public function try_remove(obj:ZPP_Body):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Body"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_Body):Void{
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
    function inlined_try_remove(obj:ZPP_Body):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Body"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_Body):ZNPNode_ZPP_Body{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_Body):ZNPNode_ZPP_Body{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Body"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_Body;
        var ret:ZNPNode_ZPP_Body;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_Body"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_Body.zpp_pool;
            ZNPNode_ZPP_Body.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_Body.POOL_CNT++;
            ZNPNode_ZPP_Body.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_Body,n:Int):ZNPNode_ZPP_Body{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_Body):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_Body):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Body"+"] has -> "+obj);
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
    function front():ZPP_Body{
        return begin().elem();
    }
    public function back():ZPP_Body{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_Body{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Body"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_Body{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Body"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_Constraint{
    public var head:ZNPNode_ZPP_Constraint=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_Constraint{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_Constraint):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_Constraint):ZPP_Constraint{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_Constraint):ZPP_Constraint{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Constraint"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_Constraint.zpp_pool==null){
                    ret=new ZNPNode_ZPP_Constraint();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Constraint.POOL_TOT++;
                    ZNPNode_ZPP_Constraint.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_Constraint.zpp_pool;
                    ZNPNode_ZPP_Constraint.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Constraint.POOL_CNT--;
                    ZNPNode_ZPP_Constraint.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_Constraint):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_Constraint"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_Constraint,o:ZPP_Constraint):ZNPNode_ZPP_Constraint{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_Constraint,o:ZPP_Constraint):ZNPNode_ZPP_Constraint{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Constraint"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_Constraint.zpp_pool==null){
                    ret=new ZNPNode_ZPP_Constraint();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Constraint.POOL_TOT++;
                    ZNPNode_ZPP_Constraint.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_Constraint.zpp_pool;
                    ZNPNode_ZPP_Constraint.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Constraint.POOL_CNT--;
                    ZNPNode_ZPP_Constraint.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Constraint"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_Constraint"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_Constraint.zpp_pool;
            ZNPNode_ZPP_Constraint.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_Constraint.POOL_CNT++;
            ZNPNode_ZPP_Constraint.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_Constraint{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_Constraint{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Constraint"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_Constraint):Void{
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
    public function try_remove(obj:ZPP_Constraint):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Constraint"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_Constraint):Void{
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
    function inlined_try_remove(obj:ZPP_Constraint):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Constraint"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_Constraint):ZNPNode_ZPP_Constraint{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_Constraint):ZNPNode_ZPP_Constraint{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Constraint"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_Constraint;
        var ret:ZNPNode_ZPP_Constraint;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_Constraint"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_Constraint.zpp_pool;
            ZNPNode_ZPP_Constraint.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_Constraint.POOL_CNT++;
            ZNPNode_ZPP_Constraint.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_Constraint,n:Int):ZNPNode_ZPP_Constraint{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_Constraint):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_Constraint):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Constraint"+"] has -> "+obj);
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
    function front():ZPP_Constraint{
        return begin().elem();
    }
    public function back():ZPP_Constraint{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_Constraint{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Constraint"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_Constraint{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Constraint"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_Compound{
    public var head:ZNPNode_ZPP_Compound=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_Compound{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_Compound):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_Compound):ZPP_Compound{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_Compound):ZPP_Compound{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Compound"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_Compound.zpp_pool==null){
                    ret=new ZNPNode_ZPP_Compound();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Compound.POOL_TOT++;
                    ZNPNode_ZPP_Compound.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_Compound.zpp_pool;
                    ZNPNode_ZPP_Compound.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Compound.POOL_CNT--;
                    ZNPNode_ZPP_Compound.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_Compound):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_Compound"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_Compound,o:ZPP_Compound):ZNPNode_ZPP_Compound{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_Compound,o:ZPP_Compound):ZNPNode_ZPP_Compound{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Compound"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_Compound.zpp_pool==null){
                    ret=new ZNPNode_ZPP_Compound();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Compound.POOL_TOT++;
                    ZNPNode_ZPP_Compound.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_Compound.zpp_pool;
                    ZNPNode_ZPP_Compound.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Compound.POOL_CNT--;
                    ZNPNode_ZPP_Compound.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Compound"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_Compound"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_Compound.zpp_pool;
            ZNPNode_ZPP_Compound.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_Compound.POOL_CNT++;
            ZNPNode_ZPP_Compound.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_Compound{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_Compound{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Compound"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_Compound):Void{
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
    public function try_remove(obj:ZPP_Compound):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Compound"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_Compound):Void{
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
    function inlined_try_remove(obj:ZPP_Compound):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Compound"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_Compound):ZNPNode_ZPP_Compound{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_Compound):ZNPNode_ZPP_Compound{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Compound"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_Compound;
        var ret:ZNPNode_ZPP_Compound;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_Compound"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_Compound.zpp_pool;
            ZNPNode_ZPP_Compound.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_Compound.POOL_CNT++;
            ZNPNode_ZPP_Compound.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_Compound,n:Int):ZNPNode_ZPP_Compound{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_Compound):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_Compound):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Compound"+"] has -> "+obj);
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
    function front():ZPP_Compound{
        return begin().elem();
    }
    public function back():ZPP_Compound{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_Compound{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Compound"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_Compound{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Compound"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_Arbiter{
    public var head:ZNPNode_ZPP_Arbiter=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_Arbiter{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_Arbiter):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_Arbiter):ZPP_Arbiter{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_Arbiter):ZPP_Arbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Arbiter"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_Arbiter.zpp_pool==null){
                    ret=new ZNPNode_ZPP_Arbiter();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Arbiter.POOL_TOT++;
                    ZNPNode_ZPP_Arbiter.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_Arbiter.zpp_pool;
                    ZNPNode_ZPP_Arbiter.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Arbiter.POOL_CNT--;
                    ZNPNode_ZPP_Arbiter.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_Arbiter):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_Arbiter"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_Arbiter,o:ZPP_Arbiter):ZNPNode_ZPP_Arbiter{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_Arbiter,o:ZPP_Arbiter):ZNPNode_ZPP_Arbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Arbiter"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_Arbiter.zpp_pool==null){
                    ret=new ZNPNode_ZPP_Arbiter();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Arbiter.POOL_TOT++;
                    ZNPNode_ZPP_Arbiter.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_Arbiter.zpp_pool;
                    ZNPNode_ZPP_Arbiter.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Arbiter.POOL_CNT--;
                    ZNPNode_ZPP_Arbiter.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Arbiter"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_Arbiter"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_Arbiter.zpp_pool;
            ZNPNode_ZPP_Arbiter.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_Arbiter.POOL_CNT++;
            ZNPNode_ZPP_Arbiter.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_Arbiter{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_Arbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Arbiter"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_Arbiter):Void{
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
    public function try_remove(obj:ZPP_Arbiter):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Arbiter"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_Arbiter):Void{
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
    function inlined_try_remove(obj:ZPP_Arbiter):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Arbiter"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_Arbiter):ZNPNode_ZPP_Arbiter{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_Arbiter):ZNPNode_ZPP_Arbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Arbiter"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_Arbiter;
        var ret:ZNPNode_ZPP_Arbiter;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_Arbiter"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_Arbiter.zpp_pool;
            ZNPNode_ZPP_Arbiter.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_Arbiter.POOL_CNT++;
            ZNPNode_ZPP_Arbiter.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_Arbiter,n:Int):ZNPNode_ZPP_Arbiter{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_Arbiter):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_Arbiter):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Arbiter"+"] has -> "+obj);
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
    function front():ZPP_Arbiter{
        return begin().elem();
    }
    public function back():ZPP_Arbiter{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_Arbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Arbiter"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_Arbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Arbiter"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_InteractionListener{
    public var head:ZNPNode_ZPP_InteractionListener=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_InteractionListener{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_InteractionListener):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_InteractionListener):ZPP_InteractionListener{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_InteractionListener):ZPP_InteractionListener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_InteractionListener"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_InteractionListener.zpp_pool==null){
                    ret=new ZNPNode_ZPP_InteractionListener();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_InteractionListener.POOL_TOT++;
                    ZNPNode_ZPP_InteractionListener.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_InteractionListener.zpp_pool;
                    ZNPNode_ZPP_InteractionListener.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_InteractionListener.POOL_CNT--;
                    ZNPNode_ZPP_InteractionListener.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_InteractionListener):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_InteractionListener"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_InteractionListener,o:ZPP_InteractionListener):ZNPNode_ZPP_InteractionListener{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_InteractionListener,o:ZPP_InteractionListener):ZNPNode_ZPP_InteractionListener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_InteractionListener"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_InteractionListener.zpp_pool==null){
                    ret=new ZNPNode_ZPP_InteractionListener();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_InteractionListener.POOL_TOT++;
                    ZNPNode_ZPP_InteractionListener.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_InteractionListener.zpp_pool;
                    ZNPNode_ZPP_InteractionListener.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_InteractionListener.POOL_CNT--;
                    ZNPNode_ZPP_InteractionListener.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_InteractionListener"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_InteractionListener"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_InteractionListener.zpp_pool;
            ZNPNode_ZPP_InteractionListener.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_InteractionListener.POOL_CNT++;
            ZNPNode_ZPP_InteractionListener.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_InteractionListener{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_InteractionListener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_InteractionListener"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_InteractionListener):Void{
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
    public function try_remove(obj:ZPP_InteractionListener):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_InteractionListener"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_InteractionListener):Void{
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
    function inlined_try_remove(obj:ZPP_InteractionListener):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_InteractionListener"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_InteractionListener):ZNPNode_ZPP_InteractionListener{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_InteractionListener):ZNPNode_ZPP_InteractionListener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_InteractionListener"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_InteractionListener;
        var ret:ZNPNode_ZPP_InteractionListener;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_InteractionListener"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_InteractionListener.zpp_pool;
            ZNPNode_ZPP_InteractionListener.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_InteractionListener.POOL_CNT++;
            ZNPNode_ZPP_InteractionListener.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_InteractionListener,n:Int):ZNPNode_ZPP_InteractionListener{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_InteractionListener):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_InteractionListener):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_InteractionListener"+"] has -> "+obj);
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
    function front():ZPP_InteractionListener{
        return begin().elem();
    }
    public function back():ZPP_InteractionListener{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_InteractionListener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_InteractionListener"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_InteractionListener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_InteractionListener"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_CbSet{
    public var head:ZNPNode_ZPP_CbSet=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_CbSet{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_CbSet):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_CbSet):ZPP_CbSet{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_CbSet):ZPP_CbSet{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_CbSet"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_CbSet.zpp_pool==null){
                    ret=new ZNPNode_ZPP_CbSet();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CbSet.POOL_TOT++;
                    ZNPNode_ZPP_CbSet.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_CbSet.zpp_pool;
                    ZNPNode_ZPP_CbSet.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CbSet.POOL_CNT--;
                    ZNPNode_ZPP_CbSet.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_CbSet):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_CbSet"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_CbSet,o:ZPP_CbSet):ZNPNode_ZPP_CbSet{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_CbSet,o:ZPP_CbSet):ZNPNode_ZPP_CbSet{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_CbSet"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_CbSet.zpp_pool==null){
                    ret=new ZNPNode_ZPP_CbSet();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CbSet.POOL_TOT++;
                    ZNPNode_ZPP_CbSet.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_CbSet.zpp_pool;
                    ZNPNode_ZPP_CbSet.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CbSet.POOL_CNT--;
                    ZNPNode_ZPP_CbSet.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_CbSet"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_CbSet"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_CbSet.zpp_pool;
            ZNPNode_ZPP_CbSet.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_CbSet.POOL_CNT++;
            ZNPNode_ZPP_CbSet.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_CbSet{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_CbSet{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_CbSet"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_CbSet):Void{
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
    public function try_remove(obj:ZPP_CbSet):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_CbSet"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_CbSet):Void{
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
    function inlined_try_remove(obj:ZPP_CbSet):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_CbSet"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_CbSet):ZNPNode_ZPP_CbSet{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_CbSet):ZNPNode_ZPP_CbSet{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_CbSet"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_CbSet;
        var ret:ZNPNode_ZPP_CbSet;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_CbSet"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_CbSet.zpp_pool;
            ZNPNode_ZPP_CbSet.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_CbSet.POOL_CNT++;
            ZNPNode_ZPP_CbSet.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_CbSet,n:Int):ZNPNode_ZPP_CbSet{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_CbSet):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_CbSet):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_CbSet"+"] has -> "+obj);
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
    function front():ZPP_CbSet{
        return begin().elem();
    }
    public function back():ZPP_CbSet{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_CbSet{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_CbSet"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_CbSet{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_CbSet"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_Interactor{
    public var head:ZNPNode_ZPP_Interactor=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_Interactor{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_Interactor):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_Interactor):ZPP_Interactor{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_Interactor):ZPP_Interactor{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Interactor"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_Interactor.zpp_pool==null){
                    ret=new ZNPNode_ZPP_Interactor();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Interactor.POOL_TOT++;
                    ZNPNode_ZPP_Interactor.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_Interactor.zpp_pool;
                    ZNPNode_ZPP_Interactor.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Interactor.POOL_CNT--;
                    ZNPNode_ZPP_Interactor.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_Interactor):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_Interactor"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_Interactor,o:ZPP_Interactor):ZNPNode_ZPP_Interactor{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_Interactor,o:ZPP_Interactor):ZNPNode_ZPP_Interactor{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Interactor"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_Interactor.zpp_pool==null){
                    ret=new ZNPNode_ZPP_Interactor();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Interactor.POOL_TOT++;
                    ZNPNode_ZPP_Interactor.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_Interactor.zpp_pool;
                    ZNPNode_ZPP_Interactor.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Interactor.POOL_CNT--;
                    ZNPNode_ZPP_Interactor.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Interactor"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_Interactor"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_Interactor.zpp_pool;
            ZNPNode_ZPP_Interactor.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_Interactor.POOL_CNT++;
            ZNPNode_ZPP_Interactor.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_Interactor{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_Interactor{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Interactor"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_Interactor):Void{
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
    public function try_remove(obj:ZPP_Interactor):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Interactor"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_Interactor):Void{
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
    function inlined_try_remove(obj:ZPP_Interactor):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Interactor"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_Interactor):ZNPNode_ZPP_Interactor{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_Interactor):ZNPNode_ZPP_Interactor{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Interactor"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_Interactor;
        var ret:ZNPNode_ZPP_Interactor;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_Interactor"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_Interactor.zpp_pool;
            ZNPNode_ZPP_Interactor.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_Interactor.POOL_CNT++;
            ZNPNode_ZPP_Interactor.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_Interactor,n:Int):ZNPNode_ZPP_Interactor{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_Interactor):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_Interactor):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Interactor"+"] has -> "+obj);
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
    function front():ZPP_Interactor{
        return begin().elem();
    }
    public function back():ZPP_Interactor{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_Interactor{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Interactor"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_Interactor{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Interactor"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_BodyListener{
    public var head:ZNPNode_ZPP_BodyListener=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_BodyListener{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_BodyListener):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_BodyListener):ZPP_BodyListener{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_BodyListener):ZPP_BodyListener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_BodyListener"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_BodyListener.zpp_pool==null){
                    ret=new ZNPNode_ZPP_BodyListener();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_BodyListener.POOL_TOT++;
                    ZNPNode_ZPP_BodyListener.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_BodyListener.zpp_pool;
                    ZNPNode_ZPP_BodyListener.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_BodyListener.POOL_CNT--;
                    ZNPNode_ZPP_BodyListener.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_BodyListener):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_BodyListener"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_BodyListener,o:ZPP_BodyListener):ZNPNode_ZPP_BodyListener{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_BodyListener,o:ZPP_BodyListener):ZNPNode_ZPP_BodyListener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_BodyListener"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_BodyListener.zpp_pool==null){
                    ret=new ZNPNode_ZPP_BodyListener();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_BodyListener.POOL_TOT++;
                    ZNPNode_ZPP_BodyListener.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_BodyListener.zpp_pool;
                    ZNPNode_ZPP_BodyListener.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_BodyListener.POOL_CNT--;
                    ZNPNode_ZPP_BodyListener.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_BodyListener"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_BodyListener"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_BodyListener.zpp_pool;
            ZNPNode_ZPP_BodyListener.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_BodyListener.POOL_CNT++;
            ZNPNode_ZPP_BodyListener.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_BodyListener{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_BodyListener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_BodyListener"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_BodyListener):Void{
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
    public function try_remove(obj:ZPP_BodyListener):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_BodyListener"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_BodyListener):Void{
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
    function inlined_try_remove(obj:ZPP_BodyListener):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_BodyListener"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_BodyListener):ZNPNode_ZPP_BodyListener{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_BodyListener):ZNPNode_ZPP_BodyListener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_BodyListener"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_BodyListener;
        var ret:ZNPNode_ZPP_BodyListener;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_BodyListener"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_BodyListener.zpp_pool;
            ZNPNode_ZPP_BodyListener.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_BodyListener.POOL_CNT++;
            ZNPNode_ZPP_BodyListener.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_BodyListener,n:Int):ZNPNode_ZPP_BodyListener{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_BodyListener):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_BodyListener):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_BodyListener"+"] has -> "+obj);
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
    function front():ZPP_BodyListener{
        return begin().elem();
    }
    public function back():ZPP_BodyListener{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_BodyListener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_BodyListener"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_BodyListener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_BodyListener"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_CbSetPair{
    public var head:ZNPNode_ZPP_CbSetPair=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_CbSetPair{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_CbSetPair):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_CbSetPair):ZPP_CbSetPair{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_CbSetPair):ZPP_CbSetPair{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_CbSetPair"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_CbSetPair.zpp_pool==null){
                    ret=new ZNPNode_ZPP_CbSetPair();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CbSetPair.POOL_TOT++;
                    ZNPNode_ZPP_CbSetPair.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_CbSetPair.zpp_pool;
                    ZNPNode_ZPP_CbSetPair.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CbSetPair.POOL_CNT--;
                    ZNPNode_ZPP_CbSetPair.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_CbSetPair):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_CbSetPair"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_CbSetPair,o:ZPP_CbSetPair):ZNPNode_ZPP_CbSetPair{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_CbSetPair,o:ZPP_CbSetPair):ZNPNode_ZPP_CbSetPair{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_CbSetPair"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_CbSetPair.zpp_pool==null){
                    ret=new ZNPNode_ZPP_CbSetPair();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CbSetPair.POOL_TOT++;
                    ZNPNode_ZPP_CbSetPair.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_CbSetPair.zpp_pool;
                    ZNPNode_ZPP_CbSetPair.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CbSetPair.POOL_CNT--;
                    ZNPNode_ZPP_CbSetPair.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_CbSetPair"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_CbSetPair"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_CbSetPair.zpp_pool;
            ZNPNode_ZPP_CbSetPair.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_CbSetPair.POOL_CNT++;
            ZNPNode_ZPP_CbSetPair.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_CbSetPair{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_CbSetPair{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_CbSetPair"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_CbSetPair):Void{
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
    public function try_remove(obj:ZPP_CbSetPair):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_CbSetPair"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_CbSetPair):Void{
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
    function inlined_try_remove(obj:ZPP_CbSetPair):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_CbSetPair"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_CbSetPair):ZNPNode_ZPP_CbSetPair{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_CbSetPair):ZNPNode_ZPP_CbSetPair{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_CbSetPair"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_CbSetPair;
        var ret:ZNPNode_ZPP_CbSetPair;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_CbSetPair"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_CbSetPair.zpp_pool;
            ZNPNode_ZPP_CbSetPair.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_CbSetPair.POOL_CNT++;
            ZNPNode_ZPP_CbSetPair.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_CbSetPair,n:Int):ZNPNode_ZPP_CbSetPair{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_CbSetPair):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_CbSetPair):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_CbSetPair"+"] has -> "+obj);
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
    function front():ZPP_CbSetPair{
        return begin().elem();
    }
    public function back():ZPP_CbSetPair{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_CbSetPair{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_CbSetPair"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_CbSetPair{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_CbSetPair"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_ConstraintListener{
    public var head:ZNPNode_ZPP_ConstraintListener=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_ConstraintListener{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_ConstraintListener):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_ConstraintListener):ZPP_ConstraintListener{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_ConstraintListener):ZPP_ConstraintListener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_ConstraintListener"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_ConstraintListener.zpp_pool==null){
                    ret=new ZNPNode_ZPP_ConstraintListener();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_ConstraintListener.POOL_TOT++;
                    ZNPNode_ZPP_ConstraintListener.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_ConstraintListener.zpp_pool;
                    ZNPNode_ZPP_ConstraintListener.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_ConstraintListener.POOL_CNT--;
                    ZNPNode_ZPP_ConstraintListener.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_ConstraintListener):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_ConstraintListener"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_ConstraintListener,o:ZPP_ConstraintListener):ZNPNode_ZPP_ConstraintListener{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_ConstraintListener,o:ZPP_ConstraintListener):ZNPNode_ZPP_ConstraintListener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_ConstraintListener"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_ConstraintListener.zpp_pool==null){
                    ret=new ZNPNode_ZPP_ConstraintListener();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_ConstraintListener.POOL_TOT++;
                    ZNPNode_ZPP_ConstraintListener.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_ConstraintListener.zpp_pool;
                    ZNPNode_ZPP_ConstraintListener.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_ConstraintListener.POOL_CNT--;
                    ZNPNode_ZPP_ConstraintListener.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_ConstraintListener"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_ConstraintListener"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_ConstraintListener.zpp_pool;
            ZNPNode_ZPP_ConstraintListener.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_ConstraintListener.POOL_CNT++;
            ZNPNode_ZPP_ConstraintListener.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_ConstraintListener{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_ConstraintListener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_ConstraintListener"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_ConstraintListener):Void{
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
    public function try_remove(obj:ZPP_ConstraintListener):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_ConstraintListener"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_ConstraintListener):Void{
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
    function inlined_try_remove(obj:ZPP_ConstraintListener):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_ConstraintListener"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_ConstraintListener):ZNPNode_ZPP_ConstraintListener{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_ConstraintListener):ZNPNode_ZPP_ConstraintListener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_ConstraintListener"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_ConstraintListener;
        var ret:ZNPNode_ZPP_ConstraintListener;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_ConstraintListener"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_ConstraintListener.zpp_pool;
            ZNPNode_ZPP_ConstraintListener.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_ConstraintListener.POOL_CNT++;
            ZNPNode_ZPP_ConstraintListener.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_ConstraintListener,n:Int):ZNPNode_ZPP_ConstraintListener{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_ConstraintListener):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_ConstraintListener):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_ConstraintListener"+"] has -> "+obj);
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
    function front():ZPP_ConstraintListener{
        return begin().elem();
    }
    public function back():ZPP_ConstraintListener{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_ConstraintListener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_ConstraintListener"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_ConstraintListener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_ConstraintListener"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_CutInt{
    public var head:ZNPNode_ZPP_CutInt=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_CutInt{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_CutInt):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_CutInt):ZPP_CutInt{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_CutInt):ZPP_CutInt{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_CutInt"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_CutInt.zpp_pool==null){
                    ret=new ZNPNode_ZPP_CutInt();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CutInt.POOL_TOT++;
                    ZNPNode_ZPP_CutInt.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_CutInt.zpp_pool;
                    ZNPNode_ZPP_CutInt.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CutInt.POOL_CNT--;
                    ZNPNode_ZPP_CutInt.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_CutInt):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_CutInt"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_CutInt,o:ZPP_CutInt):ZNPNode_ZPP_CutInt{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_CutInt,o:ZPP_CutInt):ZNPNode_ZPP_CutInt{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_CutInt"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_CutInt.zpp_pool==null){
                    ret=new ZNPNode_ZPP_CutInt();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CutInt.POOL_TOT++;
                    ZNPNode_ZPP_CutInt.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_CutInt.zpp_pool;
                    ZNPNode_ZPP_CutInt.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CutInt.POOL_CNT--;
                    ZNPNode_ZPP_CutInt.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_CutInt"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_CutInt"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_CutInt.zpp_pool;
            ZNPNode_ZPP_CutInt.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_CutInt.POOL_CNT++;
            ZNPNode_ZPP_CutInt.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_CutInt{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_CutInt{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_CutInt"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_CutInt):Void{
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
    public function try_remove(obj:ZPP_CutInt):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_CutInt"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_CutInt):Void{
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
    function inlined_try_remove(obj:ZPP_CutInt):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_CutInt"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_CutInt):ZNPNode_ZPP_CutInt{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_CutInt):ZNPNode_ZPP_CutInt{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_CutInt"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_CutInt;
        var ret:ZNPNode_ZPP_CutInt;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_CutInt"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_CutInt.zpp_pool;
            ZNPNode_ZPP_CutInt.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_CutInt.POOL_CNT++;
            ZNPNode_ZPP_CutInt.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_CutInt,n:Int):ZNPNode_ZPP_CutInt{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_CutInt):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_CutInt):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_CutInt"+"] has -> "+obj);
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
    function front():ZPP_CutInt{
        return begin().elem();
    }
    public function back():ZPP_CutInt{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_CutInt{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_CutInt"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_CutInt{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_CutInt"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_CutVert{
    public var head:ZNPNode_ZPP_CutVert=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_CutVert{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_CutVert):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_CutVert):ZPP_CutVert{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_CutVert):ZPP_CutVert{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_CutVert"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_CutVert.zpp_pool==null){
                    ret=new ZNPNode_ZPP_CutVert();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CutVert.POOL_TOT++;
                    ZNPNode_ZPP_CutVert.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_CutVert.zpp_pool;
                    ZNPNode_ZPP_CutVert.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CutVert.POOL_CNT--;
                    ZNPNode_ZPP_CutVert.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_CutVert):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_CutVert"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_CutVert,o:ZPP_CutVert):ZNPNode_ZPP_CutVert{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_CutVert,o:ZPP_CutVert):ZNPNode_ZPP_CutVert{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_CutVert"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_CutVert.zpp_pool==null){
                    ret=new ZNPNode_ZPP_CutVert();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CutVert.POOL_TOT++;
                    ZNPNode_ZPP_CutVert.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_CutVert.zpp_pool;
                    ZNPNode_ZPP_CutVert.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_CutVert.POOL_CNT--;
                    ZNPNode_ZPP_CutVert.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_CutVert"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_CutVert"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_CutVert.zpp_pool;
            ZNPNode_ZPP_CutVert.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_CutVert.POOL_CNT++;
            ZNPNode_ZPP_CutVert.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_CutVert{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_CutVert{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_CutVert"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_CutVert):Void{
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
    public function try_remove(obj:ZPP_CutVert):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_CutVert"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_CutVert):Void{
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
    function inlined_try_remove(obj:ZPP_CutVert):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_CutVert"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_CutVert):ZNPNode_ZPP_CutVert{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_CutVert):ZNPNode_ZPP_CutVert{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_CutVert"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_CutVert;
        var ret:ZNPNode_ZPP_CutVert;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_CutVert"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_CutVert.zpp_pool;
            ZNPNode_ZPP_CutVert.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_CutVert.POOL_CNT++;
            ZNPNode_ZPP_CutVert.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_CutVert,n:Int):ZNPNode_ZPP_CutVert{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_CutVert):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_CutVert):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_CutVert"+"] has -> "+obj);
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
    function front():ZPP_CutVert{
        return begin().elem();
    }
    public function back():ZPP_CutVert{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_CutVert{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_CutVert"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_CutVert{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_CutVert"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_PartitionVertex{
    public var head:ZNPNode_ZPP_PartitionVertex=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_PartitionVertex{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_PartitionVertex):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_PartitionVertex):ZPP_PartitionVertex{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_PartitionVertex):ZPP_PartitionVertex{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_PartitionVertex"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_PartitionVertex.zpp_pool==null){
                    ret=new ZNPNode_ZPP_PartitionVertex();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_PartitionVertex.POOL_TOT++;
                    ZNPNode_ZPP_PartitionVertex.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_PartitionVertex.zpp_pool;
                    ZNPNode_ZPP_PartitionVertex.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_PartitionVertex.POOL_CNT--;
                    ZNPNode_ZPP_PartitionVertex.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_PartitionVertex):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_PartitionVertex"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_PartitionVertex,o:ZPP_PartitionVertex):ZNPNode_ZPP_PartitionVertex{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_PartitionVertex,o:ZPP_PartitionVertex):ZNPNode_ZPP_PartitionVertex{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_PartitionVertex"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_PartitionVertex.zpp_pool==null){
                    ret=new ZNPNode_ZPP_PartitionVertex();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_PartitionVertex.POOL_TOT++;
                    ZNPNode_ZPP_PartitionVertex.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_PartitionVertex.zpp_pool;
                    ZNPNode_ZPP_PartitionVertex.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_PartitionVertex.POOL_CNT--;
                    ZNPNode_ZPP_PartitionVertex.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_PartitionVertex"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_PartitionVertex"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_PartitionVertex.zpp_pool;
            ZNPNode_ZPP_PartitionVertex.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_PartitionVertex.POOL_CNT++;
            ZNPNode_ZPP_PartitionVertex.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_PartitionVertex{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_PartitionVertex{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_PartitionVertex"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_PartitionVertex):Void{
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
    public function try_remove(obj:ZPP_PartitionVertex):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_PartitionVertex"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_PartitionVertex):Void{
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
    function inlined_try_remove(obj:ZPP_PartitionVertex):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_PartitionVertex"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_PartitionVertex):ZNPNode_ZPP_PartitionVertex{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_PartitionVertex):ZNPNode_ZPP_PartitionVertex{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_PartitionVertex"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_PartitionVertex;
        var ret:ZNPNode_ZPP_PartitionVertex;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_PartitionVertex"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_PartitionVertex.zpp_pool;
            ZNPNode_ZPP_PartitionVertex.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_PartitionVertex.POOL_CNT++;
            ZNPNode_ZPP_PartitionVertex.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_PartitionVertex,n:Int):ZNPNode_ZPP_PartitionVertex{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_PartitionVertex):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_PartitionVertex):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_PartitionVertex"+"] has -> "+obj);
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
    function front():ZPP_PartitionVertex{
        return begin().elem();
    }
    public function back():ZPP_PartitionVertex{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_PartitionVertex{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_PartitionVertex"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_PartitionVertex{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_PartitionVertex"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_PartitionedPoly{
    public var head:ZNPNode_ZPP_PartitionedPoly=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_PartitionedPoly{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_PartitionedPoly):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_PartitionedPoly):ZPP_PartitionedPoly{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_PartitionedPoly):ZPP_PartitionedPoly{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_PartitionedPoly"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_PartitionedPoly.zpp_pool==null){
                    ret=new ZNPNode_ZPP_PartitionedPoly();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_PartitionedPoly.POOL_TOT++;
                    ZNPNode_ZPP_PartitionedPoly.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_PartitionedPoly.zpp_pool;
                    ZNPNode_ZPP_PartitionedPoly.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_PartitionedPoly.POOL_CNT--;
                    ZNPNode_ZPP_PartitionedPoly.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_PartitionedPoly):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_PartitionedPoly"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_PartitionedPoly,o:ZPP_PartitionedPoly):ZNPNode_ZPP_PartitionedPoly{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_PartitionedPoly,o:ZPP_PartitionedPoly):ZNPNode_ZPP_PartitionedPoly{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_PartitionedPoly"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_PartitionedPoly.zpp_pool==null){
                    ret=new ZNPNode_ZPP_PartitionedPoly();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_PartitionedPoly.POOL_TOT++;
                    ZNPNode_ZPP_PartitionedPoly.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_PartitionedPoly.zpp_pool;
                    ZNPNode_ZPP_PartitionedPoly.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_PartitionedPoly.POOL_CNT--;
                    ZNPNode_ZPP_PartitionedPoly.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_PartitionedPoly"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_PartitionedPoly"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_PartitionedPoly.zpp_pool;
            ZNPNode_ZPP_PartitionedPoly.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_PartitionedPoly.POOL_CNT++;
            ZNPNode_ZPP_PartitionedPoly.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_PartitionedPoly{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_PartitionedPoly{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_PartitionedPoly"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_PartitionedPoly):Void{
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
    public function try_remove(obj:ZPP_PartitionedPoly):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_PartitionedPoly"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_PartitionedPoly):Void{
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
    function inlined_try_remove(obj:ZPP_PartitionedPoly):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_PartitionedPoly"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_PartitionedPoly):ZNPNode_ZPP_PartitionedPoly{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_PartitionedPoly):ZNPNode_ZPP_PartitionedPoly{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_PartitionedPoly"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_PartitionedPoly;
        var ret:ZNPNode_ZPP_PartitionedPoly;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_PartitionedPoly"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_PartitionedPoly.zpp_pool;
            ZNPNode_ZPP_PartitionedPoly.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_PartitionedPoly.POOL_CNT++;
            ZNPNode_ZPP_PartitionedPoly.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_PartitionedPoly,n:Int):ZNPNode_ZPP_PartitionedPoly{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_PartitionedPoly):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_PartitionedPoly):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_PartitionedPoly"+"] has -> "+obj);
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
    function front():ZPP_PartitionedPoly{
        return begin().elem();
    }
    public function back():ZPP_PartitionedPoly{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_PartitionedPoly{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_PartitionedPoly"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_PartitionedPoly{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_PartitionedPoly"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_SimplifyP{
    public var head:ZNPNode_ZPP_SimplifyP=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_SimplifyP{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_SimplifyP):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_SimplifyP):ZPP_SimplifyP{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_SimplifyP):ZPP_SimplifyP{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_SimplifyP"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_SimplifyP.zpp_pool==null){
                    ret=new ZNPNode_ZPP_SimplifyP();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_SimplifyP.POOL_TOT++;
                    ZNPNode_ZPP_SimplifyP.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_SimplifyP.zpp_pool;
                    ZNPNode_ZPP_SimplifyP.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_SimplifyP.POOL_CNT--;
                    ZNPNode_ZPP_SimplifyP.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_SimplifyP):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_SimplifyP"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_SimplifyP,o:ZPP_SimplifyP):ZNPNode_ZPP_SimplifyP{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_SimplifyP,o:ZPP_SimplifyP):ZNPNode_ZPP_SimplifyP{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_SimplifyP"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_SimplifyP.zpp_pool==null){
                    ret=new ZNPNode_ZPP_SimplifyP();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_SimplifyP.POOL_TOT++;
                    ZNPNode_ZPP_SimplifyP.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_SimplifyP.zpp_pool;
                    ZNPNode_ZPP_SimplifyP.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_SimplifyP.POOL_CNT--;
                    ZNPNode_ZPP_SimplifyP.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_SimplifyP"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_SimplifyP"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_SimplifyP.zpp_pool;
            ZNPNode_ZPP_SimplifyP.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_SimplifyP.POOL_CNT++;
            ZNPNode_ZPP_SimplifyP.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_SimplifyP{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_SimplifyP{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_SimplifyP"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_SimplifyP):Void{
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
    public function try_remove(obj:ZPP_SimplifyP):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_SimplifyP"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_SimplifyP):Void{
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
    function inlined_try_remove(obj:ZPP_SimplifyP):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_SimplifyP"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_SimplifyP):ZNPNode_ZPP_SimplifyP{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_SimplifyP):ZNPNode_ZPP_SimplifyP{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_SimplifyP"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_SimplifyP;
        var ret:ZNPNode_ZPP_SimplifyP;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_SimplifyP"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_SimplifyP.zpp_pool;
            ZNPNode_ZPP_SimplifyP.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_SimplifyP.POOL_CNT++;
            ZNPNode_ZPP_SimplifyP.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_SimplifyP,n:Int):ZNPNode_ZPP_SimplifyP{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_SimplifyP):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_SimplifyP):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_SimplifyP"+"] has -> "+obj);
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
    function front():ZPP_SimplifyP{
        return begin().elem();
    }
    public function back():ZPP_SimplifyP{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_SimplifyP{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_SimplifyP"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_SimplifyP{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_SimplifyP"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_GeomVert{
    public var head:ZNPNode_ZPP_GeomVert=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_GeomVert{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_GeomVert):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_GeomVert):ZPP_GeomVert{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_GeomVert):ZPP_GeomVert{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_GeomVert"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_GeomVert.zpp_pool==null){
                    ret=new ZNPNode_ZPP_GeomVert();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_GeomVert.POOL_TOT++;
                    ZNPNode_ZPP_GeomVert.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_GeomVert.zpp_pool;
                    ZNPNode_ZPP_GeomVert.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_GeomVert.POOL_CNT--;
                    ZNPNode_ZPP_GeomVert.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_GeomVert):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_GeomVert"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_GeomVert,o:ZPP_GeomVert):ZNPNode_ZPP_GeomVert{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_GeomVert,o:ZPP_GeomVert):ZNPNode_ZPP_GeomVert{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_GeomVert"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_GeomVert.zpp_pool==null){
                    ret=new ZNPNode_ZPP_GeomVert();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_GeomVert.POOL_TOT++;
                    ZNPNode_ZPP_GeomVert.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_GeomVert.zpp_pool;
                    ZNPNode_ZPP_GeomVert.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_GeomVert.POOL_CNT--;
                    ZNPNode_ZPP_GeomVert.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_GeomVert"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_GeomVert"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_GeomVert.zpp_pool;
            ZNPNode_ZPP_GeomVert.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_GeomVert.POOL_CNT++;
            ZNPNode_ZPP_GeomVert.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_GeomVert{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_GeomVert{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_GeomVert"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_GeomVert):Void{
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
    public function try_remove(obj:ZPP_GeomVert):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_GeomVert"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_GeomVert):Void{
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
    function inlined_try_remove(obj:ZPP_GeomVert):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_GeomVert"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_GeomVert):ZNPNode_ZPP_GeomVert{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_GeomVert):ZNPNode_ZPP_GeomVert{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_GeomVert"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_GeomVert;
        var ret:ZNPNode_ZPP_GeomVert;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_GeomVert"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_GeomVert.zpp_pool;
            ZNPNode_ZPP_GeomVert.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_GeomVert.POOL_CNT++;
            ZNPNode_ZPP_GeomVert.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_GeomVert,n:Int):ZNPNode_ZPP_GeomVert{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_GeomVert):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_GeomVert):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_GeomVert"+"] has -> "+obj);
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
    function front():ZPP_GeomVert{
        return begin().elem();
    }
    public function back():ZPP_GeomVert{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_GeomVert{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_GeomVert"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_GeomVert{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_GeomVert"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_SimpleVert{
    public var head:ZNPNode_ZPP_SimpleVert=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_SimpleVert{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_SimpleVert):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_SimpleVert):ZPP_SimpleVert{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_SimpleVert):ZPP_SimpleVert{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_SimpleVert"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_SimpleVert.zpp_pool==null){
                    ret=new ZNPNode_ZPP_SimpleVert();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_SimpleVert.POOL_TOT++;
                    ZNPNode_ZPP_SimpleVert.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_SimpleVert.zpp_pool;
                    ZNPNode_ZPP_SimpleVert.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_SimpleVert.POOL_CNT--;
                    ZNPNode_ZPP_SimpleVert.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_SimpleVert):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_SimpleVert"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_SimpleVert,o:ZPP_SimpleVert):ZNPNode_ZPP_SimpleVert{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_SimpleVert,o:ZPP_SimpleVert):ZNPNode_ZPP_SimpleVert{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_SimpleVert"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_SimpleVert.zpp_pool==null){
                    ret=new ZNPNode_ZPP_SimpleVert();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_SimpleVert.POOL_TOT++;
                    ZNPNode_ZPP_SimpleVert.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_SimpleVert.zpp_pool;
                    ZNPNode_ZPP_SimpleVert.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_SimpleVert.POOL_CNT--;
                    ZNPNode_ZPP_SimpleVert.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_SimpleVert"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_SimpleVert"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_SimpleVert.zpp_pool;
            ZNPNode_ZPP_SimpleVert.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_SimpleVert.POOL_CNT++;
            ZNPNode_ZPP_SimpleVert.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_SimpleVert{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_SimpleVert{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_SimpleVert"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_SimpleVert):Void{
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
    public function try_remove(obj:ZPP_SimpleVert):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_SimpleVert"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_SimpleVert):Void{
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
    function inlined_try_remove(obj:ZPP_SimpleVert):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_SimpleVert"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_SimpleVert):ZNPNode_ZPP_SimpleVert{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_SimpleVert):ZNPNode_ZPP_SimpleVert{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_SimpleVert"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_SimpleVert;
        var ret:ZNPNode_ZPP_SimpleVert;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_SimpleVert"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_SimpleVert.zpp_pool;
            ZNPNode_ZPP_SimpleVert.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_SimpleVert.POOL_CNT++;
            ZNPNode_ZPP_SimpleVert.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_SimpleVert,n:Int):ZNPNode_ZPP_SimpleVert{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_SimpleVert):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_SimpleVert):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_SimpleVert"+"] has -> "+obj);
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
    function front():ZPP_SimpleVert{
        return begin().elem();
    }
    public function back():ZPP_SimpleVert{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_SimpleVert{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_SimpleVert"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_SimpleVert{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_SimpleVert"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_SimpleEvent{
    public var head:ZNPNode_ZPP_SimpleEvent=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_SimpleEvent{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_SimpleEvent):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_SimpleEvent):ZPP_SimpleEvent{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_SimpleEvent):ZPP_SimpleEvent{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_SimpleEvent"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_SimpleEvent.zpp_pool==null){
                    ret=new ZNPNode_ZPP_SimpleEvent();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_SimpleEvent.POOL_TOT++;
                    ZNPNode_ZPP_SimpleEvent.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_SimpleEvent.zpp_pool;
                    ZNPNode_ZPP_SimpleEvent.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_SimpleEvent.POOL_CNT--;
                    ZNPNode_ZPP_SimpleEvent.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_SimpleEvent):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_SimpleEvent"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_SimpleEvent,o:ZPP_SimpleEvent):ZNPNode_ZPP_SimpleEvent{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_SimpleEvent,o:ZPP_SimpleEvent):ZNPNode_ZPP_SimpleEvent{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_SimpleEvent"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_SimpleEvent.zpp_pool==null){
                    ret=new ZNPNode_ZPP_SimpleEvent();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_SimpleEvent.POOL_TOT++;
                    ZNPNode_ZPP_SimpleEvent.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_SimpleEvent.zpp_pool;
                    ZNPNode_ZPP_SimpleEvent.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_SimpleEvent.POOL_CNT--;
                    ZNPNode_ZPP_SimpleEvent.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_SimpleEvent"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_SimpleEvent"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_SimpleEvent.zpp_pool;
            ZNPNode_ZPP_SimpleEvent.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_SimpleEvent.POOL_CNT++;
            ZNPNode_ZPP_SimpleEvent.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_SimpleEvent{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_SimpleEvent{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_SimpleEvent"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_SimpleEvent):Void{
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
    public function try_remove(obj:ZPP_SimpleEvent):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_SimpleEvent"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_SimpleEvent):Void{
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
    function inlined_try_remove(obj:ZPP_SimpleEvent):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_SimpleEvent"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_SimpleEvent):ZNPNode_ZPP_SimpleEvent{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_SimpleEvent):ZNPNode_ZPP_SimpleEvent{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_SimpleEvent"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_SimpleEvent;
        var ret:ZNPNode_ZPP_SimpleEvent;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_SimpleEvent"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_SimpleEvent.zpp_pool;
            ZNPNode_ZPP_SimpleEvent.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_SimpleEvent.POOL_CNT++;
            ZNPNode_ZPP_SimpleEvent.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_SimpleEvent,n:Int):ZNPNode_ZPP_SimpleEvent{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_SimpleEvent):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_SimpleEvent):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_SimpleEvent"+"] has -> "+obj);
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
    function front():ZPP_SimpleEvent{
        return begin().elem();
    }
    public function back():ZPP_SimpleEvent{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_SimpleEvent{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_SimpleEvent"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_SimpleEvent{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_SimpleEvent"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_AABBPair{
    public var head:ZNPNode_ZPP_AABBPair=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_AABBPair{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_AABBPair):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_AABBPair):ZPP_AABBPair{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_AABBPair):ZPP_AABBPair{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_AABBPair"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_AABBPair.zpp_pool==null){
                    ret=new ZNPNode_ZPP_AABBPair();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_AABBPair.POOL_TOT++;
                    ZNPNode_ZPP_AABBPair.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_AABBPair.zpp_pool;
                    ZNPNode_ZPP_AABBPair.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_AABBPair.POOL_CNT--;
                    ZNPNode_ZPP_AABBPair.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_AABBPair):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_AABBPair"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_AABBPair,o:ZPP_AABBPair):ZNPNode_ZPP_AABBPair{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_AABBPair,o:ZPP_AABBPair):ZNPNode_ZPP_AABBPair{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_AABBPair"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_AABBPair.zpp_pool==null){
                    ret=new ZNPNode_ZPP_AABBPair();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_AABBPair.POOL_TOT++;
                    ZNPNode_ZPP_AABBPair.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_AABBPair.zpp_pool;
                    ZNPNode_ZPP_AABBPair.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_AABBPair.POOL_CNT--;
                    ZNPNode_ZPP_AABBPair.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_AABBPair"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_AABBPair"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_AABBPair.zpp_pool;
            ZNPNode_ZPP_AABBPair.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_AABBPair.POOL_CNT++;
            ZNPNode_ZPP_AABBPair.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_AABBPair{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_AABBPair{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_AABBPair"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_AABBPair):Void{
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
    public function try_remove(obj:ZPP_AABBPair):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_AABBPair"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_AABBPair):Void{
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
    function inlined_try_remove(obj:ZPP_AABBPair):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_AABBPair"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_AABBPair):ZNPNode_ZPP_AABBPair{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_AABBPair):ZNPNode_ZPP_AABBPair{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_AABBPair"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_AABBPair;
        var ret:ZNPNode_ZPP_AABBPair;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_AABBPair"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_AABBPair.zpp_pool;
            ZNPNode_ZPP_AABBPair.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_AABBPair.POOL_CNT++;
            ZNPNode_ZPP_AABBPair.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_AABBPair,n:Int):ZNPNode_ZPP_AABBPair{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_AABBPair):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_AABBPair):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_AABBPair"+"] has -> "+obj);
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
    function front():ZPP_AABBPair{
        return begin().elem();
    }
    public function back():ZPP_AABBPair{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_AABBPair{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_AABBPair"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_AABBPair{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_AABBPair"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_Vec2{
    public var head:ZNPNode_ZPP_Vec2=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_Vec2{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_Vec2):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_Vec2):ZPP_Vec2{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_Vec2):ZPP_Vec2{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Vec2"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_Vec2.zpp_pool==null){
                    ret=new ZNPNode_ZPP_Vec2();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Vec2.POOL_TOT++;
                    ZNPNode_ZPP_Vec2.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_Vec2.zpp_pool;
                    ZNPNode_ZPP_Vec2.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Vec2.POOL_CNT--;
                    ZNPNode_ZPP_Vec2.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_Vec2):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_Vec2"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_Vec2,o:ZPP_Vec2):ZNPNode_ZPP_Vec2{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_Vec2,o:ZPP_Vec2):ZNPNode_ZPP_Vec2{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Vec2"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_Vec2.zpp_pool==null){
                    ret=new ZNPNode_ZPP_Vec2();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Vec2.POOL_TOT++;
                    ZNPNode_ZPP_Vec2.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_Vec2.zpp_pool;
                    ZNPNode_ZPP_Vec2.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Vec2.POOL_CNT--;
                    ZNPNode_ZPP_Vec2.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Vec2"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_Vec2"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_Vec2.zpp_pool;
            ZNPNode_ZPP_Vec2.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_Vec2.POOL_CNT++;
            ZNPNode_ZPP_Vec2.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_Vec2{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_Vec2{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Vec2"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_Vec2):Void{
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
    public function try_remove(obj:ZPP_Vec2):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Vec2"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_Vec2):Void{
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
    function inlined_try_remove(obj:ZPP_Vec2):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Vec2"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_Vec2):ZNPNode_ZPP_Vec2{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_Vec2):ZNPNode_ZPP_Vec2{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Vec2"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_Vec2;
        var ret:ZNPNode_ZPP_Vec2;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_Vec2"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_Vec2.zpp_pool;
            ZNPNode_ZPP_Vec2.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_Vec2.POOL_CNT++;
            ZNPNode_ZPP_Vec2.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_Vec2,n:Int):ZNPNode_ZPP_Vec2{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_Vec2):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_Vec2):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Vec2"+"] has -> "+obj);
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
    function front():ZPP_Vec2{
        return begin().elem();
    }
    public function back():ZPP_Vec2{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_Vec2{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Vec2"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_Vec2{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Vec2"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_Edge{
    public var head:ZNPNode_ZPP_Edge=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_Edge{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_Edge):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_Edge):ZPP_Edge{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_Edge):ZPP_Edge{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Edge"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_Edge.zpp_pool==null){
                    ret=new ZNPNode_ZPP_Edge();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Edge.POOL_TOT++;
                    ZNPNode_ZPP_Edge.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_Edge.zpp_pool;
                    ZNPNode_ZPP_Edge.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Edge.POOL_CNT--;
                    ZNPNode_ZPP_Edge.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_Edge):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_Edge"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_Edge,o:ZPP_Edge):ZNPNode_ZPP_Edge{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_Edge,o:ZPP_Edge):ZNPNode_ZPP_Edge{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Edge"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_Edge.zpp_pool==null){
                    ret=new ZNPNode_ZPP_Edge();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Edge.POOL_TOT++;
                    ZNPNode_ZPP_Edge.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_Edge.zpp_pool;
                    ZNPNode_ZPP_Edge.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Edge.POOL_CNT--;
                    ZNPNode_ZPP_Edge.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Edge"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_Edge"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_Edge.zpp_pool;
            ZNPNode_ZPP_Edge.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_Edge.POOL_CNT++;
            ZNPNode_ZPP_Edge.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_Edge{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_Edge{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Edge"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_Edge):Void{
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
    public function try_remove(obj:ZPP_Edge):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Edge"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_Edge):Void{
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
    function inlined_try_remove(obj:ZPP_Edge):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Edge"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_Edge):ZNPNode_ZPP_Edge{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_Edge):ZNPNode_ZPP_Edge{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Edge"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_Edge;
        var ret:ZNPNode_ZPP_Edge;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_Edge"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_Edge.zpp_pool;
            ZNPNode_ZPP_Edge.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_Edge.POOL_CNT++;
            ZNPNode_ZPP_Edge.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_Edge,n:Int):ZNPNode_ZPP_Edge{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_Edge):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_Edge):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Edge"+"] has -> "+obj);
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
    function front():ZPP_Edge{
        return begin().elem();
    }
    public function back():ZPP_Edge{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_Edge{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Edge"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_Edge{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Edge"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_AABBNode{
    public var head:ZNPNode_ZPP_AABBNode=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_AABBNode{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_AABBNode):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_AABBNode):ZPP_AABBNode{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_AABBNode):ZPP_AABBNode{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_AABBNode"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_AABBNode.zpp_pool==null){
                    ret=new ZNPNode_ZPP_AABBNode();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_AABBNode.POOL_TOT++;
                    ZNPNode_ZPP_AABBNode.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_AABBNode.zpp_pool;
                    ZNPNode_ZPP_AABBNode.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_AABBNode.POOL_CNT--;
                    ZNPNode_ZPP_AABBNode.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_AABBNode):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_AABBNode"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_AABBNode,o:ZPP_AABBNode):ZNPNode_ZPP_AABBNode{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_AABBNode,o:ZPP_AABBNode):ZNPNode_ZPP_AABBNode{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_AABBNode"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_AABBNode.zpp_pool==null){
                    ret=new ZNPNode_ZPP_AABBNode();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_AABBNode.POOL_TOT++;
                    ZNPNode_ZPP_AABBNode.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_AABBNode.zpp_pool;
                    ZNPNode_ZPP_AABBNode.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_AABBNode.POOL_CNT--;
                    ZNPNode_ZPP_AABBNode.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_AABBNode"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_AABBNode"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_AABBNode.zpp_pool;
            ZNPNode_ZPP_AABBNode.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_AABBNode.POOL_CNT++;
            ZNPNode_ZPP_AABBNode.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_AABBNode{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_AABBNode{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_AABBNode"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_AABBNode):Void{
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
    public function try_remove(obj:ZPP_AABBNode):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_AABBNode"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_AABBNode):Void{
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
    function inlined_try_remove(obj:ZPP_AABBNode):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_AABBNode"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_AABBNode):ZNPNode_ZPP_AABBNode{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_AABBNode):ZNPNode_ZPP_AABBNode{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_AABBNode"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_AABBNode;
        var ret:ZNPNode_ZPP_AABBNode;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_AABBNode"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_AABBNode.zpp_pool;
            ZNPNode_ZPP_AABBNode.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_AABBNode.POOL_CNT++;
            ZNPNode_ZPP_AABBNode.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_AABBNode,n:Int):ZNPNode_ZPP_AABBNode{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_AABBNode):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_AABBNode):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_AABBNode"+"] has -> "+obj);
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
    function front():ZPP_AABBNode{
        return begin().elem();
    }
    public function back():ZPP_AABBNode{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_AABBNode{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_AABBNode"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_AABBNode{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_AABBNode"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_Component{
    public var head:ZNPNode_ZPP_Component=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_Component{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_Component):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_Component):ZPP_Component{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_Component):ZPP_Component{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Component"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_Component.zpp_pool==null){
                    ret=new ZNPNode_ZPP_Component();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Component.POOL_TOT++;
                    ZNPNode_ZPP_Component.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_Component.zpp_pool;
                    ZNPNode_ZPP_Component.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Component.POOL_CNT--;
                    ZNPNode_ZPP_Component.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_Component):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_Component"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_Component,o:ZPP_Component):ZNPNode_ZPP_Component{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_Component,o:ZPP_Component):ZNPNode_ZPP_Component{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Component"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_Component.zpp_pool==null){
                    ret=new ZNPNode_ZPP_Component();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Component.POOL_TOT++;
                    ZNPNode_ZPP_Component.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_Component.zpp_pool;
                    ZNPNode_ZPP_Component.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Component.POOL_CNT--;
                    ZNPNode_ZPP_Component.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Component"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_Component"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_Component.zpp_pool;
            ZNPNode_ZPP_Component.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_Component.POOL_CNT++;
            ZNPNode_ZPP_Component.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_Component{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_Component{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Component"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_Component):Void{
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
    public function try_remove(obj:ZPP_Component):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Component"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_Component):Void{
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
    function inlined_try_remove(obj:ZPP_Component):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Component"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_Component):ZNPNode_ZPP_Component{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_Component):ZNPNode_ZPP_Component{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Component"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_Component;
        var ret:ZNPNode_ZPP_Component;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_Component"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_Component.zpp_pool;
            ZNPNode_ZPP_Component.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_Component.POOL_CNT++;
            ZNPNode_ZPP_Component.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_Component,n:Int):ZNPNode_ZPP_Component{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_Component):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_Component):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Component"+"] has -> "+obj);
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
    function front():ZPP_Component{
        return begin().elem();
    }
    public function back():ZPP_Component{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_Component{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Component"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_Component{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Component"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_InteractionGroup{
    public var head:ZNPNode_ZPP_InteractionGroup=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_InteractionGroup{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_InteractionGroup):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_InteractionGroup):ZPP_InteractionGroup{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_InteractionGroup):ZPP_InteractionGroup{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_InteractionGroup"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_InteractionGroup.zpp_pool==null){
                    ret=new ZNPNode_ZPP_InteractionGroup();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_InteractionGroup.POOL_TOT++;
                    ZNPNode_ZPP_InteractionGroup.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_InteractionGroup.zpp_pool;
                    ZNPNode_ZPP_InteractionGroup.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_InteractionGroup.POOL_CNT--;
                    ZNPNode_ZPP_InteractionGroup.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_InteractionGroup):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_InteractionGroup"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_InteractionGroup,o:ZPP_InteractionGroup):ZNPNode_ZPP_InteractionGroup{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_InteractionGroup,o:ZPP_InteractionGroup):ZNPNode_ZPP_InteractionGroup{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_InteractionGroup"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_InteractionGroup.zpp_pool==null){
                    ret=new ZNPNode_ZPP_InteractionGroup();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_InteractionGroup.POOL_TOT++;
                    ZNPNode_ZPP_InteractionGroup.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_InteractionGroup.zpp_pool;
                    ZNPNode_ZPP_InteractionGroup.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_InteractionGroup.POOL_CNT--;
                    ZNPNode_ZPP_InteractionGroup.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_InteractionGroup"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_InteractionGroup"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_InteractionGroup.zpp_pool;
            ZNPNode_ZPP_InteractionGroup.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_InteractionGroup.POOL_CNT++;
            ZNPNode_ZPP_InteractionGroup.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_InteractionGroup{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_InteractionGroup{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_InteractionGroup"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_InteractionGroup):Void{
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
    public function try_remove(obj:ZPP_InteractionGroup):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_InteractionGroup"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_InteractionGroup):Void{
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
    function inlined_try_remove(obj:ZPP_InteractionGroup):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_InteractionGroup"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_InteractionGroup):ZNPNode_ZPP_InteractionGroup{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_InteractionGroup):ZNPNode_ZPP_InteractionGroup{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_InteractionGroup"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_InteractionGroup;
        var ret:ZNPNode_ZPP_InteractionGroup;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_InteractionGroup"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_InteractionGroup.zpp_pool;
            ZNPNode_ZPP_InteractionGroup.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_InteractionGroup.POOL_CNT++;
            ZNPNode_ZPP_InteractionGroup.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_InteractionGroup,n:Int):ZNPNode_ZPP_InteractionGroup{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_InteractionGroup):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_InteractionGroup):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_InteractionGroup"+"] has -> "+obj);
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
    function front():ZPP_InteractionGroup{
        return begin().elem();
    }
    public function back():ZPP_InteractionGroup{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_InteractionGroup{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_InteractionGroup"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_InteractionGroup{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_InteractionGroup"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_ColArbiter{
    public var head:ZNPNode_ZPP_ColArbiter=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_ColArbiter{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_ColArbiter):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_ColArbiter):ZPP_ColArbiter{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_ColArbiter):ZPP_ColArbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_ColArbiter"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_ColArbiter.zpp_pool==null){
                    ret=new ZNPNode_ZPP_ColArbiter();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_ColArbiter.POOL_TOT++;
                    ZNPNode_ZPP_ColArbiter.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_ColArbiter.zpp_pool;
                    ZNPNode_ZPP_ColArbiter.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_ColArbiter.POOL_CNT--;
                    ZNPNode_ZPP_ColArbiter.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_ColArbiter):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_ColArbiter"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_ColArbiter,o:ZPP_ColArbiter):ZNPNode_ZPP_ColArbiter{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_ColArbiter,o:ZPP_ColArbiter):ZNPNode_ZPP_ColArbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_ColArbiter"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_ColArbiter.zpp_pool==null){
                    ret=new ZNPNode_ZPP_ColArbiter();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_ColArbiter.POOL_TOT++;
                    ZNPNode_ZPP_ColArbiter.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_ColArbiter.zpp_pool;
                    ZNPNode_ZPP_ColArbiter.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_ColArbiter.POOL_CNT--;
                    ZNPNode_ZPP_ColArbiter.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_ColArbiter"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_ColArbiter"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_ColArbiter.zpp_pool;
            ZNPNode_ZPP_ColArbiter.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_ColArbiter.POOL_CNT++;
            ZNPNode_ZPP_ColArbiter.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_ColArbiter{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_ColArbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_ColArbiter"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_ColArbiter):Void{
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
    public function try_remove(obj:ZPP_ColArbiter):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_ColArbiter"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_ColArbiter):Void{
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
    function inlined_try_remove(obj:ZPP_ColArbiter):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_ColArbiter"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_ColArbiter):ZNPNode_ZPP_ColArbiter{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_ColArbiter):ZNPNode_ZPP_ColArbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_ColArbiter"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_ColArbiter;
        var ret:ZNPNode_ZPP_ColArbiter;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_ColArbiter"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_ColArbiter.zpp_pool;
            ZNPNode_ZPP_ColArbiter.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_ColArbiter.POOL_CNT++;
            ZNPNode_ZPP_ColArbiter.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_ColArbiter,n:Int):ZNPNode_ZPP_ColArbiter{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_ColArbiter):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_ColArbiter):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_ColArbiter"+"] has -> "+obj);
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
    function front():ZPP_ColArbiter{
        return begin().elem();
    }
    public function back():ZPP_ColArbiter{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_ColArbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_ColArbiter"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_ColArbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_ColArbiter"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_FluidArbiter{
    public var head:ZNPNode_ZPP_FluidArbiter=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_FluidArbiter{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_FluidArbiter):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_FluidArbiter):ZPP_FluidArbiter{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_FluidArbiter):ZPP_FluidArbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_FluidArbiter"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_FluidArbiter.zpp_pool==null){
                    ret=new ZNPNode_ZPP_FluidArbiter();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_FluidArbiter.POOL_TOT++;
                    ZNPNode_ZPP_FluidArbiter.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_FluidArbiter.zpp_pool;
                    ZNPNode_ZPP_FluidArbiter.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_FluidArbiter.POOL_CNT--;
                    ZNPNode_ZPP_FluidArbiter.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_FluidArbiter):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_FluidArbiter"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_FluidArbiter,o:ZPP_FluidArbiter):ZNPNode_ZPP_FluidArbiter{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_FluidArbiter,o:ZPP_FluidArbiter):ZNPNode_ZPP_FluidArbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_FluidArbiter"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_FluidArbiter.zpp_pool==null){
                    ret=new ZNPNode_ZPP_FluidArbiter();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_FluidArbiter.POOL_TOT++;
                    ZNPNode_ZPP_FluidArbiter.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_FluidArbiter.zpp_pool;
                    ZNPNode_ZPP_FluidArbiter.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_FluidArbiter.POOL_CNT--;
                    ZNPNode_ZPP_FluidArbiter.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_FluidArbiter"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_FluidArbiter"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_FluidArbiter.zpp_pool;
            ZNPNode_ZPP_FluidArbiter.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_FluidArbiter.POOL_CNT++;
            ZNPNode_ZPP_FluidArbiter.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_FluidArbiter{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_FluidArbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_FluidArbiter"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_FluidArbiter):Void{
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
    public function try_remove(obj:ZPP_FluidArbiter):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_FluidArbiter"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_FluidArbiter):Void{
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
    function inlined_try_remove(obj:ZPP_FluidArbiter):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_FluidArbiter"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_FluidArbiter):ZNPNode_ZPP_FluidArbiter{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_FluidArbiter):ZNPNode_ZPP_FluidArbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_FluidArbiter"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_FluidArbiter;
        var ret:ZNPNode_ZPP_FluidArbiter;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_FluidArbiter"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_FluidArbiter.zpp_pool;
            ZNPNode_ZPP_FluidArbiter.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_FluidArbiter.POOL_CNT++;
            ZNPNode_ZPP_FluidArbiter.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_FluidArbiter,n:Int):ZNPNode_ZPP_FluidArbiter{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_FluidArbiter):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_FluidArbiter):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_FluidArbiter"+"] has -> "+obj);
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
    function front():ZPP_FluidArbiter{
        return begin().elem();
    }
    public function back():ZPP_FluidArbiter{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_FluidArbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_FluidArbiter"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_FluidArbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_FluidArbiter"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_SensorArbiter{
    public var head:ZNPNode_ZPP_SensorArbiter=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_SensorArbiter{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_SensorArbiter):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_SensorArbiter):ZPP_SensorArbiter{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_SensorArbiter):ZPP_SensorArbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_SensorArbiter"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_SensorArbiter.zpp_pool==null){
                    ret=new ZNPNode_ZPP_SensorArbiter();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_SensorArbiter.POOL_TOT++;
                    ZNPNode_ZPP_SensorArbiter.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_SensorArbiter.zpp_pool;
                    ZNPNode_ZPP_SensorArbiter.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_SensorArbiter.POOL_CNT--;
                    ZNPNode_ZPP_SensorArbiter.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_SensorArbiter):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_SensorArbiter"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_SensorArbiter,o:ZPP_SensorArbiter):ZNPNode_ZPP_SensorArbiter{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_SensorArbiter,o:ZPP_SensorArbiter):ZNPNode_ZPP_SensorArbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_SensorArbiter"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_SensorArbiter.zpp_pool==null){
                    ret=new ZNPNode_ZPP_SensorArbiter();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_SensorArbiter.POOL_TOT++;
                    ZNPNode_ZPP_SensorArbiter.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_SensorArbiter.zpp_pool;
                    ZNPNode_ZPP_SensorArbiter.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_SensorArbiter.POOL_CNT--;
                    ZNPNode_ZPP_SensorArbiter.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_SensorArbiter"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_SensorArbiter"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_SensorArbiter.zpp_pool;
            ZNPNode_ZPP_SensorArbiter.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_SensorArbiter.POOL_CNT++;
            ZNPNode_ZPP_SensorArbiter.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_SensorArbiter{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_SensorArbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_SensorArbiter"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_SensorArbiter):Void{
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
    public function try_remove(obj:ZPP_SensorArbiter):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_SensorArbiter"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_SensorArbiter):Void{
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
    function inlined_try_remove(obj:ZPP_SensorArbiter):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_SensorArbiter"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_SensorArbiter):ZNPNode_ZPP_SensorArbiter{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_SensorArbiter):ZNPNode_ZPP_SensorArbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_SensorArbiter"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_SensorArbiter;
        var ret:ZNPNode_ZPP_SensorArbiter;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_SensorArbiter"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_SensorArbiter.zpp_pool;
            ZNPNode_ZPP_SensorArbiter.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_SensorArbiter.POOL_CNT++;
            ZNPNode_ZPP_SensorArbiter.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_SensorArbiter,n:Int):ZNPNode_ZPP_SensorArbiter{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_SensorArbiter):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_SensorArbiter):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_SensorArbiter"+"] has -> "+obj);
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
    function front():ZPP_SensorArbiter{
        return begin().elem();
    }
    public function back():ZPP_SensorArbiter{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_SensorArbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_SensorArbiter"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_SensorArbiter{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_SensorArbiter"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_Listener{
    public var head:ZNPNode_ZPP_Listener=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_Listener{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_Listener):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_Listener):ZPP_Listener{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_Listener):ZPP_Listener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Listener"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_Listener.zpp_pool==null){
                    ret=new ZNPNode_ZPP_Listener();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Listener.POOL_TOT++;
                    ZNPNode_ZPP_Listener.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_Listener.zpp_pool;
                    ZNPNode_ZPP_Listener.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Listener.POOL_CNT--;
                    ZNPNode_ZPP_Listener.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_Listener):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_Listener"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_Listener,o:ZPP_Listener):ZNPNode_ZPP_Listener{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_Listener,o:ZPP_Listener):ZNPNode_ZPP_Listener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_Listener"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_Listener.zpp_pool==null){
                    ret=new ZNPNode_ZPP_Listener();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Listener.POOL_TOT++;
                    ZNPNode_ZPP_Listener.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_Listener.zpp_pool;
                    ZNPNode_ZPP_Listener.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_Listener.POOL_CNT--;
                    ZNPNode_ZPP_Listener.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Listener"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_Listener"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_Listener.zpp_pool;
            ZNPNode_ZPP_Listener.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_Listener.POOL_CNT++;
            ZNPNode_ZPP_Listener.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_Listener{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_Listener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Listener"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_Listener):Void{
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
    public function try_remove(obj:ZPP_Listener):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Listener"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_Listener):Void{
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
    function inlined_try_remove(obj:ZPP_Listener):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Listener"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_Listener):ZNPNode_ZPP_Listener{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_Listener):ZNPNode_ZPP_Listener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_Listener"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_Listener;
        var ret:ZNPNode_ZPP_Listener;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_Listener"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_Listener.zpp_pool;
            ZNPNode_ZPP_Listener.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_Listener.POOL_CNT++;
            ZNPNode_ZPP_Listener.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_Listener,n:Int):ZNPNode_ZPP_Listener{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_Listener):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_Listener):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_Listener"+"] has -> "+obj);
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
    function front():ZPP_Listener{
        return begin().elem();
    }
    public function back():ZPP_Listener{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_Listener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Listener"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_Listener{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_Listener"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_ToiEvent{
    public var head:ZNPNode_ZPP_ToiEvent=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_ToiEvent{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_ToiEvent):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_ToiEvent):ZPP_ToiEvent{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_ToiEvent):ZPP_ToiEvent{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_ToiEvent"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_ToiEvent.zpp_pool==null){
                    ret=new ZNPNode_ZPP_ToiEvent();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_ToiEvent.POOL_TOT++;
                    ZNPNode_ZPP_ToiEvent.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_ToiEvent.zpp_pool;
                    ZNPNode_ZPP_ToiEvent.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_ToiEvent.POOL_CNT--;
                    ZNPNode_ZPP_ToiEvent.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_ToiEvent):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_ToiEvent"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_ToiEvent,o:ZPP_ToiEvent):ZNPNode_ZPP_ToiEvent{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_ToiEvent,o:ZPP_ToiEvent):ZNPNode_ZPP_ToiEvent{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_ToiEvent"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_ToiEvent.zpp_pool==null){
                    ret=new ZNPNode_ZPP_ToiEvent();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_ToiEvent.POOL_TOT++;
                    ZNPNode_ZPP_ToiEvent.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_ToiEvent.zpp_pool;
                    ZNPNode_ZPP_ToiEvent.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_ToiEvent.POOL_CNT--;
                    ZNPNode_ZPP_ToiEvent.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_ToiEvent"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_ToiEvent"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_ToiEvent.zpp_pool;
            ZNPNode_ZPP_ToiEvent.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_ToiEvent.POOL_CNT++;
            ZNPNode_ZPP_ToiEvent.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_ToiEvent{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_ToiEvent{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_ToiEvent"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_ToiEvent):Void{
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
    public function try_remove(obj:ZPP_ToiEvent):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_ToiEvent"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_ToiEvent):Void{
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
    function inlined_try_remove(obj:ZPP_ToiEvent):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_ToiEvent"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_ToiEvent):ZNPNode_ZPP_ToiEvent{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_ToiEvent):ZNPNode_ZPP_ToiEvent{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_ToiEvent"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_ToiEvent;
        var ret:ZNPNode_ZPP_ToiEvent;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_ToiEvent"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_ToiEvent.zpp_pool;
            ZNPNode_ZPP_ToiEvent.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_ToiEvent.POOL_CNT++;
            ZNPNode_ZPP_ToiEvent.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_ToiEvent,n:Int):ZNPNode_ZPP_ToiEvent{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_ToiEvent):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_ToiEvent):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_ToiEvent"+"] has -> "+obj);
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
    function front():ZPP_ToiEvent{
        return begin().elem();
    }
    public function back():ZPP_ToiEvent{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_ToiEvent{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_ToiEvent"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_ToiEvent{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_ToiEvent"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ConvexResult{
    public var head:ZNPNode_ConvexResult=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ConvexResult{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ConvexResult):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ConvexResult):ConvexResult{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ConvexResult):ConvexResult{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ConvexResult"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ConvexResult.zpp_pool==null){
                    ret=new ZNPNode_ConvexResult();
                    #if NAPE_POOL_STATS ZNPNode_ConvexResult.POOL_TOT++;
                    ZNPNode_ConvexResult.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ConvexResult.zpp_pool;
                    ZNPNode_ConvexResult.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ConvexResult.POOL_CNT--;
                    ZNPNode_ConvexResult.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ConvexResult):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ConvexResult"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ConvexResult,o:ConvexResult):ZNPNode_ConvexResult{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ConvexResult,o:ConvexResult):ZNPNode_ConvexResult{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ConvexResult"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ConvexResult.zpp_pool==null){
                    ret=new ZNPNode_ConvexResult();
                    #if NAPE_POOL_STATS ZNPNode_ConvexResult.POOL_TOT++;
                    ZNPNode_ConvexResult.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ConvexResult.zpp_pool;
                    ZNPNode_ConvexResult.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ConvexResult.POOL_CNT--;
                    ZNPNode_ConvexResult.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ConvexResult"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ConvexResult"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ConvexResult.zpp_pool;
            ZNPNode_ConvexResult.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ConvexResult.POOL_CNT++;
            ZNPNode_ConvexResult.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ConvexResult{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ConvexResult{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ConvexResult"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ConvexResult):Void{
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
    public function try_remove(obj:ConvexResult):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ConvexResult"+"] remove -> "+obj);
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
    function inlined_remove(obj:ConvexResult):Void{
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
    function inlined_try_remove(obj:ConvexResult):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ConvexResult"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ConvexResult):ZNPNode_ConvexResult{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ConvexResult):ZNPNode_ConvexResult{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ConvexResult"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ConvexResult;
        var ret:ZNPNode_ConvexResult;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ConvexResult"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ConvexResult.zpp_pool;
            ZNPNode_ConvexResult.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ConvexResult.POOL_CNT++;
            ZNPNode_ConvexResult.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ConvexResult,n:Int):ZNPNode_ConvexResult{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ConvexResult):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ConvexResult):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ConvexResult"+"] has -> "+obj);
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
    function front():ConvexResult{
        return begin().elem();
    }
    public function back():ConvexResult{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ConvexResult{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ConvexResult"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ConvexResult{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ConvexResult"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_ZPP_GeomPoly{
    public var head:ZNPNode_ZPP_GeomPoly=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_ZPP_GeomPoly{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_ZPP_GeomPoly):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_GeomPoly):ZPP_GeomPoly{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_GeomPoly):ZPP_GeomPoly{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_GeomPoly"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_GeomPoly.zpp_pool==null){
                    ret=new ZNPNode_ZPP_GeomPoly();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_GeomPoly.POOL_TOT++;
                    ZNPNode_ZPP_GeomPoly.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_GeomPoly.zpp_pool;
                    ZNPNode_ZPP_GeomPoly.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_GeomPoly.POOL_CNT--;
                    ZNPNode_ZPP_GeomPoly.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_ZPP_GeomPoly):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_GeomPoly"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_ZPP_GeomPoly,o:ZPP_GeomPoly):ZNPNode_ZPP_GeomPoly{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_ZPP_GeomPoly,o:ZPP_GeomPoly):ZNPNode_ZPP_GeomPoly{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_GeomPoly"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_ZPP_GeomPoly.zpp_pool==null){
                    ret=new ZNPNode_ZPP_GeomPoly();
                    #if NAPE_POOL_STATS ZNPNode_ZPP_GeomPoly.POOL_TOT++;
                    ZNPNode_ZPP_GeomPoly.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_ZPP_GeomPoly.zpp_pool;
                    ZNPNode_ZPP_GeomPoly.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_ZPP_GeomPoly.POOL_CNT--;
                    ZNPNode_ZPP_GeomPoly.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_GeomPoly"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_GeomPoly"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_GeomPoly.zpp_pool;
            ZNPNode_ZPP_GeomPoly.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_GeomPoly.POOL_CNT++;
            ZNPNode_ZPP_GeomPoly.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():ZPP_GeomPoly{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_GeomPoly{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_GeomPoly"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_GeomPoly):Void{
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
    public function try_remove(obj:ZPP_GeomPoly):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_GeomPoly"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_GeomPoly):Void{
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
    function inlined_try_remove(obj:ZPP_GeomPoly):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_GeomPoly"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_ZPP_GeomPoly):ZNPNode_ZPP_GeomPoly{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_ZPP_GeomPoly):ZNPNode_ZPP_GeomPoly{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_GeomPoly"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_ZPP_GeomPoly;
        var ret:ZNPNode_ZPP_GeomPoly;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_ZPP_GeomPoly"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_ZPP_GeomPoly.zpp_pool;
            ZNPNode_ZPP_GeomPoly.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_ZPP_GeomPoly.POOL_CNT++;
            ZNPNode_ZPP_GeomPoly.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_ZPP_GeomPoly,n:Int):ZNPNode_ZPP_GeomPoly{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:ZPP_GeomPoly):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_GeomPoly):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_GeomPoly"+"] has -> "+obj);
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
    function front():ZPP_GeomPoly{
        return begin().elem();
    }
    public function back():ZPP_GeomPoly{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_ZPP_GeomPoly{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_GeomPoly"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_GeomPoly{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_GeomPoly"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}
#if nape_swc@:keep #end
class ZNPList_RayResult{
    public var head:ZNPNode_RayResult=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZNPNode_RayResult{
        return head;
    }
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZNPNode_RayResult):Void{
        head=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:RayResult):RayResult{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:RayResult):RayResult{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"RayResult"+"] add -> o="+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_RayResult.zpp_pool==null){
                    ret=new ZNPNode_RayResult();
                    #if NAPE_POOL_STATS ZNPNode_RayResult.POOL_TOT++;
                    ZNPNode_RayResult.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_RayResult.zpp_pool;
                    ZNPNode_RayResult.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_RayResult.POOL_CNT--;
                    ZNPNode_RayResult.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        temp.next=begin();
        head=temp;
        modified=true;
        length++;
        return o;
    }
    public function addAll(x:ZNPList_RayResult):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"RayResult"+"] addAll -> "+x);
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
    public function insert(cur:ZNPNode_RayResult,o:RayResult):ZNPNode_RayResult{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZNPNode_RayResult,o:RayResult):ZNPNode_RayResult{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"RayResult"+"] cur -> "+cur+" -> "+o);
            #end
        };
        var temp={
            var ret;
            {
                if(ZNPNode_RayResult.zpp_pool==null){
                    ret=new ZNPNode_RayResult();
                    #if NAPE_POOL_STATS ZNPNode_RayResult.POOL_TOT++;
                    ZNPNode_RayResult.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZNPNode_RayResult.zpp_pool;
                    ZNPNode_RayResult.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZNPNode_RayResult.POOL_CNT--;
                    ZNPNode_RayResult.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            ret.elt=o;
            ret;
        };
        if(cur==null){
            temp.next=begin();
            head=temp;
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"RayResult"+"] pop");
            #end
        };
        var ret=begin();
        head=ret.next;
        {};
        {
            var o=ret;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_RayResult"+", in obj: "+"ret"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_RayResult.zpp_pool;
            ZNPNode_RayResult.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_RayResult.POOL_CNT++;
            ZNPNode_RayResult.POOL_SUB++;
            #end
        };
        if(empty())pushmod=true;
        modified=true;
        length--;
    }
    public function pop_unsafe():RayResult{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():RayResult{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"RayResult"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:RayResult):Void{
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
    public function try_remove(obj:RayResult):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"RayResult"+"] remove -> "+obj);
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
    function inlined_remove(obj:RayResult):Void{
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
    function inlined_try_remove(obj:RayResult):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"RayResult"+"] remove -> "+obj);
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
    public function erase(pre:ZNPNode_RayResult):ZNPNode_RayResult{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZNPNode_RayResult):ZNPNode_RayResult{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"RayResult"+"] erase -> "+pre);
            #end
        };
        var old:ZNPNode_RayResult;
        var ret:ZNPNode_RayResult;
        if(pre==null){
            old=begin();
            ret=old.next;
            head=ret;
            if(empty())pushmod=true;
        }
        else{
            old=pre.next;
            ret=old.next;
            pre.next=ret;
            if(ret==null)pushmod=true;
        }
        {};
        {
            var o=old;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZNPNode_RayResult"+", in obj: "+"old"+")");
                #end
            };
            o.free();
            o.next=ZNPNode_RayResult.zpp_pool;
            ZNPNode_RayResult.zpp_pool=o;
            #if NAPE_POOL_STATS ZNPNode_RayResult.POOL_CNT++;
            ZNPNode_RayResult.POOL_SUB++;
            #end
        };
        modified=true;
        length--;
        pushmod=true;
        return ret;
    }
    public function splice(pre:ZNPNode_RayResult,n:Int):ZNPNode_RayResult{
        while(n-->0&&pre.next!=null)erase(pre);
        return pre.next;
    }
    public function clear():Void{
        inlined_clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_clear():Void{
        if(true){
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
            head=cur;
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
    public function has(obj:RayResult):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:RayResult):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"RayResult"+"] has -> "+obj);
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
    function front():RayResult{
        return begin().elem();
    }
    public function back():RayResult{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZNPNode_RayResult{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"RayResult"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):RayResult{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"RayResult"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
}

#if nape_swc@:keep #end
class ZNPNode_ZPP_CbType{
    static public var zpp_pool:ZNPNode_ZPP_CbType=null;
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
    
    public var next:ZNPNode_ZPP_CbType=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_CbType=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_CbType{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_CallbackSet{
    static public var zpp_pool:ZNPNode_ZPP_CallbackSet=null;
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
    
    public var next:ZNPNode_ZPP_CallbackSet=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_CallbackSet=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_CallbackSet{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_Shape{
    static public var zpp_pool:ZNPNode_ZPP_Shape=null;
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
    
    public var next:ZNPNode_ZPP_Shape=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_Shape=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_Shape{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_Body{
    static public var zpp_pool:ZNPNode_ZPP_Body=null;
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
    
    public var next:ZNPNode_ZPP_Body=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_Body=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_Body{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_Constraint{
    static public var zpp_pool:ZNPNode_ZPP_Constraint=null;
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
    
    public var next:ZNPNode_ZPP_Constraint=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_Constraint=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_Constraint{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_Compound{
    static public var zpp_pool:ZNPNode_ZPP_Compound=null;
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
    
    public var next:ZNPNode_ZPP_Compound=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_Compound=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_Compound{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_Arbiter{
    static public var zpp_pool:ZNPNode_ZPP_Arbiter=null;
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
    
    public var next:ZNPNode_ZPP_Arbiter=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_Arbiter=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_Arbiter{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_InteractionListener{
    static public var zpp_pool:ZNPNode_ZPP_InteractionListener=null;
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
    
    public var next:ZNPNode_ZPP_InteractionListener=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_InteractionListener=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_InteractionListener{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_CbSet{
    static public var zpp_pool:ZNPNode_ZPP_CbSet=null;
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
    
    public var next:ZNPNode_ZPP_CbSet=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_CbSet=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_CbSet{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_Interactor{
    static public var zpp_pool:ZNPNode_ZPP_Interactor=null;
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
    
    public var next:ZNPNode_ZPP_Interactor=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_Interactor=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_Interactor{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_BodyListener{
    static public var zpp_pool:ZNPNode_ZPP_BodyListener=null;
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
    
    public var next:ZNPNode_ZPP_BodyListener=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_BodyListener=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_BodyListener{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_CbSetPair{
    static public var zpp_pool:ZNPNode_ZPP_CbSetPair=null;
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
    
    public var next:ZNPNode_ZPP_CbSetPair=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_CbSetPair=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_CbSetPair{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_ConstraintListener{
    static public var zpp_pool:ZNPNode_ZPP_ConstraintListener=null;
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
    
    public var next:ZNPNode_ZPP_ConstraintListener=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_ConstraintListener=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_ConstraintListener{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_CutInt{
    static public var zpp_pool:ZNPNode_ZPP_CutInt=null;
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
    
    public var next:ZNPNode_ZPP_CutInt=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_CutInt=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_CutInt{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_CutVert{
    static public var zpp_pool:ZNPNode_ZPP_CutVert=null;
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
    
    public var next:ZNPNode_ZPP_CutVert=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_CutVert=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_CutVert{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_PartitionVertex{
    static public var zpp_pool:ZNPNode_ZPP_PartitionVertex=null;
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
    
    public var next:ZNPNode_ZPP_PartitionVertex=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_PartitionVertex=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_PartitionVertex{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_PartitionedPoly{
    static public var zpp_pool:ZNPNode_ZPP_PartitionedPoly=null;
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
    
    public var next:ZNPNode_ZPP_PartitionedPoly=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_PartitionedPoly=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_PartitionedPoly{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_SimplifyP{
    static public var zpp_pool:ZNPNode_ZPP_SimplifyP=null;
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
    
    public var next:ZNPNode_ZPP_SimplifyP=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_SimplifyP=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_SimplifyP{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_GeomVert{
    static public var zpp_pool:ZNPNode_ZPP_GeomVert=null;
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
    
    public var next:ZNPNode_ZPP_GeomVert=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_GeomVert=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_GeomVert{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_SimpleVert{
    static public var zpp_pool:ZNPNode_ZPP_SimpleVert=null;
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
    
    public var next:ZNPNode_ZPP_SimpleVert=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_SimpleVert=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_SimpleVert{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_SimpleEvent{
    static public var zpp_pool:ZNPNode_ZPP_SimpleEvent=null;
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
    
    public var next:ZNPNode_ZPP_SimpleEvent=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_SimpleEvent=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_SimpleEvent{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_AABBPair{
    static public var zpp_pool:ZNPNode_ZPP_AABBPair=null;
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
    
    public var next:ZNPNode_ZPP_AABBPair=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_AABBPair=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_AABBPair{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_Vec2{
    static public var zpp_pool:ZNPNode_ZPP_Vec2=null;
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
    
    public var next:ZNPNode_ZPP_Vec2=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_Vec2=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_Vec2{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_Edge{
    static public var zpp_pool:ZNPNode_ZPP_Edge=null;
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
    
    public var next:ZNPNode_ZPP_Edge=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_Edge=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_Edge{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_AABBNode{
    static public var zpp_pool:ZNPNode_ZPP_AABBNode=null;
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
    
    public var next:ZNPNode_ZPP_AABBNode=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_AABBNode=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_AABBNode{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_Component{
    static public var zpp_pool:ZNPNode_ZPP_Component=null;
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
    
    public var next:ZNPNode_ZPP_Component=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_Component=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_Component{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_InteractionGroup{
    static public var zpp_pool:ZNPNode_ZPP_InteractionGroup=null;
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
    
    public var next:ZNPNode_ZPP_InteractionGroup=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_InteractionGroup=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_InteractionGroup{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_ColArbiter{
    static public var zpp_pool:ZNPNode_ZPP_ColArbiter=null;
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
    
    public var next:ZNPNode_ZPP_ColArbiter=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_ColArbiter=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_ColArbiter{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_FluidArbiter{
    static public var zpp_pool:ZNPNode_ZPP_FluidArbiter=null;
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
    
    public var next:ZNPNode_ZPP_FluidArbiter=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_FluidArbiter=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_FluidArbiter{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_SensorArbiter{
    static public var zpp_pool:ZNPNode_ZPP_SensorArbiter=null;
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
    
    public var next:ZNPNode_ZPP_SensorArbiter=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_SensorArbiter=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_SensorArbiter{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_Listener{
    static public var zpp_pool:ZNPNode_ZPP_Listener=null;
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
    
    public var next:ZNPNode_ZPP_Listener=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_Listener=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_Listener{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_ToiEvent{
    static public var zpp_pool:ZNPNode_ZPP_ToiEvent=null;
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
    
    public var next:ZNPNode_ZPP_ToiEvent=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_ToiEvent=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_ToiEvent{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ConvexResult{
    static public var zpp_pool:ZNPNode_ConvexResult=null;
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
    
    public var next:ZNPNode_ConvexResult=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ConvexResult=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ConvexResult{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_ZPP_GeomPoly{
    static public var zpp_pool:ZNPNode_ZPP_GeomPoly=null;
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
    
    public var next:ZNPNode_ZPP_GeomPoly=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:ZPP_GeomPoly=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_GeomPoly{
        return elt;
    }
}
#if nape_swc@:keep #end
class ZNPNode_RayResult{
    static public var zpp_pool:ZNPNode_RayResult=null;
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
    
    public var next:ZNPNode_RayResult=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        elt=null;
    }
    public var elt:RayResult=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():RayResult{
        return elt;
    }
}

#if nape_swc@:keep #end
class ZPP_MixVec2List extends Vec2List{
    public var inner:ZPP_Vec2=null;
    public var _length:Int=0;
    public var zip_length:Bool=false;
    public var at_ite:ZPP_Vec2=null;
    public var at_index:Int=0;
    public static function get(list:ZPP_Vec2,immutable=false):ZPP_MixVec2List{
        var ret=new ZPP_MixVec2List();
        ret.inner=list;
        ret.zpp_inner.immutable=immutable;
        return ret;
    }
    public function new(){
        super();
        at_ite=null;
        at_index=0;
        zip_length=true;
        _length=0;
    }
    public override function zpp_gl():Int{
        zpp_vm();
        if(zip_length){
            _length=0;
            {
                var cx_ite=inner.begin();
                while(cx_ite!=null){
                    var i=cx_ite.elem();
                    if(true)_length++;
                    cx_ite=cx_ite.next;
                }
            };
            zip_length=false;
        }
        return _length;
    }
    public override function zpp_vm():Void{
        zpp_inner.validate();
        if(inner.modified){
            zip_length=true;
            _length=0;
            at_ite=null;
        }
    }
    #if nape_swc@:keep #end
    public override function at(index:Int):Vec2{
        zpp_vm();
        #if(!NAPE_RELEASE_BUILD)
        if(index<0||index>=length)throw "Error: Index out of bounds";
        #end
        if(zpp_inner.reverse_flag)index=length-1-index;
        if(index<at_index||at_ite==null){
            at_index=0;
            at_ite=inner.begin();
            while(true){
                var x=at_ite.elem();
                if(true)break;
                at_ite=at_ite.next;
            }
        }
        while(at_index!=index){
            at_index++;
            at_ite=at_ite.next;
            while(true){
                var x=at_ite.elem();
                if(true)break;
                at_ite=at_ite.next;
            }
        }
        return at_ite.elem().wrapper();
    }
    #if nape_swc@:keep #end
    public override function push(obj:Vec2):Bool{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.immutable)throw "Error: "+"Vec2"+"List is immutable";
        #end
        zpp_inner.modify_test();
        zpp_vm();
        #if(!NAPE_RELEASE_BUILD)
        if(obj.zpp_inner._inuse)throw "Error: "+"Vec2"+" is already in use";
        #end
        var cont=if(zpp_inner.adder!=null)zpp_inner.adder(obj)else true;
        if(cont){
            if(zpp_inner.reverse_flag)inner.add(obj.zpp_inner);
            else{
                var ite=inner.iterator_at(length-1);
                inner.insert(ite,obj.zpp_inner);
            }
            zpp_inner.invalidate();
            if(zpp_inner.post_adder!=null)zpp_inner.post_adder(obj);
        }
        return cont;
    }
    #if nape_swc@:keep #end
    public override function unshift(obj:Vec2):Bool{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.immutable)throw "Error: "+"Vec2"+"List is immutable";
        #end
        zpp_inner.modify_test();
        zpp_vm();
        #if(!NAPE_RELEASE_BUILD)
        if(obj.zpp_inner._inuse)throw "Error: "+"Vec2"+" is already in use";
        #end
        var cont=if(zpp_inner.adder!=null)zpp_inner.adder(obj)else true;
        if(cont){
            if(zpp_inner.reverse_flag){
                var ite=inner.iterator_at(length-1);
                inner.insert(ite,obj.zpp_inner);
            }
            else inner.add(obj.zpp_inner);
            zpp_inner.invalidate();
            if(zpp_inner.post_adder!=null)zpp_inner.post_adder(obj);
        }
        return cont;
    }
    #if nape_swc@:keep #end
    public override function pop():Vec2{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.immutable)throw "Error: "+"Vec2"+"List is immutable";
        #end
        zpp_inner.modify_test();
        #if(!NAPE_RELEASE_BUILD)
        if(empty())throw "Error: Cannot remove from empty list";
        #end
        zpp_vm();
        var ret=null;
        if(zpp_inner.reverse_flag){
            ret=inner.front();
            var retx=ret.wrapper();
            if(zpp_inner.subber!=null)zpp_inner.subber(retx);
            if(!zpp_inner.dontremove)inner.pop();
        }
        else{
            if(at_ite!=null&&at_ite.next==null)at_ite=null;
            var ite=length==1?null:inner.iterator_at(length-2);
            ret=ite==null?inner.front():ite.next.elem();
            var retx=ret.wrapper();
            if(zpp_inner.subber!=null)zpp_inner.subber(retx);
            if(!zpp_inner.dontremove)inner.erase(ite);
        }
        zpp_inner.invalidate();
        var retx=ret.wrapper();
        return retx;
    }
    #if nape_swc@:keep #end
    public override function shift():Vec2{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.immutable)throw "Error: "+"Vec2"+"List is immutable";
        #end
        zpp_inner.modify_test();
        #if(!NAPE_RELEASE_BUILD)
        if(empty())throw "Error: Cannot remove from empty list";
        #end
        zpp_vm();
        var ret=null;
        if(zpp_inner.reverse_flag){
            if(at_ite!=null&&at_ite.next==null)at_ite=null;
            var ite=length==1?null:inner.iterator_at(length-2);
            ret=ite==null?inner.front():ite.next.elem();
            var retx=ret.wrapper();
            if(zpp_inner.subber!=null)zpp_inner.subber(retx);
            if(!zpp_inner.dontremove)inner.erase(ite);
        }
        else{
            ret=inner.front();
            var retx=ret.wrapper();
            if(zpp_inner.subber!=null)zpp_inner.subber(retx);
            if(!zpp_inner.dontremove)inner.pop();
        }
        zpp_inner.invalidate();
        var retx=ret.wrapper();
        return retx;
    }
    #if nape_swc@:keep #end
    public override function remove(obj:Vec2):Bool{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.immutable)throw "Error: "+"Vec2"+"List is immutable";
        #end
        zpp_inner.modify_test();
        zpp_vm();
        var ret;
        {
            ret=false;
            {
                var cx_ite=inner.begin();
                while(cx_ite!=null){
                    var x=cx_ite.elem();
                    {
                        if(obj.zpp_inner==x){
                            ret=true;
                            break;
                        }
                    };
                    cx_ite=cx_ite.next;
                }
            };
        };
        if(ret){
            if(zpp_inner.subber!=null)zpp_inner.subber(obj);
            if(!zpp_inner.dontremove)inner.remove(obj.zpp_inner);
            zpp_inner.invalidate();
        }
        return ret;
    }
    #if nape_swc@:keep #end
    public override function clear():Void{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.immutable)throw "Error: "+"Vec2"+"List is immutable";
        #end
        if(zpp_inner.reverse_flag){
            while(!empty())pop();
        }
        else{
            while(!empty())shift();
        }
    }
}

#if nape_swc@:keep #end
class ZPP_ConstraintList{
    public var outer:ConstraintList=null;
    public var inner:ZNPList_ZPP_Constraint=null;
    public var immutable:Bool=false;
    public var _invalidated:Bool=false;
    public var _invalidate:ZPP_ConstraintList->Void=null;
    public var _validate:Void->Void=null;
    public var _modifiable:Void->Void=null;
    public static var internal:Bool=false;
    public var adder:Constraint->Bool=null;
    public var post_adder:Constraint->Void=null;
    public var subber:Constraint->Void=null;
    public var dontremove:Bool=false;
    public var reverse_flag:Bool=false;
     public static function get(list:ZNPList_ZPP_Constraint,imm:Bool=false):ConstraintList{
        var ret=new ConstraintList();
        ret.zpp_inner.inner=list;
        if(imm)ret.zpp_inner.immutable=true;
        ret.zpp_inner.zip_length=true;
        return ret;
    }
    public function valmod():Void{
        validate();
        if(inner.modified){
            if(inner.pushmod)push_ite=null;
            at_ite=null;
            inner.modified=false;
            inner.pushmod=false;
            zip_length=true;
        }
    }
    public function modified():Void{
        zip_length=true;
        at_ite=null;
        push_ite=null;
    }
    public function modify_test():Void{
        #if(!NAPE_RELEASE_BUILD)
        if(_modifiable!=null)_modifiable();
        #end
    }
    public function validate():Void{
        if(_invalidated){
            _invalidated=false;
            if(_validate!=null)_validate();
        }
    }
    public function invalidate():Void{
        _invalidated=true;
        if(_invalidate!=null)_invalidate(this);
    }
    public var at_index:Int=0;
    public var at_ite:ZNPNode_ZPP_Constraint=null;
    public var push_ite:ZNPNode_ZPP_Constraint=null;
    public var zip_length:Bool=false;
    public var user_length:Int=0;
    public function new(){
        inner=new ZNPList_ZPP_Constraint();
        _invalidated=true;
    }
}
#if nape_swc@:keep #end
class ZPP_BodyList{
    public var outer:BodyList=null;
    public var inner:ZNPList_ZPP_Body=null;
    public var immutable:Bool=false;
    public var _invalidated:Bool=false;
    public var _invalidate:ZPP_BodyList->Void=null;
    public var _validate:Void->Void=null;
    public var _modifiable:Void->Void=null;
    public static var internal:Bool=false;
    public var adder:Body->Bool=null;
    public var post_adder:Body->Void=null;
    public var subber:Body->Void=null;
    public var dontremove:Bool=false;
    public var reverse_flag:Bool=false;
     public static function get(list:ZNPList_ZPP_Body,imm:Bool=false):BodyList{
        var ret=new BodyList();
        ret.zpp_inner.inner=list;
        if(imm)ret.zpp_inner.immutable=true;
        ret.zpp_inner.zip_length=true;
        return ret;
    }
    public function valmod():Void{
        validate();
        if(inner.modified){
            if(inner.pushmod)push_ite=null;
            at_ite=null;
            inner.modified=false;
            inner.pushmod=false;
            zip_length=true;
        }
    }
    public function modified():Void{
        zip_length=true;
        at_ite=null;
        push_ite=null;
    }
    public function modify_test():Void{
        #if(!NAPE_RELEASE_BUILD)
        if(_modifiable!=null)_modifiable();
        #end
    }
    public function validate():Void{
        if(_invalidated){
            _invalidated=false;
            if(_validate!=null)_validate();
        }
    }
    public function invalidate():Void{
        _invalidated=true;
        if(_invalidate!=null)_invalidate(this);
    }
    public var at_index:Int=0;
    public var at_ite:ZNPNode_ZPP_Body=null;
    public var push_ite:ZNPNode_ZPP_Body=null;
    public var zip_length:Bool=false;
    public var user_length:Int=0;
    public function new(){
        inner=new ZNPList_ZPP_Body();
        _invalidated=true;
    }
}
#if nape_swc@:keep #end
class ZPP_InteractorList{
    public var outer:InteractorList=null;
    public var inner:ZNPList_ZPP_Interactor=null;
    public var immutable:Bool=false;
    public var _invalidated:Bool=false;
    public var _invalidate:ZPP_InteractorList->Void=null;
    public var _validate:Void->Void=null;
    public var _modifiable:Void->Void=null;
    public static var internal:Bool=false;
    public var adder:Interactor->Bool=null;
    public var post_adder:Interactor->Void=null;
    public var subber:Interactor->Void=null;
    public var dontremove:Bool=false;
    public var reverse_flag:Bool=false;
     public static function get(list:ZNPList_ZPP_Interactor,imm:Bool=false):InteractorList{
        var ret=new InteractorList();
        ret.zpp_inner.inner=list;
        if(imm)ret.zpp_inner.immutable=true;
        ret.zpp_inner.zip_length=true;
        return ret;
    }
    public function valmod():Void{
        validate();
        if(inner.modified){
            if(inner.pushmod)push_ite=null;
            at_ite=null;
            inner.modified=false;
            inner.pushmod=false;
            zip_length=true;
        }
    }
    public function modified():Void{
        zip_length=true;
        at_ite=null;
        push_ite=null;
    }
    public function modify_test():Void{
        #if(!NAPE_RELEASE_BUILD)
        if(_modifiable!=null)_modifiable();
        #end
    }
    public function validate():Void{
        if(_invalidated){
            _invalidated=false;
            if(_validate!=null)_validate();
        }
    }
    public function invalidate():Void{
        _invalidated=true;
        if(_invalidate!=null)_invalidate(this);
    }
    public var at_index:Int=0;
    public var at_ite:ZNPNode_ZPP_Interactor=null;
    public var push_ite:ZNPNode_ZPP_Interactor=null;
    public var zip_length:Bool=false;
    public var user_length:Int=0;
    public function new(){
        inner=new ZNPList_ZPP_Interactor();
        _invalidated=true;
    }
}
#if nape_swc@:keep #end
class ZPP_CompoundList{
    public var outer:CompoundList=null;
    public var inner:ZNPList_ZPP_Compound=null;
    public var immutable:Bool=false;
    public var _invalidated:Bool=false;
    public var _invalidate:ZPP_CompoundList->Void=null;
    public var _validate:Void->Void=null;
    public var _modifiable:Void->Void=null;
    public static var internal:Bool=false;
    public var adder:Compound->Bool=null;
    public var post_adder:Compound->Void=null;
    public var subber:Compound->Void=null;
    public var dontremove:Bool=false;
    public var reverse_flag:Bool=false;
     public static function get(list:ZNPList_ZPP_Compound,imm:Bool=false):CompoundList{
        var ret=new CompoundList();
        ret.zpp_inner.inner=list;
        if(imm)ret.zpp_inner.immutable=true;
        ret.zpp_inner.zip_length=true;
        return ret;
    }
    public function valmod():Void{
        validate();
        if(inner.modified){
            if(inner.pushmod)push_ite=null;
            at_ite=null;
            inner.modified=false;
            inner.pushmod=false;
            zip_length=true;
        }
    }
    public function modified():Void{
        zip_length=true;
        at_ite=null;
        push_ite=null;
    }
    public function modify_test():Void{
        #if(!NAPE_RELEASE_BUILD)
        if(_modifiable!=null)_modifiable();
        #end
    }
    public function validate():Void{
        if(_invalidated){
            _invalidated=false;
            if(_validate!=null)_validate();
        }
    }
    public function invalidate():Void{
        _invalidated=true;
        if(_invalidate!=null)_invalidate(this);
    }
    public var at_index:Int=0;
    public var at_ite:ZNPNode_ZPP_Compound=null;
    public var push_ite:ZNPNode_ZPP_Compound=null;
    public var zip_length:Bool=false;
    public var user_length:Int=0;
    public function new(){
        inner=new ZNPList_ZPP_Compound();
        _invalidated=true;
    }
}
#if nape_swc@:keep #end
class ZPP_ListenerList{
    public var outer:ListenerList=null;
    public var inner:ZNPList_ZPP_Listener=null;
    public var immutable:Bool=false;
    public var _invalidated:Bool=false;
    public var _invalidate:ZPP_ListenerList->Void=null;
    public var _validate:Void->Void=null;
    public var _modifiable:Void->Void=null;
    public static var internal:Bool=false;
    public var adder:Listener->Bool=null;
    public var post_adder:Listener->Void=null;
    public var subber:Listener->Void=null;
    public var dontremove:Bool=false;
    public var reverse_flag:Bool=false;
     public static function get(list:ZNPList_ZPP_Listener,imm:Bool=false):ListenerList{
        var ret=new ListenerList();
        ret.zpp_inner.inner=list;
        if(imm)ret.zpp_inner.immutable=true;
        ret.zpp_inner.zip_length=true;
        return ret;
    }
    public function valmod():Void{
        validate();
        if(inner.modified){
            if(inner.pushmod)push_ite=null;
            at_ite=null;
            inner.modified=false;
            inner.pushmod=false;
            zip_length=true;
        }
    }
    public function modified():Void{
        zip_length=true;
        at_ite=null;
        push_ite=null;
    }
    public function modify_test():Void{
        #if(!NAPE_RELEASE_BUILD)
        if(_modifiable!=null)_modifiable();
        #end
    }
    public function validate():Void{
        if(_invalidated){
            _invalidated=false;
            if(_validate!=null)_validate();
        }
    }
    public function invalidate():Void{
        _invalidated=true;
        if(_invalidate!=null)_invalidate(this);
    }
    public var at_index:Int=0;
    public var at_ite:ZNPNode_ZPP_Listener=null;
    public var push_ite:ZNPNode_ZPP_Listener=null;
    public var zip_length:Bool=false;
    public var user_length:Int=0;
    public function new(){
        inner=new ZNPList_ZPP_Listener();
        _invalidated=true;
    }
}
#if nape_swc@:keep #end
class ZPP_CbTypeList{
    public var outer:CbTypeList=null;
    public var inner:ZNPList_ZPP_CbType=null;
    public var immutable:Bool=false;
    public var _invalidated:Bool=false;
    public var _invalidate:ZPP_CbTypeList->Void=null;
    public var _validate:Void->Void=null;
    public var _modifiable:Void->Void=null;
    public static var internal:Bool=false;
    public var adder:CbType->Bool=null;
    public var post_adder:CbType->Void=null;
    public var subber:CbType->Void=null;
    public var dontremove:Bool=false;
    public var reverse_flag:Bool=false;
     public static function get(list:ZNPList_ZPP_CbType,imm:Bool=false):CbTypeList{
        var ret=new CbTypeList();
        ret.zpp_inner.inner=list;
        if(imm)ret.zpp_inner.immutable=true;
        ret.zpp_inner.zip_length=true;
        return ret;
    }
    public function valmod():Void{
        validate();
        if(inner.modified){
            if(inner.pushmod)push_ite=null;
            at_ite=null;
            inner.modified=false;
            inner.pushmod=false;
            zip_length=true;
        }
    }
    public function modified():Void{
        zip_length=true;
        at_ite=null;
        push_ite=null;
    }
    public function modify_test():Void{
        #if(!NAPE_RELEASE_BUILD)
        if(_modifiable!=null)_modifiable();
        #end
    }
    public function validate():Void{
        if(_invalidated){
            _invalidated=false;
            if(_validate!=null)_validate();
        }
    }
    public function invalidate():Void{
        _invalidated=true;
        if(_invalidate!=null)_invalidate(this);
    }
    public var at_index:Int=0;
    public var at_ite:ZNPNode_ZPP_CbType=null;
    public var push_ite:ZNPNode_ZPP_CbType=null;
    public var zip_length:Bool=false;
    public var user_length:Int=0;
    public function new(){
        inner=new ZNPList_ZPP_CbType();
        _invalidated=true;
    }
}
#if nape_swc@:keep #end
class ZPP_Vec2List{
    public var outer:Vec2List=null;
    public var inner:ZNPList_ZPP_Vec2=null;
    public var immutable:Bool=false;
    public var _invalidated:Bool=false;
    public var _invalidate:ZPP_Vec2List->Void=null;
    public var _validate:Void->Void=null;
    public var _modifiable:Void->Void=null;
    public static var internal:Bool=false;
    public var adder:Vec2->Bool=null;
    public var post_adder:Vec2->Void=null;
    public var subber:Vec2->Void=null;
    public var dontremove:Bool=false;
    public var reverse_flag:Bool=false;
     public static function get(list:ZNPList_ZPP_Vec2,imm:Bool=false):Vec2List{
        var ret=new Vec2List();
        ret.zpp_inner.inner=list;
        if(imm)ret.zpp_inner.immutable=true;
        ret.zpp_inner.zip_length=true;
        return ret;
    }
    public function valmod():Void{
        validate();
        if(inner.modified){
            if(inner.pushmod)push_ite=null;
            at_ite=null;
            inner.modified=false;
            inner.pushmod=false;
            zip_length=true;
        }
    }
    public function modified():Void{
        zip_length=true;
        at_ite=null;
        push_ite=null;
    }
    public function modify_test():Void{
        #if(!NAPE_RELEASE_BUILD)
        if(_modifiable!=null)_modifiable();
        #end
    }
    public function validate():Void{
        if(_invalidated){
            _invalidated=false;
            if(_validate!=null)_validate();
        }
    }
    public function invalidate():Void{
        _invalidated=true;
        if(_invalidate!=null)_invalidate(this);
    }
    public var at_index:Int=0;
    public var at_ite:ZNPNode_ZPP_Vec2=null;
    public var push_ite:ZNPNode_ZPP_Vec2=null;
    public var zip_length:Bool=false;
    public var user_length:Int=0;
    public function new(){
        inner=new ZNPList_ZPP_Vec2();
        _invalidated=true;
    }
}
#if nape_swc@:keep #end
class ZPP_GeomPolyList{
    public var outer:GeomPolyList=null;
    public var inner:ZNPList_ZPP_GeomPoly=null;
    public var immutable:Bool=false;
    public var _invalidated:Bool=false;
    public var _invalidate:ZPP_GeomPolyList->Void=null;
    public var _validate:Void->Void=null;
    public var _modifiable:Void->Void=null;
    public static var internal:Bool=false;
    public var adder:GeomPoly->Bool=null;
    public var post_adder:GeomPoly->Void=null;
    public var subber:GeomPoly->Void=null;
    public var dontremove:Bool=false;
    public var reverse_flag:Bool=false;
     public static function get(list:ZNPList_ZPP_GeomPoly,imm:Bool=false):GeomPolyList{
        var ret=new GeomPolyList();
        ret.zpp_inner.inner=list;
        if(imm)ret.zpp_inner.immutable=true;
        ret.zpp_inner.zip_length=true;
        return ret;
    }
    public function valmod():Void{
        validate();
        if(inner.modified){
            if(inner.pushmod)push_ite=null;
            at_ite=null;
            inner.modified=false;
            inner.pushmod=false;
            zip_length=true;
        }
    }
    public function modified():Void{
        zip_length=true;
        at_ite=null;
        push_ite=null;
    }
    public function modify_test():Void{
        #if(!NAPE_RELEASE_BUILD)
        if(_modifiable!=null)_modifiable();
        #end
    }
    public function validate():Void{
        if(_invalidated){
            _invalidated=false;
            if(_validate!=null)_validate();
        }
    }
    public function invalidate():Void{
        _invalidated=true;
        if(_invalidate!=null)_invalidate(this);
    }
    public var at_index:Int=0;
    public var at_ite:ZNPNode_ZPP_GeomPoly=null;
    public var push_ite:ZNPNode_ZPP_GeomPoly=null;
    public var zip_length:Bool=false;
    public var user_length:Int=0;
    public function new(){
        inner=new ZNPList_ZPP_GeomPoly();
        _invalidated=true;
    }
}
#if nape_swc@:keep #end
class ZPP_RayResultList{
    public var outer:RayResultList=null;
    public var inner:ZNPList_RayResult=null;
    public var immutable:Bool=false;
    public var _invalidated:Bool=false;
    public var _invalidate:ZPP_RayResultList->Void=null;
    public var _validate:Void->Void=null;
    public var _modifiable:Void->Void=null;
    public static var internal:Bool=false;
    public var adder:RayResult->Bool=null;
    public var post_adder:RayResult->Void=null;
    public var subber:RayResult->Void=null;
    public var dontremove:Bool=false;
    public var reverse_flag:Bool=false;
     public static function get(list:ZNPList_RayResult,imm:Bool=false):RayResultList{
        var ret=new RayResultList();
        ret.zpp_inner.inner=list;
        if(imm)ret.zpp_inner.immutable=true;
        ret.zpp_inner.zip_length=true;
        return ret;
    }
    public function valmod():Void{
        validate();
        if(inner.modified){
            if(inner.pushmod)push_ite=null;
            at_ite=null;
            inner.modified=false;
            inner.pushmod=false;
            zip_length=true;
        }
    }
    public function modified():Void{
        zip_length=true;
        at_ite=null;
        push_ite=null;
    }
    public function modify_test():Void{
        #if(!NAPE_RELEASE_BUILD)
        if(_modifiable!=null)_modifiable();
        #end
    }
    public function validate():Void{
        if(_invalidated){
            _invalidated=false;
            if(_validate!=null)_validate();
        }
    }
    public function invalidate():Void{
        _invalidated=true;
        if(_invalidate!=null)_invalidate(this);
    }
    public var at_index:Int=0;
    public var at_ite:ZNPNode_RayResult=null;
    public var push_ite:ZNPNode_RayResult=null;
    public var zip_length:Bool=false;
    public var user_length:Int=0;
    public function new(){
        inner=new ZNPList_RayResult();
        _invalidated=true;
    }
}
#if nape_swc@:keep #end
class ZPP_ConvexResultList{
    public var outer:ConvexResultList=null;
    public var inner:ZNPList_ConvexResult=null;
    public var immutable:Bool=false;
    public var _invalidated:Bool=false;
    public var _invalidate:ZPP_ConvexResultList->Void=null;
    public var _validate:Void->Void=null;
    public var _modifiable:Void->Void=null;
    public static var internal:Bool=false;
    public var adder:ConvexResult->Bool=null;
    public var post_adder:ConvexResult->Void=null;
    public var subber:ConvexResult->Void=null;
    public var dontremove:Bool=false;
    public var reverse_flag:Bool=false;
     public static function get(list:ZNPList_ConvexResult,imm:Bool=false):ConvexResultList{
        var ret=new ConvexResultList();
        ret.zpp_inner.inner=list;
        if(imm)ret.zpp_inner.immutable=true;
        ret.zpp_inner.zip_length=true;
        return ret;
    }
    public function valmod():Void{
        validate();
        if(inner.modified){
            if(inner.pushmod)push_ite=null;
            at_ite=null;
            inner.modified=false;
            inner.pushmod=false;
            zip_length=true;
        }
    }
    public function modified():Void{
        zip_length=true;
        at_ite=null;
        push_ite=null;
    }
    public function modify_test():Void{
        #if(!NAPE_RELEASE_BUILD)
        if(_modifiable!=null)_modifiable();
        #end
    }
    public function validate():Void{
        if(_invalidated){
            _invalidated=false;
            if(_validate!=null)_validate();
        }
    }
    public function invalidate():Void{
        _invalidated=true;
        if(_invalidate!=null)_invalidate(this);
    }
    public var at_index:Int=0;
    public var at_ite:ZNPNode_ConvexResult=null;
    public var push_ite:ZNPNode_ConvexResult=null;
    public var zip_length:Bool=false;
    public var user_length:Int=0;
    public function new(){
        inner=new ZNPList_ConvexResult();
        _invalidated=true;
    }
}
#if nape_swc@:keep #end
class ZPP_EdgeList{
    public var outer:EdgeList=null;
    public var inner:ZNPList_ZPP_Edge=null;
    public var immutable:Bool=false;
    public var _invalidated:Bool=false;
    public var _invalidate:ZPP_EdgeList->Void=null;
    public var _validate:Void->Void=null;
    public var _modifiable:Void->Void=null;
    public static var internal:Bool=false;
    public var adder:Edge->Bool=null;
    public var post_adder:Edge->Void=null;
    public var subber:Edge->Void=null;
    public var dontremove:Bool=false;
    public var reverse_flag:Bool=false;
     public static function get(list:ZNPList_ZPP_Edge,imm:Bool=false):EdgeList{
        var ret=new EdgeList();
        ret.zpp_inner.inner=list;
        if(imm)ret.zpp_inner.immutable=true;
        ret.zpp_inner.zip_length=true;
        return ret;
    }
    public function valmod():Void{
        validate();
        if(inner.modified){
            if(inner.pushmod)push_ite=null;
            at_ite=null;
            inner.modified=false;
            inner.pushmod=false;
            zip_length=true;
        }
    }
    public function modified():Void{
        zip_length=true;
        at_ite=null;
        push_ite=null;
    }
    public function modify_test():Void{
        #if(!NAPE_RELEASE_BUILD)
        if(_modifiable!=null)_modifiable();
        #end
    }
    public function validate():Void{
        if(_invalidated){
            _invalidated=false;
            if(_validate!=null)_validate();
        }
    }
    public function invalidate():Void{
        _invalidated=true;
        if(_invalidate!=null)_invalidate(this);
    }
    public var at_index:Int=0;
    public var at_ite:ZNPNode_ZPP_Edge=null;
    public var push_ite:ZNPNode_ZPP_Edge=null;
    public var zip_length:Bool=false;
    public var user_length:Int=0;
    public function new(){
        inner=new ZNPList_ZPP_Edge();
        _invalidated=true;
    }
}
#if nape_swc@:keep #end
class ZPP_ShapeList{
    public var outer:ShapeList=null;
    public var inner:ZNPList_ZPP_Shape=null;
    public var immutable:Bool=false;
    public var _invalidated:Bool=false;
    public var _invalidate:ZPP_ShapeList->Void=null;
    public var _validate:Void->Void=null;
    public var _modifiable:Void->Void=null;
    public static var internal:Bool=false;
    public var adder:Shape->Bool=null;
    public var post_adder:Shape->Void=null;
    public var subber:Shape->Void=null;
    public var dontremove:Bool=false;
    public var reverse_flag:Bool=false;
     public static function get(list:ZNPList_ZPP_Shape,imm:Bool=false):ShapeList{
        var ret=new ShapeList();
        ret.zpp_inner.inner=list;
        if(imm)ret.zpp_inner.immutable=true;
        ret.zpp_inner.zip_length=true;
        return ret;
    }
    public function valmod():Void{
        validate();
        if(inner.modified){
            if(inner.pushmod)push_ite=null;
            at_ite=null;
            inner.modified=false;
            inner.pushmod=false;
            zip_length=true;
        }
    }
    public function modified():Void{
        zip_length=true;
        at_ite=null;
        push_ite=null;
    }
    public function modify_test():Void{
        #if(!NAPE_RELEASE_BUILD)
        if(_modifiable!=null)_modifiable();
        #end
    }
    public function validate():Void{
        if(_invalidated){
            _invalidated=false;
            if(_validate!=null)_validate();
        }
    }
    public function invalidate():Void{
        _invalidated=true;
        if(_invalidate!=null)_invalidate(this);
    }
    public var at_index:Int=0;
    public var at_ite:ZNPNode_ZPP_Shape=null;
    public var push_ite:ZNPNode_ZPP_Shape=null;
    public var zip_length:Bool=false;
    public var user_length:Int=0;
    public function new(){
        inner=new ZNPList_ZPP_Shape();
        _invalidated=true;
    }
}
#if nape_swc@:keep #end
class ZPP_InteractionGroupList{
    public var outer:InteractionGroupList=null;
    public var inner:ZNPList_ZPP_InteractionGroup=null;
    public var immutable:Bool=false;
    public var _invalidated:Bool=false;
    public var _invalidate:ZPP_InteractionGroupList->Void=null;
    public var _validate:Void->Void=null;
    public var _modifiable:Void->Void=null;
    public static var internal:Bool=false;
    public var adder:InteractionGroup->Bool=null;
    public var post_adder:InteractionGroup->Void=null;
    public var subber:InteractionGroup->Void=null;
    public var dontremove:Bool=false;
    public var reverse_flag:Bool=false;
     public static function get(list:ZNPList_ZPP_InteractionGroup,imm:Bool=false):InteractionGroupList{
        var ret=new InteractionGroupList();
        ret.zpp_inner.inner=list;
        if(imm)ret.zpp_inner.immutable=true;
        ret.zpp_inner.zip_length=true;
        return ret;
    }
    public function valmod():Void{
        validate();
        if(inner.modified){
            if(inner.pushmod)push_ite=null;
            at_ite=null;
            inner.modified=false;
            inner.pushmod=false;
            zip_length=true;
        }
    }
    public function modified():Void{
        zip_length=true;
        at_ite=null;
        push_ite=null;
    }
    public function modify_test():Void{
        #if(!NAPE_RELEASE_BUILD)
        if(_modifiable!=null)_modifiable();
        #end
    }
    public function validate():Void{
        if(_invalidated){
            _invalidated=false;
            if(_validate!=null)_validate();
        }
    }
    public function invalidate():Void{
        _invalidated=true;
        if(_invalidate!=null)_invalidate(this);
    }
    public var at_index:Int=0;
    public var at_ite:ZNPNode_ZPP_InteractionGroup=null;
    public var push_ite:ZNPNode_ZPP_InteractionGroup=null;
    public var zip_length:Bool=false;
    public var user_length:Int=0;
    public function new(){
        inner=new ZNPList_ZPP_InteractionGroup();
        _invalidated=true;
    }
}
#if nape_swc@:keep #end
class ZPP_ArbiterList{
    public var outer:ArbiterList=null;
    public var inner:ZNPList_ZPP_Arbiter=null;
    public var immutable:Bool=false;
    public var _invalidated:Bool=false;
    public var _invalidate:ZPP_ArbiterList->Void=null;
    public var _validate:Void->Void=null;
    public var _modifiable:Void->Void=null;
    public static var internal:Bool=false;
    public var adder:Arbiter->Bool=null;
    public var post_adder:Arbiter->Void=null;
    public var subber:Arbiter->Void=null;
    public var dontremove:Bool=false;
    public var reverse_flag:Bool=false;
     public static function get(list:ZNPList_ZPP_Arbiter,imm:Bool=false):ArbiterList{
        var ret=new ArbiterList();
        ret.zpp_inner.inner=list;
        if(imm)ret.zpp_inner.immutable=true;
        ret.zpp_inner.zip_length=true;
        return ret;
    }
    public function valmod():Void{
        validate();
        if(inner.modified){
            if(inner.pushmod)push_ite=null;
            at_ite=null;
            inner.modified=false;
            inner.pushmod=false;
            zip_length=true;
        }
    }
    public function modified():Void{
        zip_length=true;
        at_ite=null;
        push_ite=null;
    }
    public function modify_test():Void{
        #if(!NAPE_RELEASE_BUILD)
        if(_modifiable!=null)_modifiable();
        #end
    }
    public function validate():Void{
        if(_invalidated){
            _invalidated=false;
            if(_validate!=null)_validate();
        }
    }
    public function invalidate():Void{
        _invalidated=true;
        if(_invalidate!=null)_invalidate(this);
    }
    public var at_index:Int=0;
    public var at_ite:ZNPNode_ZPP_Arbiter=null;
    public var push_ite:ZNPNode_ZPP_Arbiter=null;
    public var zip_length:Bool=false;
    public var user_length:Int=0;
    public function new(){
        inner=new ZNPList_ZPP_Arbiter();
        _invalidated=true;
    }
}
#if nape_swc@:keep #end
class ZPP_ContactList{
    public var outer:ContactList=null;
    public var inner:ZPP_Contact=null;
    public var immutable:Bool=false;
    public var _invalidated:Bool=false;
    public var _invalidate:ZPP_ContactList->Void=null;
    public var _validate:Void->Void=null;
    public var _modifiable:Void->Void=null;
    public static var internal:Bool=false;
    public var adder:Contact->Bool=null;
    public var post_adder:Contact->Void=null;
    public var subber:Contact->Void=null;
    public var dontremove:Bool=false;
    public var reverse_flag:Bool=false;
     public static function get(list:ZPP_Contact,imm:Bool=false):ContactList{
        var ret=new ContactList();
        ret.zpp_inner.inner=list;
        if(imm)ret.zpp_inner.immutable=true;
        ret.zpp_inner.zip_length=true;
        return ret;
    }
    public function valmod():Void{
        validate();
        if(inner.modified){
            if(inner.pushmod)push_ite=null;
            at_ite=null;
            inner.modified=false;
            inner.pushmod=false;
            zip_length=true;
        }
    }
    public function modified():Void{
        zip_length=true;
        at_ite=null;
        push_ite=null;
    }
    public function modify_test():Void{
        #if(!NAPE_RELEASE_BUILD)
        if(_modifiable!=null)_modifiable();
        #end
    }
    public function validate():Void{
        if(_invalidated){
            _invalidated=false;
            if(_validate!=null)_validate();
        }
    }
    public function invalidate():Void{
        _invalidated=true;
        if(_invalidate!=null)_invalidate(this);
    }
    public var at_index:Int=0;
    public var at_ite:ZPP_Contact=null;
    public var push_ite:ZPP_Contact=null;
    public var zip_length:Bool=false;
    public var user_length:Int=0;
    public function new(){
        inner=new ZPP_Contact();
        _invalidated=true;
    }
}
