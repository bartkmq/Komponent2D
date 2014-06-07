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
class ZPP_PartitionPair{
    static public var zpp_pool:ZPP_PartitionPair=null;
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
    
    public var next:ZPP_PartitionPair=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function elem():ZPP_PartitionPair{
        return this;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function begin():ZPP_PartitionPair{
        return next;
    }
    public var _inuse:Bool=false;
    public var modified:Bool=false;
    public var pushmod:Bool=false;
    public var length:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function setbegin(i:ZPP_PartitionPair):Void{
        next=i;
        modified=true;
        pushmod=true;
    }
    public function add(o:ZPP_PartitionPair):ZPP_PartitionPair{
        return inlined_add(o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_add(o:ZPP_PartitionPair):ZPP_PartitionPair{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_PartitionPair"+"] add -> o="+o);
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
    public function addAll(x:ZPP_PartitionPair):Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                x!=null;
            };
            if(!res)throw "assert("+"x!=null"+") :: "+("[ListMixin("+"ZPP_PartitionPair"+"] addAll -> "+x);
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
    public function insert(cur:ZPP_PartitionPair,o:ZPP_PartitionPair):ZPP_PartitionPair{
        return inlined_insert(cur,o);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insert(cur:ZPP_PartitionPair,o:ZPP_PartitionPair):ZPP_PartitionPair{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                o!=null&&!has(o);
            };
            if(!res)throw "assert("+"o!=null&&!has(o)"+") :: "+("[ListMixin("+"ZPP_PartitionPair"+"] cur -> "+cur+" -> "+o);
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
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_PartitionPair"+"] pop");
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
    public function pop_unsafe():ZPP_PartitionPair{
        return inlined_pop_unsafe();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_pop_unsafe():ZPP_PartitionPair{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_PartitionPair"+"] pop_unsafe");
            #end
        };
        var ret=front();
        pop();
        return ret;
    }
    public function remove(obj:ZPP_PartitionPair):Void{
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
    public function try_remove(obj:ZPP_PartitionPair):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_PartitionPair"+"] remove -> "+obj);
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
    function inlined_remove(obj:ZPP_PartitionPair):Void{
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
    function inlined_try_remove(obj:ZPP_PartitionPair):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_PartitionPair"+"] remove -> "+obj);
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
    public function erase(pre:ZPP_PartitionPair):ZPP_PartitionPair{
        return inlined_erase(pre);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_erase(pre:ZPP_PartitionPair):ZPP_PartitionPair{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !empty();
            };
            if(!res)throw "assert("+"!empty()"+") :: "+("[ListMixin("+"ZPP_PartitionPair"+"] erase -> "+pre);
            #end
        };
        var old:ZPP_PartitionPair;
        var ret:ZPP_PartitionPair;
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
    public function splice(pre:ZPP_PartitionPair,n:Int):ZPP_PartitionPair{
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
    public function has(obj:ZPP_PartitionPair):Bool{
        return inlined_has(obj);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_has(obj:ZPP_PartitionPair):Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                obj!=null;
            };
            if(!res)throw "assert("+"obj!=null"+") :: "+("[ListMixin("+"ZPP_PartitionPair"+"] has -> "+obj);
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
    function front():ZPP_PartitionPair{
        return begin().elem();
    }
    public function back():ZPP_PartitionPair{
        var ret=begin();
        var cur=ret;
        while(cur!=null){
            ret=cur;
            cur=cur.next;
        }
        return ret.elem();
    }
    public function iterator_at(ind:Int):ZPP_PartitionPair{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=-1&&ind<size();
            };
            if(!res)throw "assert("+"ind>=-1&&ind<size()"+") :: "+("[ListMixin("+"ZPP_PartitionPair"+"] iterator_at -> "+ind);
            #end
        };
        var ret=begin();
        while(ind-->0&&ret!=null)ret=ret.next;
        return ret;
    }
    public function at(ind:Int):ZPP_PartitionPair{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ind>=0&&ind<size();
            };
            if(!res)throw "assert("+"ind>=0&&ind<size()"+") :: "+("[ListMixin("+"ZPP_PartitionPair"+"] at -> "+ind);
            #end
        };
        var it=iterator_at(ind);
        return if(it!=null)it.elem()else null;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free(){
        a=b=null;
        node=null;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc(){}
    public var a:ZPP_PartitionVertex=null;
    public var b:ZPP_PartitionVertex=null;
    public var id:Int=0;
    public var di:Int=0;
    public function new(){}
    public static#if NAPE_NO_INLINE#else inline #end
    function get(a:ZPP_PartitionVertex,b:ZPP_PartitionVertex){
        var ret;
        {
            if(ZPP_PartitionPair.zpp_pool==null){
                ret=new ZPP_PartitionPair();
                #if NAPE_POOL_STATS ZPP_PartitionPair.POOL_TOT++;
                ZPP_PartitionPair.POOL_ADDNEW++;
                #end
            }
            else{
                ret=ZPP_PartitionPair.zpp_pool;
                ZPP_PartitionPair.zpp_pool=ret.next;
                ret.next=null;
                #if NAPE_POOL_STATS ZPP_PartitionPair.POOL_CNT--;
                ZPP_PartitionPair.POOL_ADD++;
                #end
            }
            ret.alloc();
        };
        ret.a=a;
        ret.b=b;
        if(a.id<b.id){
            ret.id=a.id;
            ret.di=b.id;
        }
        else{
            ret.id=b.id;
            ret.di=a.id;
        }
        return ret;
    }
    public var node:ZPP_Set_ZPP_PartitionPair=null;
    public static function edge_swap(a:ZPP_PartitionPair,b:ZPP_PartitionPair){
        {
            var t=a.node;
            a.node=b.node;
            b.node=t;
        };
    }
    public static function edge_lt(a:ZPP_PartitionPair,b:ZPP_PartitionPair){
        return a.id<b.id||(a.id==b.id&&a.di<b.di);
    }
}
#if nape_swc@:keep #end
class ZPP_Triangular{
    #if NAPE_NO_INLINE#else inline #end
    static function lt(p:ZPP_PartitionVertex,q:ZPP_PartitionVertex){
        return p.y<q.y||(p.y==q.y&&p.x<q.x);
    }
    #if NAPE_NO_INLINE#else inline #end
    static function right_turn(a:ZPP_PartitionVertex,b:ZPP_PartitionVertex,c:ZPP_PartitionVertex){
        var ux:Float=0.0;
        var uy:Float=0.0;
        {
            ux=c.x-b.x;
            uy=c.y-b.y;
        };
        var vx:Float=0.0;
        var vy:Float=0.0;
        {
            vx=b.x-a.x;
            vy=b.y-a.y;
        };
        return(vy*ux-vx*uy);
    }
    static var queue:ZNPList_ZPP_PartitionVertex=null;
    static var stack:ZNPList_ZPP_PartitionVertex=null;
    public static function delaunay(A:ZPP_PartitionVertex,B:ZPP_PartitionVertex,C:ZPP_PartitionVertex,D:ZPP_PartitionVertex){
        var ux:Float=0.0;
        var uy:Float=0.0;
        var vx:Float=0.0;
        var vy:Float=0.0;
        if(({
            {
                ux=C.x-B.x;
                uy=C.y-B.y;
            };
            {
                vx=B.x-A.x;
                vy=B.y-A.y;
            };
            (vy*ux-vx*uy)>=0;
        })||({
            {
                ux=D.x-C.x;
                uy=D.y-C.y;
            };
            {
                vx=C.x-B.x;
                vy=C.y-B.y;
            };
            (vy*ux-vx*uy)>=0;
        })||({
            {
                ux=A.x-D.x;
                uy=A.y-D.y;
            };
            {
                vx=D.x-C.x;
                vy=D.y-C.y;
            };
            (vy*ux-vx*uy)>=0;
        })||({
            {
                ux=B.x-A.x;
                uy=B.y-A.y;
            };
            {
                vx=A.x-D.x;
                vy=A.y-D.y;
            };
            (vy*ux-vx*uy)>=0;
        })){
            return true;
        }
        return(B.x*(C.y*D.mag-C.mag*D.y)-C.x*(B.y*D.mag-B.mag*D.y)+D.x*(B.y*C.mag-B.mag*C.y))-(A.x*(C.y*D.mag-C.mag*D.y)-C.x*(A.y*D.mag-A.mag*D.y)+D.x*(A.y*C.mag-A.mag*C.y))+(A.x*(B.y*D.mag-B.mag*D.y)-B.x*(A.y*D.mag-A.mag*D.y)+D.x*(A.y*B.mag-A.mag*B.y))-(A.x*(B.y*C.mag-B.mag*C.y)-B.x*(A.y*C.mag-A.mag*C.y)+C.x*(A.y*B.mag-A.mag*B.y))>0;
        return(B.x*(C.y*D.mag-C.mag*D.y)+B.y*(C.mag*D.x-C.x*D.mag)+B.mag*(C.x*D.y-C.y*D.x)+A.x*(C.mag*D.y-C.y*D.mag+B.mag*(C.y-D.y)+B.y*(D.mag-C.mag))+A.y*(C.x*D.mag-C.mag*D.x+B.mag*(D.x-C.x)+B.x*(C.mag-D.mag))+A.mag*(C.y*D.x-C.x*D.y+B.x*(D.y-C.y)+B.y*(C.x-D.x)))>0;
    }
    static var edgeSet:ZPP_Set_ZPP_PartitionPair=null;
    public static function optimise(P:ZPP_PartitionedPoly){
        {
            var F=P.vertices;
            var L=P.vertices;
            if(F!=null){
                var nite=F;
                do{
                    var p=nite;
                    {
                        {
                            p.sort();
                            p.mag=(p.x*p.x+p.y*p.y);
                        };
                    }
                    nite=nite.next;
                }
                while(nite!=L);
            }
        };
        if(edgeSet==null){
            {
                if(ZPP_Set_ZPP_PartitionPair.zpp_pool==null){
                    edgeSet=new ZPP_Set_ZPP_PartitionPair();
                    #if NAPE_POOL_STATS ZPP_Set_ZPP_PartitionPair.POOL_TOT++;
                    ZPP_Set_ZPP_PartitionPair.POOL_ADDNEW++;
                    #end
                }
                else{
                    edgeSet=ZPP_Set_ZPP_PartitionPair.zpp_pool;
                    ZPP_Set_ZPP_PartitionPair.zpp_pool=edgeSet.next;
                    edgeSet.next=null;
                    #if NAPE_POOL_STATS ZPP_Set_ZPP_PartitionPair.POOL_CNT--;
                    ZPP_Set_ZPP_PartitionPair.POOL_ADD++;
                    #end
                }
                edgeSet.alloc();
            };
            edgeSet.lt=ZPP_PartitionPair.edge_lt;
            edgeSet.swapped=ZPP_PartitionPair.edge_swap;
        }
        var edgeStack:ZPP_PartitionPair;
        {
            if(ZPP_PartitionPair.zpp_pool==null){
                edgeStack=new ZPP_PartitionPair();
                #if NAPE_POOL_STATS ZPP_PartitionPair.POOL_TOT++;
                ZPP_PartitionPair.POOL_ADDNEW++;
                #end
            }
            else{
                edgeStack=ZPP_PartitionPair.zpp_pool;
                ZPP_PartitionPair.zpp_pool=edgeStack.next;
                edgeStack.next=null;
                #if NAPE_POOL_STATS ZPP_PartitionPair.POOL_CNT--;
                ZPP_PartitionPair.POOL_ADD++;
                #end
            }
            edgeStack.alloc();
        };
        {
            var F=P.vertices;
            var L=P.vertices;
            if(F!=null){
                var nite=F;
                do{
                    var p=nite;
                    {
                        {
                            var q0=p.next;
                            p.diagonals.reverse();
                            {
                                var cx_ite=p.diagonals.begin();
                                while(cx_ite!=null){
                                    var q=cx_ite.elem();
                                    {
                                        if(q.id<p.id){
                                            q0=q;
                                            {
                                                cx_ite=cx_ite.next;
                                                continue;
                                            };
                                        }
                                        var q1=(cx_ite.next==null?p.prev:cx_ite.next.elem());
                                        if(!delaunay(p,q0,q,q1)){
                                            var edge=ZPP_PartitionPair.get(p,q);
                                            edgeStack.add(edge);
                                            edge.node=edgeSet.insert(edge);
                                        }
                                        q0=q;
                                    };
                                    cx_ite=cx_ite.next;
                                }
                            };
                        };
                    }
                    nite=nite.next;
                }
                while(nite!=L);
            }
        };
        while(!edgeStack.empty()){
            var edge=edgeStack.pop_unsafe();
            var A=edge.a;
            var C=edge.b;
            var B=A.next;
            var D=null;
            {
                var cx_ite=A.diagonals.begin();
                while(cx_ite!=null){
                    var p=cx_ite.elem();
                    {
                        if(p==C){
                            cx_ite=cx_ite.next;
                            D=if(cx_ite==null)A.prev else cx_ite.elem();
                            break;
                        }
                        B=p;
                    };
                    cx_ite=cx_ite.next;
                }
            };
            A.diagonals.remove(C);
            C.diagonals.remove(A);
            {
                if(C==B.next){
                    B.diagonals.add(D);
                }
                else{
                    {
                        var cx_ite=B.diagonals.begin();
                        while(cx_ite!=null){
                            var p=cx_ite.elem();
                            {
                                if(p==C){
                                    B.diagonals.insert(cx_ite,D);
                                    break;
                                }
                            };
                            cx_ite=cx_ite.next;
                        }
                    };
                }
            };
            {
                if(A==D.next){
                    D.diagonals.add(B);
                }
                else{
                    {
                        var cx_ite=D.diagonals.begin();
                        while(cx_ite!=null){
                            var p=cx_ite.elem();
                            {
                                if(p==A){
                                    D.diagonals.insert(cx_ite,B);
                                    break;
                                }
                            };
                            cx_ite=cx_ite.next;
                        }
                    };
                }
            };
            edgeSet.remove_node(edge.node);
            {
                var o=edge;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_PartitionPair"+", in obj: "+"edge"+")");
                    #end
                };
                o.free();
                o.next=ZPP_PartitionPair.zpp_pool;
                ZPP_PartitionPair.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_PartitionPair.POOL_CNT++;
                ZPP_PartitionPair.POOL_SUB++;
                #end
            };
        }
        {
            var o=edgeStack;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_PartitionPair"+", in obj: "+"edgeStack"+")");
                #end
            };
            o.free();
            o.next=ZPP_PartitionPair.zpp_pool;
            ZPP_PartitionPair.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_PartitionPair.POOL_CNT++;
            ZPP_PartitionPair.POOL_SUB++;
            #end
        };
    }
    public static function triangulate(P:ZPP_PartitionedPoly){
        var min=P.vertices;
        var max=P.vertices;
        {
            var F=P.vertices.next;
            var L=P.vertices;
            if(F!=null){
                var nite=F;
                do{
                    var p=nite;
                    {
                        {
                            if(lt(p,min))min=p;
                            if(lt(max,p))max=p;
                        };
                    }
                    nite=nite.next;
                }
                while(nite!=L);
            }
        };
        if(queue==null)queue=new ZNPList_ZPP_PartitionVertex();
        var rp=max.prev;
        var lp=max.next;
        queue.add(max);
        while(rp!=min||lp!=min){
            if(rp==min||(lp!=min&&lt(rp,lp))){
                queue.add(lp);
                lp.rightchain=false;
                lp=lp.next;
            }
            else{
                queue.add(rp);
                rp.rightchain=true;
                rp=rp.prev;
            }
        }
        queue.add(min);
        if(stack==null)stack=new ZNPList_ZPP_PartitionVertex();
        stack.add(queue.pop_unsafe());
        var pre:ZPP_PartitionVertex=queue.pop_unsafe();
        stack.add(pre);
        while(true){
            var p=queue.pop_unsafe();
            if(queue.empty())break;
            if(p.rightchain!=stack.front().rightchain){
                while(true){
                    var s=stack.pop_unsafe();
                    if(stack.empty())break;
                    P.add_diagonal(s,p);
                }
                stack.add(pre);
            }
            else{
                var q=stack.pop_unsafe();
                while(!stack.empty()){
                    var s=stack.front();
                    var right=right_turn(s,q,p);
                    if((p.rightchain&&right>=0)||(!p.rightchain&&right<=0))break;
                    P.add_diagonal(s,p);
                    q=s;
                    stack.pop();
                }
                stack.add(q);
            }
            stack.add(p);
            pre=p;
        }
        if(!stack.empty()){
            stack.pop();
            while(!stack.empty()){
                var s=stack.pop_unsafe();
                if(stack.empty())break;
                P.add_diagonal(max,s);
            }
        }
        return P;
    }
}
