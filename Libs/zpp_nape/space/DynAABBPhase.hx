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
class ZPP_AABBNode{
    public var aabb:ZPP_AABB=null;
    public var shape:ZPP_Shape=null;
    public var dyn:Bool=false;
    public var parent:ZPP_AABBNode=null;
    public var child1:ZPP_AABBNode=null;
    public var child2:ZPP_AABBNode=null;
    public var height:Int=0;
    public var rayt:Float=0.0;
    public function new(){
        height=-1;
    }
    public var next:ZPP_AABBNode=null;
    static public var zpp_pool:ZPP_AABBNode=null;
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
        moved=false;
        synced=false;
        first_sync=false;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free(){
        height=-1;
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
        child1=child2=parent=null;
        next=null;
        snext=null;
        mnext=null;
    }
    public var mnext:ZPP_AABBNode=null;
    public var moved:Bool=false;
    public var snext:ZPP_AABBNode=null;
    public var synced:Bool=false;
    public var first_sync:Bool=false;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function isLeaf(){
        return child1==null;
    }
}
#if nape_swc@:keep #end
class ZPP_AABBPair{
    public var n1:ZPP_AABBNode=null;
    public var n2:ZPP_AABBNode=null;
    public var first:Bool=false;
    public var sleeping:Bool=false;
    public var id:Int=0;
    public var di:Int=0;
    public var arb:ZPP_Arbiter=null;
    public var next:ZPP_AABBPair=null;
    static public var zpp_pool:ZPP_AABBPair=null;
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
    
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc(){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                arb==null;
            };
            if(!res)throw "assert("+"arb==null"+") :: "+("node still has an arbiter when going into pool??");
            #end
        };
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free(){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                arb==null;
            };
            if(!res)throw "assert("+"arb==null"+") :: "+("node still has an arbiter when going into pool??");
            #end
        };
        n1=n2=null;
        sleeping=false;
    }
}
#if nape_swc@:keep #end
class ZPP_AABBTree{
    public var root:ZPP_AABBNode=null;
    public function new(){}
    public function clear(){
        if(root==null)return;
        var stack:ZPP_AABBNode=null;
        {
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    root.next==null;
                };
                if(!res)throw "assert("+"root.next==null"+") :: "+("object already in list");
                #end
            };
            root.next=stack;
            stack=root;
        };
        while(stack!=null){
            var node=({
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        stack!=null;
                    };
                    if(!res)throw "assert("+"stack!=null"+") :: "+("pop from empty list!");
                    #end
                };
                var ret=stack;
                stack=ret.next;
                ret.next=null;
                ret;
            });
            if(node.isLeaf()){
                node.shape.node=null;
                node.shape.removedFromSpace();
                node.shape=null;
            }
            else{
                if(node.child1!=null){
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            node.child1.next==null;
                        };
                        if(!res)throw "assert("+"node.child1.next==null"+") :: "+("object already in list");
                        #end
                    };
                    node.child1.next=stack;
                    stack=node.child1;
                };
                if(node.child2!=null){
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            node.child2.next==null;
                        };
                        if(!res)throw "assert("+"node.child2.next==null"+") :: "+("object already in list");
                        #end
                    };
                    node.child2.next=stack;
                    stack=node.child2;
                };
            }
            {
                var o=node;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_AABBNode"+", in obj: "+"node"+")");
                    #end
                };
                o.free();
                o.next=ZPP_AABBNode.zpp_pool;
                ZPP_AABBNode.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_AABBNode.POOL_CNT++;
                ZPP_AABBNode.POOL_SUB++;
                #end
            };
        }
        root=null;
    }
    public static var tmpaabb=new ZPP_AABB();
    function insertLeaf(leaf:ZPP_AABBNode){
        inlined_insertLeaf(leaf);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_insertLeaf(leaf:ZPP_AABBNode){
        if(root==null){
            root=leaf;
            root.parent=null;
        }
        else{
            var leafaabb=leaf.aabb;
            var node=root;
            while(!node.isLeaf()){
                var child1=node.child1;
                var child2=node.child2;
                var area=node.aabb.perimeter();
                tmpaabb.setCombine(node.aabb,leafaabb);
                var carea=tmpaabb.perimeter();
                var cost=2*carea;
                var icost=2*(carea-area);
                var cost1=({
                    tmpaabb.setCombine(leafaabb,child1.aabb);
                    if(child1.isLeaf())tmpaabb.perimeter()+icost;
                    else{
                        var oarea=child1.aabb.perimeter();
                        var narea=tmpaabb.perimeter();
                        (narea-oarea)+icost;
                    }
                });
                var cost2=({
                    tmpaabb.setCombine(leafaabb,child2.aabb);
                    if(child2.isLeaf())tmpaabb.perimeter()+icost;
                    else{
                        var oarea=child2.aabb.perimeter();
                        var narea=tmpaabb.perimeter();
                        (narea-oarea)+icost;
                    }
                });
                if(cost<cost1&&cost<cost2)break;
                else node=cost1<cost2?child1:child2;
            }
            var sibling=node;
            var oparent=sibling.parent;
            var nparent;
            {
                if(ZPP_AABBNode.zpp_pool==null){
                    nparent=new ZPP_AABBNode();
                    #if NAPE_POOL_STATS ZPP_AABBNode.POOL_TOT++;
                    ZPP_AABBNode.POOL_ADDNEW++;
                    #end
                }
                else{
                    nparent=ZPP_AABBNode.zpp_pool;
                    ZPP_AABBNode.zpp_pool=nparent.next;
                    nparent.next=null;
                    #if NAPE_POOL_STATS ZPP_AABBNode.POOL_CNT--;
                    ZPP_AABBNode.POOL_ADD++;
                    #end
                }
                nparent.alloc();
            };
            nparent.parent=oparent;
            nparent.aabb.setCombine(leafaabb,sibling.aabb);
            nparent.height=sibling.height+1;
            if(oparent!=null){
                if(oparent.child1==sibling)oparent.child1=nparent;
                else oparent.child2=nparent;
                nparent.child1=sibling;
                nparent.child2=leaf;
                sibling.parent=nparent;
                leaf.parent=nparent;
            }
            else{
                nparent.child1=sibling;
                nparent.child2=leaf;
                sibling.parent=nparent;
                leaf.parent=nparent;
                root=nparent;
            }
            node=leaf.parent;
            while(node!=null){
                node=balance(node);
                var child1=node.child1;
                var child2=node.child2;
                node.height=1+({
                    var x=child1.height;
                    var y=child2.height;
                    x>y?x:y;
                });
                node.aabb.setCombine(child1.aabb,child2.aabb);
                node=node.parent;
            }
        }
    }
    public function removeLeaf(leaf:ZPP_AABBNode){
        inlined_removeLeaf(leaf);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function inlined_removeLeaf(leaf:ZPP_AABBNode){
        if(leaf==root){
            root=null;
            return;
        }
        else{
            var parent=leaf.parent;
            var gparent=parent.parent;
            var sibling=if(parent.child1==leaf)parent.child2 else parent.child1;
            if(gparent!=null){
                if(gparent.child1==parent)gparent.child1=sibling;
                else gparent.child2=sibling;
                sibling.parent=gparent;
                {
                    var o=parent;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            o!=null;
                        };
                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_AABBNode"+", in obj: "+"parent"+")");
                        #end
                    };
                    o.free();
                    o.next=ZPP_AABBNode.zpp_pool;
                    ZPP_AABBNode.zpp_pool=o;
                    #if NAPE_POOL_STATS ZPP_AABBNode.POOL_CNT++;
                    ZPP_AABBNode.POOL_SUB++;
                    #end
                };
                var node=gparent;
                while(node!=null){
                    node=balance(node);
                    var child1=node.child1;
                    var child2=node.child2;
                    node.aabb.setCombine(child1.aabb,child2.aabb);
                    node.height=1+({
                        var x=child1.height;
                        var y=child2.height;
                        x>y?x:y;
                    });
                    node=node.parent;
                }
            }
            else{
                root=sibling;
                sibling.parent=null;
                {
                    var o=parent;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            o!=null;
                        };
                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_AABBNode"+", in obj: "+"parent"+")");
                        #end
                    };
                    o.free();
                    o.next=ZPP_AABBNode.zpp_pool;
                    ZPP_AABBNode.zpp_pool=o;
                    #if NAPE_POOL_STATS ZPP_AABBNode.POOL_CNT++;
                    ZPP_AABBNode.POOL_SUB++;
                    #end
                };
            }
        }
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function balance(a:ZPP_AABBNode){
        if(a.isLeaf()||a.height<2)return a;
        else{
            var b=a.child1;
            var c=a.child2;
            var balance=c.height-b.height;
            if(balance>1){
                var f=c.child1;
                var g=c.child2;
                c.child1=a;
                c.parent=a.parent;
                a.parent=c;
                if(c.parent!=null){
                    if(c.parent.child1==a)c.parent.child1=c;
                    else c.parent.child2=c;
                }
                else root=c;
                if(f.height>g.height){
                    c.child2=f;
                    a.child2=g;
                    g.parent=a;
                    a.aabb.setCombine(b.aabb,g.aabb);
                    c.aabb.setCombine(a.aabb,f.aabb);
                    a.height=1+({
                        var x=b.height;
                        var y=g.height;
                        x>y?x:y;
                    });
                    c.height=1+({
                        var x=a.height;
                        var y=f.height;
                        x>y?x:y;
                    });
                };
                else{
                    c.child2=g;
                    a.child2=f;
                    f.parent=a;
                    a.aabb.setCombine(b.aabb,f.aabb);
                    c.aabb.setCombine(a.aabb,g.aabb);
                    a.height=1+({
                        var x=b.height;
                        var y=f.height;
                        x>y?x:y;
                    });
                    c.height=1+({
                        var x=a.height;
                        var y=g.height;
                        x>y?x:y;
                    });
                };
                return c;
            };
            else if(balance<-1){
                var f=b.child1;
                var g=b.child2;
                b.child1=a;
                b.parent=a.parent;
                a.parent=b;
                if(b.parent!=null){
                    if(b.parent.child1==a)b.parent.child1=b;
                    else b.parent.child2=b;
                }
                else root=b;
                if(f.height>g.height){
                    b.child2=f;
                    a.child1=g;
                    g.parent=a;
                    a.aabb.setCombine(c.aabb,g.aabb);
                    b.aabb.setCombine(a.aabb,f.aabb);
                    a.height=1+({
                        var x=c.height;
                        var y=g.height;
                        x>y?x:y;
                    });
                    b.height=1+({
                        var x=a.height;
                        var y=f.height;
                        x>y?x:y;
                    });
                };
                else{
                    b.child2=g;
                    a.child1=f;
                    f.parent=a;
                    a.aabb.setCombine(c.aabb,f.aabb);
                    b.aabb.setCombine(a.aabb,g.aabb);
                    a.height=1+({
                        var x=c.height;
                        var y=f.height;
                        x>y?x:y;
                    });
                    b.height=1+({
                        var x=a.height;
                        var y=g.height;
                        x>y?x:y;
                    });
                };
                return b;
            };
            else return a;
        }
    }
}
#if nape_swc@:keep #end
class ZPP_DynAABBPhase extends ZPP_Broadphase{
    #if NAPE_NO_INLINE#else inline #end
    static var FATTEN=3.0;
    #if NAPE_NO_INLINE#else inline #end
    static var VEL_STEPS=2.0;
    public var stree:ZPP_AABBTree=null;
    public var dtree:ZPP_AABBTree=null;
    public var pairs:ZPP_AABBPair=null;
    public var syncs:ZPP_AABBNode=null;
    public var moves:ZPP_AABBNode=null;
    public function new(space:ZPP_Space){
        this.space=space;
        is_sweep=false;
        dynab=this;
        stree=new ZPP_AABBTree();
        dtree=new ZPP_AABBTree();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function dyn(shape:ZPP_Shape){
        if(shape.body.isStatic())return false;
        else return!shape.body.component.sleeping;
    }
    public function __insert(shape:ZPP_Shape){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                shape.node==null;
            };
            if(!res)throw "assert("+"shape.node==null"+") :: "+("shape has been inserted already");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                shape.body!=null;
            };
            if(!res)throw "assert("+"shape.body!=null"+") :: "+("shape inserted with no body");
            #end
        };
        var node;
        {
            if(ZPP_AABBNode.zpp_pool==null){
                node=new ZPP_AABBNode();
                #if NAPE_POOL_STATS ZPP_AABBNode.POOL_TOT++;
                ZPP_AABBNode.POOL_ADDNEW++;
                #end
            }
            else{
                node=ZPP_AABBNode.zpp_pool;
                ZPP_AABBNode.zpp_pool=node.next;
                node.next=null;
                #if NAPE_POOL_STATS ZPP_AABBNode.POOL_CNT--;
                ZPP_AABBNode.POOL_ADD++;
                #end
            }
            node.alloc();
        };
        node.shape=shape;
        shape.node=node;
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !node.synced;
            };
            if(!res)throw "assert("+"!node.synced"+") :: "+("node is already in sync list?");
            #end
        };
        node.synced=true;
        node.first_sync=true;
        {
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    node.snext==null;
                };
                if(!res)throw "assert("+"node.snext==null"+") :: "+("object already in list");
                #end
            };
            node.snext=syncs;
            syncs=node;
        };
    }
    public function __remove(shape:ZPP_Shape){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                shape.body!=null;
            };
            if(!res)throw "assert("+"shape.body!=null"+") :: "+("do i need this assertion?");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                shape.node!=null;
            };
            if(!res)throw "assert("+"shape.node!=null"+") :: "+("shape has no node?");
            #end
        };
        var node=shape.node;
        if(!node.first_sync){
            if(node.dyn)dtree.removeLeaf(node);
            else stree.removeLeaf(node);
        }
        shape.node=null;
        if(node.synced){
            {
                var pre=null;
                var cur=syncs;
                while(cur!=null){
                    if(cur==node)break;
                    pre=cur;
                    cur=cur.snext;
                }
                {
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            cur!=null;
                        };
                        if(!res)throw "assert("+"cur!=null"+") :: "+("cur = null");
                        #end
                    };
                    if(pre==null)syncs=cur.snext;
                    else pre.snext=cur.snext;
                    cur.snext=null;
                };
            };
            node.synced=false;
        }
        if(node.moved){
            {
                var pre=null;
                var cur=moves;
                while(cur!=null){
                    if(cur==node)break;
                    pre=cur;
                    cur=cur.mnext;
                }
                {
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            cur!=null;
                        };
                        if(!res)throw "assert("+"cur!=null"+") :: "+("cur = null");
                        #end
                    };
                    if(pre==null)moves=cur.mnext;
                    else pre.mnext=cur.mnext;
                    cur.mnext=null;
                };
            };
            node.moved=false;
        }
        var pre=null;
        var cur=pairs;
        while(cur!=null){
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !cur.sleeping;
                };
                if(!res)throw "assert("+"!cur.sleeping"+") :: "+("sleeping pair in pairs");
                #end
            };
            var nxt=cur.next;
            if(cur.n1==node||cur.n2==node){
                if(pre==null)pairs=nxt;
                else pre.next=nxt;
                if(cur.arb!=null)cur.arb.pair=null;
                cur.arb=null;
                cur.n1.shape.pairs.remove(cur);
                cur.n2.shape.pairs.remove(cur);
                {
                    var o=cur;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            o!=null;
                        };
                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_AABBPair"+", in obj: "+"cur"+")");
                        #end
                    };
                    o.free();
                    o.next=ZPP_AABBPair.zpp_pool;
                    ZPP_AABBPair.zpp_pool=o;
                    #if NAPE_POOL_STATS ZPP_AABBPair.POOL_CNT++;
                    ZPP_AABBPair.POOL_SUB++;
                    #end
                };
                cur=nxt;
                continue;
            }
            pre=cur;
            cur=nxt;
        }
        while(!shape.pairs.empty()){
            var cur=shape.pairs.pop_unsafe();
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    cur.sleeping;
                };
                if(!res)throw "assert("+"cur.sleeping"+") :: "+("non-sleeping pair remaining after removing them from pairs");
                #end
            };
            if(cur.n1==node)cur.n2.shape.pairs.remove(cur);
            else cur.n1.shape.pairs.remove(cur);
            if(cur.arb!=null)cur.arb.pair=null;
            cur.arb=null;
            {
                var o=cur;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_AABBPair"+", in obj: "+"cur"+")");
                    #end
                };
                o.free();
                o.next=ZPP_AABBPair.zpp_pool;
                ZPP_AABBPair.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_AABBPair.POOL_CNT++;
                ZPP_AABBPair.POOL_SUB++;
                #end
            };
        }
        {
            var o=node;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_AABBNode"+", in obj: "+"node"+")");
                #end
            };
            o.free();
            o.next=ZPP_AABBNode.zpp_pool;
            ZPP_AABBNode.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_AABBNode.POOL_CNT++;
            ZPP_AABBNode.POOL_SUB++;
            #end
        };
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function __sync(shape:ZPP_Shape){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !shape.body.isStatic();
            };
            if(!res)throw "assert("+"!shape.body.isStatic()"+") :: "+("static shape being synced?");
            #end
        };
        var node=shape.node;
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                node!=null;
            };
            if(!res)throw "assert("+"node!=null"+") :: "+("shape has no node?");
            #end
        };
        if(!node.synced){
            if(!space.continuous)shape.validate_aabb();
            var sync=node.dyn!=dyn(shape)||!node.aabb.contains(shape.aabb);
            if(sync){
                #if NAPE_TIMES Debug.BROADCLASH++;
                #end
                node.synced=true;
                {
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            node.snext==null;
                        };
                        if(!res)throw "assert("+"node.snext==null"+") :: "+("object already in list");
                        #end
                    };
                    node.snext=syncs;
                    syncs=node;
                };
            }
            #if NAPE_TIMES Debug.BROADTOTAL++;
            #end
        }
    }
    public function sync_broadphase(){
        space.validation();
        if(syncs!=null){
            if(moves==null){
                var node=syncs;
                while(node!=null){
                    {
                        var shape=node.shape;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                node.synced;
                            };
                            if(!res)throw "assert("+"node.synced"+") :: "+("node doesn't need syncing?");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                shape.body!=null;
                            };
                            if(!res)throw "assert("+"shape.body!=null"+") :: "+("shape with no body can't be synced");
                            #end
                        };
                        if(!node.first_sync){
                            var tree=if(node.dyn)dtree else stree;
                            tree.inlined_removeLeaf(node);
                        }
                        else node.first_sync=false;
                        var aabb=node.aabb;
                        if(!space.continuous)shape.validate_aabb();
                        aabb.setExpand(shape.aabb,FATTEN);
                        var tree=if(node.dyn=dyn(shape))dtree else stree;
                        tree.inlined_insertLeaf(node);
                        node.synced=false;
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !node.moved;
                        };
                        if(!res)throw "assert("+"!node.moved"+") :: "+("node already moved apparently?");
                        #end
                    };
                    node.moved=true;
                    node.mnext=node.snext;
                    node.snext=null;
                    node=node.mnext;
                }
                {
                    var t=syncs;
                    syncs=moves;
                    moves=t;
                }
            }
            else{
                while(syncs!=null){
                    var node=({
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                syncs!=null;
                            };
                            if(!res)throw "assert("+"syncs!=null"+") :: "+("pop from empty list!");
                            #end
                        };
                        var ret=syncs;
                        syncs=ret.snext;
                        ret.snext=null;
                        ret;
                    });
                    {
                        var shape=node.shape;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                node.synced;
                            };
                            if(!res)throw "assert("+"node.synced"+") :: "+("node doesn't need syncing?");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                shape.body!=null;
                            };
                            if(!res)throw "assert("+"shape.body!=null"+") :: "+("shape with no body can't be synced");
                            #end
                        };
                        if(!node.first_sync){
                            var tree=if(node.dyn)dtree else stree;
                            tree.inlined_removeLeaf(node);
                        }
                        else node.first_sync=false;
                        var aabb=node.aabb;
                        if(!space.continuous)shape.validate_aabb();
                        aabb.setExpand(shape.aabb,FATTEN);
                        var tree=if(node.dyn=dyn(shape))dtree else stree;
                        tree.inlined_insertLeaf(node);
                        node.synced=false;
                    };
                    if(!node.moved){
                        node.moved=true;
                        {
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    node.mnext==null;
                                };
                                if(!res)throw "assert("+"node.mnext==null"+") :: "+("object already in list");
                                #end
                            };
                            node.mnext=moves;
                            moves=node;
                        };
                    }
                }
            }
        }
    }
    public override function broadphase(space:ZPP_Space,discrete:Bool){
        var node=syncs;
        while(node!=null){
            {
                var shape=node.shape;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        node.synced;
                    };
                    if(!res)throw "assert("+"node.synced"+") :: "+("node doesn't need syncing?");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        shape.body!=null;
                    };
                    if(!res)throw "assert("+"shape.body!=null"+") :: "+("shape with no body can't be synced");
                    #end
                };
                if(!node.first_sync){
                    var tree=if(node.dyn)dtree else stree;
                    tree.inlined_removeLeaf(node);
                }
                else node.first_sync=false;
                var aabb=node.aabb;
                if(!space.continuous)shape.validate_aabb();
                aabb.setExpand(shape.aabb,FATTEN);
                var tree=if(node.dyn=dyn(shape))dtree else stree;
                tree.inlined_insertLeaf(node);
                node.synced=false;
            };
            node=node.snext;
        }
        {
            while(syncs!=null){
                var leaf=({
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            syncs!=null;
                        };
                        if(!res)throw "assert("+"syncs!=null"+") :: "+("pop from empty list!");
                        #end
                    };
                    var ret=syncs;
                    syncs=ret.snext;
                    ret.snext=null;
                    ret;
                });
                if("syncs"!="moves"&&leaf.moved)continue;
                leaf.moved=false;
                var lshape=leaf.shape;
                var lbody=lshape.body;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !(lbody.component.sleeping&&lbody.isStatic());
                    };
                    if(!res)throw "assert("+"!(lbody.component.sleeping&&lbody.isStatic())"+") :: "+("a sleeping static?");
                    #end
                };
                if(lbody.component.sleeping)continue;
                var ab=leaf.aabb;
                var stack=null;
                {
                    if(dtree.root!=null){
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                dtree.root.next==null;
                            };
                            if(!res)throw "assert("+"dtree.root.next==null"+") :: "+("object already in list");
                            #end
                        };
                        dtree.root.next=stack;
                        stack=dtree.root;
                    };
                    while(stack!=null){
                        var node=({
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    stack!=null;
                                };
                                if(!res)throw "assert("+"stack!=null"+") :: "+("pop from empty list!");
                                #end
                            };
                            var ret=stack;
                            stack=ret.next;
                            ret.next=null;
                            ret;
                        });
                        if(node==leaf)continue;
                        if(node.isLeaf()){
                            var shape=node.shape;
                            if(shape.body!=lshape.body&&!(shape.body.isStatic()&&lshape.body.isStatic())){
                                if(ab.intersect(node.aabb)){
                                    var id:Int;
                                    var di:Int;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            lshape!=shape;
                                        };
                                        if(!res)throw "assert("+"lshape!=shape"+") :: "+("narrowphase area of dyn-aabb with same shape?");
                                        #end
                                    };
                                    if(lshape.id<shape.id){
                                        id=lshape.id;
                                        di=shape.id;
                                    }
                                    else{
                                        id=shape.id;
                                        di=lshape.id;
                                    }
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            id<di;
                                        };
                                        if(!res)throw "assert("+"id<di"+") :: "+("id's not well ordered in dyn-aabb narrowphase");
                                        #end
                                    };
                                    var s=if(lshape.pairs.length<shape.pairs.length)lshape else shape;
                                    var p:ZPP_AABBPair=null;
                                    {
                                        var cx_ite=s.pairs.begin();
                                        while(cx_ite!=null){
                                            var px=cx_ite.elem();
                                            {
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        !(px.id==di&&px.di==id);
                                                    };
                                                    if(!res)throw "assert("+"!(px.id==di&&px.di==id)"+") :: "+("dyn-pair id didn't match shape id's");
                                                    #end
                                                };
                                                if(px.id==id&&px.di==di){
                                                    p=px;
                                                    break;
                                                }
                                            };
                                            cx_ite=cx_ite.next;
                                        }
                                    };
                                    if(p!=null){
                                        if(p.sleeping){
                                            p.sleeping=false;
                                            p.next=pairs;
                                            pairs=p;
                                            p.first=true;
                                        }
                                        continue;
                                    }
                                    {
                                        if(ZPP_AABBPair.zpp_pool==null){
                                            p=new ZPP_AABBPair();
                                            #if NAPE_POOL_STATS ZPP_AABBPair.POOL_TOT++;
                                            ZPP_AABBPair.POOL_ADDNEW++;
                                            #end
                                        }
                                        else{
                                            p=ZPP_AABBPair.zpp_pool;
                                            ZPP_AABBPair.zpp_pool=p.next;
                                            p.next=null;
                                            #if NAPE_POOL_STATS ZPP_AABBPair.POOL_CNT--;
                                            ZPP_AABBPair.POOL_ADD++;
                                            #end
                                        }
                                        p.alloc();
                                    };
                                    p.n1=leaf;
                                    p.n2=node;
                                    p.id=id;
                                    p.di=di;
                                    p.next=pairs;
                                    pairs=p;
                                    p.first=true;
                                    lshape.pairs.inlined_add(p);
                                    shape.pairs.inlined_add(p);
                                }
                            }
                        }
                        else if(ab.intersect(node.aabb)){
                            if(node.child1!=null){
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        node.child1.next==null;
                                    };
                                    if(!res)throw "assert("+"node.child1.next==null"+") :: "+("object already in list");
                                    #end
                                };
                                node.child1.next=stack;
                                stack=node.child1;
                            };
                            if(node.child2!=null){
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        node.child2.next==null;
                                    };
                                    if(!res)throw "assert("+"node.child2.next==null"+") :: "+("object already in list");
                                    #end
                                };
                                node.child2.next=stack;
                                stack=node.child2;
                            };
                        }
                    }
                };
                {
                    if(stree.root!=null){
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                stree.root.next==null;
                            };
                            if(!res)throw "assert("+"stree.root.next==null"+") :: "+("object already in list");
                            #end
                        };
                        stree.root.next=stack;
                        stack=stree.root;
                    };
                    while(stack!=null){
                        var node=({
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    stack!=null;
                                };
                                if(!res)throw "assert("+"stack!=null"+") :: "+("pop from empty list!");
                                #end
                            };
                            var ret=stack;
                            stack=ret.next;
                            ret.next=null;
                            ret;
                        });
                        if(node==leaf)continue;
                        if(node.isLeaf()){
                            var shape=node.shape;
                            if(shape.body!=lshape.body&&!(shape.body.isStatic()&&lshape.body.isStatic())){
                                if(ab.intersect(node.aabb)){
                                    var id:Int;
                                    var di:Int;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            lshape!=shape;
                                        };
                                        if(!res)throw "assert("+"lshape!=shape"+") :: "+("narrowphase area of dyn-aabb with same shape?");
                                        #end
                                    };
                                    if(lshape.id<shape.id){
                                        id=lshape.id;
                                        di=shape.id;
                                    }
                                    else{
                                        id=shape.id;
                                        di=lshape.id;
                                    }
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            id<di;
                                        };
                                        if(!res)throw "assert("+"id<di"+") :: "+("id's not well ordered in dyn-aabb narrowphase");
                                        #end
                                    };
                                    var s=if(lshape.pairs.length<shape.pairs.length)lshape else shape;
                                    var p:ZPP_AABBPair=null;
                                    {
                                        var cx_ite=s.pairs.begin();
                                        while(cx_ite!=null){
                                            var px=cx_ite.elem();
                                            {
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        !(px.id==di&&px.di==id);
                                                    };
                                                    if(!res)throw "assert("+"!(px.id==di&&px.di==id)"+") :: "+("dyn-pair id didn't match shape id's");
                                                    #end
                                                };
                                                if(px.id==id&&px.di==di){
                                                    p=px;
                                                    break;
                                                }
                                            };
                                            cx_ite=cx_ite.next;
                                        }
                                    };
                                    if(p!=null){
                                        if(p.sleeping){
                                            p.sleeping=false;
                                            p.next=pairs;
                                            pairs=p;
                                            p.first=true;
                                        }
                                        continue;
                                    }
                                    {
                                        if(ZPP_AABBPair.zpp_pool==null){
                                            p=new ZPP_AABBPair();
                                            #if NAPE_POOL_STATS ZPP_AABBPair.POOL_TOT++;
                                            ZPP_AABBPair.POOL_ADDNEW++;
                                            #end
                                        }
                                        else{
                                            p=ZPP_AABBPair.zpp_pool;
                                            ZPP_AABBPair.zpp_pool=p.next;
                                            p.next=null;
                                            #if NAPE_POOL_STATS ZPP_AABBPair.POOL_CNT--;
                                            ZPP_AABBPair.POOL_ADD++;
                                            #end
                                        }
                                        p.alloc();
                                    };
                                    p.n1=leaf;
                                    p.n2=node;
                                    p.id=id;
                                    p.di=di;
                                    p.next=pairs;
                                    pairs=p;
                                    p.first=true;
                                    lshape.pairs.inlined_add(p);
                                    shape.pairs.inlined_add(p);
                                }
                            }
                        }
                        else if(ab.intersect(node.aabb)){
                            if(node.child1!=null){
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        node.child1.next==null;
                                    };
                                    if(!res)throw "assert("+"node.child1.next==null"+") :: "+("object already in list");
                                    #end
                                };
                                node.child1.next=stack;
                                stack=node.child1;
                            };
                            if(node.child2!=null){
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        node.child2.next==null;
                                    };
                                    if(!res)throw "assert("+"node.child2.next==null"+") :: "+("object already in list");
                                    #end
                                };
                                node.child2.next=stack;
                                stack=node.child2;
                            };
                        }
                    }
                };
            }
        }
        {
            while(moves!=null){
                var leaf=({
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            moves!=null;
                        };
                        if(!res)throw "assert("+"moves!=null"+") :: "+("pop from empty list!");
                        #end
                    };
                    var ret=moves;
                    moves=ret.mnext;
                    ret.mnext=null;
                    ret;
                });
                if("moves"!="moves"&&leaf.moved)continue;
                leaf.moved=false;
                var lshape=leaf.shape;
                var lbody=lshape.body;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !(lbody.component.sleeping&&lbody.isStatic());
                    };
                    if(!res)throw "assert("+"!(lbody.component.sleeping&&lbody.isStatic())"+") :: "+("a sleeping static?");
                    #end
                };
                if(lbody.component.sleeping)continue;
                var ab=leaf.aabb;
                var stack=null;
                {
                    if(dtree.root!=null){
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                dtree.root.next==null;
                            };
                            if(!res)throw "assert("+"dtree.root.next==null"+") :: "+("object already in list");
                            #end
                        };
                        dtree.root.next=stack;
                        stack=dtree.root;
                    };
                    while(stack!=null){
                        var node=({
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    stack!=null;
                                };
                                if(!res)throw "assert("+"stack!=null"+") :: "+("pop from empty list!");
                                #end
                            };
                            var ret=stack;
                            stack=ret.next;
                            ret.next=null;
                            ret;
                        });
                        if(node==leaf)continue;
                        if(node.isLeaf()){
                            var shape=node.shape;
                            if(shape.body!=lshape.body&&!(shape.body.isStatic()&&lshape.body.isStatic())){
                                if(ab.intersect(node.aabb)){
                                    var id:Int;
                                    var di:Int;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            lshape!=shape;
                                        };
                                        if(!res)throw "assert("+"lshape!=shape"+") :: "+("narrowphase area of dyn-aabb with same shape?");
                                        #end
                                    };
                                    if(lshape.id<shape.id){
                                        id=lshape.id;
                                        di=shape.id;
                                    }
                                    else{
                                        id=shape.id;
                                        di=lshape.id;
                                    }
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            id<di;
                                        };
                                        if(!res)throw "assert("+"id<di"+") :: "+("id's not well ordered in dyn-aabb narrowphase");
                                        #end
                                    };
                                    var s=if(lshape.pairs.length<shape.pairs.length)lshape else shape;
                                    var p:ZPP_AABBPair=null;
                                    {
                                        var cx_ite=s.pairs.begin();
                                        while(cx_ite!=null){
                                            var px=cx_ite.elem();
                                            {
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        !(px.id==di&&px.di==id);
                                                    };
                                                    if(!res)throw "assert("+"!(px.id==di&&px.di==id)"+") :: "+("dyn-pair id didn't match shape id's");
                                                    #end
                                                };
                                                if(px.id==id&&px.di==di){
                                                    p=px;
                                                    break;
                                                }
                                            };
                                            cx_ite=cx_ite.next;
                                        }
                                    };
                                    if(p!=null){
                                        if(p.sleeping){
                                            p.sleeping=false;
                                            p.next=pairs;
                                            pairs=p;
                                            p.first=true;
                                        }
                                        continue;
                                    }
                                    {
                                        if(ZPP_AABBPair.zpp_pool==null){
                                            p=new ZPP_AABBPair();
                                            #if NAPE_POOL_STATS ZPP_AABBPair.POOL_TOT++;
                                            ZPP_AABBPair.POOL_ADDNEW++;
                                            #end
                                        }
                                        else{
                                            p=ZPP_AABBPair.zpp_pool;
                                            ZPP_AABBPair.zpp_pool=p.next;
                                            p.next=null;
                                            #if NAPE_POOL_STATS ZPP_AABBPair.POOL_CNT--;
                                            ZPP_AABBPair.POOL_ADD++;
                                            #end
                                        }
                                        p.alloc();
                                    };
                                    p.n1=leaf;
                                    p.n2=node;
                                    p.id=id;
                                    p.di=di;
                                    p.next=pairs;
                                    pairs=p;
                                    p.first=true;
                                    lshape.pairs.inlined_add(p);
                                    shape.pairs.inlined_add(p);
                                }
                            }
                        }
                        else if(ab.intersect(node.aabb)){
                            if(node.child1!=null){
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        node.child1.next==null;
                                    };
                                    if(!res)throw "assert("+"node.child1.next==null"+") :: "+("object already in list");
                                    #end
                                };
                                node.child1.next=stack;
                                stack=node.child1;
                            };
                            if(node.child2!=null){
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        node.child2.next==null;
                                    };
                                    if(!res)throw "assert("+"node.child2.next==null"+") :: "+("object already in list");
                                    #end
                                };
                                node.child2.next=stack;
                                stack=node.child2;
                            };
                        }
                    }
                };
                {
                    if(stree.root!=null){
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                stree.root.next==null;
                            };
                            if(!res)throw "assert("+"stree.root.next==null"+") :: "+("object already in list");
                            #end
                        };
                        stree.root.next=stack;
                        stack=stree.root;
                    };
                    while(stack!=null){
                        var node=({
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    stack!=null;
                                };
                                if(!res)throw "assert("+"stack!=null"+") :: "+("pop from empty list!");
                                #end
                            };
                            var ret=stack;
                            stack=ret.next;
                            ret.next=null;
                            ret;
                        });
                        if(node==leaf)continue;
                        if(node.isLeaf()){
                            var shape=node.shape;
                            if(shape.body!=lshape.body&&!(shape.body.isStatic()&&lshape.body.isStatic())){
                                if(ab.intersect(node.aabb)){
                                    var id:Int;
                                    var di:Int;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            lshape!=shape;
                                        };
                                        if(!res)throw "assert("+"lshape!=shape"+") :: "+("narrowphase area of dyn-aabb with same shape?");
                                        #end
                                    };
                                    if(lshape.id<shape.id){
                                        id=lshape.id;
                                        di=shape.id;
                                    }
                                    else{
                                        id=shape.id;
                                        di=lshape.id;
                                    }
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            id<di;
                                        };
                                        if(!res)throw "assert("+"id<di"+") :: "+("id's not well ordered in dyn-aabb narrowphase");
                                        #end
                                    };
                                    var s=if(lshape.pairs.length<shape.pairs.length)lshape else shape;
                                    var p:ZPP_AABBPair=null;
                                    {
                                        var cx_ite=s.pairs.begin();
                                        while(cx_ite!=null){
                                            var px=cx_ite.elem();
                                            {
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        !(px.id==di&&px.di==id);
                                                    };
                                                    if(!res)throw "assert("+"!(px.id==di&&px.di==id)"+") :: "+("dyn-pair id didn't match shape id's");
                                                    #end
                                                };
                                                if(px.id==id&&px.di==di){
                                                    p=px;
                                                    break;
                                                }
                                            };
                                            cx_ite=cx_ite.next;
                                        }
                                    };
                                    if(p!=null){
                                        if(p.sleeping){
                                            p.sleeping=false;
                                            p.next=pairs;
                                            pairs=p;
                                            p.first=true;
                                        }
                                        continue;
                                    }
                                    {
                                        if(ZPP_AABBPair.zpp_pool==null){
                                            p=new ZPP_AABBPair();
                                            #if NAPE_POOL_STATS ZPP_AABBPair.POOL_TOT++;
                                            ZPP_AABBPair.POOL_ADDNEW++;
                                            #end
                                        }
                                        else{
                                            p=ZPP_AABBPair.zpp_pool;
                                            ZPP_AABBPair.zpp_pool=p.next;
                                            p.next=null;
                                            #if NAPE_POOL_STATS ZPP_AABBPair.POOL_CNT--;
                                            ZPP_AABBPair.POOL_ADD++;
                                            #end
                                        }
                                        p.alloc();
                                    };
                                    p.n1=leaf;
                                    p.n2=node;
                                    p.id=id;
                                    p.di=di;
                                    p.next=pairs;
                                    pairs=p;
                                    p.first=true;
                                    lshape.pairs.inlined_add(p);
                                    shape.pairs.inlined_add(p);
                                }
                            }
                        }
                        else if(ab.intersect(node.aabb)){
                            if(node.child1!=null){
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        node.child1.next==null;
                                    };
                                    if(!res)throw "assert("+"node.child1.next==null"+") :: "+("object already in list");
                                    #end
                                };
                                node.child1.next=stack;
                                stack=node.child1;
                            };
                            if(node.child2!=null){
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        node.child2.next==null;
                                    };
                                    if(!res)throw "assert("+"node.child2.next==null"+") :: "+("object already in list");
                                    #end
                                };
                                node.child2.next=stack;
                                stack=node.child2;
                            };
                        }
                    }
                };
            }
        }
        var pre:ZPP_AABBPair=null;
        var cur=pairs;
        while(cur!=null){
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !cur.sleeping;
                };
                if(!res)throw "assert("+"!cur.sleeping"+") :: "+("non-sleeping pair in pairs");
                #end
            };
            if(!cur.first&&!cur.n1.aabb.intersect(cur.n2.aabb)){
                if(pre==null)pairs=cur.next;
                else pre.next=cur.next;
                cur.n1.shape.pairs.inlined_remove(cur);
                cur.n2.shape.pairs.inlined_remove(cur);
                var nxt=cur.next;
                if(cur.arb!=null)cur.arb.pair=null;
                cur.arb=null;
                {
                    var o=cur;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            o!=null;
                        };
                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_AABBPair"+", in obj: "+"cur"+")");
                        #end
                    };
                    o.free();
                    o.next=ZPP_AABBPair.zpp_pool;
                    ZPP_AABBPair.zpp_pool=o;
                    #if NAPE_POOL_STATS ZPP_AABBPair.POOL_CNT++;
                    ZPP_AABBPair.POOL_SUB++;
                    #end
                };
                cur=nxt;
                continue;
            }
            var s1=cur.n1.shape;
            var b1=s1.body;
            var s2=cur.n2.shape;
            var b2=s2.body;
            if(!cur.first){
                if((b1.component.sleeping||b1.isStatic())&&(b2.component.sleeping||b2.isStatic())){
                    cur.sleeping=true;
                    if(pre==null)pairs=cur.next;
                    else pre.next=cur.next;
                    cur=cur.next;
                    continue;
                }
            }
            cur.first=false;
            if(s1.aabb.intersect(s2.aabb)){
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        cur.arb==null||cur.arb.pair==cur;
                    };
                    if(!res)throw "assert("+"cur.arb==null||cur.arb.pair==cur"+") :: "+("arbiter/pair don't match up");
                    #end
                };
                var oarb=cur.arb;
                if(discrete)cur.arb=space.narrowPhase(s1,s2,!b1.isDynamic()||!b2.isDynamic(),cur.arb,false);
                else cur.arb=space.continuousEvent(s1,s2,!b1.isDynamic()||!b2.isDynamic(),cur.arb,false);
                if(cur.arb==null){
                    if(oarb!=null)oarb.pair=null;
                }
                else cur.arb.pair=cur;
                #if NAPE_ASSERT if(cur.arb!=oarb){
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        oarb==null||oarb.pair==null;
                    };
                    if(!res)throw "assert("+"oarb==null||oarb.pair==null"+") :: "+("oh deary me");
                    #end
                };
                #end
            }
            pre=cur;
            cur=cur.next;
        }
    }
    public override function clear(){
        while(syncs!=null){
            var next=syncs.snext;
            syncs.snext=null;
            if(syncs.first_sync){
                syncs.shape.node=null;
                syncs.shape.removedFromSpace();
                syncs.shape=null;
            }
            syncs=next;
        }
        while(moves!=null){
            var next=moves.mnext;
            moves.mnext=null;
            if(moves.first_sync){
                moves.shape.node=null;
                moves.shape.removedFromSpace();
                moves.shape=null;
            }
            moves=next;
        }
        while(pairs!=null){
            var nxt=pairs.next;
            if(pairs.arb!=null)pairs.arb.pair=null;
            pairs.arb=null;
            pairs.n1.shape.pairs.inlined_remove(pairs);
            pairs.n2.shape.pairs.inlined_remove(pairs);
            {
                var o=pairs;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_AABBPair"+", in obj: "+"pairs"+")");
                    #end
                };
                o.free();
                o.next=ZPP_AABBPair.zpp_pool;
                ZPP_AABBPair.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_AABBPair.POOL_CNT++;
                ZPP_AABBPair.POOL_SUB++;
                #end
            };
            pairs=nxt;
        }
        dtree.clear();
        stree.clear();
    }
    var treeStack:ZNPList_ZPP_AABBNode=null;
    public override function shapesUnderPoint(x:Float,y:Float,filter:ZPP_InteractionFilter,output:ShapeList){
        sync_broadphase();
        var v=ZPP_Vec2.get(x,y);
        var ret=(output==null?new ShapeList():output);
        {
            {
                if(stree.root!=null){
                    if(treeStack==null)treeStack=new ZNPList_ZPP_AABBNode();
                    treeStack.add(stree.root);
                    while(!treeStack.empty()){
                        var node=treeStack.pop_unsafe();
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                node!=null;
                            };
                            if(!res)throw "assert("+"node!=null"+") :: "+("null node");
                            #end
                        };
                        if(node.aabb.containsPoint(v)){
                            if(node.isLeaf()){
                                {
                                    if(filter==null||node.shape.filter.shouldCollide(filter)){
                                        if(node.shape.isCircle()){
                                            if(ZPP_Collide.circleContains(node.shape.circle,v))ret.push(node.shape.outer);
                                        }
                                        else{
                                            if(ZPP_Collide.polyContains(node.shape.polygon,v))ret.push(node.shape.outer);
                                        }
                                    }
                                };
                            }
                            else{
                                if(node.child1!=null)treeStack.add(node.child1);
                                if(node.child2!=null)treeStack.add(node.child2);
                            }
                        }
                    }
                }
            }
        }
        {
            {
                if(dtree.root!=null){
                    if(treeStack==null)treeStack=new ZNPList_ZPP_AABBNode();
                    treeStack.add(dtree.root);
                    while(!treeStack.empty()){
                        var node=treeStack.pop_unsafe();
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                node!=null;
                            };
                            if(!res)throw "assert("+"node!=null"+") :: "+("null node");
                            #end
                        };
                        if(node.aabb.containsPoint(v)){
                            if(node.isLeaf()){
                                {
                                    if(filter==null||node.shape.filter.shouldCollide(filter)){
                                        if(node.shape.isCircle()){
                                            if(ZPP_Collide.circleContains(node.shape.circle,v))ret.push(node.shape.outer);
                                        }
                                        else{
                                            if(ZPP_Collide.polyContains(node.shape.polygon,v))ret.push(node.shape.outer);
                                        }
                                    }
                                };
                            }
                            else{
                                if(node.child1!=null)treeStack.add(node.child1);
                                if(node.child2!=null)treeStack.add(node.child2);
                            }
                        }
                    }
                }
            }
        }
        {
            var o=v;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Vec2"+", in obj: "+"v"+")");
                #end
            };
            o.free();
            o.next=ZPP_Vec2.zpp_pool;
            ZPP_Vec2.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_Vec2.POOL_CNT++;
            ZPP_Vec2.POOL_SUB++;
            #end
        };
        return ret;
    }
    public override function bodiesUnderPoint(x:Float,y:Float,filter:ZPP_InteractionFilter,output:BodyList){
        sync_broadphase();
        var v=ZPP_Vec2.get(x,y);
        var ret=(output==null?new BodyList():output);
        {
            {
                if(stree.root!=null){
                    if(treeStack==null)treeStack=new ZNPList_ZPP_AABBNode();
                    treeStack.add(stree.root);
                    while(!treeStack.empty()){
                        var node=treeStack.pop_unsafe();
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                node!=null;
                            };
                            if(!res)throw "assert("+"node!=null"+") :: "+("null node");
                            #end
                        };
                        if(node.aabb.containsPoint(v)){
                            if(node.isLeaf()){
                                {
                                    var body=node.shape.body.outer;
                                    if(!ret.has(body)){
                                        if(filter==null||node.shape.filter.shouldCollide(filter)){
                                            if(node.shape.isCircle()){
                                                if(ZPP_Collide.circleContains(node.shape.circle,v))ret.push(body);
                                            }
                                            else{
                                                if(ZPP_Collide.polyContains(node.shape.polygon,v))ret.push(body);
                                            }
                                        }
                                    }
                                };
                            }
                            else{
                                if(node.child1!=null)treeStack.add(node.child1);
                                if(node.child2!=null)treeStack.add(node.child2);
                            }
                        }
                    }
                }
            }
        }
        {
            {
                if(dtree.root!=null){
                    if(treeStack==null)treeStack=new ZNPList_ZPP_AABBNode();
                    treeStack.add(dtree.root);
                    while(!treeStack.empty()){
                        var node=treeStack.pop_unsafe();
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                node!=null;
                            };
                            if(!res)throw "assert("+"node!=null"+") :: "+("null node");
                            #end
                        };
                        if(node.aabb.containsPoint(v)){
                            if(node.isLeaf()){
                                {
                                    var body=node.shape.body.outer;
                                    if(!ret.has(body)){
                                        if(filter==null||node.shape.filter.shouldCollide(filter)){
                                            if(node.shape.isCircle()){
                                                if(ZPP_Collide.circleContains(node.shape.circle,v))ret.push(body);
                                            }
                                            else{
                                                if(ZPP_Collide.polyContains(node.shape.polygon,v))ret.push(body);
                                            }
                                        }
                                    }
                                };
                            }
                            else{
                                if(node.child1!=null)treeStack.add(node.child1);
                                if(node.child2!=null)treeStack.add(node.child2);
                            }
                        }
                    }
                }
            }
        }
        {
            var o=v;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Vec2"+", in obj: "+"v"+")");
                #end
            };
            o.free();
            o.next=ZPP_Vec2.zpp_pool;
            ZPP_Vec2.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_Vec2.POOL_CNT++;
            ZPP_Vec2.POOL_SUB++;
            #end
        };
        return ret;
    }
    public var treeStack2:ZNPList_ZPP_AABBNode=null;
    public override function shapesInAABB(aabb:ZPP_AABB,strict:Bool,containment:Bool,filter:ZPP_InteractionFilter,output:ShapeList){
        sync_broadphase();
        updateAABBShape(aabb);
        var ab=aabbShape.zpp_inner.aabb;
        var ret=(output==null?new ShapeList():output);
        {
            {
                if(stree.root!=null){
                    if(treeStack==null)treeStack=new ZNPList_ZPP_AABBNode();
                    treeStack.add(stree.root);
                    while(!treeStack.empty()){
                        var node=treeStack.pop_unsafe();
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                node!=null;
                            };
                            if(!res)throw "assert("+"node!=null"+") :: "+("null node");
                            #end
                        };
                        if(ab.contains(node.aabb)){
                            if(node.isLeaf()){
                                {
                                    if(filter==null||node.shape.filter.shouldCollide(filter)){
                                        ret.push(node.shape.outer);
                                    }
                                };
                            }
                            else{
                                if(treeStack2==null){
                                    treeStack2=new ZNPList_ZPP_AABBNode();
                                }
                                treeStack2.add(node);
                                while(!treeStack2.empty()){
                                    var node=treeStack2.pop_unsafe();
                                    if(node.isLeaf()){
                                        if(filter==null||node.shape.filter.shouldCollide(filter)){
                                            ret.push(node.shape.outer);
                                        }
                                    };
                                    else{
                                        if(node.child1!=null)treeStack2.add(node.child1);
                                        if(node.child2!=null)treeStack2.add(node.child2);
                                    }
                                }
                            }
                        }
                        else if(node.aabb.intersect(ab)){
                            if(node.isLeaf()){
                                {
                                    if(filter==null||node.shape.filter.shouldCollide(filter)){
                                        if(strict){
                                            if(containment){
                                                if(ZPP_Collide.containTest(aabbShape.zpp_inner,node.shape))ret.push(node.shape.outer);
                                            }
                                            else{
                                                if(ab.contains(node.shape.aabb))ret.push(node.shape.outer);
                                                else if(ZPP_Collide.testCollide_safe(node.shape,aabbShape.zpp_inner))ret.push(node.shape.outer);
                                            }
                                        }
                                        else if(!containment||ab.contains(node.shape.aabb))ret.push(node.shape.outer);
                                    }
                                };
                            }
                            else{
                                if(node.child1!=null)treeStack.add(node.child1);
                                if(node.child2!=null)treeStack.add(node.child2);
                            }
                        }
                    }
                }
            }
        }
        {
            {
                if(dtree.root!=null){
                    if(treeStack==null)treeStack=new ZNPList_ZPP_AABBNode();
                    treeStack.add(dtree.root);
                    while(!treeStack.empty()){
                        var node=treeStack.pop_unsafe();
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                node!=null;
                            };
                            if(!res)throw "assert("+"node!=null"+") :: "+("null node");
                            #end
                        };
                        if(ab.contains(node.aabb)){
                            if(node.isLeaf()){
                                {
                                    if(filter==null||node.shape.filter.shouldCollide(filter)){
                                        ret.push(node.shape.outer);
                                    }
                                };
                            }
                            else{
                                if(treeStack2==null){
                                    treeStack2=new ZNPList_ZPP_AABBNode();
                                }
                                treeStack2.add(node);
                                while(!treeStack2.empty()){
                                    var node=treeStack2.pop_unsafe();
                                    if(node.isLeaf()){
                                        if(filter==null||node.shape.filter.shouldCollide(filter)){
                                            ret.push(node.shape.outer);
                                        }
                                    };
                                    else{
                                        if(node.child1!=null)treeStack2.add(node.child1);
                                        if(node.child2!=null)treeStack2.add(node.child2);
                                    }
                                }
                            }
                        }
                        else if(node.aabb.intersect(ab)){
                            if(node.isLeaf()){
                                {
                                    if(filter==null||node.shape.filter.shouldCollide(filter)){
                                        if(strict){
                                            if(containment){
                                                if(ZPP_Collide.containTest(aabbShape.zpp_inner,node.shape))ret.push(node.shape.outer);
                                            }
                                            else{
                                                if(ab.contains(node.shape.aabb))ret.push(node.shape.outer);
                                                else if(ZPP_Collide.testCollide_safe(node.shape,aabbShape.zpp_inner))ret.push(node.shape.outer);
                                            }
                                        }
                                        else if(!containment||ab.contains(node.shape.aabb))ret.push(node.shape.outer);
                                    }
                                };
                            }
                            else{
                                if(node.child1!=null)treeStack.add(node.child1);
                                if(node.child2!=null)treeStack.add(node.child2);
                            }
                        }
                    }
                }
            }
        }
        return ret;
    }
    public var failed:BodyList=null;
    public override function bodiesInAABB(aabb:ZPP_AABB,strict:Bool,containment:Bool,filter:ZPP_InteractionFilter,output:BodyList){
        sync_broadphase();
        updateAABBShape(aabb);
        var ab=aabbShape.zpp_inner.aabb;
        var ret=(output==null?new BodyList():output);
        if(failed==null)failed=new BodyList();
        {
            {
                if(stree.root!=null){
                    if(treeStack==null)treeStack=new ZNPList_ZPP_AABBNode();
                    treeStack.add(stree.root);
                    while(!treeStack.empty()){
                        var node=treeStack.pop_unsafe();
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                node!=null;
                            };
                            if(!res)throw "assert("+"node!=null"+") :: "+("null node");
                            #end
                        };
                        if(ab.contains(node.aabb)){
                            if(node.isLeaf()){
                                {
                                    if(filter==null||node.shape.filter.shouldCollide(filter)){
                                        var body=node.shape.body.outer;
                                        if(!ret.has(body))ret.push(body);
                                    }
                                };
                            }
                            else{
                                if(treeStack2==null){
                                    treeStack2=new ZNPList_ZPP_AABBNode();
                                }
                                treeStack2.add(node);
                                while(!treeStack2.empty()){
                                    var node=treeStack2.pop_unsafe();
                                    if(node.isLeaf()){
                                        if(filter==null||node.shape.filter.shouldCollide(filter)){
                                            var body=node.shape.body.outer;
                                            if(!ret.has(body))ret.push(body);
                                        }
                                    };
                                    else{
                                        if(node.child1!=null)treeStack2.add(node.child1);
                                        if(node.child2!=null)treeStack2.add(node.child2);
                                    }
                                }
                            }
                        }
                        else if(node.aabb.intersect(ab)){
                            if(node.isLeaf()){
                                {
                                    var body=node.shape.body.outer;
                                    if(filter==null||node.shape.filter.shouldCollide(filter)){
                                        if(strict){
                                            if(containment){
                                                if(!failed.has(body)){
                                                    var col=ZPP_Collide.containTest(aabbShape.zpp_inner,node.shape);
                                                    if(!ret.has(body)&&col)ret.push(body);
                                                    else if(!col){
                                                        ret.remove(body);
                                                        failed.push(body);
                                                    }
                                                }
                                            }
                                            else if(!ret.has(body)&&ZPP_Collide.testCollide_safe(node.shape,aabbShape.zpp_inner)){
                                                ret.push(body);
                                            }
                                        }
                                        else{
                                            if(containment){
                                                if(!failed.has(body)){
                                                    var col=ab.contains(node.shape.aabb);
                                                    if(!ret.has(body)&&col)ret.push(body);
                                                    else if(!col){
                                                        ret.remove(body);
                                                        failed.push(body);
                                                    }
                                                }
                                            }
                                            else if(!ret.has(body)&&ab.contains(node.shape.aabb)){
                                                ret.push(body);
                                            }
                                        }
                                    }
                                };
                            }
                            else{
                                if(node.child1!=null)treeStack.add(node.child1);
                                if(node.child2!=null)treeStack.add(node.child2);
                            }
                        }
                    }
                }
            }
        }
        {
            {
                if(dtree.root!=null){
                    if(treeStack==null)treeStack=new ZNPList_ZPP_AABBNode();
                    treeStack.add(dtree.root);
                    while(!treeStack.empty()){
                        var node=treeStack.pop_unsafe();
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                node!=null;
                            };
                            if(!res)throw "assert("+"node!=null"+") :: "+("null node");
                            #end
                        };
                        if(ab.contains(node.aabb)){
                            if(node.isLeaf()){
                                {
                                    if(filter==null||node.shape.filter.shouldCollide(filter)){
                                        var body=node.shape.body.outer;
                                        if(!ret.has(body))ret.push(body);
                                    }
                                };
                            }
                            else{
                                if(treeStack2==null){
                                    treeStack2=new ZNPList_ZPP_AABBNode();
                                }
                                treeStack2.add(node);
                                while(!treeStack2.empty()){
                                    var node=treeStack2.pop_unsafe();
                                    if(node.isLeaf()){
                                        if(filter==null||node.shape.filter.shouldCollide(filter)){
                                            var body=node.shape.body.outer;
                                            if(!ret.has(body))ret.push(body);
                                        }
                                    };
                                    else{
                                        if(node.child1!=null)treeStack2.add(node.child1);
                                        if(node.child2!=null)treeStack2.add(node.child2);
                                    }
                                }
                            }
                        }
                        else if(node.aabb.intersect(ab)){
                            if(node.isLeaf()){
                                {
                                    var body=node.shape.body.outer;
                                    if(filter==null||node.shape.filter.shouldCollide(filter)){
                                        if(strict){
                                            if(containment){
                                                if(!failed.has(body)){
                                                    var col=ZPP_Collide.containTest(aabbShape.zpp_inner,node.shape);
                                                    if(!ret.has(body)&&col)ret.push(body);
                                                    else if(!col){
                                                        ret.remove(body);
                                                        failed.push(body);
                                                    }
                                                }
                                            }
                                            else if(!ret.has(body)&&ZPP_Collide.testCollide_safe(node.shape,aabbShape.zpp_inner)){
                                                ret.push(body);
                                            }
                                        }
                                        else{
                                            if(containment){
                                                if(!failed.has(body)){
                                                    var col=ab.contains(node.shape.aabb);
                                                    if(!ret.has(body)&&col)ret.push(body);
                                                    else if(!col){
                                                        ret.remove(body);
                                                        failed.push(body);
                                                    }
                                                }
                                            }
                                            else if(!ret.has(body)&&ab.contains(node.shape.aabb)){
                                                ret.push(body);
                                            }
                                        }
                                    }
                                };
                            }
                            else{
                                if(node.child1!=null)treeStack.add(node.child1);
                                if(node.child2!=null)treeStack.add(node.child2);
                            }
                        }
                    }
                }
            }
        }
        failed.clear();
        return ret;
    }
    public override function shapesInCircle(x:Float,y:Float,r:Float,containment:Bool,filter:ZPP_InteractionFilter,output:ShapeList){
        sync_broadphase();
        updateCircShape(x,y,r);
        var ab=circShape.zpp_inner.aabb;
        var ret=(output==null?new ShapeList():output);
        {
            {
                if(stree.root!=null){
                    if(treeStack==null)treeStack=new ZNPList_ZPP_AABBNode();
                    treeStack.add(stree.root);
                    while(!treeStack.empty()){
                        var node=treeStack.pop_unsafe();
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                node!=null;
                            };
                            if(!res)throw "assert("+"node!=null"+") :: "+("null node");
                            #end
                        };
                        if(node.aabb.intersect(ab)){
                            if(node.isLeaf()){
                                {
                                    if(filter==null||node.shape.filter.shouldCollide(filter)){
                                        if(containment){
                                            if(ZPP_Collide.containTest(circShape.zpp_inner,node.shape))ret.push(node.shape.outer);
                                        }
                                        else if(ZPP_Collide.testCollide_safe(node.shape,circShape.zpp_inner))ret.push(node.shape.outer);
                                    }
                                };
                            }
                            else{
                                if(node.child1!=null)treeStack.add(node.child1);
                                if(node.child2!=null)treeStack.add(node.child2);
                            }
                        }
                    }
                }
            }
        }
        {
            {
                if(dtree.root!=null){
                    if(treeStack==null)treeStack=new ZNPList_ZPP_AABBNode();
                    treeStack.add(dtree.root);
                    while(!treeStack.empty()){
                        var node=treeStack.pop_unsafe();
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                node!=null;
                            };
                            if(!res)throw "assert("+"node!=null"+") :: "+("null node");
                            #end
                        };
                        if(node.aabb.intersect(ab)){
                            if(node.isLeaf()){
                                {
                                    if(filter==null||node.shape.filter.shouldCollide(filter)){
                                        if(containment){
                                            if(ZPP_Collide.containTest(circShape.zpp_inner,node.shape))ret.push(node.shape.outer);
                                        }
                                        else if(ZPP_Collide.testCollide_safe(node.shape,circShape.zpp_inner))ret.push(node.shape.outer);
                                    }
                                };
                            }
                            else{
                                if(node.child1!=null)treeStack.add(node.child1);
                                if(node.child2!=null)treeStack.add(node.child2);
                            }
                        }
                    }
                }
            }
        }
        return ret;
    }
    public override function bodiesInCircle(x:Float,y:Float,r:Float,containment:Bool,filter:ZPP_InteractionFilter,output:BodyList){
        sync_broadphase();
        updateCircShape(x,y,r);
        var ab=circShape.zpp_inner.aabb;
        var ret=(output==null?new BodyList():output);
        if(failed==null)failed=new BodyList();
        {
            {
                if(stree.root!=null){
                    if(treeStack==null)treeStack=new ZNPList_ZPP_AABBNode();
                    treeStack.add(stree.root);
                    while(!treeStack.empty()){
                        var node=treeStack.pop_unsafe();
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                node!=null;
                            };
                            if(!res)throw "assert("+"node!=null"+") :: "+("null node");
                            #end
                        };
                        if(node.aabb.intersect(ab)){
                            if(node.isLeaf()){
                                {
                                    var body=node.shape.body.outer;
                                    if(filter==null||node.shape.filter.shouldCollide(filter)){
                                        if(containment){
                                            if(!failed.has(body)){
                                                var col=ZPP_Collide.containTest(circShape.zpp_inner,node.shape);
                                                if(!ret.has(body)&&col)ret.push(body);
                                                else if(!col){
                                                    ret.remove(body);
                                                    failed.push(body);
                                                }
                                            }
                                        }
                                        else if(!ret.has(body)&&ZPP_Collide.testCollide_safe(node.shape,circShape.zpp_inner)){
                                            ret.push(body);
                                        }
                                    }
                                };
                            }
                            else{
                                if(node.child1!=null)treeStack.add(node.child1);
                                if(node.child2!=null)treeStack.add(node.child2);
                            }
                        }
                    }
                }
            }
        }
        {
            {
                if(dtree.root!=null){
                    if(treeStack==null)treeStack=new ZNPList_ZPP_AABBNode();
                    treeStack.add(dtree.root);
                    while(!treeStack.empty()){
                        var node=treeStack.pop_unsafe();
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                node!=null;
                            };
                            if(!res)throw "assert("+"node!=null"+") :: "+("null node");
                            #end
                        };
                        if(node.aabb.intersect(ab)){
                            if(node.isLeaf()){
                                {
                                    var body=node.shape.body.outer;
                                    if(filter==null||node.shape.filter.shouldCollide(filter)){
                                        if(containment){
                                            if(!failed.has(body)){
                                                var col=ZPP_Collide.containTest(circShape.zpp_inner,node.shape);
                                                if(!ret.has(body)&&col)ret.push(body);
                                                else if(!col){
                                                    ret.remove(body);
                                                    failed.push(body);
                                                }
                                            }
                                        }
                                        else if(!ret.has(body)&&ZPP_Collide.testCollide_safe(node.shape,circShape.zpp_inner)){
                                            ret.push(body);
                                        }
                                    }
                                };
                            }
                            else{
                                if(node.child1!=null)treeStack.add(node.child1);
                                if(node.child2!=null)treeStack.add(node.child2);
                            }
                        }
                    }
                }
            }
        }
        failed.clear();
        return ret;
    }
    public override function shapesInShape(shp:ZPP_Shape,containment:Bool,filter:ZPP_InteractionFilter,output:ShapeList){
        sync_broadphase();
        validateShape(shp);
        var ab=shp.aabb;
        var ret=(output==null?new ShapeList():output);
        {
            {
                if(stree.root!=null){
                    if(treeStack==null)treeStack=new ZNPList_ZPP_AABBNode();
                    treeStack.add(stree.root);
                    while(!treeStack.empty()){
                        var node=treeStack.pop_unsafe();
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                node!=null;
                            };
                            if(!res)throw "assert("+"node!=null"+") :: "+("null node");
                            #end
                        };
                        if(node.aabb.intersect(ab)){
                            if(node.isLeaf()){
                                {
                                    if(filter==null||node.shape.filter.shouldCollide(filter)){
                                        if(containment){
                                            if(ZPP_Collide.containTest(shp,node.shape))ret.push(node.shape.outer);
                                        }
                                        else if(ZPP_Collide.testCollide_safe(node.shape,shp))ret.push(node.shape.outer);
                                    }
                                };
                            }
                            else{
                                if(node.child1!=null)treeStack.add(node.child1);
                                if(node.child2!=null)treeStack.add(node.child2);
                            }
                        }
                    }
                }
            }
        }
        {
            {
                if(dtree.root!=null){
                    if(treeStack==null)treeStack=new ZNPList_ZPP_AABBNode();
                    treeStack.add(dtree.root);
                    while(!treeStack.empty()){
                        var node=treeStack.pop_unsafe();
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                node!=null;
                            };
                            if(!res)throw "assert("+"node!=null"+") :: "+("null node");
                            #end
                        };
                        if(node.aabb.intersect(ab)){
                            if(node.isLeaf()){
                                {
                                    if(filter==null||node.shape.filter.shouldCollide(filter)){
                                        if(containment){
                                            if(ZPP_Collide.containTest(shp,node.shape))ret.push(node.shape.outer);
                                        }
                                        else if(ZPP_Collide.testCollide_safe(node.shape,shp))ret.push(node.shape.outer);
                                    }
                                };
                            }
                            else{
                                if(node.child1!=null)treeStack.add(node.child1);
                                if(node.child2!=null)treeStack.add(node.child2);
                            }
                        }
                    }
                }
            }
        }
        return ret;
    }
    public override function bodiesInShape(shp:ZPP_Shape,containment:Bool,filter:ZPP_InteractionFilter,output:BodyList){
        sync_broadphase();
        validateShape(shp);
        var ab=shp.aabb;
        var ret=(output==null?new BodyList():output);
        if(failed==null)failed=new BodyList();
        {
            {
                if(stree.root!=null){
                    if(treeStack==null)treeStack=new ZNPList_ZPP_AABBNode();
                    treeStack.add(stree.root);
                    while(!treeStack.empty()){
                        var node=treeStack.pop_unsafe();
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                node!=null;
                            };
                            if(!res)throw "assert("+"node!=null"+") :: "+("null node");
                            #end
                        };
                        if(node.aabb.intersect(ab)){
                            if(node.isLeaf()){
                                {
                                    var body=node.shape.body.outer;
                                    if(filter==null||node.shape.filter.shouldCollide(filter)){
                                        if(containment){
                                            if(!failed.has(body)){
                                                var col=ZPP_Collide.containTest(shp,node.shape);
                                                if(!ret.has(body)&&col)ret.push(body);
                                                else if(!col){
                                                    ret.remove(body);
                                                    failed.push(body);
                                                }
                                            }
                                        }
                                        else if(!ret.has(body)&&ZPP_Collide.testCollide_safe(node.shape,shp)){
                                            ret.push(body);
                                        }
                                    }
                                };
                            }
                            else{
                                if(node.child1!=null)treeStack.add(node.child1);
                                if(node.child2!=null)treeStack.add(node.child2);
                            }
                        }
                    }
                }
            }
        }
        {
            {
                if(dtree.root!=null){
                    if(treeStack==null)treeStack=new ZNPList_ZPP_AABBNode();
                    treeStack.add(dtree.root);
                    while(!treeStack.empty()){
                        var node=treeStack.pop_unsafe();
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                node!=null;
                            };
                            if(!res)throw "assert("+"node!=null"+") :: "+("null node");
                            #end
                        };
                        if(node.aabb.intersect(ab)){
                            if(node.isLeaf()){
                                {
                                    var body=node.shape.body.outer;
                                    if(filter==null||node.shape.filter.shouldCollide(filter)){
                                        if(containment){
                                            if(!failed.has(body)){
                                                var col=ZPP_Collide.containTest(shp,node.shape);
                                                if(!ret.has(body)&&col)ret.push(body);
                                                else if(!col){
                                                    ret.remove(body);
                                                    failed.push(body);
                                                }
                                            }
                                        }
                                        else if(!ret.has(body)&&ZPP_Collide.testCollide_safe(node.shape,shp)){
                                            ret.push(body);
                                        }
                                    }
                                };
                            }
                            else{
                                if(node.child1!=null)treeStack.add(node.child1);
                                if(node.child2!=null)treeStack.add(node.child2);
                            }
                        }
                    }
                }
            }
        }
        failed.clear();
        return ret;
    }
    public var openlist:ZNPList_ZPP_AABBNode=null;
    public override function rayCast(ray:ZPP_Ray,inner:Bool,filter:ZPP_InteractionFilter){
        if(openlist==null)openlist=new ZNPList_ZPP_AABBNode();
        sync_broadphase();
        ray.validate_dir();
        var mint=ray.maxdist;
        {
            if(dtree.root!=null){
                if(ray.aabbtest(dtree.root.aabb)){
                    var t=ray.aabbsect(dtree.root.aabb);
                    if(t>=0&&t<mint){
                        dtree.root.rayt=t;
                        {
                            var pre=null;
                            {
                                var cx_ite=openlist.begin();
                                while(cx_ite!=null){
                                    var j=cx_ite.elem();
                                    {
                                        if((dtree.root.rayt<j.rayt))break;
                                        pre=cx_ite;
                                    };
                                    cx_ite=cx_ite.next;
                                }
                            };
                            openlist.inlined_insert(pre,dtree.root);
                        };
                    }
                }
            }
        };
        {
            if(stree.root!=null){
                if(ray.aabbtest(stree.root.aabb)){
                    var t=ray.aabbsect(stree.root.aabb);
                    if(t>=0&&t<mint){
                        stree.root.rayt=t;
                        {
                            var pre=null;
                            {
                                var cx_ite=openlist.begin();
                                while(cx_ite!=null){
                                    var j=cx_ite.elem();
                                    {
                                        if((stree.root.rayt<j.rayt))break;
                                        pre=cx_ite;
                                    };
                                    cx_ite=cx_ite.next;
                                }
                            };
                            openlist.inlined_insert(pre,stree.root);
                        };
                    }
                }
            }
        };
        var minres:RayResult=null;
        while(!openlist.empty()){
            var cnode=openlist.pop_unsafe();
            if(cnode.rayt>=mint)break;
            if(cnode.isLeaf()){
                var shape=cnode.shape;
                if(filter==null||shape.filter.shouldCollide(filter)){
                    var result=if(shape.isCircle())ray.circlesect(shape.circle,inner,mint)else if(ray.aabbtest(shape.aabb))ray.polysect(shape.polygon,inner,mint);
                    else null;
                    if(result!=null){
                        mint=result.distance;
                        if(minres!=null){
                            minres.dispose();
                        }
                        minres=result;
                    }
                }
            }
            else{
                {
                    if(cnode.child1!=null){
                        if(ray.aabbtest(cnode.child1.aabb)){
                            var t=ray.aabbsect(cnode.child1.aabb);
                            if(t>=0&&t<mint){
                                cnode.child1.rayt=t;
                                {
                                    var pre=null;
                                    {
                                        var cx_ite=openlist.begin();
                                        while(cx_ite!=null){
                                            var j=cx_ite.elem();
                                            {
                                                if((cnode.child1.rayt<j.rayt))break;
                                                pre=cx_ite;
                                            };
                                            cx_ite=cx_ite.next;
                                        }
                                    };
                                    openlist.inlined_insert(pre,cnode.child1);
                                };
                            }
                        }
                    }
                };
                {
                    if(cnode.child2!=null){
                        if(ray.aabbtest(cnode.child2.aabb)){
                            var t=ray.aabbsect(cnode.child2.aabb);
                            if(t>=0&&t<mint){
                                cnode.child2.rayt=t;
                                {
                                    var pre=null;
                                    {
                                        var cx_ite=openlist.begin();
                                        while(cx_ite!=null){
                                            var j=cx_ite.elem();
                                            {
                                                if((cnode.child2.rayt<j.rayt))break;
                                                pre=cx_ite;
                                            };
                                            cx_ite=cx_ite.next;
                                        }
                                    };
                                    openlist.inlined_insert(pre,cnode.child2);
                                };
                            }
                        }
                    }
                };
            }
        }
        openlist.clear();
        return minres;
    }
    public override function rayMultiCast(ray:ZPP_Ray,inner:Bool,filter:ZPP_InteractionFilter,output:RayResultList){
        if(openlist==null)openlist=new ZNPList_ZPP_AABBNode();
        sync_broadphase();
        ray.validate_dir();
        var inf=ray.maxdist>=ZPP_Const.POSINF();
        var ret=(output==null?new RayResultList():output);
        {
            if(dtree.root!=null){
                if(ray.aabbtest(dtree.root.aabb)){
                    if(inf)openlist.add(dtree.root);
                    else{
                        var t=ray.aabbsect(dtree.root.aabb);
                        if(t>=0&&t<ray.maxdist)openlist.add(dtree.root);
                    }
                }
            }
        };
        {
            if(stree.root!=null){
                if(ray.aabbtest(stree.root.aabb)){
                    if(inf)openlist.add(stree.root);
                    else{
                        var t=ray.aabbsect(stree.root.aabb);
                        if(t>=0&&t<ray.maxdist)openlist.add(stree.root);
                    }
                }
            }
        };
        while(!openlist.empty()){
            var cnode=openlist.pop_unsafe();
            if(cnode.isLeaf()){
                var shape=cnode.shape;
                if(filter==null||shape.filter.shouldCollide(filter)){
                    if(shape.isCircle())ray.circlesect2(shape.circle,inner,ret);
                    else if(ray.aabbtest(shape.aabb))ray.polysect2(shape.polygon,inner,ret);
                }
            }
            else{
                {
                    if(cnode.child1!=null){
                        if(ray.aabbtest(cnode.child1.aabb)){
                            if(inf)openlist.add(cnode.child1);
                            else{
                                var t=ray.aabbsect(cnode.child1.aabb);
                                if(t>=0&&t<ray.maxdist)openlist.add(cnode.child1);
                            }
                        }
                    }
                };
                {
                    if(cnode.child2!=null){
                        if(ray.aabbtest(cnode.child2.aabb)){
                            if(inf)openlist.add(cnode.child2);
                            else{
                                var t=ray.aabbsect(cnode.child2.aabb);
                                if(t>=0&&t<ray.maxdist)openlist.add(cnode.child2);
                            }
                        }
                    }
                };
            }
        }
        openlist.clear();
        return ret;
    }
}
