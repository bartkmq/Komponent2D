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
import zpp_nape.geom.Triangular;
import zpp_nape.geom.PartitionedPoly;
import zpp_nape.geom.Simplify;
import zpp_nape.geom.AABB;
import zpp_nape.geom.SweepDistance;
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
class ZPP_SimpleVert{
    public var forced:Bool=false;
    public var x:Float=0.0;
    public var y:Float=0.0;
    public var links:ZPP_Set_ZPP_SimpleVert=null;
    public var id:Int=0;
    public var next:ZPP_SimpleVert=null;
    static public var zpp_pool:ZPP_SimpleVert=null;
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
    
    public var node:ZPP_Set_ZPP_SimpleVert=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free(){
        links.clear();
        node=null;
        forced=false;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc(){}
    function new(){
        id=ZPP_ID.ZPP_SimpleVert();
        {
            if(ZPP_Set_ZPP_SimpleVert.zpp_pool==null){
                links=new ZPP_Set_ZPP_SimpleVert();
                #if NAPE_POOL_STATS ZPP_Set_ZPP_SimpleVert.POOL_TOT++;
                ZPP_Set_ZPP_SimpleVert.POOL_ADDNEW++;
                #end
            }
            else{
                links=ZPP_Set_ZPP_SimpleVert.zpp_pool;
                ZPP_Set_ZPP_SimpleVert.zpp_pool=links.next;
                links.next=null;
                #if NAPE_POOL_STATS ZPP_Set_ZPP_SimpleVert.POOL_CNT--;
                ZPP_Set_ZPP_SimpleVert.POOL_ADD++;
                #end
            }
            links.alloc();
        };
        links.lt=less_xy;
    }
    public static function less_xy(p:ZPP_SimpleVert,q:ZPP_SimpleVert){
        return p.y<q.y||(p.y==q.y&&p.x<q.x);
    }
    public static function swap_nodes(p:ZPP_SimpleVert,q:ZPP_SimpleVert){
        var t=p.node;
        p.node=q.node;
        q.node=t;
    }
    public static#if NAPE_NO_INLINE#else inline #end
    function get(x:Float,y:Float){
        var ret;
        {
            if(ZPP_SimpleVert.zpp_pool==null){
                ret=new ZPP_SimpleVert();
                #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_TOT++;
                ZPP_SimpleVert.POOL_ADDNEW++;
                #end
            }
            else{
                ret=ZPP_SimpleVert.zpp_pool;
                ZPP_SimpleVert.zpp_pool=ret.next;
                ret.next=null;
                #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT--;
                ZPP_SimpleVert.POOL_ADD++;
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
}
#if nape_swc@:keep #end
class ZPP_SimpleSeg{
    public var left:ZPP_SimpleVert=null;
    public var right:ZPP_SimpleVert=null;
    public var vertices:ZPP_Set_ZPP_SimpleVert=null;
    public var id:Int=0;
    public var next:ZPP_SimpleSeg=null;
    static public var zpp_pool:ZPP_SimpleSeg=null;
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
        left=right=null;
        prev=null;
        node=null;
        vertices.clear();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc(){}
    public var prev:ZPP_SimpleSeg=null;
    public var node:ZPP_Set_ZPP_SimpleSeg=null;
    public function less_xy(a:ZPP_SimpleVert,b:ZPP_SimpleVert){
        return a.x<b.x||(a.x==b.x&&a.y<b.y);
    }
    function new(){
        id=ZPP_ID.ZPP_SimpleSeg();
        {
            if(ZPP_Set_ZPP_SimpleVert.zpp_pool==null){
                vertices=new ZPP_Set_ZPP_SimpleVert();
                #if NAPE_POOL_STATS ZPP_Set_ZPP_SimpleVert.POOL_TOT++;
                ZPP_Set_ZPP_SimpleVert.POOL_ADDNEW++;
                #end
            }
            else{
                vertices=ZPP_Set_ZPP_SimpleVert.zpp_pool;
                ZPP_Set_ZPP_SimpleVert.zpp_pool=vertices.next;
                vertices.next=null;
                #if NAPE_POOL_STATS ZPP_Set_ZPP_SimpleVert.POOL_CNT--;
                ZPP_Set_ZPP_SimpleVert.POOL_ADD++;
                #end
            }
            vertices.alloc();
        };
        vertices.lt=less_xy;
    }
    public static function get(left:ZPP_SimpleVert,right:ZPP_SimpleVert){
        var ret;
        {
            if(ZPP_SimpleSeg.zpp_pool==null){
                ret=new ZPP_SimpleSeg();
                #if NAPE_POOL_STATS ZPP_SimpleSeg.POOL_TOT++;
                ZPP_SimpleSeg.POOL_ADDNEW++;
                #end
            }
            else{
                ret=ZPP_SimpleSeg.zpp_pool;
                ZPP_SimpleSeg.zpp_pool=ret.next;
                ret.next=null;
                #if NAPE_POOL_STATS ZPP_SimpleSeg.POOL_CNT--;
                ZPP_SimpleSeg.POOL_ADD++;
                #end
            }
            ret.alloc();
        };
        ret.left=left;
        ret.right=right;
        ret.vertices.insert(left);
        ret.vertices.insert(right);
        return ret;
    }
}
#if nape_swc@:keep #end
class ZPP_SimpleEvent{
    public var type:Int=0;
    public var vertex:ZPP_SimpleVert=null;
    public var segment:ZPP_SimpleSeg=null;
    public var segment2:ZPP_SimpleSeg=null;
    public var node:ZPP_Set_ZPP_SimpleEvent=null;
    public static function swap_nodes(a:ZPP_SimpleEvent,b:ZPP_SimpleEvent){
        var t=a.node;
        a.node=b.node;
        b.node=t;
    }
    public static function less_xy(a:ZPP_SimpleEvent,b:ZPP_SimpleEvent){
        if(a.vertex.x<b.vertex.x)return true;
        else if(a.vertex.x>b.vertex.x)return false;
        else{
            if(a.vertex.y<b.vertex.y)return true;
            else if(a.vertex.y>b.vertex.y)return false;
            else return a.type<b.type;
        }
    }
    public var next:ZPP_SimpleEvent=null;
    static public var zpp_pool:ZPP_SimpleEvent=null;
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
        vertex=null;
        segment=segment2=null;
        node=null;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc(){}
    function new(){}
    public static#if NAPE_NO_INLINE#else inline #end
    function get(v:ZPP_SimpleVert){
        var ret;
        {
            if(ZPP_SimpleEvent.zpp_pool==null){
                ret=new ZPP_SimpleEvent();
                #if NAPE_POOL_STATS ZPP_SimpleEvent.POOL_TOT++;
                ZPP_SimpleEvent.POOL_ADDNEW++;
                #end
            }
            else{
                ret=ZPP_SimpleEvent.zpp_pool;
                ZPP_SimpleEvent.zpp_pool=ret.next;
                ret.next=null;
                #if NAPE_POOL_STATS ZPP_SimpleEvent.POOL_CNT--;
                ZPP_SimpleEvent.POOL_ADD++;
                #end
            }
            ret.alloc();
        };
        ret.vertex=v;
        return ret;
    }
}
#if nape_swc@:keep #end
class ZPP_SimpleSweep{
    public var sweepx:Float=0.0;
    public var tree:ZPP_Set_ZPP_SimpleSeg=null;
    public function new(){
        {
            if(ZPP_Set_ZPP_SimpleSeg.zpp_pool==null){
                tree=new ZPP_Set_ZPP_SimpleSeg();
                #if NAPE_POOL_STATS ZPP_Set_ZPP_SimpleSeg.POOL_TOT++;
                ZPP_Set_ZPP_SimpleSeg.POOL_ADDNEW++;
                #end
            }
            else{
                tree=ZPP_Set_ZPP_SimpleSeg.zpp_pool;
                ZPP_Set_ZPP_SimpleSeg.zpp_pool=tree.next;
                tree.next=null;
                #if NAPE_POOL_STATS ZPP_Set_ZPP_SimpleSeg.POOL_CNT--;
                ZPP_Set_ZPP_SimpleSeg.POOL_ADD++;
                #end
            }
            tree.alloc();
        };
        tree.lt=edge_lt;
        tree.swapped=swap_nodes;
    }
    public function swap_nodes(p:ZPP_SimpleSeg,q:ZPP_SimpleSeg){
        var t=p.node;
        p.node=q.node;
        q.node=t;
    }
    public function edge_lt(p:ZPP_SimpleSeg,q:ZPP_SimpleSeg){
        var ux:Float=0.0;
        var uy:Float=0.0;
        var vx:Float=0.0;
        var vy:Float=0.0;
        var flip:Bool;
        if(p.left==q.left&&p.right==q.right)return false;
        else if(p.left==q.right)return({
            if(p.left.x==p.right.x){
                if(p.left.y<p.right.y)p.left.y>q.left.y;
                else p.right.y>q.left.y;
            }
            else({
                flip=p.right.x<p.left.x;
                {
                    ux=p.right.x-p.left.x;
                    uy=p.right.y-p.left.y;
                };
                {
                    vx=q.left.x-p.left.x;
                    vy=q.left.y-p.left.y;
                };
                flip?(uy*vx-ux*vy):(vy*ux-vx*uy);
            })<0;
        });
        else if(p.right==q.left)return!({
            if(q.left.x==q.right.x){
                if(q.left.y<q.right.y)q.left.y>p.left.y;
                else q.right.y>p.left.y;
            }
            else({
                flip=q.right.x<q.left.x;
                {
                    ux=q.right.x-q.left.x;
                    uy=q.right.y-q.left.y;
                };
                {
                    vx=p.left.x-q.left.x;
                    vy=p.left.y-q.left.y;
                };
                flip?(uy*vx-ux*vy):(vy*ux-vx*uy);
            })<0;
        });
        else if(p.left==q.left)return({
            if(p.left.x==p.right.x){
                if(p.left.y<p.right.y)p.left.y>q.right.y;
                else p.right.y>q.right.y;
            }
            else({
                flip=p.right.x<p.left.x;
                {
                    ux=p.right.x-p.left.x;
                    uy=p.right.y-p.left.y;
                };
                {
                    vx=q.right.x-p.left.x;
                    vy=q.right.y-p.left.y;
                };
                flip?(uy*vx-ux*vy):(vy*ux-vx*uy);
            })<0;
        });
        else if(p.right==q.right)return({
            if(p.left.x==p.right.x){
                if(p.left.y<p.right.y)p.left.y>q.left.y;
                else p.right.y>q.left.y;
            }
            else({
                flip=p.right.x<p.left.x;
                {
                    ux=p.right.x-p.left.x;
                    uy=p.right.y-p.left.y;
                };
                {
                    vx=q.left.x-p.left.x;
                    vy=q.left.y-p.left.y;
                };
                flip?(uy*vx-ux*vy):(vy*ux-vx*uy);
            })<0;
        });
        if(p.left.x==p.right.x){
            if(q.left.x==q.right.x){
                var pmax=if(p.left.y<p.right.y)p.right else p.left;
                var qmax=if(q.left.y<q.right.y)q.right else q.left;
                return pmax.y>qmax.y;
            }
            else{
                var plrg=({
                    flip=q.right.x<q.left.x;
                    {
                        ux=q.right.x-q.left.x;
                        uy=q.right.y-q.left.y;
                    };
                    {
                        vx=p.left.x-q.left.x;
                        vy=p.left.y-q.left.y;
                    };
                    flip?(uy*vx-ux*vy):(vy*ux-vx*uy);
                });
                var aplrg=({
                    flip=q.right.x<q.left.x;
                    {
                        ux=q.right.x-q.left.x;
                        uy=q.right.y-q.left.y;
                    };
                    {
                        vx=p.right.x-q.left.x;
                        vy=p.right.y-q.left.y;
                    };
                    flip?(uy*vx-ux*vy):(vy*ux-vx*uy);
                });
                if(plrg*aplrg>=0)return plrg>=0.0;
                else return sweepx>=p.left.x;
            }
        }
        else if(q.left.x==q.right.x){
            var qlrg=({
                flip=p.right.x<p.left.x;
                {
                    ux=p.right.x-p.left.x;
                    uy=p.right.y-p.left.y;
                };
                {
                    vx=q.left.x-p.left.x;
                    vy=q.left.y-p.left.y;
                };
                flip?(uy*vx-ux*vy):(vy*ux-vx*uy);
            });
            var aqlrg=({
                flip=p.right.x<p.left.x;
                {
                    ux=p.right.x-p.left.x;
                    uy=p.right.y-p.left.y;
                };
                {
                    vx=q.right.x-p.left.x;
                    vy=q.right.y-p.left.y;
                };
                flip?(uy*vx-ux*vy):(vy*ux-vx*uy);
            });
            if(qlrg*aqlrg>=0)return qlrg<0.0;
            else return sweepx<q.left.x;
        }
        else{
            var qlrg=({
                flip=p.right.x<p.left.x;
                {
                    ux=p.right.x-p.left.x;
                    uy=p.right.y-p.left.y;
                };
                {
                    vx=q.left.x-p.left.x;
                    vy=q.left.y-p.left.y;
                };
                flip?(uy*vx-ux*vy):(vy*ux-vx*uy);
            })<0.0;
            var aqlrg=({
                flip=p.right.x<p.left.x;
                {
                    ux=p.right.x-p.left.x;
                    uy=p.right.y-p.left.y;
                };
                {
                    vx=q.right.x-p.left.x;
                    vy=q.right.y-p.left.y;
                };
                flip?(uy*vx-ux*vy):(vy*ux-vx*uy);
            })<0.0;
            if(qlrg==aqlrg)return qlrg;
            else{
                var plrg=({
                    flip=q.right.x<q.left.x;
                    {
                        ux=q.right.x-q.left.x;
                        uy=q.right.y-q.left.y;
                    };
                    {
                        vx=p.left.x-q.left.x;
                        vy=p.left.y-q.left.y;
                    };
                    flip?(uy*vx-ux*vy):(vy*ux-vx*uy);
                })>=0.0;
                var aplrg=({
                    flip=q.right.x<q.left.x;
                    {
                        ux=q.right.x-q.left.x;
                        uy=q.right.y-q.left.y;
                    };
                    {
                        vx=p.right.x-q.left.x;
                        vy=p.right.y-q.left.y;
                    };
                    flip?(uy*vx-ux*vy):(vy*ux-vx*uy);
                })>=0.0;
                if(plrg==aplrg)return plrg;
                var py=(sweepx-p.left.x)/(p.right.x-p.left.x)*(p.right.y-p.left.y)+p.left.y;
                var qy=(sweepx-q.left.x)/(q.right.x-q.left.x)*(q.right.y-q.left.y)+q.left.y;
                return py>qy;
            }
        }
    }
    public function clear(){
        tree.clear();
    }
    public function add(e:ZPP_SimpleSeg){
        e.node=tree.insert(e);
        var nxt=tree.successor_node(e.node);
        var pre=tree.predecessor_node(e.node);
        if(nxt!=null){
            e.next=nxt.data;
            nxt.data.prev=e;
        }
        if(pre!=null){
            e.prev=pre.data;
            pre.data.next=e;
        }
        return e;
    }
    public function remove(e:ZPP_SimpleSeg){
        var nxt=tree.successor_node(e.node);
        var pre=tree.predecessor_node(e.node);
        if(nxt!=null)nxt.data.prev=e.prev;
        if(pre!=null)pre.data.next=e.next;
        tree.remove_node(e.node);
        e.node=null;
    }
    public function intersect(p:ZPP_SimpleSeg,q:ZPP_SimpleSeg){
        if(p==null||q==null)return false;
        else if(p.left==q.left||p.left==q.right||p.right==q.left||p.right==q.right)return false;
        else{
            var lsign=((q.left.x-p.left.x)*(p.right.y-p.left.y)-(p.right.x-p.left.x)*(q.left.y-p.left.y));
            var rsign=((q.right.x-p.left.x)*(p.right.y-p.left.y)-(p.right.x-p.left.x)*(q.right.y-p.left.y));
            if(lsign*rsign>0)return false;
            else{
                var lsign2=((p.left.x-q.left.x)*(q.right.y-q.left.y)-(q.right.x-q.left.x)*(p.left.y-q.left.y));
                var rsign2=((p.right.x-q.left.x)*(q.right.y-q.left.y)-(q.right.x-q.left.x)*(p.right.y-q.left.y));
                if(lsign2*rsign2>0)return false;
                else if(lsign*rsign>=0&&lsign2*rsign2>=0){
                    return true;
                }
                else return true;
            }
        }
    }
    public function intersection(p:ZPP_SimpleSeg,q:ZPP_SimpleSeg){
        if(p==null||q==null)return null;
        else if(p.left==q.left||p.left==q.right||p.right==q.left||p.right==q.right)return null;
        else{
            var ux:Float=0.0;
            var uy:Float=0.0;
            {
                ux=p.right.x-p.left.x;
                uy=p.right.y-p.left.y;
            };
            var vx:Float=0.0;
            var vy:Float=0.0;
            {
                vx=q.right.x-q.left.x;
                vy=q.right.y-q.left.y;
            };
            var denom=(vy*ux-vx*uy);
            if(denom==0.0)return null;
            denom=1/denom;
            var cx:Float=0.0;
            var cy:Float=0.0;
            {
                cx=q.left.x-p.left.x;
                cy=q.left.y-p.left.y;
            };
            var t=(vy*cx-vx*cy)*denom;
            if(t<0||t>1)return null;
            var s=(uy*cx-ux*cy)*denom;
            if(s<0||s>1)return null;
            var vet:ZPP_SimpleVert;
            if(s==0||s==1||t==0||t==1){
                #if(!NAPE_RELEASE_BUILD)
                var cases=s==0;
                if(s==1&&cases)throw "corner case 1a";
                else if(s==1)cases=true;
                if(t==0&&cases)throw "corner case 1b";
                else if(t==0)cases=true;
                if(t==1&&cases)throw "corner case 1c";
                #end
                vet=if(s==0)q.left else if(s==1)q.right else if(t==0)p.left else p.right;
            }
            else vet=ZPP_SimpleVert.get(0.5*(p.left.x+ux*t+q.left.x+vx*s),0.5*(p.left.y+uy*t+q.left.y+vy*s));
            var ret=ZPP_SimpleEvent.get(vet);
            ret.type=0;
            ret.segment=p;
            ret.segment2=q;
            return ret;
        }
    }
}
#if nape_swc@:keep #end
class ZPP_Simple{
    static var sweep:ZPP_SimpleSweep=null;
    static var inthash:FastHash2_Hashable2_Boolfalse=null;
    static var vertices:ZPP_Set_ZPP_SimpleVert=null;
    static var queue:ZPP_Set_ZPP_SimpleEvent=null;
    static var ints:ZPP_Set_ZPP_SimpleEvent=null;
    public static function decompose(poly:ZPP_GeomVert,?rets:ZNPList_ZPP_GeomVert){
        if(sweep==null){
            sweep=new ZPP_SimpleSweep();
            inthash=new FastHash2_Hashable2_Boolfalse();
        }
        if(vertices==null){
            {
                if(ZPP_Set_ZPP_SimpleVert.zpp_pool==null){
                    vertices=new ZPP_Set_ZPP_SimpleVert();
                    #if NAPE_POOL_STATS ZPP_Set_ZPP_SimpleVert.POOL_TOT++;
                    ZPP_Set_ZPP_SimpleVert.POOL_ADDNEW++;
                    #end
                }
                else{
                    vertices=ZPP_Set_ZPP_SimpleVert.zpp_pool;
                    ZPP_Set_ZPP_SimpleVert.zpp_pool=vertices.next;
                    vertices.next=null;
                    #if NAPE_POOL_STATS ZPP_Set_ZPP_SimpleVert.POOL_CNT--;
                    ZPP_Set_ZPP_SimpleVert.POOL_ADD++;
                    #end
                }
                vertices.alloc();
            };
            vertices.lt=ZPP_SimpleVert.less_xy;
            vertices.swapped=ZPP_SimpleVert.swap_nodes;
        }
        if(queue==null){
            {
                if(ZPP_Set_ZPP_SimpleEvent.zpp_pool==null){
                    queue=new ZPP_Set_ZPP_SimpleEvent();
                    #if NAPE_POOL_STATS ZPP_Set_ZPP_SimpleEvent.POOL_TOT++;
                    ZPP_Set_ZPP_SimpleEvent.POOL_ADDNEW++;
                    #end
                }
                else{
                    queue=ZPP_Set_ZPP_SimpleEvent.zpp_pool;
                    ZPP_Set_ZPP_SimpleEvent.zpp_pool=queue.next;
                    queue.next=null;
                    #if NAPE_POOL_STATS ZPP_Set_ZPP_SimpleEvent.POOL_CNT--;
                    ZPP_Set_ZPP_SimpleEvent.POOL_ADD++;
                    #end
                }
                queue.alloc();
            };
            queue.lt=ZPP_SimpleEvent.less_xy;
            queue.swapped=ZPP_SimpleEvent.swap_nodes;
        }
        var fst:ZPP_SimpleVert=null;
        var pre:ZPP_SimpleVert=null;
        {
            var F=poly;
            var L=poly;
            if(F!=null){
                var nite=F;
                do{
                    var v=nite;
                    {
                        {
                            var vert=ZPP_SimpleVert.get(v.x,v.y);
                            var vx=({
                                var cur=vertices.parent;
                                while(cur!=null){
                                    if(vertices.lt(vert,cur.data))cur=cur.prev;
                                    else if(vertices.lt(cur.data,vert))cur=cur.next;
                                    else break;
                                }
                                cur;
                            });
                            if(vx!=null){
                                {
                                    var o=vert;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o!=null;
                                        };
                                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleVert"+", in obj: "+"vert"+")");
                                        #end
                                    };
                                    o.free();
                                    o.next=ZPP_SimpleVert.zpp_pool;
                                    ZPP_SimpleVert.zpp_pool=o;
                                    #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT++;
                                    ZPP_SimpleVert.POOL_SUB++;
                                    #end
                                };
                                vert=vx.data;
                            }
                            else vert.node=vertices.insert(vert);
                            if(pre!=null){
                                var e1=ZPP_SimpleEvent.get(pre);
                                var e2=ZPP_SimpleEvent.get(vert);
                                var seg:ZPP_SimpleSeg;
                                if(ZPP_SimpleEvent.less_xy(e1,e2)){
                                    e1.type=1;
                                    e2.type=2;
                                    seg=ZPP_SimpleSeg.get(pre,vert);
                                }
                                else{
                                    e1.type=2;
                                    e2.type=1;
                                    seg=ZPP_SimpleSeg.get(vert,pre);
                                }
                                e1.segment=e2.segment=seg;
                                queue.insert(e1);
                                queue.insert(e2);
                                pre.links.insert(vert);
                                vert.links.insert(pre);
                            };
                            pre=vert;
                            if(fst==null)fst=vert;
                        };
                    }
                    nite=nite.next;
                }
                while(nite!=L);
            }
        };
        {
            var e1=ZPP_SimpleEvent.get(pre);
            var e2=ZPP_SimpleEvent.get(fst);
            var seg:ZPP_SimpleSeg;
            if(ZPP_SimpleEvent.less_xy(e1,e2)){
                e1.type=1;
                e2.type=2;
                seg=ZPP_SimpleSeg.get(pre,fst);
            }
            else{
                e1.type=2;
                e2.type=1;
                seg=ZPP_SimpleSeg.get(fst,pre);
            }
            e1.segment=e2.segment=seg;
            queue.insert(e1);
            queue.insert(e2);
            pre.links.insert(fst);
            fst.links.insert(pre);
        };
        if(ints==null){
            {
                if(ZPP_Set_ZPP_SimpleEvent.zpp_pool==null){
                    ints=new ZPP_Set_ZPP_SimpleEvent();
                    #if NAPE_POOL_STATS ZPP_Set_ZPP_SimpleEvent.POOL_TOT++;
                    ZPP_Set_ZPP_SimpleEvent.POOL_ADDNEW++;
                    #end
                }
                else{
                    ints=ZPP_Set_ZPP_SimpleEvent.zpp_pool;
                    ZPP_Set_ZPP_SimpleEvent.zpp_pool=ints.next;
                    ints.next=null;
                    #if NAPE_POOL_STATS ZPP_Set_ZPP_SimpleEvent.POOL_CNT--;
                    ZPP_Set_ZPP_SimpleEvent.POOL_ADD++;
                    #end
                }
                ints.alloc();
            };
            ints.lt=ZPP_SimpleEvent.less_xy;
        }
        while(!queue.empty()){
            var e=queue.pop_front();
            sweep.sweepx=e.vertex.x;
            if(e.type==1){
                var s=e.segment;
                sweep.add(s);
                {
                    if(s.next!=null&&s!=null&&!(s.next.id<s.id?inthash.has(s.next.id,s.id):inthash.has(s.id,s.next.id))){
                        var intx=sweep.intersection(s.next,s);
                        if(intx!=null){
                            if(intx.vertex.x>=sweep.sweepx){
                                var ex=({
                                    var cur=queue.parent;
                                    while(cur!=null){
                                        if(queue.lt(intx,cur.data))cur=cur.prev;
                                        else if(queue.lt(cur.data,intx))cur=cur.next;
                                        else break;
                                    }
                                    cur;
                                });
                                if(ex==null){
                                    var vx=({
                                        var cur=ints.parent;
                                        while(cur!=null){
                                            if(ints.lt(intx,cur.data))cur=cur.prev;
                                            else if(ints.lt(cur.data,intx))cur=cur.next;
                                            else break;
                                        }
                                        cur;
                                    });
                                    if(vx!=null){
                                        {
                                            var o=intx.vertex;
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    o!=null;
                                                };
                                                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleVert"+", in obj: "+"intx.vertex"+")");
                                                #end
                                            };
                                            o.free();
                                            o.next=ZPP_SimpleVert.zpp_pool;
                                            ZPP_SimpleVert.zpp_pool=o;
                                            #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT++;
                                            ZPP_SimpleVert.POOL_SUB++;
                                            #end
                                        };
                                        intx.vertex=vx.data.vertex;
                                        vx.data=intx;
                                        queue.insert(intx);
                                    }
                                    else{
                                        queue.insert(intx);
                                        ints.insert(intx);
                                    }
                                    if(s.next.id<s.id)inthash.add(Hashable2_Boolfalse.get(s.next.id,s.id,true));
                                    else inthash.add(Hashable2_Boolfalse.get(s.id,s.next.id,true));
                                }
                                else{
                                    var x=ex.data;
                                    #if(!NAPE_RELEASE_BUILD)
                                    if(x.segment!=intx.segment||intx.segment2!=x.segment2)throw "corner case 2, shiiiit.";
                                    #end
                                    {
                                        var o=intx.vertex;
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                o!=null;
                                            };
                                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleVert"+", in obj: "+"intx.vertex"+")");
                                            #end
                                        };
                                        o.free();
                                        o.next=ZPP_SimpleVert.zpp_pool;
                                        ZPP_SimpleVert.zpp_pool=o;
                                        #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT++;
                                        ZPP_SimpleVert.POOL_SUB++;
                                        #end
                                    };
                                    {
                                        var o=intx;
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                o!=null;
                                            };
                                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleEvent"+", in obj: "+"intx"+")");
                                            #end
                                        };
                                        o.free();
                                        o.next=ZPP_SimpleEvent.zpp_pool;
                                        ZPP_SimpleEvent.zpp_pool=o;
                                        #if NAPE_POOL_STATS ZPP_SimpleEvent.POOL_CNT++;
                                        ZPP_SimpleEvent.POOL_SUB++;
                                        #end
                                    };
                                }
                            }
                            else{
                                {
                                    var o=intx.vertex;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o!=null;
                                        };
                                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleVert"+", in obj: "+"intx.vertex"+")");
                                        #end
                                    };
                                    o.free();
                                    o.next=ZPP_SimpleVert.zpp_pool;
                                    ZPP_SimpleVert.zpp_pool=o;
                                    #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT++;
                                    ZPP_SimpleVert.POOL_SUB++;
                                    #end
                                };
                                {
                                    var o=intx;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o!=null;
                                        };
                                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleEvent"+", in obj: "+"intx"+")");
                                        #end
                                    };
                                    o.free();
                                    o.next=ZPP_SimpleEvent.zpp_pool;
                                    ZPP_SimpleEvent.zpp_pool=o;
                                    #if NAPE_POOL_STATS ZPP_SimpleEvent.POOL_CNT++;
                                    ZPP_SimpleEvent.POOL_SUB++;
                                    #end
                                };
                            }
                        }
                    }
                };
                {
                    if(s!=null&&s.prev!=null&&!(s.id<s.prev.id?inthash.has(s.id,s.prev.id):inthash.has(s.prev.id,s.id))){
                        var intx=sweep.intersection(s,s.prev);
                        if(intx!=null){
                            if(intx.vertex.x>=sweep.sweepx){
                                var ex=({
                                    var cur=queue.parent;
                                    while(cur!=null){
                                        if(queue.lt(intx,cur.data))cur=cur.prev;
                                        else if(queue.lt(cur.data,intx))cur=cur.next;
                                        else break;
                                    }
                                    cur;
                                });
                                if(ex==null){
                                    var vx=({
                                        var cur=ints.parent;
                                        while(cur!=null){
                                            if(ints.lt(intx,cur.data))cur=cur.prev;
                                            else if(ints.lt(cur.data,intx))cur=cur.next;
                                            else break;
                                        }
                                        cur;
                                    });
                                    if(vx!=null){
                                        {
                                            var o=intx.vertex;
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    o!=null;
                                                };
                                                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleVert"+", in obj: "+"intx.vertex"+")");
                                                #end
                                            };
                                            o.free();
                                            o.next=ZPP_SimpleVert.zpp_pool;
                                            ZPP_SimpleVert.zpp_pool=o;
                                            #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT++;
                                            ZPP_SimpleVert.POOL_SUB++;
                                            #end
                                        };
                                        intx.vertex=vx.data.vertex;
                                        vx.data=intx;
                                        queue.insert(intx);
                                    }
                                    else{
                                        queue.insert(intx);
                                        ints.insert(intx);
                                    }
                                    if(s.id<s.prev.id)inthash.add(Hashable2_Boolfalse.get(s.id,s.prev.id,true));
                                    else inthash.add(Hashable2_Boolfalse.get(s.prev.id,s.id,true));
                                }
                                else{
                                    var x=ex.data;
                                    #if(!NAPE_RELEASE_BUILD)
                                    if(x.segment!=intx.segment||intx.segment2!=x.segment2)throw "corner case 2, shiiiit.";
                                    #end
                                    {
                                        var o=intx.vertex;
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                o!=null;
                                            };
                                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleVert"+", in obj: "+"intx.vertex"+")");
                                            #end
                                        };
                                        o.free();
                                        o.next=ZPP_SimpleVert.zpp_pool;
                                        ZPP_SimpleVert.zpp_pool=o;
                                        #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT++;
                                        ZPP_SimpleVert.POOL_SUB++;
                                        #end
                                    };
                                    {
                                        var o=intx;
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                o!=null;
                                            };
                                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleEvent"+", in obj: "+"intx"+")");
                                            #end
                                        };
                                        o.free();
                                        o.next=ZPP_SimpleEvent.zpp_pool;
                                        ZPP_SimpleEvent.zpp_pool=o;
                                        #if NAPE_POOL_STATS ZPP_SimpleEvent.POOL_CNT++;
                                        ZPP_SimpleEvent.POOL_SUB++;
                                        #end
                                    };
                                }
                            }
                            else{
                                {
                                    var o=intx.vertex;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o!=null;
                                        };
                                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleVert"+", in obj: "+"intx.vertex"+")");
                                        #end
                                    };
                                    o.free();
                                    o.next=ZPP_SimpleVert.zpp_pool;
                                    ZPP_SimpleVert.zpp_pool=o;
                                    #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT++;
                                    ZPP_SimpleVert.POOL_SUB++;
                                    #end
                                };
                                {
                                    var o=intx;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o!=null;
                                        };
                                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleEvent"+", in obj: "+"intx"+")");
                                        #end
                                    };
                                    o.free();
                                    o.next=ZPP_SimpleEvent.zpp_pool;
                                    ZPP_SimpleEvent.zpp_pool=o;
                                    #if NAPE_POOL_STATS ZPP_SimpleEvent.POOL_CNT++;
                                    ZPP_SimpleEvent.POOL_SUB++;
                                    #end
                                };
                            }
                        }
                    }
                };
            }
            else if(e.type==2){
                var s=e.segment;
                if(s.node!=null){
                    var nxt=s.next;
                    var pre=s.prev;
                    sweep.remove(s);
                    {
                        var o=s;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                o!=null;
                            };
                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleSeg"+", in obj: "+"s"+")");
                            #end
                        };
                        o.free();
                        o.next=ZPP_SimpleSeg.zpp_pool;
                        ZPP_SimpleSeg.zpp_pool=o;
                        #if NAPE_POOL_STATS ZPP_SimpleSeg.POOL_CNT++;
                        ZPP_SimpleSeg.POOL_SUB++;
                        #end
                    };
                    {
                        if(nxt!=null&&pre!=null&&!(nxt.id<pre.id?inthash.has(nxt.id,pre.id):inthash.has(pre.id,nxt.id))){
                            var intx=sweep.intersection(nxt,pre);
                            if(intx!=null){
                                if(intx.vertex.x>=sweep.sweepx){
                                    var ex=({
                                        var cur=queue.parent;
                                        while(cur!=null){
                                            if(queue.lt(intx,cur.data))cur=cur.prev;
                                            else if(queue.lt(cur.data,intx))cur=cur.next;
                                            else break;
                                        }
                                        cur;
                                    });
                                    if(ex==null){
                                        var vx=({
                                            var cur=ints.parent;
                                            while(cur!=null){
                                                if(ints.lt(intx,cur.data))cur=cur.prev;
                                                else if(ints.lt(cur.data,intx))cur=cur.next;
                                                else break;
                                            }
                                            cur;
                                        });
                                        if(vx!=null){
                                            {
                                                var o=intx.vertex;
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        o!=null;
                                                    };
                                                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleVert"+", in obj: "+"intx.vertex"+")");
                                                    #end
                                                };
                                                o.free();
                                                o.next=ZPP_SimpleVert.zpp_pool;
                                                ZPP_SimpleVert.zpp_pool=o;
                                                #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT++;
                                                ZPP_SimpleVert.POOL_SUB++;
                                                #end
                                            };
                                            intx.vertex=vx.data.vertex;
                                            vx.data=intx;
                                            queue.insert(intx);
                                        }
                                        else{
                                            queue.insert(intx);
                                            ints.insert(intx);
                                        }
                                        if(nxt.id<pre.id)inthash.add(Hashable2_Boolfalse.get(nxt.id,pre.id,true));
                                        else inthash.add(Hashable2_Boolfalse.get(pre.id,nxt.id,true));
                                    }
                                    else{
                                        var x=ex.data;
                                        #if(!NAPE_RELEASE_BUILD)
                                        if(x.segment!=intx.segment||intx.segment2!=x.segment2)throw "corner case 2, shiiiit.";
                                        #end
                                        {
                                            var o=intx.vertex;
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    o!=null;
                                                };
                                                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleVert"+", in obj: "+"intx.vertex"+")");
                                                #end
                                            };
                                            o.free();
                                            o.next=ZPP_SimpleVert.zpp_pool;
                                            ZPP_SimpleVert.zpp_pool=o;
                                            #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT++;
                                            ZPP_SimpleVert.POOL_SUB++;
                                            #end
                                        };
                                        {
                                            var o=intx;
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    o!=null;
                                                };
                                                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleEvent"+", in obj: "+"intx"+")");
                                                #end
                                            };
                                            o.free();
                                            o.next=ZPP_SimpleEvent.zpp_pool;
                                            ZPP_SimpleEvent.zpp_pool=o;
                                            #if NAPE_POOL_STATS ZPP_SimpleEvent.POOL_CNT++;
                                            ZPP_SimpleEvent.POOL_SUB++;
                                            #end
                                        };
                                    }
                                }
                                else{
                                    {
                                        var o=intx.vertex;
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                o!=null;
                                            };
                                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleVert"+", in obj: "+"intx.vertex"+")");
                                            #end
                                        };
                                        o.free();
                                        o.next=ZPP_SimpleVert.zpp_pool;
                                        ZPP_SimpleVert.zpp_pool=o;
                                        #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT++;
                                        ZPP_SimpleVert.POOL_SUB++;
                                        #end
                                    };
                                    {
                                        var o=intx;
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                o!=null;
                                            };
                                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleEvent"+", in obj: "+"intx"+")");
                                            #end
                                        };
                                        o.free();
                                        o.next=ZPP_SimpleEvent.zpp_pool;
                                        ZPP_SimpleEvent.zpp_pool=o;
                                        #if NAPE_POOL_STATS ZPP_SimpleEvent.POOL_CNT++;
                                        ZPP_SimpleEvent.POOL_SUB++;
                                        #end
                                    };
                                }
                            }
                        }
                    };
                }
            }
            else{
                var intx=e.vertex;
                var pnull=intx.node==null;
                var a=e.segment;
                var b=e.segment2;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        (a.prev==b&&b.next==a)||(b.prev==a&&a.next==b);
                    };
                    if(!res)throw "assert("+"(a.prev==b&&b.next==a)||(b.prev==a&&a.next==b)"+") :: "+("no.!!!");
                    #end
                };
                if(b.next!=a){
                    var t=a;
                    a=b;
                    b=t;
                };
                var anew=({
                    var cur=a.vertices.parent;
                    while(cur!=null){
                        if(a.vertices.lt(intx,cur.data))cur=cur.prev;
                        else if(a.vertices.lt(cur.data,intx))cur=cur.next;
                        else break;
                    }
                    cur;
                })==null;
                var bnew=({
                    var cur=b.vertices.parent;
                    while(cur!=null){
                        if(b.vertices.lt(intx,cur.data))cur=cur.prev;
                        else if(b.vertices.lt(cur.data,intx))cur=cur.next;
                        else break;
                    }
                    cur;
                })==null;
                if(anew){
                    var aint=a.vertices.insert(intx);
                    var naleft=intx==a.left?intx:a.vertices.predecessor_node(aint).data;
                    var naright=intx==a.right?intx:a.vertices.successor_node(aint).data;
                    naleft.links.remove(naright);
                    if(intx!=naleft)naleft.links.insert(intx);
                    naright.links.remove(naleft);
                    if(intx!=naright)naright.links.insert(intx);
                    if(intx!=naleft)intx.links.insert(naleft);
                    if(intx!=naright)intx.links.insert(naright);
                }
                if(bnew){
                    var bint=b.vertices.insert(intx);
                    var nbleft=intx==b.left?intx:b.vertices.predecessor_node(bint).data;
                    var nbright=intx==b.right?intx:b.vertices.successor_node(bint).data;
                    nbleft.links.remove(nbright);
                    if(intx!=nbleft)nbleft.links.insert(intx);
                    nbright.links.remove(nbleft);
                    if(intx!=nbright)nbright.links.insert(intx);
                    if(intx!=nbleft)intx.links.insert(nbleft);
                    if(intx!=nbright)intx.links.insert(nbright);
                }
                if(pnull)intx.node=vertices.insert(intx);
                intx.forced=true;
                if(pnull){
                    var an=a.node;
                    var bn=b.node;
                    an.data=b;
                    bn.data=a;
                    a.node=bn;
                    b.node=an;
                    b.next=a.next;
                    a.next=b;
                    a.prev=b.prev;
                    b.prev=a;
                    if(a.prev!=null)a.prev.next=a;
                    if(b.next!=null)b.next.prev=b;
                }
                {
                    if(b.next!=null&&b!=null&&!(b.next.id<b.id?inthash.has(b.next.id,b.id):inthash.has(b.id,b.next.id))){
                        var intx=sweep.intersection(b.next,b);
                        if(intx!=null){
                            if(intx.vertex.x>=sweep.sweepx){
                                var ex=({
                                    var cur=queue.parent;
                                    while(cur!=null){
                                        if(queue.lt(intx,cur.data))cur=cur.prev;
                                        else if(queue.lt(cur.data,intx))cur=cur.next;
                                        else break;
                                    }
                                    cur;
                                });
                                if(ex==null){
                                    var vx=({
                                        var cur=ints.parent;
                                        while(cur!=null){
                                            if(ints.lt(intx,cur.data))cur=cur.prev;
                                            else if(ints.lt(cur.data,intx))cur=cur.next;
                                            else break;
                                        }
                                        cur;
                                    });
                                    if(vx!=null){
                                        {
                                            var o=intx.vertex;
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    o!=null;
                                                };
                                                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleVert"+", in obj: "+"intx.vertex"+")");
                                                #end
                                            };
                                            o.free();
                                            o.next=ZPP_SimpleVert.zpp_pool;
                                            ZPP_SimpleVert.zpp_pool=o;
                                            #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT++;
                                            ZPP_SimpleVert.POOL_SUB++;
                                            #end
                                        };
                                        intx.vertex=vx.data.vertex;
                                        vx.data=intx;
                                        queue.insert(intx);
                                    }
                                    else{
                                        queue.insert(intx);
                                        ints.insert(intx);
                                    }
                                    if(b.next.id<b.id)inthash.add(Hashable2_Boolfalse.get(b.next.id,b.id,true));
                                    else inthash.add(Hashable2_Boolfalse.get(b.id,b.next.id,true));
                                }
                                else{
                                    var x=ex.data;
                                    #if(!NAPE_RELEASE_BUILD)
                                    if(x.segment!=intx.segment||intx.segment2!=x.segment2)throw "corner case 2, shiiiit.";
                                    #end
                                    {
                                        var o=intx.vertex;
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                o!=null;
                                            };
                                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleVert"+", in obj: "+"intx.vertex"+")");
                                            #end
                                        };
                                        o.free();
                                        o.next=ZPP_SimpleVert.zpp_pool;
                                        ZPP_SimpleVert.zpp_pool=o;
                                        #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT++;
                                        ZPP_SimpleVert.POOL_SUB++;
                                        #end
                                    };
                                    {
                                        var o=intx;
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                o!=null;
                                            };
                                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleEvent"+", in obj: "+"intx"+")");
                                            #end
                                        };
                                        o.free();
                                        o.next=ZPP_SimpleEvent.zpp_pool;
                                        ZPP_SimpleEvent.zpp_pool=o;
                                        #if NAPE_POOL_STATS ZPP_SimpleEvent.POOL_CNT++;
                                        ZPP_SimpleEvent.POOL_SUB++;
                                        #end
                                    };
                                }
                            }
                            else{
                                {
                                    var o=intx.vertex;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o!=null;
                                        };
                                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleVert"+", in obj: "+"intx.vertex"+")");
                                        #end
                                    };
                                    o.free();
                                    o.next=ZPP_SimpleVert.zpp_pool;
                                    ZPP_SimpleVert.zpp_pool=o;
                                    #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT++;
                                    ZPP_SimpleVert.POOL_SUB++;
                                    #end
                                };
                                {
                                    var o=intx;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o!=null;
                                        };
                                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleEvent"+", in obj: "+"intx"+")");
                                        #end
                                    };
                                    o.free();
                                    o.next=ZPP_SimpleEvent.zpp_pool;
                                    ZPP_SimpleEvent.zpp_pool=o;
                                    #if NAPE_POOL_STATS ZPP_SimpleEvent.POOL_CNT++;
                                    ZPP_SimpleEvent.POOL_SUB++;
                                    #end
                                };
                            }
                        }
                    }
                };
                {
                    if(a!=null&&a.prev!=null&&!(a.id<a.prev.id?inthash.has(a.id,a.prev.id):inthash.has(a.prev.id,a.id))){
                        var intx=sweep.intersection(a,a.prev);
                        if(intx!=null){
                            if(intx.vertex.x>=sweep.sweepx){
                                var ex=({
                                    var cur=queue.parent;
                                    while(cur!=null){
                                        if(queue.lt(intx,cur.data))cur=cur.prev;
                                        else if(queue.lt(cur.data,intx))cur=cur.next;
                                        else break;
                                    }
                                    cur;
                                });
                                if(ex==null){
                                    var vx=({
                                        var cur=ints.parent;
                                        while(cur!=null){
                                            if(ints.lt(intx,cur.data))cur=cur.prev;
                                            else if(ints.lt(cur.data,intx))cur=cur.next;
                                            else break;
                                        }
                                        cur;
                                    });
                                    if(vx!=null){
                                        {
                                            var o=intx.vertex;
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    o!=null;
                                                };
                                                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleVert"+", in obj: "+"intx.vertex"+")");
                                                #end
                                            };
                                            o.free();
                                            o.next=ZPP_SimpleVert.zpp_pool;
                                            ZPP_SimpleVert.zpp_pool=o;
                                            #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT++;
                                            ZPP_SimpleVert.POOL_SUB++;
                                            #end
                                        };
                                        intx.vertex=vx.data.vertex;
                                        vx.data=intx;
                                        queue.insert(intx);
                                    }
                                    else{
                                        queue.insert(intx);
                                        ints.insert(intx);
                                    }
                                    if(a.id<a.prev.id)inthash.add(Hashable2_Boolfalse.get(a.id,a.prev.id,true));
                                    else inthash.add(Hashable2_Boolfalse.get(a.prev.id,a.id,true));
                                }
                                else{
                                    var x=ex.data;
                                    #if(!NAPE_RELEASE_BUILD)
                                    if(x.segment!=intx.segment||intx.segment2!=x.segment2)throw "corner case 2, shiiiit.";
                                    #end
                                    {
                                        var o=intx.vertex;
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                o!=null;
                                            };
                                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleVert"+", in obj: "+"intx.vertex"+")");
                                            #end
                                        };
                                        o.free();
                                        o.next=ZPP_SimpleVert.zpp_pool;
                                        ZPP_SimpleVert.zpp_pool=o;
                                        #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT++;
                                        ZPP_SimpleVert.POOL_SUB++;
                                        #end
                                    };
                                    {
                                        var o=intx;
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                o!=null;
                                            };
                                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleEvent"+", in obj: "+"intx"+")");
                                            #end
                                        };
                                        o.free();
                                        o.next=ZPP_SimpleEvent.zpp_pool;
                                        ZPP_SimpleEvent.zpp_pool=o;
                                        #if NAPE_POOL_STATS ZPP_SimpleEvent.POOL_CNT++;
                                        ZPP_SimpleEvent.POOL_SUB++;
                                        #end
                                    };
                                }
                            }
                            else{
                                {
                                    var o=intx.vertex;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o!=null;
                                        };
                                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleVert"+", in obj: "+"intx.vertex"+")");
                                        #end
                                    };
                                    o.free();
                                    o.next=ZPP_SimpleVert.zpp_pool;
                                    ZPP_SimpleVert.zpp_pool=o;
                                    #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT++;
                                    ZPP_SimpleVert.POOL_SUB++;
                                    #end
                                };
                                {
                                    var o=intx;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o!=null;
                                        };
                                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleEvent"+", in obj: "+"intx"+")");
                                        #end
                                    };
                                    o.free();
                                    o.next=ZPP_SimpleEvent.zpp_pool;
                                    ZPP_SimpleEvent.zpp_pool=o;
                                    #if NAPE_POOL_STATS ZPP_SimpleEvent.POOL_CNT++;
                                    ZPP_SimpleEvent.POOL_SUB++;
                                    #end
                                };
                            }
                        }
                    }
                };
                ints.remove(e);
            }
            {
                var o=e;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleEvent"+", in obj: "+"e"+")");
                    #end
                };
                o.free();
                o.next=ZPP_SimpleEvent.zpp_pool;
                ZPP_SimpleEvent.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_SimpleEvent.POOL_CNT++;
                ZPP_SimpleEvent.POOL_SUB++;
                #end
            };
        }
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                queue.empty();
            };
            if(!res)throw "assert("+"queue.empty()"+") :: "+("clean up fail");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                ints.empty();
            };
            if(!res)throw "assert("+"ints.empty()"+") :: "+("clean up fail");
            #end
        };
        {
            for(i in 0...inthash.table.length){
                var n=inthash.table[i];
                if(n==null)continue;
                while(n!=null){
                    var t=n.hnext;
                    n.hnext=null;
                    {
                        var o=n;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                o!=null;
                            };
                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"Hashable2_Boolfalse"+", in obj: "+"n"+")");
                            #end
                        };
                        o.free();
                        o.next=Hashable2_Boolfalse.zpp_pool;
                        Hashable2_Boolfalse.zpp_pool=o;
                        #if NAPE_POOL_STATS Hashable2_Boolfalse.POOL_CNT++;
                        Hashable2_Boolfalse.POOL_SUB++;
                        #end
                    };
                    n=t;
                }
                inthash.table[i]=null;
            }
        };
        if(rets==null)rets=new ZNPList_ZPP_GeomVert();
        while(!vertices.empty())clip_polygon(vertices,rets);
        return rets;
    }
    public static function clip_polygon(vertices:ZPP_Set_ZPP_SimpleVert,rets:ZNPList_ZPP_GeomVert){
        var ret:ZPP_GeomVert=null;
        var cur=vertices.first();
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                cur.links.size()==2;
            };
            if(!res)throw "assert("+"cur.links.size()==2"+") :: "+("damnit!");
            #end
        };
        var fst=cur;
        var pren=cur.links.parent;
        var nxtn=if(pren.prev==null)pren.next else pren.prev;
        var pre=pren.data;
        var nxt=nxtn.data;
        if(({
            var ux:Float=0.0;
            var uy:Float=0.0;
            {
                ux=cur.x-pre.x;
                uy=cur.y-pre.y;
            };
            var vx:Float=0.0;
            var vy:Float=0.0;
            {
                vx=nxt.x-cur.x;
                vy=nxt.y-cur.y;
            };
            (vy*ux-vx*uy);
        })<0)nxt=pre;
        ret={
            var obj=ZPP_GeomVert.get(cur.x,cur.y);
            if(ret==null)ret=obj.prev=obj.next=obj;
            else{
                obj.prev=ret;
                obj.next=ret.next;
                ret.next.prev=obj;
                ret.next=obj;
            }
            obj;
        };
        ret.forced=cur.forced;
        while(true){
            cur.links.remove(nxt);
            nxt.links.remove(cur);
            if(nxt==fst){
                if(cur.links.empty()){
                    vertices.remove(cur);
                    {
                        var o=cur;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                o!=null;
                            };
                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleVert"+", in obj: "+"cur"+")");
                            #end
                        };
                        o.free();
                        o.next=ZPP_SimpleVert.zpp_pool;
                        ZPP_SimpleVert.zpp_pool=o;
                        #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT++;
                        ZPP_SimpleVert.POOL_SUB++;
                        #end
                    };
                };
                break;
            }
            ret={
                var obj=ZPP_GeomVert.get(nxt.x,nxt.y);
                if(ret==null)ret=obj.prev=obj.next=obj;
                else{
                    obj.prev=ret;
                    obj.next=ret.next;
                    ret.next.prev=obj;
                    ret.next=obj;
                }
                obj;
            };
            ret.forced=nxt.forced;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !nxt.links.empty();
                };
                if(!res)throw "assert("+"!nxt.links.empty()"+") :: "+(".. no where left to go?");
                #end
            };
            if(nxt.links.singular()){
                if(cur.links.empty()){
                    vertices.remove(cur);
                    {
                        var o=cur;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                o!=null;
                            };
                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleVert"+", in obj: "+"cur"+")");
                            #end
                        };
                        o.free();
                        o.next=ZPP_SimpleVert.zpp_pool;
                        ZPP_SimpleVert.zpp_pool=o;
                        #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT++;
                        ZPP_SimpleVert.POOL_SUB++;
                        #end
                    };
                };
                cur=nxt;
                nxt=nxt.links.parent.data;
            }
            else{
                var min=null;
                var minl:Float=0.0;
                {
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            nxt.links!=null;
                        };
                        if(!res)throw "assert("+"nxt.links!=null"+") :: "+("Iterate  null set?");
                        #end
                    };
                    if(!nxt.links.empty()){
                        var set_ite=nxt.links.parent;
                        while(set_ite.prev!=null)set_ite=set_ite.prev;
                        while(set_ite!=null){
                            var p=set_ite.data;
                            {
                                if(min==null){
                                    min=p;
                                    minl=({
                                        var ux:Float=0.0;
                                        var uy:Float=0.0;
                                        {
                                            ux=nxt.x-cur.x;
                                            uy=nxt.y-cur.y;
                                        };
                                        var vx:Float=0.0;
                                        var vy:Float=0.0;
                                        {
                                            vx=p.x-nxt.x;
                                            vy=p.y-nxt.y;
                                        };
                                        (vy*ux-vx*uy);
                                    });
                                }
                                else{
                                    var nleft=({
                                        var ux:Float=0.0;
                                        var uy:Float=0.0;
                                        {
                                            ux=nxt.x-cur.x;
                                            uy=nxt.y-cur.y;
                                        };
                                        var vx:Float=0.0;
                                        var vy:Float=0.0;
                                        {
                                            vx=p.x-nxt.x;
                                            vy=p.y-nxt.y;
                                        };
                                        (vy*ux-vx*uy);
                                    });
                                    if(nleft>0&&minl<=0){
                                        min=p;
                                        minl=nleft;
                                    }
                                    else if(minl*nleft>=0){
                                        var pleft=({
                                            var ux:Float=0.0;
                                            var uy:Float=0.0;
                                            {
                                                ux=nxt.x-p.x;
                                                uy=nxt.y-p.y;
                                            };
                                            var vx:Float=0.0;
                                            var vy:Float=0.0;
                                            {
                                                vx=min.x-nxt.x;
                                                vy=min.y-nxt.y;
                                            };
                                            (vy*ux-vx*uy);
                                        });
                                        if(pleft>0){
                                            min=p;
                                            minl=nleft;
                                        }
                                    }
                                }
                            };
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
                if(cur.links.empty()){
                    vertices.remove(cur);
                    {
                        var o=cur;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                o!=null;
                            };
                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleVert"+", in obj: "+"cur"+")");
                            #end
                        };
                        o.free();
                        o.next=ZPP_SimpleVert.zpp_pool;
                        ZPP_SimpleVert.zpp_pool=o;
                        #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT++;
                        ZPP_SimpleVert.POOL_SUB++;
                        #end
                    };
                };
                cur=nxt;
                nxt=min;
            }
        }
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                fst.links.empty();
            };
            if(!res)throw "assert("+"fst.links.empty()"+") :: "+("hmm, shouldn't have any left");
            #end
        };
        {
            vertices.remove(fst);
            {
                var o=fst;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleVert"+", in obj: "+"fst"+")");
                    #end
                };
                o.free();
                o.next=ZPP_SimpleVert.zpp_pool;
                ZPP_SimpleVert.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT++;
                ZPP_SimpleVert.POOL_SUB++;
                #end
            };
        };
        rets.add(ret);
    }
    static var list_vertices:ZNPList_ZPP_SimpleVert=null;
    static var list_queue:ZNPList_ZPP_SimpleEvent=null;
    public static function isSimple(poly:ZPP_GeomVert){
        if(sweep==null){
            sweep=new ZPP_SimpleSweep();
            inthash=new FastHash2_Hashable2_Boolfalse();
        }
        var vertices=list_vertices;
        if(vertices==null)vertices=list_vertices=new ZNPList_ZPP_SimpleVert();
        {
            var F=poly;
            var L=poly;
            if(F!=null){
                var nite=F;
                do{
                    var v=nite;
                    {
                        {
                            vertices.add(ZPP_SimpleVert.get(v.x,v.y));
                        };
                    }
                    nite=nite.next;
                }
                while(nite!=L);
            }
        };
        var queue=list_queue;
        if(queue==null)queue=list_queue=new ZNPList_ZPP_SimpleEvent();
        {
            var cx_ite=vertices.begin();
            var u=cx_ite.elem();
            cx_ite=cx_ite.next;
            while(cx_ite!=null){
                var v=cx_ite.elem();
                {
                    var e1=queue.add(ZPP_SimpleEvent.get(u));
                    var e2=queue.add(ZPP_SimpleEvent.get(v));
                    e1.segment=e2.segment=if(ZPP_SimpleEvent.less_xy(e1,e2)){
                        e1.type=1;
                        e2.type=2;
                        ZPP_SimpleSeg.get(u,v);
                    }
                    else{
                        e1.type=2;
                        e2.type=1;
                        ZPP_SimpleSeg.get(v,u);
                    }
                };
                u=v;
                cx_ite=cx_ite.next;
            }
            var v=vertices.front();
            {
                var e1=queue.add(ZPP_SimpleEvent.get(u));
                var e2=queue.add(ZPP_SimpleEvent.get(v));
                e1.segment=e2.segment=if(ZPP_SimpleEvent.less_xy(e1,e2)){
                    e1.type=1;
                    e2.type=2;
                    ZPP_SimpleSeg.get(u,v);
                }
                else{
                    e1.type=2;
                    e2.type=1;
                    ZPP_SimpleSeg.get(v,u);
                }
            };
        };
        {
            var xxlist=queue;
            if(!xxlist.empty()&&xxlist.begin().next!=null){
                var head:ZNPNode_ZPP_SimpleEvent=xxlist.begin();
                var tail:ZNPNode_ZPP_SimpleEvent=null;
                var left:ZNPNode_ZPP_SimpleEvent=null;
                var right:ZNPNode_ZPP_SimpleEvent=null;
                var nxt:ZNPNode_ZPP_SimpleEvent=null;
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
                            else if(ZPP_SimpleEvent.less_xy(left.elem(),right.elem())){
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
        var ret=true;
        while(!queue.empty()){
            var e=queue.pop_unsafe();
            var seg=e.segment;
            if(e.type==1){
                sweep.add(seg);
                if(sweep.intersect(seg,seg.next)||sweep.intersect(seg,seg.prev)){
                    ret=false;
                    break;
                }
            }
            else if(e.type==2){
                if(sweep.intersect(seg.prev,seg.next)){
                    ret=false;
                    break;
                }
                sweep.remove(seg);
                {
                    var o=seg;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            o!=null;
                        };
                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleSeg"+", in obj: "+"seg"+")");
                        #end
                    };
                    o.free();
                    o.next=ZPP_SimpleSeg.zpp_pool;
                    ZPP_SimpleSeg.zpp_pool=o;
                    #if NAPE_POOL_STATS ZPP_SimpleSeg.POOL_CNT++;
                    ZPP_SimpleSeg.POOL_SUB++;
                    #end
                };
            }
            {
                var o=e;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleEvent"+", in obj: "+"e"+")");
                    #end
                };
                o.free();
                o.next=ZPP_SimpleEvent.zpp_pool;
                ZPP_SimpleEvent.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_SimpleEvent.POOL_CNT++;
                ZPP_SimpleEvent.POOL_SUB++;
                #end
            };
        }
        while(!queue.empty()){
            var e=queue.pop_unsafe();
            if(e.type==2){
                var o=e.segment;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleSeg"+", in obj: "+"e.segment"+")");
                    #end
                };
                o.free();
                o.next=ZPP_SimpleSeg.zpp_pool;
                ZPP_SimpleSeg.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_SimpleSeg.POOL_CNT++;
                ZPP_SimpleSeg.POOL_SUB++;
                #end
            };
            {
                var o=e;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleEvent"+", in obj: "+"e"+")");
                    #end
                };
                o.free();
                o.next=ZPP_SimpleEvent.zpp_pool;
                ZPP_SimpleEvent.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_SimpleEvent.POOL_CNT++;
                ZPP_SimpleEvent.POOL_SUB++;
                #end
            };
        }
        sweep.clear();
        while(!vertices.empty()){
            var o=vertices.pop_unsafe();
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SimpleVert"+", in obj: "+"vertices.pop_unsafe()"+")");
                #end
            };
            o.free();
            o.next=ZPP_SimpleVert.zpp_pool;
            ZPP_SimpleVert.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_CNT++;
            ZPP_SimpleVert.POOL_SUB++;
            #end
        };
        return ret;
    }
}
