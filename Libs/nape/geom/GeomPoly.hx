package nape.geom;
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
/**
 * Polygon class with various geometric methods
 * <br/><br/>
 * This class represents a general Polygon, rather than the Polygon class
 * which is physics shape.
 * <br/><br/>
 * Internally this polygon is stored as a circularly linked list of special
 * vertex types that are exposed via a Vec2 that is lazily constructed whenever
 * necessary to the API.
 */
@:final#if nape_swc@:keep #end
class GeomPoly{
    /**
     * @private
     */
    public var zpp_pool:GeomPoly=null;
    #if(!NAPE_RELEASE_BUILD)
    /**
     * @private
     */
    public var zpp_disp:Bool;
    #end
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
    
    /**
     * @private
     */
    public var zpp_inner:ZPP_GeomPoly=null;
    /**
     * Determine if polygon is empty.
     *
     * @return True if polygon is empty.
     * @throws # If this GeomPoly has been disposed.
     */
    public#if NAPE_NO_INLINE#else inline #end
    function empty():Bool{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        return(zpp_inner.vertices==null);
    }
    /**
     * Determine number of vertices in polygon
     *
     * @return The number of vertices.
     * @throws # If this GeomPoly has been disposed.
     */
    public function size():Int{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        return({
            var ret=0;
            {
                var F=zpp_inner.vertices;
                var L=zpp_inner.vertices;
                if(F!=null){
                    var nite=F;
                    do{
                        var i=nite;
                        {
                            ret++;
                        }
                        nite=nite.next;
                    }
                    while(nite!=L);
                }
            };
            ret;
        });
    }
    /**
     * Haxe iterator over vertices of polygon.
     *
     * @return A Haxe iterator over the vertices of the polygon.
     * @throws # If this GeomPoly has been disposed.
     */
    public#if NAPE_NO_INLINE#else inline #end
    function iterator():GeomVertexIterator{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        return ZPP_GeomVertexIterator.get(zpp_inner.vertices,true);
    }
    /**
     * Haxe iterator over vertices of polygon.
     *
     * @return A Haxe iterator over the vertices of the polygon.
     *         Iterating in a forward direction.
     * @throws # If this GeomPoly has been disposed.
     */
    public#if NAPE_NO_INLINE#else inline #end
    function forwardIterator():GeomVertexIterator{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        return ZPP_GeomVertexIterator.get(zpp_inner.vertices,true);
    }
    /**
     * Haxe iterator over vertices of polygon.
     *
     * @return A Haxe iterator over the vertices of the polygon.
     *         Iterating in a backwards direction.
     * @throws # If this GeomPoly has been disposed.
     */
    public#if NAPE_NO_INLINE#else inline #end
    function backwardsIterator():GeomVertexIterator{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        return ZPP_GeomVertexIterator.get(zpp_inner.vertices,false);
    }
    /**
     * Current vertex at head of polygon.
     * <br/><br/>
     * The current vertex will not be changed by this access.
     * <br/><br/>
     * This function returns a Vec2 which will be intrinsically tied
     * to the values of the internal vertex so that modifications to
     * this Vec2 will be reflected in the vertex of the polygon.
     * <br/><br/>
     * If invoked again with the head of the polygon pointing to the
     * same vertex, then the same Vec2 will be returned; this Vec2 is
     * not able to be disposed of.
     *
     * @return A Vec2 representing the current vertex of polygon.
     * @throws # If polygon is empty.
     * @throws # If this GeomPoly has been disposed.
     */
    public#if NAPE_NO_INLINE#else inline #end
    function current():Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(empty()){
            throw "Error: GeomPoly is empty";
        }
        #end
        return zpp_inner.vertices.wrapper();
    }
    /**
     * Push vertex to polygon.
     * <br/><br/>
     * A vertex will be allocated from a global object pool, and initialised
     * with the values of the given Vec2.
     * <br/><br/>
     * This vertex will be inserted after the current head, and the head
     * advanced to the newly inserted vertex, in this way successive pushes
     * will insert elements in order.
     * <br/><br/>
     * Note that the Vec2 supplied as argument is only used to initialise the
     * inner Vertex.
     * <pre>
     * poly := -> A <-> B <-> C <-> D <-> E <-
     *                (head)
     *
     * poly.push(X);
     *
     * poly := -> A <-> B <-> X <-> C <-> D <-> E <-
     *                      (head)
     * </pre>
     *
     * @param vertex The Vec2 to be used in initialising the inner vertex.
     * @return A reference to this polygon.
     * @throws # If Vec2 is null, or has been disposed.
     * @throws # If this GeomPoly has been disposed.
     */
    public function push(vertex:Vec2):GeomPoly{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(vertex!=null&&vertex.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(vertex==null){
            throw "Error: Cannot push null vertex";
        }
        #end
        zpp_inner.vertices={
            var obj=ZPP_GeomVert.get(vertex.x,vertex.y);
            if(zpp_inner.vertices==null)zpp_inner.vertices=obj.prev=obj.next=obj;
            else{
                obj.prev=zpp_inner.vertices;
                obj.next=zpp_inner.vertices.next;
                zpp_inner.vertices.next.prev=obj;
                zpp_inner.vertices.next=obj;
            }
            obj;
        };
        ({
            if(({
                vertex.zpp_inner.weak;
            })){
                vertex.dispose();
                true;
            }
            else{
                false;
            }
        });
        return this;
    }
    /**
     * Pop vertex from polygon.
     * <br/><br/>
     * Pop the current vertex at head of polygon, retreating the 'current'
     * vertex to point to the previous vertex in polygon. This inner vertex
     * will be released to the global object pool.
     * <br/><br/>
     * In this way a pop which follows a push will act to reset the push.
     * <pre>
     * poly := -> A <-> B <-> C <-> D <-> E <-
     *                (head)
     *
     * poly.pop();
     *
     * poly := -> A <-> C <-> D <-> E <-
     *          (head)
     * </pre>
     *
     * @return A reference to this polygon.
     * @throws # If polygon is empty.
     * @throws # If this GeomPoly has been disposed.
     */
    public function pop():GeomPoly{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if((zpp_inner.vertices==null)){
            throw "Error: Cannot pop from empty polygon";
        }
        #end
        var retv=zpp_inner.vertices;
        zpp_inner.vertices={
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !(zpp_inner.vertices==null);
                };
                if(!res)throw "assert("+"!(zpp_inner.vertices==null)"+") :: "+("can't pop from empty list derpiderp");
                #end
            };
            if((zpp_inner.vertices!=null&&zpp_inner.vertices.prev==zpp_inner.vertices)){
                zpp_inner.vertices.next=zpp_inner.vertices.prev=null;
                {};
                null;
            }
            else{
                var retnodes=zpp_inner.vertices.prev;
                zpp_inner.vertices.prev.next=zpp_inner.vertices.next;
                zpp_inner.vertices.next.prev=zpp_inner.vertices.prev;
                zpp_inner.vertices.next=zpp_inner.vertices.prev=null;
                {};
                zpp_inner.vertices=null;
                retnodes;
            }
        };
        {
            var o=retv;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"retv"+")");
                #end
            };
            o.free();
            o.next=ZPP_GeomVert.zpp_pool;
            ZPP_GeomVert.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
            ZPP_GeomVert.POOL_SUB++;
            #end
        };
        return this;
    }
    /**
     * Unshift vertex to polygon.
     * <br/><br/>
     * A vertex will be allocated from a global object pool, and initialised
     * with the values of the given Vec2.
     * <br/><br/>
     * This vertex will be inserted before the current head, and the head
     * retreated to the newly inserted vertex, in this way successive unshifts
     * will insert elements in the expected reverse order.
     * <br/><br/>
     * Note that the Vec2 supplied as argument is only used to initialise the
     * inner Vertex.
     * <pre>
     * poly := -> A <-> B <-> C <-> D <-> E <-
     *                (head)
     *
     * poly.unshift(X);
     *
     * poly := -> A <-> X <-> B <-> C <-> D <-> E <-
     *                (head)
     * </pre>
     *
     * @param vertex The Vec2 to be used in initialising the inner vertex.
     * @return A reference to this polygon.
     * @throws # If Vec2 is null, or has been disposed.
     * @throws # If this GeomPoly has been disposed.
     */
    public function unshift(vertex:Vec2):GeomPoly{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(vertex!=null&&vertex.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(vertex==null){
            throw "Error: Cannot unshift null vertex";
        }
        #end
        zpp_inner.vertices={
            var obj=ZPP_GeomVert.get(vertex.x,vertex.y);
            if(zpp_inner.vertices==null)zpp_inner.vertices=obj.prev=obj.next=obj;
            else{
                obj.next=zpp_inner.vertices;
                obj.prev=zpp_inner.vertices.prev;
                zpp_inner.vertices.prev.next=obj;
                zpp_inner.vertices.prev=obj;
            }
            obj;
        };
        ({
            if(({
                vertex.zpp_inner.weak;
            })){
                vertex.dispose();
                true;
            }
            else{
                false;
            }
        });
        return this;
    }
    /**
     * Shift vertex from polygon.
     * <br/><br/>
     * Shift the current vertex at head of polygon, advancing the 'current'
     * vertex to point to the next vertex in polygon. This inner vertex
     * will be released to the global object pool.
     * <br/><br/>
     * In this way a shift which follows an unshift will act to reset the
     * unshift operation.
     * <pre>
     * poly := -> A <-> B <-> C <-> D <-> E <-
     *                (head)
     *
     * poly.shift();
     *
     * poly := -> A <-> C <-> D <-> E <-
     *                (head)
     * </pre>
     *
     * @return A reference to this polygon.
     * @throws # If polygon is empty.
     * @throws # If this GeomPoly has been disposed.
     */
    public function shift():GeomPoly{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if((zpp_inner.vertices==null)){
            throw "Error: Cannot shift from empty polygon";
        }
        #end
        var retv=zpp_inner.vertices;
        zpp_inner.vertices={
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !(zpp_inner.vertices==null);
                };
                if(!res)throw "assert("+"!(zpp_inner.vertices==null)"+") :: "+("can't pop from empty list herpaderp");
                #end
            };
            if((zpp_inner.vertices!=null&&zpp_inner.vertices.prev==zpp_inner.vertices)){
                zpp_inner.vertices.next=zpp_inner.vertices.prev=null;
                {};
                zpp_inner.vertices=null;
            }
            else{
                var retnodes=zpp_inner.vertices.next;
                zpp_inner.vertices.prev.next=zpp_inner.vertices.next;
                zpp_inner.vertices.next.prev=zpp_inner.vertices.prev;
                zpp_inner.vertices.next=zpp_inner.vertices.prev=null;
                {};
                zpp_inner.vertices=null;
                retnodes;
            }
        };
        {
            var o=retv;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"retv"+")");
                #end
            };
            o.free();
            o.next=ZPP_GeomVert.zpp_pool;
            ZPP_GeomVert.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
            ZPP_GeomVert.POOL_SUB++;
            #end
        };
        return this;
    }
    /**
     * Advance head of polygon forward.
     * <br/><br/>
     * The current head of polygon will be moved forwards
     * the given number of times, with a negative value
     * being equivalent to performing a backwards advance.
     * <br/><br/>
     * <code>poly.skip_forwards(times)</code> is equivalent to
     * <code>poly.skip_backwards(-times)</code>
     * <pre>
     * poly := -> A <-> B <-> C <-> D <-> E <-
     *                (head)
     *
     * poly.skipForwards(2);
     *
     * poly := -> A <-> B <-> C <-> D <-> E <-
     *                            (head)
     * </pre>
     *
     * @params times The number of times to advance head forward.
     *               This value can be negative indicating a backwards
     *               advance.
     * @return A reference to this polygon.
     * @throws # If this GeomPoly has been disposed.
     */
    public function skipForward(times:Int):GeomPoly{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        if(!(zpp_inner.vertices==null)){
            if(times>0){
                while(times-->0)zpp_inner.vertices=zpp_inner.vertices.next;
            }
            else if(times<0){
                while(times++<0)zpp_inner.vertices=zpp_inner.vertices.prev;
            }
        }
        return this;
    }
    /**
     * Advance head of polygon backwards.
     * <br/><br/>
     * The current head of polygon will be moved backwards
     * the given number of times, with a negative value
     * being equivalent to performing a forwards advance.
     * <br/><br/>
     * <code>poly.skip_backwards(times)</code> is equivalent to
     * <code>poly.skip_forwards(-times)</code>
     * <pre>
     * poly := -> A <-> B <-> C <-> D <-> E <-
     *                (head)
     *
     * poly.skipBackwards(2);
     *
     * poly := -> A <-> B <-> C <-> D <-> E <-
     *                                  (head)
     * </pre>
     *
     * @params times The number of times to advance head backwards.
     *               This value can be negative indicating a forwards
     *               advance.
     * @return A reference to this polygon.
     * @throws # If this GeomPoly has been disposed.
     */
    public#if NAPE_NO_INLINE#else inline #end
    function skipBackwards(times:Int){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        return skipForward(-times);
    }
    /**
     * Erase count number of elements
     * <br/><br/>
     * For positive values of count, this is equivalent to successive
     * unshift operations.
     * <br/><br/>
     * For negative values of count, this is equivalent to successive
     * pop operations.
     * <pre>
     * poly := -> A <-> B <-> C <-> D <-> E <-> F <-> G <-
     *                (head)
     *
     * poly.erase(2);
     *
     * poly := -> A <-> D <-> E <-> F <-> G <-
     *                (head)
     *
     * poly.erase(-3);
     *
     * poly := -> E <-> F <-
     *                (head)
     * </pre>
     * In this case that the specified number of elements to erase is
     * greater than the size of the polygon, the method will simply
     * terminate with the polygon being empty.
     *
     * @param count The number of vertices to erase, with sign indicating
     *              the direction for erasing.
     * @return A reference to this polygon.
     * @throws # If this GeomPoly has been disposed.
     */
    public function erase(count:Int):GeomPoly{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        while(count!=0&&!(zpp_inner.vertices==null)){
            var retv=zpp_inner.vertices;
            if(count>0){
                zpp_inner.vertices={
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !(zpp_inner.vertices==null);
                        };
                        if(!res)throw "assert("+"!(zpp_inner.vertices==null)"+") :: "+("can't pop from empty list herpaderp");
                        #end
                    };
                    if((zpp_inner.vertices!=null&&zpp_inner.vertices.prev==zpp_inner.vertices)){
                        zpp_inner.vertices.next=zpp_inner.vertices.prev=null;
                        {};
                        zpp_inner.vertices=null;
                    }
                    else{
                        var retnodes=zpp_inner.vertices.next;
                        zpp_inner.vertices.prev.next=zpp_inner.vertices.next;
                        zpp_inner.vertices.next.prev=zpp_inner.vertices.prev;
                        zpp_inner.vertices.next=zpp_inner.vertices.prev=null;
                        {};
                        zpp_inner.vertices=null;
                        retnodes;
                    }
                };
                count--;
            }
            else if(count<0){
                zpp_inner.vertices={
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !(zpp_inner.vertices==null);
                        };
                        if(!res)throw "assert("+"!(zpp_inner.vertices==null)"+") :: "+("can't pop from empty list derpiderp");
                        #end
                    };
                    if((zpp_inner.vertices!=null&&zpp_inner.vertices.prev==zpp_inner.vertices)){
                        zpp_inner.vertices.next=zpp_inner.vertices.prev=null;
                        {};
                        null;
                    }
                    else{
                        var retnodes=zpp_inner.vertices.prev;
                        zpp_inner.vertices.prev.next=zpp_inner.vertices.next;
                        zpp_inner.vertices.next.prev=zpp_inner.vertices.prev;
                        zpp_inner.vertices.next=zpp_inner.vertices.prev=null;
                        {};
                        zpp_inner.vertices=null;
                        retnodes;
                    }
                };
                count++;
            }
            {
                var o=retv;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"retv"+")");
                    #end
                };
                o.free();
                o.next=ZPP_GeomVert.zpp_pool;
                ZPP_GeomVert.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
                ZPP_GeomVert.POOL_SUB++;
                #end
            };
        }
        return this;
    }
    /**
     * Clear all vertices from polygon.
     * <br/><br/>
     * All of the vertices will be released to the global object pool.
     *
     * @return A reference to this polygon.
     * @throws # If this GeomPoly has been disposed.
     */
    public function clear():GeomPoly{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        {
            while(!(zpp_inner.vertices==null)){
                var tmp=zpp_inner.vertices;
                zpp_inner.vertices={
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !(zpp_inner.vertices==null);
                        };
                        if(!res)throw "assert("+"!(zpp_inner.vertices==null)"+") :: "+("can't pop from empty list herpaderp");
                        #end
                    };
                    if((zpp_inner.vertices!=null&&zpp_inner.vertices.prev==zpp_inner.vertices)){
                        zpp_inner.vertices.next=zpp_inner.vertices.prev=null;
                        {};
                        zpp_inner.vertices=null;
                    }
                    else{
                        var retnodes=zpp_inner.vertices.next;
                        zpp_inner.vertices.prev.next=zpp_inner.vertices.next;
                        zpp_inner.vertices.next.prev=zpp_inner.vertices.prev;
                        zpp_inner.vertices.next=zpp_inner.vertices.prev=null;
                        {};
                        zpp_inner.vertices=null;
                        retnodes;
                    }
                };
                {
                    var o=tmp;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            o!=null;
                        };
                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"tmp"+")");
                        #end
                    };
                    o.free();
                    o.next=ZPP_GeomVert.zpp_pool;
                    ZPP_GeomVert.zpp_pool=o;
                    #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
                    ZPP_GeomVert.POOL_SUB++;
                    #end
                };
            }
        };
        return this;
    }
    /**
     * Copy this polygon.
     * <br/><br/>
     * The copy will have its vertices in the same order as 'this' polygon.
     * It will also have its current vertex at head, as the same vertex
     * this polygon has.
     * <br/><br/>
     * This polygon will not be modified in any way.
     * <pre>
     * poly := -> A <-> B <-> C <-> D <-> E <-
     *                (head)
     *
     * poly2 = poly.copy();
     *
     * poly2 := -> A' <-> B' <-> C' <-> D' <-> E' <-
     *                  (head)
     * </pre>
     *
     * @return The new GeomPoly representing the copy.
     * @throws # If this GeomPoly has been disposed.
     */
    public function copy():GeomPoly{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        var ret=GeomPoly.get();
        {
            var F=zpp_inner.vertices;
            var L=zpp_inner.vertices;
            if(F!=null){
                var nite=F;
                do{
                    var v=nite;
                    {
                        {
                            ret.zpp_inner.vertices={
                                var obj=ZPP_GeomVert.get(v.x,v.y);
                                if(ret.zpp_inner.vertices==null)ret.zpp_inner.vertices=obj.prev=obj.next=obj;
                                else{
                                    obj.prev=ret.zpp_inner.vertices;
                                    obj.next=ret.zpp_inner.vertices.next;
                                    ret.zpp_inner.vertices.next.prev=obj;
                                    ret.zpp_inner.vertices.next=obj;
                                }
                                obj;
                            };
                        };
                    }
                    nite=nite.next;
                }
                while(nite!=L);
            }
        };
        return ret.skipForward(1);
    }
    /**
     * Create a new GeomPoly polygon.
     * <br/><br/>
     * The vertices argument is typed Dynamic (* in AS3), and is permitted
     * to be one of: <code>Array&lt;Vec2&gt;, flash.Vector&lt;Vec2&gt;, Vec2List, GeomPoly</code>
     * <br/><br/>
     * The input will be used to initialise the vertices of the polygon with
     * the head of the polygon pointing to the first vertex in input with vertices
     * inserted in forward order.
     * <br/><br/>
     * You should use the static 'get' method in preference to make use of object pool.
     *
     * @param vertices Vertex data to initialise polygon, or null for empty polygon.
     * @return New GeomPoly representing input vertex data.
     * @throws # If input data is not of an expected Type.
     */
    public function new(vertices:Dynamic=null){
        zpp_inner=new ZPP_GeomPoly(this);
        if(vertices!=null){
            {
                if(#if flash untyped __is__(vertices,Array)#else Std.is(vertices,Array)#end){
                    var lv:Array<Dynamic>=vertices;
                    for(vite in lv){
                        #if(!NAPE_RELEASE_BUILD)
                        if(vite==null)throw "Error: Array<Vec2> contains null objects";
                        #end
                        #if(!NAPE_RELEASE_BUILD)
                        if(!#if flash untyped __is__(vite,Vec2)#else Std.is(vite,Vec2)#end)throw "Error: Array<Vec2> contains non Vec2 objects";
                        #end
                        var v:Vec2=vite;
                        {
                            #if(!NAPE_RELEASE_BUILD)
                            if(v!=null&&v.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                            #end
                        };
                        {
                            zpp_inner.vertices={
                                var obj=ZPP_GeomVert.get(v.x,v.y);
                                if(zpp_inner.vertices==null)zpp_inner.vertices=obj.prev=obj.next=obj;
                                else{
                                    obj.prev=zpp_inner.vertices;
                                    obj.next=zpp_inner.vertices.next;
                                    zpp_inner.vertices.next.prev=obj;
                                    zpp_inner.vertices.next=obj;
                                }
                                obj;
                            };
                        };
                    }
                }
                else if(#if flash10 untyped __is__(vertices,ZPP_Const.vec2vector)#else false #end){
                    #if flash10 var lv:flash.Vector<Vec2>=vertices;
                    for(vite in lv){
                        #if(!NAPE_RELEASE_BUILD)
                        if(vite==null)throw "Error: flash.Vector<Vec2> contains null objects";
                        #end
                        var v:Vec2=vite;
                        {
                            #if(!NAPE_RELEASE_BUILD)
                            if(v!=null&&v.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                            #end
                        };
                        {
                            zpp_inner.vertices={
                                var obj=ZPP_GeomVert.get(v.x,v.y);
                                if(zpp_inner.vertices==null)zpp_inner.vertices=obj.prev=obj.next=obj;
                                else{
                                    obj.prev=zpp_inner.vertices;
                                    obj.next=zpp_inner.vertices.next;
                                    zpp_inner.vertices.next.prev=obj;
                                    zpp_inner.vertices.next=obj;
                                }
                                obj;
                            };
                        };
                    }
                    #end
                }
                else if(#if flash untyped __is__(vertices,Vec2List)#else Std.is(vertices,Vec2List)#end){
                    var lv:Vec2List=vertices;
                    for(v in lv){
                        #if(!NAPE_RELEASE_BUILD)
                        if(v==null)throw "Error: Vec2List contains null objects";
                        #end
                        {
                            #if(!NAPE_RELEASE_BUILD)
                            if(v!=null&&v.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                            #end
                        };
                        {
                            zpp_inner.vertices={
                                var obj=ZPP_GeomVert.get(v.x,v.y);
                                if(zpp_inner.vertices==null)zpp_inner.vertices=obj.prev=obj.next=obj;
                                else{
                                    obj.prev=zpp_inner.vertices;
                                    obj.next=zpp_inner.vertices.next;
                                    zpp_inner.vertices.next.prev=obj;
                                    zpp_inner.vertices.next=obj;
                                }
                                obj;
                            };
                        };
                    }
                }
                else if(#if flash untyped __is__(vertices,GeomPoly)#else Std.is(vertices,GeomPoly)#end){
                    var lv:GeomPoly=vertices;
                    {
                        #if(!NAPE_RELEASE_BUILD)
                        if(lv!=null&&lv.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
                        #end
                    };
                    var verts:ZPP_GeomVert=lv.zpp_inner.vertices;
                    if(verts!=null){
                        var vite=verts;
                        do{
                            var v=Vec2.get(vite.x,vite.y);
                            vite=vite.next;
                            {
                                zpp_inner.vertices={
                                    var obj=ZPP_GeomVert.get(v.x,v.y);
                                    if(zpp_inner.vertices==null)zpp_inner.vertices=obj.prev=obj.next=obj;
                                    else{
                                        obj.prev=zpp_inner.vertices;
                                        obj.next=zpp_inner.vertices.next;
                                        zpp_inner.vertices.next.prev=obj;
                                        zpp_inner.vertices.next=obj;
                                    }
                                    obj;
                                };
                            };
                            v.dispose();
                        }
                        while(vite!=verts);
                    }
                }
                else{
                    #if(!NAPE_RELEASE_BUILD)
                    throw "Error: Invalid type for polygon object, should be Array<Vec2>, Vec2List, GeomPoly or for flash10+ flash.Vector<Vec2>";
                    #end
                }
            };
            skipForward(1);
            {
                if(#if flash untyped __is__(vertices,Array)#else Std.is(vertices,Array)#end){
                    var lv:Array<Vec2>=vertices;
                    var i=0;
                    while(i<lv.length){
                        var cur=lv[i];
                        if(({
                            if(({
                                cur.zpp_inner.weak;
                            })){
                                cur.dispose();
                                true;
                            }
                            else{
                                false;
                            }
                        })){
                            lv.splice(i,1);
                            continue;
                        }
                        i++;
                    }
                }
                else if(#if flash10 untyped __is__(vertices,ZPP_Const.vec2vector)#else false #end){
                    #if flash10 var lv:flash.Vector<Vec2>=vertices;
                    if(!lv.fixed){
                        var i:Int=0;
                        while(i<cast lv.length){
                            var cur=lv[i];
                            if(({
                                if(({
                                    cur.zpp_inner.weak;
                                })){
                                    cur.dispose();
                                    true;
                                }
                                else{
                                    false;
                                }
                            })){
                                lv.splice(i,1);
                                continue;
                            }
                            i++;
                        }
                    }
                    #end
                }
                else if(#if flash untyped __is__(vertices,Vec2List)#else Std.is(vertices,Vec2List)#end){
                    var lv:Vec2List=vertices;
                    if(lv.zpp_inner._validate!=null)lv.zpp_inner._validate();
                    var ins=lv.zpp_inner.inner;
                    var pre=null;
                    var cur=ins.begin();
                    while(cur!=null){
                        var x=cur.elem();
                        if(({
                            x.outer.zpp_inner.weak;
                        })){
                            cur=ins.erase(pre);
                            ({
                                if(({
                                    x.outer.zpp_inner.weak;
                                })){
                                    x.outer.dispose();
                                    true;
                                }
                                else{
                                    false;
                                }
                            });
                        }
                        else{
                            pre=cur;
                            cur=cur.next;
                        }
                    }
                }
            };
        }
    }
    /**
     * Allocate GeomPoly from object pool.
     * <br/><br/>
     * The vertices argument is typed Dynamic (* in AS3), and is permitted
     * to be one of: <code>Array&lt;Vec2&gt;, flash.Vector&lt;Vec2&gt;, Vec2List, GeomPoly</code>
     * <br/><br/>
     * The input will be used to initialise the vertices of the polygon with
     * the head of the polygon pointing to the first vertex in input with vertices
     * inserted in forward order.
     *
     * @param vertices Vertex data to initialise polygon, or null for empty polygon.
     * @return New GeomPoly representing input vertex data, allocated from object pool.
     * @throws # If input data is not of an expected Type.
     */
    public static function get(vertices:Dynamic=null){
        var ret;
        {
            if(ZPP_PubPool.poolGeomPoly==null){
                ret=new GeomPoly();
                #if NAPE_POOL_STATS GeomPoly.POOL_TOT++;
                GeomPoly.POOL_ADDNEW++;
                #end
            }
            else{
                ret=ZPP_PubPool.poolGeomPoly;
                ZPP_PubPool.poolGeomPoly=ret.zpp_pool;
                ret.zpp_pool=null;
                #if(!NAPE_RELEASE_BUILD)
                ret.zpp_disp=false;
                if(ret==ZPP_PubPool.nextGeomPoly)ZPP_PubPool.nextGeomPoly=null;
                #end
                #if NAPE_POOL_STATS GeomPoly.POOL_CNT--;
                GeomPoly.POOL_ADD++;
                #end
            }
        };
        if(vertices!=null){
            {
                if(#if flash untyped __is__(vertices,Array)#else Std.is(vertices,Array)#end){
                    var lv:Array<Dynamic>=vertices;
                    for(vite in lv){
                        #if(!NAPE_RELEASE_BUILD)
                        if(vite==null)throw "Error: Array<Vec2> contains null objects";
                        #end
                        #if(!NAPE_RELEASE_BUILD)
                        if(!#if flash untyped __is__(vite,Vec2)#else Std.is(vite,Vec2)#end)throw "Error: Array<Vec2> contains non Vec2 objects";
                        #end
                        var v:Vec2=vite;
                        {
                            #if(!NAPE_RELEASE_BUILD)
                            if(v!=null&&v.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                            #end
                        };
                        {
                            ret.zpp_inner.vertices={
                                var obj=ZPP_GeomVert.get(v.x,v.y);
                                if(ret.zpp_inner.vertices==null)ret.zpp_inner.vertices=obj.prev=obj.next=obj;
                                else{
                                    obj.prev=ret.zpp_inner.vertices;
                                    obj.next=ret.zpp_inner.vertices.next;
                                    ret.zpp_inner.vertices.next.prev=obj;
                                    ret.zpp_inner.vertices.next=obj;
                                }
                                obj;
                            };
                        };
                    }
                }
                else if(#if flash10 untyped __is__(vertices,ZPP_Const.vec2vector)#else false #end){
                    #if flash10 var lv:flash.Vector<Vec2>=vertices;
                    for(vite in lv){
                        #if(!NAPE_RELEASE_BUILD)
                        if(vite==null)throw "Error: flash.Vector<Vec2> contains null objects";
                        #end
                        var v:Vec2=vite;
                        {
                            #if(!NAPE_RELEASE_BUILD)
                            if(v!=null&&v.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                            #end
                        };
                        {
                            ret.zpp_inner.vertices={
                                var obj=ZPP_GeomVert.get(v.x,v.y);
                                if(ret.zpp_inner.vertices==null)ret.zpp_inner.vertices=obj.prev=obj.next=obj;
                                else{
                                    obj.prev=ret.zpp_inner.vertices;
                                    obj.next=ret.zpp_inner.vertices.next;
                                    ret.zpp_inner.vertices.next.prev=obj;
                                    ret.zpp_inner.vertices.next=obj;
                                }
                                obj;
                            };
                        };
                    }
                    #end
                }
                else if(#if flash untyped __is__(vertices,Vec2List)#else Std.is(vertices,Vec2List)#end){
                    var lv:Vec2List=vertices;
                    for(v in lv){
                        #if(!NAPE_RELEASE_BUILD)
                        if(v==null)throw "Error: Vec2List contains null objects";
                        #end
                        {
                            #if(!NAPE_RELEASE_BUILD)
                            if(v!=null&&v.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                            #end
                        };
                        {
                            ret.zpp_inner.vertices={
                                var obj=ZPP_GeomVert.get(v.x,v.y);
                                if(ret.zpp_inner.vertices==null)ret.zpp_inner.vertices=obj.prev=obj.next=obj;
                                else{
                                    obj.prev=ret.zpp_inner.vertices;
                                    obj.next=ret.zpp_inner.vertices.next;
                                    ret.zpp_inner.vertices.next.prev=obj;
                                    ret.zpp_inner.vertices.next=obj;
                                }
                                obj;
                            };
                        };
                    }
                }
                else if(#if flash untyped __is__(vertices,GeomPoly)#else Std.is(vertices,GeomPoly)#end){
                    var lv:GeomPoly=vertices;
                    {
                        #if(!NAPE_RELEASE_BUILD)
                        if(lv!=null&&lv.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
                        #end
                    };
                    var verts:ZPP_GeomVert=lv.zpp_inner.vertices;
                    if(verts!=null){
                        var vite=verts;
                        do{
                            var v=Vec2.get(vite.x,vite.y);
                            vite=vite.next;
                            {
                                ret.zpp_inner.vertices={
                                    var obj=ZPP_GeomVert.get(v.x,v.y);
                                    if(ret.zpp_inner.vertices==null)ret.zpp_inner.vertices=obj.prev=obj.next=obj;
                                    else{
                                        obj.prev=ret.zpp_inner.vertices;
                                        obj.next=ret.zpp_inner.vertices.next;
                                        ret.zpp_inner.vertices.next.prev=obj;
                                        ret.zpp_inner.vertices.next=obj;
                                    }
                                    obj;
                                };
                            };
                            v.dispose();
                        }
                        while(vite!=verts);
                    }
                }
                else{
                    #if(!NAPE_RELEASE_BUILD)
                    throw "Error: Invalid type for polygon object, should be Array<Vec2>, Vec2List, GeomPoly or for flash10+ flash.Vector<Vec2>";
                    #end
                }
            };
            ret.skipForward(1);
            {
                if(#if flash untyped __is__(vertices,Array)#else Std.is(vertices,Array)#end){
                    var lv:Array<Vec2>=vertices;
                    var i=0;
                    while(i<lv.length){
                        var cur=lv[i];
                        if(({
                            if(({
                                cur.zpp_inner.weak;
                            })){
                                cur.dispose();
                                true;
                            }
                            else{
                                false;
                            }
                        })){
                            lv.splice(i,1);
                            continue;
                        }
                        i++;
                    }
                }
                else if(#if flash10 untyped __is__(vertices,ZPP_Const.vec2vector)#else false #end){
                    #if flash10 var lv:flash.Vector<Vec2>=vertices;
                    if(!lv.fixed){
                        var i:Int=0;
                        while(i<cast lv.length){
                            var cur=lv[i];
                            if(({
                                if(({
                                    cur.zpp_inner.weak;
                                })){
                                    cur.dispose();
                                    true;
                                }
                                else{
                                    false;
                                }
                            })){
                                lv.splice(i,1);
                                continue;
                            }
                            i++;
                        }
                    }
                    #end
                }
                else if(#if flash untyped __is__(vertices,Vec2List)#else Std.is(vertices,Vec2List)#end){
                    var lv:Vec2List=vertices;
                    if(lv.zpp_inner._validate!=null)lv.zpp_inner._validate();
                    var ins=lv.zpp_inner.inner;
                    var pre=null;
                    var cur=ins.begin();
                    while(cur!=null){
                        var x=cur.elem();
                        if(({
                            x.outer.zpp_inner.weak;
                        })){
                            cur=ins.erase(pre);
                            ({
                                if(({
                                    x.outer.zpp_inner.weak;
                                })){
                                    x.outer.dispose();
                                    true;
                                }
                                else{
                                    false;
                                }
                            });
                        }
                        else{
                            pre=cur;
                            cur=cur.next;
                        }
                    }
                }
            };
        }
        return ret;
    }
    /**
     * Release this GeomPoly to global object pool.
     * <br/><br/>
     * Once disposed this GeomPoly will be accessible to Nape internals for re-allocation
     * and should not be touched (Good practice would be to set any references to this
     * GeomPoly to null to help ensure this).
     * <br/><br/>
     * In debug mode, should you attempt to access this GeomPoly after disposal
     * and the GeomPoly is still in the object pool, you will be given an Error.
     * The object pool operates on a First-In-Last-Out principal in debug mode to help
     * catch these sort of errors.
     * @throws # If this GeomPoly has already been disposed.
     */
    public function dispose():Void{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        clear();
        {
            var o=this;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("PublicFree(in T: "+"GeomPoly"+", in obj: "+"this"+")");
                #end
            };
            #if(!NAPE_RELEASE_BUILD)
            o.zpp_pool=null;
            if(ZPP_PubPool.nextGeomPoly!=null)ZPP_PubPool.nextGeomPoly.zpp_pool=o;
            else ZPP_PubPool.poolGeomPoly=o;
            ZPP_PubPool.nextGeomPoly=o;
            #end
            #if NAPE_RELEASE_BUILD 
            o.zpp_pool=ZPP_PubPool.poolGeomPoly;
            ZPP_PubPool.poolGeomPoly=o;
            #end
            #if NAPE_POOL_STATS GeomPoly.POOL_CNT++;
            GeomPoly.POOL_SUB++;
            #end
            #if(!NAPE_RELEASE_BUILD)
            o.zpp_disp=true;
            #end
        };
    }
    /**
     * @private
     */
    @:keep public function toString():String{
        var ret="GeomPoly[";
        {
            {
                var F=zpp_inner.vertices;
                var L=zpp_inner.vertices;
                if(F!=null){
                    var nite=F;
                    do{
                        var v=nite;
                        {
                            {
                                if(v!=zpp_inner.vertices)ret+=",";
                                ret+="{"+v.x+","+v.y+"}";
                            };
                        }
                        nite=nite.next;
                    }
                    while(nite!=L);
                }
            };
        }
        return ret+"]";
    }
    /**
     * Compute area of weakly-simple polygon.
     * <br/><br/>
     * For complex polygons, this function will return an underestimate
     * to the true area.
     *
     * @return The area of the polygon.
     * @throws # If this GeomPoly has been disposed.
     */
    public function area():Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        return if(({
            zpp_inner.vertices==null||zpp_inner.vertices.next==null||zpp_inner.vertices.prev==zpp_inner.vertices.next;
        }))0.0 else({
            var ret=({
                {
                    #if NAPE_ASSERT if(({
                        var ret=0;
                        {
                            var F=zpp_inner.vertices;
                            var L=zpp_inner.vertices;
                            if(F!=null){
                                var nite=F;
                                do{
                                    var i=nite;
                                    {
                                        ret++;
                                    }
                                    nite=nite.next;
                                }
                                while(nite!=L);
                            }
                        };
                        ret;
                    })<3){
                        throw "Error: Method requires that polygon has atleast 3 vertices";
                    }
                    #end
                };
                var area=0.0;
                {
                    var F=zpp_inner.vertices;
                    var L=zpp_inner.vertices;
                    if(F!=null){
                        var nite=F;
                        do{
                            var v=nite;
                            {
                                {
                                    area+=v.x*(v.next.y-v.prev.y);
                                };
                            }
                            nite=nite.next;
                        }
                        while(nite!=L);
                    }
                };
                area*0.5;
            });
            if(ret<0)(-ret)else ret;
        });
    }
    /**
     * Compute the winding order for this polygon.
     * <br/><br/>
     * The winding order can be conceptualised by thinking of an analog
     * clock face, if your polygon is the numbers on the clock then a
     * clockwise winding would have your polygon's vertices in numerical
     * order.
     * <br/><br/>
     * In the case of a non-simple polygon with self intersections then the
     * winding order is decided by how 'much' of the polygon is locally
     * clockwise wound, and how much is locally anti-clockwise wound.
     * <br/>
     * (Think of a figure 8 style polygon where one loop is larger than the
     * other. This larger loop will dictate the winding of the polygon.)
     * <br/><br/>
     * If no winding can be computed, then <code>Winding.UNDEFINED</code>
     * will be returned.
     *
     * @return The winding of the polygon.
     * @throws # If this GeomPoly has been disposed.
     */
    public function winding():Winding{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        if(({
            zpp_inner.vertices==null||zpp_inner.vertices.next==null||zpp_inner.vertices.prev==zpp_inner.vertices.next;
        })){
            return Winding.UNDEFINED;
        }
        else{
            var area=({
                {
                    #if NAPE_ASSERT if(({
                        var ret=0;
                        {
                            var F=zpp_inner.vertices;
                            var L=zpp_inner.vertices;
                            if(F!=null){
                                var nite=F;
                                do{
                                    var i=nite;
                                    {
                                        ret++;
                                    }
                                    nite=nite.next;
                                }
                                while(nite!=L);
                            }
                        };
                        ret;
                    })<3){
                        throw "Error: Method requires that polygon has atleast 3 vertices";
                    }
                    #end
                };
                var area=0.0;
                {
                    var F=zpp_inner.vertices;
                    var L=zpp_inner.vertices;
                    if(F!=null){
                        var nite=F;
                        do{
                            var v=nite;
                            {
                                {
                                    area+=v.x*(v.next.y-v.prev.y);
                                };
                            }
                            nite=nite.next;
                        }
                        while(nite!=L);
                    }
                };
                area*0.5;
            });
            return if(area>0)Winding.CLOCKWISE else if(area==0)Winding.UNDEFINED else Winding.ANTICLOCKWISE;
        }
    }
    /**
     * Determine if point is contained in polygon.
     * <br/><br/>
     * Polygon containment is performed with a ray cast through polygon
     * from the vertex and counting the number of intersections. In this
     * way containment will be defined for self-intersecting polygons based
     * on how such a polygon would be rendered with areas of self-intersection
     * treat as being 'outside' the polygon.
     * <br/><br/>
     * This algorithm operates in O(n) time.
     *
     * @param point The point to test for containment.
     * @return True if point is contained in the polygon.
     * @throws # If point is null or has been disposed.
     * @throws # If this GeomPoly has been disposed.
     */
    public function contains(point:Vec2):Bool{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(point!=null&&point.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(point==null){
            throw "Error: GeomPoly::contains point cannot be null";
        }
        #end
        var ret=({
            var x=point.x;
            var y=point.y;
            var ret=false;
            {
                var F=zpp_inner.vertices;
                var L=zpp_inner.vertices;
                if(F!=null){
                    var nite=F;
                    do{
                        var p=nite;
                        {
                            {
                                var q=p.prev;
                                if((p.y<y&&q.y>=y||q.y<y&&p.y>=y)&&(p.x<=x||q.x<=x)){
                                    if((p.x+(y-p.y)/(q.y-p.y)*(q.x-p.x))<x){
                                        ret=!ret;
                                    }
                                }
                            };
                        }
                        nite=nite.next;
                    }
                    while(nite!=L);
                }
            };
            ret;
        });
        ({
            if(({
                point.zpp_inner.weak;
            })){
                point.dispose();
                true;
            }
            else{
                false;
            }
        });
        return ret;
    }
    /**
     * Determine if polygon is clockwise wound.
     * <br/><br/>
     * This is equivalent to <code>poly.winding() == Winding.CLOCKWISE</code>.
     *
     * @return True if polygon is clockwise wound.
     * @throws # If this GeomPoly has been disposed.
     */
    public#if NAPE_NO_INLINE#else inline #end
    function isClockwise():Bool{
        return winding()==Winding.CLOCKWISE;
    }
    /**
     * Determine if weakly-simple polygon is convex.
     * <br/><br/>
     * This algorithm assumes that the polygon is weakly-simple. Otherwise it may
     * fail (It is very easy to construct a self intersecting polygon which
     * will return True for isConvex()).
     * <br/><br/>
     * You may wish to instead use <code>isSimple() && isConvex()</code> if
     * you cannot be sure of the polygon being simple, noting that this will
     * of course return false in the case of a weakly-simple polygon.
     * <pre>
     *  _____
     * |     |
     * |     |  <-- convex
     * |____/
     *  __
     * |  &#92;___
     * |     /  <-- concave
     * |____/
     * </pre>
     * This algorithm operates in O(n) time.
     *
     * @return True if polygon is found to be convex.
     * @throws # If this GeomPoly has been disposed.
     */
    public function isConvex():Bool{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        return if(({
            zpp_inner.vertices==null||zpp_inner.vertices.next==null||zpp_inner.vertices.prev==zpp_inner.vertices.next;
        }))true else({
            {
                #if NAPE_ASSERT if(({
                    var ret=0;
                    {
                        var F=zpp_inner.vertices;
                        var L=zpp_inner.vertices;
                        if(F!=null){
                            var nite=F;
                            do{
                                var i=nite;
                                {
                                    ret++;
                                }
                                nite=nite.next;
                            }
                            while(nite!=L);
                        }
                    };
                    ret;
                })<3){
                    throw "Error: Method requires that polygon has atleast 3 vertices";
                }
                #end
            };
            var neg=false;
            var pos=false;
            var ret=true;
            {
                var F=zpp_inner.vertices;
                var L=zpp_inner.vertices;
                if(F!=null){
                    var nite=F;
                    do{
                        var v=nite;
                        {
                            {
                                var u=v.prev;
                                var w=v.next;
                                var ax:Float=0.0;
                                var ay:Float=0.0;
                                {
                                    ax=w.x-v.x;
                                    ay=w.y-v.y;
                                };
                                var bx:Float=0.0;
                                var by:Float=0.0;
                                {
                                    bx=v.x-u.x;
                                    by=v.y-u.y;
                                };
                                var dot=(by*ax-bx*ay);
                                if(dot>0.0){
                                    pos=true;
                                }
                                else if(dot<0.0){
                                    neg=true;
                                }
                                if(pos&&neg){
                                    ret=false;
                                    break;
                                }
                            };
                        }
                        nite=nite.next;
                    }
                    while(nite!=L);
                }
            };
            ret;
        });
    }
    /**
     * Determine if polygon is strictly simple.
     * <br/><br/>
     * By strict simplicity, we refer to not permitting 'glancing'
     * self intersections (where boundary of polygon 'touches' but does not
     * pass through another area of the polygon's boundary). This property
     * is instead referred to as being 'weakly simple' for which there is no
     * easy test!
     * <pre>
     *  _______
     * |   __  |  <-- strictly simple polygon.
     * |   &#92; &#92;_|
     *  &#92;__/
     *  _______
     * |   |   |
     * |  /_&#92;  | <-- weakly simple polygon.
     *  &#92;_____/
     *  ____
     * | __/
     *  X_  __   <-- complex polygon.
     * |  &#92;/  &#92;
     * &#92;__/&#92;__|
     * </pre>
     * This algorithm operates in O(n.log(n)) time.
     *
     * @return True if polygon is strictly simple.
     * @throws # If this GeomPoly has been disposed.
     */
    public#if NAPE_NO_INLINE#else inline #end
    function isSimple():Bool{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        return if(({
            zpp_inner.vertices==null||zpp_inner.vertices.next==null||zpp_inner.vertices.prev==zpp_inner.vertices.next;
        }))true else ZPP_Simple.isSimple(zpp_inner.vertices);
    }
    /**
     * Determine if polygon is y-monotone.
     * <br/><br/>
     * To be classed as y-monotone, the polygon must be such that any horizontal
     * line intersects the polygon in at most 2 intersections.
     * <pre>
     *  ___
     * |   |
     * |   |  <-- y-monotone
     * |___|
     *
     * |&#92;
     * | &#92;/|  <-- not y-monotone, offending vertex at bottom of the V.
     * |___|
     * </pre>
     * This algorithm operates in O(n) time.
     *
     * @return True if polygon is y-monotone.
     * @throws # If this GeomPoly has been disposed.
     */
    public#if NAPE_NO_INLINE#else inline #end
    function isMonotone():Bool{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        return if(({
            zpp_inner.vertices==null||zpp_inner.vertices.next==null||zpp_inner.vertices.prev==zpp_inner.vertices.next;
        }))true else ZPP_Monotone.isMonotone(zpp_inner.vertices);
    }
    /**
     * Determine if weakly-simple polygon is degenerate.
     * <br/><br/>
     * Degeneracy is determined by having a zero area, if polygon is complex,
     * then this function may report degeneracy erroneously.
     *
     * @return True if polygon is degenerate.
     * @throws # If this GeomPoly has been disposed.
     */
    public#if NAPE_NO_INLINE#else inline #end
    function isDegenerate():Bool{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        return if(({
            zpp_inner.vertices==null||zpp_inner.vertices.next==null||zpp_inner.vertices.prev==zpp_inner.vertices.next;
        }))true else area()<Config.epsilon;
    }
    /**
     * Simplify polygon.
     * <br/><br/>
     * Simplification is performed with an implementation of the Ramer-Douglas-Peucker
     * algorithm. The output polygon is formed via subset of the vertices in
     * the input polygon such that any discarded vertex is at most 'epsilon' pixels
     * away from the local output polygon.
     * <br/><br/>
     * This algorithm works on both simple and complex polygons, but please note
     * that this algorithm makes no guarantees on a simple polygon remaining simple
     * after simplification. This should not generally be a problem unless the epsilon
     * value is large with respect to the size of the features on the polygon.
     * <br/><br/>
     * Many of the geometric algorithms will mark vertices as important, such that
     * they will be guaranteed to exist after simplification (Such as preventing
     * gaps from opening up in marching squares when simplifying output polygons).
     * <br/><br/>
     * The average runtime of this algorithm is O(n.log(n)). This algorithm is
     * not stable in the sense that adding a new vertex to the polygon may drastically
     * change the result of simplifying the polygon.
     *
     * @param epsilon The distance from polygon at which vertices are ignored.
     * @return A new GeomPoly representing the result of the simplification.
     * @throws # If epsilon is <= 0.
     * @throws # If this GeomPoly has been disposed.
     */
    public function simplify(epsilon:Float):GeomPoly{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(epsilon<=0.0){
            throw "Error: Epsilon should be > 0 for simplifying a GeomPoly";
        }
        #end
        if(({
            zpp_inner.vertices==null||zpp_inner.vertices.next==null||zpp_inner.vertices.prev==zpp_inner.vertices.next;
        })){
            return copy();
        }
        else{
            var x=ZPP_Simplify.simplify(zpp_inner.vertices,epsilon);
            var ret=GeomPoly.get();
            ret.zpp_inner.vertices=x;
            return ret;
        }
    }
    /**
     * Produce a decomposition of complex polygon into simple components.
     * <br/><br/>
     * WARNING: This method is 'not' 100% robust. It may fail!
     * <br/><br/>
     * Produce a decomposition of a self intersecting, complex polygon into
     * a set of weakly-simple components.
     * <br/><br/>
     * This algorithm operates in O(n.log(n)) time and is based on the
     * Bentley-Ottmann algorithm.
     *
     * @param output If supplied, polygons will be appended to this list via 'add'
     *        instead of a new list being constructed.
     * @return A Nape list of GeomPoly's representing the decomposition.
     * @throws # If polygon is degenerate.
     * @throws # Any other error may be thrown if algorithm has failed, even
     *           in release builds!
     * @throws # If this GeomPoly has been disposed.
     */
    public function simpleDecomposition(output:GeomPolyList=null):GeomPolyList{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(({
            zpp_inner.vertices==null||zpp_inner.vertices.next==null||zpp_inner.vertices.prev==zpp_inner.vertices.next;
        })){
            throw "Error: Cannot decompose a degenerate polygon";
        }
        #end
        var MPs=ZPP_Simple.decompose(zpp_inner.vertices,ZPP_PartitionedPoly.getShared());
        var ret=(output==null?new GeomPolyList():output);
        while(!MPs.empty()){
            var MP=MPs.pop_unsafe();
            var x=GeomPoly.get();
            x.zpp_inner.vertices=MP;
            ret.add(x);
        }
        return ret;
    }
    /**
     * Produce a decomposition of weakly-simple polygon into monotone components.
     * <br/><br/>
     * This algorithm 'should' be 100% robust and has been well tested on for
     * example, the output of the Marching Squares utility which produces many
     * degenerate cases of weakly-simple polygons that have not yet broken this
     * algorithm!.
     * <br/><br/>
     * This algorithm operates in O(n.log(n)) time and may strip vertices from
     * the polygon in degenerate cases where vertex is not needed to define the
     * polygon.
     * <br/><br/>
     * This algorithm is an improved version of the one presented in: Mark de
     * Berg, Marc van Kreveld, Mark Overmars, and Otfried Schwarzkopf.
     * Computational Geometry: Algorithms and Applications. Springer-Verlag,
     * Berlin, 1997.
    
     * @param output If supplied, polygons will be appended to this list via 'add'
     *        instead of a new list being constructed.
     * @return A Nape list of GeomPoly's defining the decomposition.
     * @throws # If polygon is degenerate.
     * @throws # If this GeomPoly has been disposed.
     */
    public function monotoneDecomposition(output:GeomPolyList=null):GeomPolyList{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(({
            zpp_inner.vertices==null||zpp_inner.vertices.next==null||zpp_inner.vertices.prev==zpp_inner.vertices.next;
        })){
            throw "Error: Cannot decompose a degenerate polygon";
        }
        #end
        var poly=ZPP_Monotone.decompose(zpp_inner.vertices,ZPP_Monotone.getShared());
        var MPs=poly.extract(ZPP_PartitionedPoly.getShared());
        var ret=(output==null?new GeomPolyList():output);
        while(!MPs.empty()){
            var MP=MPs.pop_unsafe();
            var x=GeomPoly.get();
            x.zpp_inner.vertices=MP;
            ret.add(x);
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    x.isMonotone();
                };
                if(!res)throw "assert("+"x.isMonotone()"+") :: "+("monotone decomposition gave non-monotone output?");
                #end
            };
        }
        return ret;
    }
    /**
     * Produce a decomposition of weakly-simple polygon into convex components.
     * <br/><br/>
     * This algorithm 'should' be 100% robust and has been well test on for
     * example, the output of the Marching Squars utility which produces many
     * degenerate cases of weakly-simple polygons that have not yet broken this
     * algorithm!.
     * <br/><br/>
     * This algorithm operates in O(n.log(n)) time and will produce no more than
     * 4 times the number of convex poylgons in a minimal decomposition in the
     * worst case scenario.
     * <br/><br/>
     * Vertices may be stripped from the polygon that are found to not be
     * necessary as part of making this algorithm robust.
     *
     * @param delaunay This algorithm first performs a triangulation, if this field
     *                 is true, then this triangulation will be made delaunay and may
     *                 produce better convex polygons resultanly (default false).
     * @param output If supplied, polygons will be appended to this list via 'add'
     *        instead of a new list being constructed.
     * @return A Nape list of GeomPoly's defining the decomposition.
     * @throws # If polygon is degenerate.
     * @throws # If this GeomPoly has been disposed.
     */
    public function convexDecomposition(delaunay:Bool=false,output:GeomPolyList=null):GeomPolyList{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(({
            zpp_inner.vertices==null||zpp_inner.vertices.next==null||zpp_inner.vertices.prev==zpp_inner.vertices.next;
        })){
            throw "Error: Cannot decompose a degenerate polygon";
        }
        #end
        var poly=ZPP_Monotone.decompose(zpp_inner.vertices,ZPP_Monotone.getShared());
        var MPs=poly.extract_partitions(ZPP_PartitionedPoly.getSharedPP());
        var ret=(output==null?new GeomPolyList():output);
        while(!MPs.empty()){
            var MP=MPs.pop_unsafe();
            ZPP_Triangular.triangulate(MP);
            if(delaunay){
                ZPP_Triangular.optimise(MP);
            }
            ZPP_Convex.optimise(MP);
            var MQs=MP.extract(ZPP_PartitionedPoly.getShared());
            {
                var o=MP;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_PartitionedPoly"+", in obj: "+"MP"+")");
                    #end
                };
                o.free();
                o.next=ZPP_PartitionedPoly.zpp_pool;
                ZPP_PartitionedPoly.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_PartitionedPoly.POOL_CNT++;
                ZPP_PartitionedPoly.POOL_SUB++;
                #end
            };
            while(!MQs.empty()){
                var MQ=MQs.pop_unsafe();
                var x=GeomPoly.get();
                x.zpp_inner.vertices=MQ;
                ret.add(x);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        x.isConvex();
                    };
                    if(!res)throw "assert("+"x.isConvex()"+") :: "+("convex decomposition gave non-convex output?");
                    #end
                };
            }
        }
        return ret;
    }
    /**
     * Produce a decomposition of weakly-simple polygon into triangles.
     * <br/><br/>
     * This algorithm 'should' be 100% robust and has been well test on for
     * example, the output of the Marching Squars utility which produces many
     * degenerate cases of weakly-simple polygons that have not yet broken this
     * algorithm!.
     * <br/><br/>
     * This algorithm operates in O(n.log(n)) time.
     * <br/><br/>
     * Vertices may be stripped from the polygon that are found to not be
     * necessary as part of making this algorithm robust.
     *
     * @param delaunay If true, then an O(n^2) pass will be made to mutate the original
     *                 triangulation to push it into a delanuay triangulation. (default false)
     * @param output If supplied, polygons will be appended to this list via 'add'
     *        instead of a new list being constructed.
     * @return A Nape list of GeomPoly's defining the decomposition.
     * @throws # If polygon is degenerate.
     * @throws # If this GeomPoly has been disposed.
     */
    public function triangularDecomposition(delaunay:Bool=false,output:GeomPolyList=null):GeomPolyList{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(({
            zpp_inner.vertices==null||zpp_inner.vertices.next==null||zpp_inner.vertices.prev==zpp_inner.vertices.next;
        })){
            throw "Error: Cannot decompose a degenerate polygon";
        }
        #end
        var poly=ZPP_Monotone.decompose(zpp_inner.vertices,ZPP_Monotone.getShared());
        var MPs=poly.extract_partitions(ZPP_PartitionedPoly.getSharedPP());
        var ret=(output==null?new GeomPolyList():output);
        while(!MPs.empty()){
            var MP=MPs.pop_unsafe();
            ZPP_Triangular.triangulate(MP);
            if(delaunay){
                ZPP_Triangular.optimise(MP);
            }
            var MQs=MP.extract(ZPP_PartitionedPoly.getShared());
            {
                var o=MP;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_PartitionedPoly"+", in obj: "+"MP"+")");
                    #end
                };
                o.free();
                o.next=ZPP_PartitionedPoly.zpp_pool;
                ZPP_PartitionedPoly.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_PartitionedPoly.POOL_CNT++;
                ZPP_PartitionedPoly.POOL_SUB++;
                #end
            };
            while(!MQs.empty()){
                var MQ=MQs.pop_unsafe();
                var x=GeomPoly.get();
                x.zpp_inner.vertices=MQ;
                ret.add(x);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        x.size()==3;
                    };
                    if(!res)throw "assert("+"x.size()==3"+") :: "+("triangular decomosition gave non-triangle output?");
                    #end
                };
            }
        }
        return ret;
    }
    /**
     * Inflate/Deflate polygon.
     * <br/><br/>
     * This algorithm does not attempt to deal with any self-intersections which may
     * result from the process. Gaps are joined with a miter joint.
     * <br/><br/>
     * This algorithm will work for self-intersecting polygons, though the results
     * may not be what you expect; some parts will be inflated, and some deflated
     * depending on the local winding. You should probably avoid using this on
     * self-intersecting polygons.
     *
     * @param inflation The number of pixels to inflate polygon by. To deflate
     *                  use a negative value.
     * @return The inflated polygon.
     * @throws # If this GeomPoly has been disposed.
     */
    public function inflate(inflation:Float):GeomPoly{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        var ret=GeomPoly.get();
        if(isClockwise())inflation=-inflation;
        {
            var F=zpp_inner.vertices;
            var L=zpp_inner.vertices;
            if(F!=null){
                var nite=F;
                do{
                    var p=nite;
                    {
                        {
                            var prev=p.prev;
                            var next=p.next;
                            var ax:Float=0.0;
                            var ay:Float=0.0;
                            var bx:Float=0.0;
                            var by:Float=0.0;
                            {
                                ax=p.x-prev.x;
                                ay=p.y-prev.y;
                            };
                            {
                                bx=next.x-p.x;
                                by=next.y-p.y;
                            };
                            var apx:Float=0.0;
                            var apy:Float=0.0;
                            var bpx:Float=0.0;
                            var bpy:Float=0.0;
                            {
                                apx=ax;
                                apy=ay;
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((apx!=apx));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(apx)"+") :: "+("vec_set(in n: "+"ap"+",in x: "+"ax"+",in y: "+"ay"+")");
                                    #end
                                };
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((apy!=apy));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(apy)"+") :: "+("vec_set(in n: "+"ap"+",in x: "+"ax"+",in y: "+"ay"+")");
                                    #end
                                };
                            };
                            {
                                {
                                    var d=(apx*apx+apy*apy);
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            d!=0.0;
                                        };
                                        if(!res)throw "assert("+"d!=0.0"+") :: "+("vec_normalise(in n: "+"ap"+")");
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
                                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"ap"+",in s: "+"imag"+")");
                                            #end
                                        };
                                        apx*=t;
                                        apy*=t;
                                    };
                                };
                                {
                                    var t=apx;
                                    apx=-apy;
                                    apy=t;
                                };
                            };
                            {
                                var t=(inflation);
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((t!=t));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"ap"+",in s: "+"inflation"+")");
                                    #end
                                };
                                apx*=t;
                                apy*=t;
                            };
                            {
                                bpx=bx;
                                bpy=by;
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((bpx!=bpx));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(bpx)"+") :: "+("vec_set(in n: "+"bp"+",in x: "+"bx"+",in y: "+"by"+")");
                                    #end
                                };
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((bpy!=bpy));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(bpy)"+") :: "+("vec_set(in n: "+"bp"+",in x: "+"bx"+",in y: "+"by"+")");
                                    #end
                                };
                            };
                            {
                                {
                                    var d=(bpx*bpx+bpy*bpy);
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            d!=0.0;
                                        };
                                        if(!res)throw "assert("+"d!=0.0"+") :: "+("vec_normalise(in n: "+"bp"+")");
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
                                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"bp"+",in s: "+"imag"+")");
                                            #end
                                        };
                                        bpx*=t;
                                        bpy*=t;
                                    };
                                };
                                {
                                    var t=bpx;
                                    bpx=-bpy;
                                    bpy=t;
                                };
                            };
                            {
                                var t=(inflation);
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((t!=t));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"bp"+",in s: "+"inflation"+")");
                                    #end
                                };
                                bpx*=t;
                                bpy*=t;
                            };
                            var bapx:Float=0.0;
                            var bapy:Float=0.0;
                            {
                                bapx=bpx-apx;
                                bapy=bpy-apy;
                            };
                            var num=(by*bapx-bx*bapy);
                            var t=if(num==0)0 else(num/(by*ax-bx*ay));
                            var px:Float=0.0;
                            var py:Float=0.0;
                            {
                                px=p.x+apx;
                                py=p.y+apy;
                            };
                            {
                                var t=(t);
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((t!=t));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"p"+",in b: "+"a"+",in s: "+"t"+")");
                                    #end
                                };
                                px+=ax*t;
                                py+=ay*t;
                            };
                            ret.push(Vec2.get(px,py));
                        };
                    }
                    nite=nite.next;
                }
                while(nite!=L);
            }
        };
        return ret.skipForward(1);
    }
    /**
     * Cut simple polygon with line.
     * <br/><br/>
     * The result of this operation will be a list of new GeomPoly representing
     * the connected regions of the polygon after an imaginary cut is made.
     * <pre>
     * (Result of cut assuming
     *  boundedStart = true)       _
     *   /&#92;    _             /&#92;   / &#92;
     *  /  &#92;  / &#92;           /  &#92; '---'
     * / o--&#92;/---&#92;-->  =>  /    &#92;,---,
     * &#92;_________/         &#92;_________/
     * </pre>
     * This algorithm runs in average case O(n.log(n)) time and worst case O(n^2).
     * For convex polygons, this algorithm runs in guaranteed O(n) time.
     *
     * @param start The start point for line segment
     * @param end The end point for line segment.
     * @param boundedStart If true, then the cut will not extend
     *                     beyond the start of the line segment.
     *                     (default false)
     * @param boundedEnd   If true, then the cut will not extend
     *                     beyond the end of the line segment.
     *                     (default false)
     * @param output A GeomPolyList to append results to if supplied,
     *               otherwise a new list is created (default null)
     * @return A list of GeomPoly representing the result of the cut.
     * @throws # If polygon is not simple.
     * @throws # If start or end Vec2 are null or disposed of.
     * @throws # If this GeomPoly has been disposed.
     */
    public function cut(start:Vec2,end:Vec2,boundedStart:Bool=false,boundedEnd:Bool=false,output:GeomPolyList=null):GeomPolyList{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(!isSimple()){
            throw "Error: Cut requires a truly simple polygon";
        }
        if(start==null||end==null){
            throw "Error: Cannot cut with null start/end's";
        }
        #end
        {
            #if(!NAPE_RELEASE_BUILD)
            if(start!=null&&start.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(end!=null&&end.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        var ret=ZPP_Cutter.run(zpp_inner.vertices,start,end,boundedStart,boundedEnd,output);
        ({
            if(({
                start.zpp_inner.weak;
            })){
                start.dispose();
                true;
            }
            else{
                false;
            }
        });
        ({
            if(({
                end.zpp_inner.weak;
            })){
                end.dispose();
                true;
            }
            else{
                false;
            }
        });
        return ret;
    }
    /**
     * Transform polygon by given matrix.
     * <br/><br/>
     * Any transformation (not just equiorthogonal ones) are permitted, though
     * a transformation that causes polygon to be come degenerate is a bit
     * pointless.
     *
     * @param matrix The matrix to transform polygon by.
     * @return A reference to this polygon.
     * @throws # If matrix is null.
     * @throws # If this GeomPoly has been disposed.
     */
    public function transform(matrix:Mat23):GeomPoly{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(matrix==null){
            throw "Error: Cannot transform by null matrix";
        }
        #end
        {
            var F=zpp_inner.vertices;
            var L=zpp_inner.vertices;
            if(F!=null){
                var nite=F;
                do{
                    var v=nite;
                    {
                        {
                            {
                                var t=matrix.a*v.x+matrix.b*v.y+matrix.tx;
                                v.y=matrix.c*v.x+matrix.d*v.y+matrix.ty;
                                v.x=t;
                            };
                        };
                    }
                    nite=nite.next;
                }
                while(nite!=L);
            }
        };
        return this;
    }
    /**
     * Determine bounds of polygon.
     *
     * @return A new AABB representing bounds of polygon.
     * @throws # If polygon is empty.
     * @throws # If this GeomPoly has been disposed.
     */
    public function bounds():AABB{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(empty()){
            throw "Error: empty GeomPoly has no defineable bounds";
        }
        #end
        var minx:Float=ZPP_Const.FMAX;
        var miny:Float=ZPP_Const.FMAX;
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((minx!=minx));
            };
            if(!res)throw "assert("+"!assert_isNaN(minx)"+") :: "+("vec_new(in n: "+"min"+",in x: "+"ZPP_Const.FMAX"+",in y: "+"ZPP_Const.FMAX"+")");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((miny!=miny));
            };
            if(!res)throw "assert("+"!assert_isNaN(miny)"+") :: "+("vec_new(in n: "+"min"+",in x: "+"ZPP_Const.FMAX"+",in y: "+"ZPP_Const.FMAX"+")");
            #end
        };
        var maxx:Float=-ZPP_Const.FMAX;
        var maxy:Float=-ZPP_Const.FMAX;
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((maxx!=maxx));
            };
            if(!res)throw "assert("+"!assert_isNaN(maxx)"+") :: "+("vec_new(in n: "+"max"+",in x: "+"-ZPP_Const.FMAX"+",in y: "+"-ZPP_Const.FMAX"+")");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((maxy!=maxy));
            };
            if(!res)throw "assert("+"!assert_isNaN(maxy)"+") :: "+("vec_new(in n: "+"max"+",in x: "+"-ZPP_Const.FMAX"+",in y: "+"-ZPP_Const.FMAX"+")");
            #end
        };
        {
            var F=zpp_inner.vertices;
            var L=zpp_inner.vertices;
            if(F!=null){
                var nite=F;
                do{
                    var v=nite;
                    {
                        {
                            if(v.x<minx)minx=v.x;
                            if(v.y<miny)miny=v.y;
                            if(v.x>maxx)maxx=v.x;
                            if(v.y>maxy)maxy=v.y;
                        };
                    }
                    nite=nite.next;
                }
                while(nite!=L);
            }
        };
        return new AABB(minx,miny,maxx-minx,maxy-miny);
    }
    /**
     * Find top most vertex of polygon.
     * <br/><br/>
     * If there is more than one such vertex then the result is indeterminate.
     * <br/><br/>
     * The Vec2 returned is intrinsically tied to the inner vertex like that
     * returned by current(). This method will not alter the current vertex.
     * This Vec2 is not able to be disposed of.
     *
     * @return A Vec2 representing the top most vertex.
     * @throws # If polygon is empty.
     * @throws # If this GeomPoly has been disposed.
     */
    public function top():Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(empty()){
            throw "Error: empty GeomPoly has no defineable topmost vertex";
        }
        #end
        var min=zpp_inner.vertices;
        {
            var F=zpp_inner.vertices.next;
            var L=zpp_inner.vertices;
            if(F!=null){
                var nite=F;
                do{
                    var v=nite;
                    {
                        if(v.y<min.y)min=v;
                    }
                    nite=nite.next;
                }
                while(nite!=L);
            }
        };
        return min.wrapper();
    }
    /**
     * Find bottom most vertex of polygon.
     * <br/><br/>
     * If there is more than one such vertex then the result is indeterminate.
     * <br/><br/>
     * The Vec2 returned is intrinsically tied to the inner vertex like that
     * returned by current(). This method will not alter the current vertex.
     * This Vec2 is not able to be disposed of.
     *
     * @return A Vec2 representing the bottom most vertex.
     * @throws # If polygon is empty.
     * @throws # If this GeomPoly has been disposed.
     */
    public function bottom():Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(empty()){
            throw "Error: empty GeomPoly has no defineable bottommost vertex";
        }
        #end
        var max=zpp_inner.vertices;
        {
            var F=zpp_inner.vertices.next;
            var L=zpp_inner.vertices;
            if(F!=null){
                var nite=F;
                do{
                    var v=nite;
                    {
                        if(v.y>max.y)max=v;
                    }
                    nite=nite.next;
                }
                while(nite!=L);
            }
        };
        return max.wrapper();
    }
    /**
     * Find left most vertex of polygon.
     * <br/><br/>
     * If there is more than one such vertex then the result is indeterminate.
     * <br/><br/>
     * The Vec2 returned is intrinsically tied to the inner vertex like that
     * returned by current(). This method will not alter the current vertex.
     * This Vec2 is not able to be disposed of.
     *
     * @return A Vec2 representing the left most vertex.
     * @throws # If polygon is empty.
     * @throws # If this GeomPoly has been disposed.
     */
    public function left():Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(empty()){
            throw "Error: empty GeomPoly has no defineable leftmost vertex";
        }
        #end
        var min=zpp_inner.vertices;
        {
            var F=zpp_inner.vertices.next;
            var L=zpp_inner.vertices;
            if(F!=null){
                var nite=F;
                do{
                    var v=nite;
                    {
                        if(v.x<min.x)min=v;
                    }
                    nite=nite.next;
                }
                while(nite!=L);
            }
        };
        return min.wrapper();
    }
    /**
     * Find right most vertex of polygon.
     * <br/><br/>
     * If there is more than one such vertex then the result is indeterminate.
     * <br/><br/>
     * The Vec2 returned is intrinsically tied to the inner vertex like that
     * returned by current(). This method will not alter the current vertex.
     * This Vec2 is not able to be disposed of.
     *
     * @return A Vec2 representing the right most vertex.
     * @throws # If polygon is empty.
     * @throws # If this GeomPoly has been disposed.
     */
    public function right():Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(empty()){
            throw "Error: empty GeomPoly has no defineable rightmmost vertex";
        }
        #end
        var max=zpp_inner.vertices;
        {
            var F=zpp_inner.vertices.next;
            var L=zpp_inner.vertices;
            if(F!=null){
                var nite=F;
                do{
                    var v=nite;
                    {
                        if(v.x>max.x)max=v;
                    }
                    nite=nite.next;
                }
                while(nite!=L);
            }
        };
        return max.wrapper();
    }
}
