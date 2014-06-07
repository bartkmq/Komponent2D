package nape.shape;
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
 * Polygon subtype of Shape.
 * <br/><br/>
 * Can be used to simulate any convex polygon.
 */
@:final#if nape_swc@:keep #end
class Polygon extends Shape{
    /**
     * @private
     */
    public var zpp_inner_zn:ZPP_Polygon=null;
    /**
     * Construct a polygon representing a rectangle.
     * <br/><br/>
     * For a dynamic object, you may consider use of the box method instead
     * as dynamic bodies will only respond as expected if their centre of mass
     * is equal to the origin.
     * <br/><br/>
     * The generate polygon will have coordinates equal to:
     * <pre>
     * (x, y) -> (x + width, y + height)
     * </pre>
     * Negative values of width/height are permitted so that the given x/y values
     * may not necessarigly be the top-left corner of rectangle.
     *
     * @param x The x coordinate of rectangle.
     * @param y The y coordinate of rectangle.
     * @param width The width of the ractangle. This value may be negative.
     * @param height The height of the rectangle This value may be negative.
     * @param weak If true, the generated list of vertices will be allocated as
     *             weak Vec2s so that when this list is passed to a Nape function
     *             these Vec2s will be automatically sent back to object pool.
     *             (default false)
     * @return An array of Vec2 representing the given rectangle.
     */
    #if nape_swc@:keep #end
    public static function rect(x:Float,y:Float,width:Float,height:Float,weak:Bool=false):Array<Vec2>{
        #if(!NAPE_RELEASE_BUILD)
        if((x!=x)||(y!=y)||(width!=width)||(height!=height))throw "Error: Polygon.rect cannot accept NaN arguments";
        #end
        return[Vec2.get(x,y,weak),Vec2.get(x+width,y,weak),Vec2.get(x+width,y+height,weak),Vec2.get(x,y+height,weak)];
    }
    /**
     * Construct a polygon representing an origin centred box.
     * <br/><br/>
     * This method is equivalent to calling: <code>Polygon.rect(-width/2,-height/2,width,height)</code>
     *
     * @param width The width of the box (This value may be negative but will
     *              make no difference).
     * @param height The height of the box (This value may be negative but will
     *              make no difference).
     * @param weak If true, the generated list of vertices will be allocated as
     *             weak Vec2s so that when this list is passed to a Nape function
     *             these Vec2s will be automatically sent back to object pool.
     *             (default false)
     * @return An array of Vec2 representing the given box.
     */
    #if nape_swc@:keep #end
    public static function box(width:Float,height:Float,weak:Bool=false):Array<Vec2>{
        #if(!NAPE_RELEASE_BUILD)
        if((width!=width)||(height!=height))throw "Error: Polygon.box cannot accept NaN arguments";
        #end
        return rect(-width/2,-height/2,width,height,weak);
    }
    /**
     * Construct a regular polygon centred at origin.
     * <br/><br/>
     * Vertices are created at positions on the edge of an ellipsoid of given
     * radii, when radii are not equal the vertices will not have an equal
     * angle between them; it will be as though an actual regular polygon were
     * created, and then squashed to conform to the input radii.
     *
     * @param xRadius The x radius of polygon before angleOffset rotation.
     * @param yRadius The y radius of polygon before angleOffset rotation.
     * @param edgeCount The number of edges/vertices in polygon.
     * @param angleOffset The clockwise angular offset to generate vertices at
     *                    in radians. (default 0.0)
     * @param weak If true, the generated list of vertices will be allocated as
     *             weak Vec2s so that when this list is passed to a Nape function
     *             these Vec2s will be automatically sent back to object pool.
     *             (default false)
     * @return An array of Vec2 representing the polygon.
     */
    #if nape_swc@:keep #end
    public static function regular(xRadius:Float,yRadius:Float,edgeCount:Int,angleOffset=0.0,weak:Bool=false):Array<Vec2>{
        #if(!NAPE_RELEASE_BUILD)
        if((xRadius!=xRadius)||(yRadius!=yRadius)||(angleOffset!=angleOffset))throw "Error: Polygon.regular cannot accept NaN arguments";
        #end
        var ret=[];
        var dangle=Math.PI*2/edgeCount;
        for(i in 0...edgeCount){
            var ang=i*dangle+angleOffset;
            var x=Vec2.get(Math.cos(ang)*xRadius,Math.sin(ang)*yRadius,weak);
            ret.push(x);
        }
        return ret;
    }
    /**
     * Local coordinates of vertices.
     * <br/><br/>
     * This list can be modified, but modifications to a Polygon that is
     * part of a static Body inside of a Space will given an error in
     * debug builds.
     */
    #if nape_swc@:isVar #end
    public var localVerts(get_localVerts,never):Vec2List;
    inline function get_localVerts():Vec2List{
        if(zpp_inner_zn.wrap_lverts==null)zpp_inner_zn.getlverts();
        return zpp_inner_zn.wrap_lverts;
    }
    /**
     * World coordinates of vertices.
     * <br/><br/>
     * This list can be accessed, but any queries of values will result
     * in an error in debug builds unless this Polygon is part of a Body.
     * <br/><br/>
     * This list is immutable.
     */
    #if nape_swc@:isVar #end
    public var worldVerts(get_worldVerts,never):Vec2List;
    inline function get_worldVerts():Vec2List{
        if(zpp_inner_zn.wrap_gverts==null)zpp_inner_zn.getgverts();
        return zpp_inner_zn.wrap_gverts;
    }
    /**
     * Set of edges on polygon.
     * <br/><br/>
     * This list is immutable.
     */
    #if nape_swc@:isVar #end
    public var edges(get_edges,never):EdgeList;
    inline function get_edges():EdgeList{
        if(zpp_inner_zn.wrap_edges==null)zpp_inner_zn.getedges();
        return zpp_inner_zn.wrap_edges;
    }
    /**
     * Determine validity of polygon for use in a Nape simulation.
     */
    #if nape_swc@:keep #end
    public function validity():ValidationResult{
        return zpp_inner_zn.valid();
    }
    /**
     * Construct a new Polygon.
     * <br/><br/>
     * The localVerts parameter is typed Dynamic and may be one of:
     * <code>Array&lt;Vec2&gt;, flash.Vector&lt;Vec2&gt;, Vec2List, GeomPoly</code>
     *
     * @param localVerts The local vertices of polygon.
     * @param material The material for this polygon. (default new Material&#40;&#41;)
     * @param filter The interaction filter for this polygon.
     *               (default new InteractionFilter&#40;&#41;)
     * @return The constructed Polygon.
     * @throws # If localVerts is null, or not of the expected Type.
     * @throws # If localVerts contains any disposed or null Vec2.
     */
    #if flib@:keep function flibopts_2(){}
    #end
    public function new(localVerts:Dynamic,material:Material=null,filter:InteractionFilter=null){
        #if(!NAPE_RELEASE_BUILD)
        try{
            super();
        }
        catch(e:Dynamic){}
        #end
        #if NAPE_RELEASE_BUILD 
        super();
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(localVerts==null)throw "Error: localVerts cannot be null";
        #end
        zpp_inner_zn=new ZPP_Polygon();
        zpp_inner_zn.outer=this;
        zpp_inner_zn.outer_zn=this;
        zpp_inner=zpp_inner_zn;
        zpp_inner_i=zpp_inner;
        zpp_inner_i.outer_i=this;
        {
            if(#if flash untyped __is__(localVerts,Array)#else Std.is(localVerts,Array)#end){
                var lv:Array<Dynamic>=localVerts;
                for(vite in lv){
                    #if(!NAPE_RELEASE_BUILD)
                    if(vite==null)throw "Error: Array<Vec2> contains null objects";
                    #end
                    #if(!NAPE_RELEASE_BUILD)
                    if(!#if flash untyped __is__(vite,Vec2)#else Std.is(vite,Vec2)#end)throw "Error: Array<Vec2> contains non Vec2 objects";
                    #end
                    var x:Vec2=vite;
                    {
                        #if(!NAPE_RELEASE_BUILD)
                        if(x!=null&&x.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                        #end
                    };
                    this.localVerts.push(x.copy());
                }
            }
            else if(#if flash10 untyped __is__(localVerts,ZPP_Const.vec2vector)#else false #end){
                #if flash10 var lv:flash.Vector<Vec2>=localVerts;
                for(vite in lv){
                    #if(!NAPE_RELEASE_BUILD)
                    if(vite==null)throw "Error: flash.Vector<Vec2> contains null objects";
                    #end
                    var x:Vec2=vite;
                    {
                        #if(!NAPE_RELEASE_BUILD)
                        if(x!=null&&x.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                        #end
                    };
                    this.localVerts.push(x.copy());
                }
                #end
            }
            else if(#if flash untyped __is__(localVerts,Vec2List)#else Std.is(localVerts,Vec2List)#end){
                var lv:Vec2List=localVerts;
                for(x in lv){
                    #if(!NAPE_RELEASE_BUILD)
                    if(x==null)throw "Error: Vec2List contains null objects";
                    #end
                    {
                        #if(!NAPE_RELEASE_BUILD)
                        if(x!=null&&x.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                        #end
                    };
                    this.localVerts.push(x.copy());
                }
            }
            else if(#if flash untyped __is__(localVerts,GeomPoly)#else Std.is(localVerts,GeomPoly)#end){
                var lv:GeomPoly=localVerts;
                {
                    #if(!NAPE_RELEASE_BUILD)
                    if(lv!=null&&lv.zpp_disp)throw "Error: "+"GeomPoly"+" has been disposed and cannot be used!";
                    #end
                };
                var verts:ZPP_GeomVert=lv.zpp_inner.vertices;
                if(verts!=null){
                    var vite=verts;
                    do{
                        var x=Vec2.get(vite.x,vite.y);
                        vite=vite.next;
                        this.localVerts.push(x.copy());
                        x.dispose();
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
        {
            if(#if flash untyped __is__(localVerts,Array)#else Std.is(localVerts,Array)#end){
                var lv:Array<Vec2>=localVerts;
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
            else if(#if flash10 untyped __is__(localVerts,ZPP_Const.vec2vector)#else false #end){
                #if flash10 var lv:flash.Vector<Vec2>=localVerts;
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
            else if(#if flash untyped __is__(localVerts,Vec2List)#else Std.is(localVerts,Vec2List)#end){
                var lv:Vec2List=localVerts;
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
        if(material==null){
            if(ZPP_Material.zpp_pool==null){
                zpp_inner.material=new ZPP_Material();
                #if NAPE_POOL_STATS ZPP_Material.POOL_TOT++;
                ZPP_Material.POOL_ADDNEW++;
                #end
            }
            else{
                zpp_inner.material=ZPP_Material.zpp_pool;
                ZPP_Material.zpp_pool=zpp_inner.material.next;
                zpp_inner.material.next=null;
                #if NAPE_POOL_STATS ZPP_Material.POOL_CNT--;
                ZPP_Material.POOL_ADD++;
                #end
            }
            zpp_inner.material.alloc();
        };
        else this.material=material;
        if(filter==null){
            if(ZPP_InteractionFilter.zpp_pool==null){
                zpp_inner.filter=new ZPP_InteractionFilter();
                #if NAPE_POOL_STATS ZPP_InteractionFilter.POOL_TOT++;
                ZPP_InteractionFilter.POOL_ADDNEW++;
                #end
            }
            else{
                zpp_inner.filter=ZPP_InteractionFilter.zpp_pool;
                ZPP_InteractionFilter.zpp_pool=zpp_inner.filter.next;
                zpp_inner.filter.next=null;
                #if NAPE_POOL_STATS ZPP_InteractionFilter.POOL_CNT--;
                ZPP_InteractionFilter.POOL_ADD++;
                #end
            }
            zpp_inner.filter.alloc();
        };
        else this.filter=filter;
        zpp_inner_i.insert_cbtype(CbType.ANY_SHAPE.zpp_inner);
    }
}
