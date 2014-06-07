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
import nape.shape.Polygon;
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
 * Edge class providing internal details of Polygon.
 */
@:final#if nape_swc@:keep #end
class Edge{
    /**
     * @private
     */
    public var zpp_inner:ZPP_Edge=null;
    /**
     * @private
     */
    public function new(){
        #if(!NAPE_RELEASE_BUILD)
        if(!ZPP_Edge.internal)throw "Error: Cannot instantiate an Edge derp!";
        #end
    }
    /**
     * Reference to Polygon this Edge belongs to.
     */
    #if nape_swc@:isVar #end
    public var polygon(get_polygon,never):Polygon;
    inline function get_polygon():Polygon{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.polygon==null)throw "Error: Edge not current in use";
        #end
        return zpp_inner.polygon.outer_zn;
    }
    /**
     * Normal of edge in local coordinates.
     * <br/><br/>
     * This Vec2 is immutable.
     */
    #if nape_swc@:isVar #end
    public var localNormal(get_localNormal,never):Vec2;
    inline function get_localNormal():Vec2{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.polygon==null)throw "Error: Edge not current in use";
        #end
        if(zpp_inner.wrap_lnorm==null)zpp_inner.getlnorm();
        return zpp_inner.wrap_lnorm;
    }
    /**
     * Normal of edge in world coordinates.
     * <br/><br/>
     * This Vec2 is immutable, and may be accessed even if the related Polygon
     * is not part of a Body but queries to its values will result in a debug
     * build error.
     */
    #if nape_swc@:isVar #end
    public var worldNormal(get_worldNormal,never):Vec2;
    inline function get_worldNormal():Vec2{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.polygon==null)throw "Error: Edge not current in use";
        #end
        if(zpp_inner.wrap_gnorm==null)zpp_inner.getgnorm();
        return zpp_inner.wrap_gnorm;
    }
    /**
     * Length of edge.
     */
    #if nape_swc@:isVar #end
    public var length(get_length,never):Float;
    inline function get_length():Float{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.polygon==null)throw "Error: Edge not current in use";
        #end
        zpp_inner.polygon.validate_laxi();
        return zpp_inner.length;
    }
    /**
     * Local projection of polygon onto edge axis.
     */
    #if nape_swc@:isVar #end
    public var localProjection(get_localProjection,never):Float;
    inline function get_localProjection():Float{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.polygon==null)throw "Error: Edge not current in use";
        #end
        zpp_inner.polygon.validate_laxi();
        return zpp_inner.lprojection;
    }
    /**
     * World projection of polygon to edge axis.
     * <br/><br/>
     * This value can only be accessed if related Polygon is part of a Body.
     */
    #if nape_swc@:isVar #end
    public var worldProjection(get_worldProjection,never):Float;
    inline function get_worldProjection():Float{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.polygon==null)throw "Error: Edge not current in use";
        if(zpp_inner.polygon.body==null)throw "Error: Edge world projection only makes sense for Polygons contained within a rigid body";
        #end
        zpp_inner.polygon.validate_gaxi();
        return zpp_inner.gprojection;
    }
    /**
     * Reference to first local vertex for edge.
     */
    #if nape_swc@:isVar #end
    public var localVertex1(get_localVertex1,never):Vec2;
    inline function get_localVertex1():Vec2{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.polygon==null)throw "Error: Edge not current in use";
        #end
        zpp_inner.polygon.validate_laxi();
        return zpp_inner.lp0.wrapper();
    }
    /**
     * Reference to second local vertex for edge.
     */
    #if nape_swc@:isVar #end
    public var localVertex2(get_localVertex2,never):Vec2;
    inline function get_localVertex2():Vec2{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.polygon==null)throw "Error: Edge not current in use";
        #end
        zpp_inner.polygon.validate_laxi();
        return zpp_inner.lp1.wrapper();
    }
    /**
     * Reference to first world vertex for edge.
     */
    #if nape_swc@:isVar #end
    public var worldVertex1(get_worldVertex1,never):Vec2;
    inline function get_worldVertex1():Vec2{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.polygon==null)throw "Error: Edge not current in use";
        #end
        zpp_inner.polygon.validate_gaxi();
        return zpp_inner.gp0.wrapper();
    }
    /**
     * Reference to second world vertex for edge.
     */
    #if nape_swc@:isVar #end
    public var worldVertex2(get_worldVertex2,never):Vec2;
    inline function get_worldVertex2():Vec2{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.polygon==null)throw "Error: Edge not current in use";
        #end
        zpp_inner.polygon.validate_gaxi();
        return zpp_inner.gp1.wrapper();
    }
    /**
     * @private
     */
    @:keep public function toString(){
        if(zpp_inner.polygon==null)return "Edge(object-pooled)";
        else if(zpp_inner.polygon.body==null){
            zpp_inner.polygon.validate_laxi();
            return "{ localNormal : "+("{ x: "+zpp_inner.lnormx+" y: "+zpp_inner.lnormy+" }")+" }";
        }
        else{
            zpp_inner.polygon.validate_gaxi();
            return "{ localNormal : "+("{ x: "+zpp_inner.lnormx+" y: "+zpp_inner.lnormy+" }")+" worldNormal : "+("{ x: "+zpp_inner.gnormx+" y: "+zpp_inner.gnormy+" }")+" }";
        }
    }
}
