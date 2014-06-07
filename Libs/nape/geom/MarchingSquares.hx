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
 * Iso-surface extraction into polygons.
 * <br/><br/>
 * This class, with only one static method provides an interface to
 * an algorithm which will, when given a function mapping each point
 * in a given AABB to a scalar value extract approximated polygons
 * which represent the region of the AABB where the function returns
 * a negative value.
 * <br/><br/>
 * This function could be a mathematical function like the equation of
 * a circle: <code> function (x, y) return (x*x + y*y) - r*r </code>
 * <br/>
 * Or something more practical like the biased alpha value interpolated
 * from a Bitmap:
 * <pre>
 * function (x, y) {
 *    var ix = if (x < 0) 0 else if (x >= bitmap.width - 1) bitmap.width - 2 else Std.int(x);
 *    var iy = if (y < 0) 0 else if (y >= bitmap.height - 1) bitmap.height - 2 else Std.int(y);
 *    var fx = x - ix;
 *    var fy = y - iy;
 *    var gx = 1 - fx;
 *    var gy = 1 - fy;
 *
 *    var a00 = bitmap.getPixel32(ix, iy) >>> 24;
 *    var a01 = bitmap.getPixel32(ix, iy + 1) >>> 24;
 *    var a10 = bitmap.getPixel32(ix + 1, iy) >>> 24;
 *    var a11 = bitmap.getPixel32(ix + 1, iy + 1) >>> 24;
 *
 *    return 0x80 - (gx*gy*a00 + fx*gy*a10 + gx*fy*a01 + fx*fy*a11);
 * }
 * </pre>
 * For 'flash', we must wrap this in an IsoFunction interface to be used
 * by MarchingSquares for performance reasons:
 * <pre>
 * class BitmapIsoFunction implements nape.geom.IsoFunction {
 *     public function iso(x:Float, y:Float):Float {
 *         ...
 *     }
 * }
 * </pre>
 * This function is converted into a set of polygons by sampling along regular
 * grid points, and then recursively interpolating along cell edges based on
 * the function provided to find the point in space along that edge where the
 * function is approximately 0.
 * <br/><br/>
 * From this we generate polygons in each grid cell, which are then by default
 * combined into larger, weakly simply polygons suitable for use in the
 * decomposition routines of GeomPoly like convexDecomposition!
 * <br/><br/>
 * The runtime of the algorithm is O(N+K) for N number of cells and K number
 * of output vertices (A final pass is made to remove unnecessary vertices).
 */
@:final#if nape_swc@:keep #end
class MarchingSquares{
    /**
     * Execute marching squares algorithm over region of space.
     * <br/><br/>
     * We can, optionally provide a subgrid argument which, when non null
     * will invoke this algorithm seperately on each subgrid cell of the region
     * in space, instead of on the entire region at once. This can be very useful
     * as shown in the DestructibleTerrain demo where regions of a terrain are
     * recomputed with marching squares without needing to regenerate the whole
     * of the terrain.
     *
     * @param iso The iso-function defining the regions where polygons should
     *            be extracted, a negative return indicates a region to be extracted.
     *            This function need not be continuous, but if it is continuous
     *            then more accurate results will be given for the same input
     *            parameters.
     * @param bounds The AABB representing the region of space to be converted.
     *               The arguments to the iso-function will be in the same region.
     * @param cellsize The dimensions of each cell used individual polygon extraction.
     *                 Smaller cells will give more accurate results at a greater
     *                 cost permitting smaller features to be extracted.
     * @param quality This is the number of recursive interpolations which will be
     *                performed along cell edges. If the iso-function is not
     *                continuous, then this value should be increased to get better
     *                accuracy. (default 2)
     * @param subgrid When supplied, the region of space will be first subdivided
     *                into cells with these given dimensions, and each cell treated
     *                as a seperate invocation of this method, this value should
     *                obviously be greater than cellsize or it would be a bit
     *                pointless. (default null)
     * @param combine When True, the polygons generated in each cell of the grid
     *                will be combined into the largest possible weakly-simple
     *                polygons representing the same area. These polygons will
     *                always be suitable for decomposition in Nape. (default true)
     * @param output When supplied, GeomPoly will be inserted into the list (via add)
     *               instead of creating a new GeomPolyList object.
     * @return A list of GeomPoly representing the results of the extraction.
     * @throws # If iso, bounds or cellsize argument is null.
     * @throws # If cellsize is disposed, or its components have 0, or negative values.
     * @throws # If quality is less than 0.
     * @throws # If subgrid is not null, but is disposed or has zero or negative
     *           component values.
     */
    static public function run(iso:IsoFunctionDef,bounds:AABB,cellsize:Vec2,quality:Int=2,subgrid:Vec2=null,combine:Bool=true,output:GeomPolyList=null){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(cellsize!=null&&cellsize.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(subgrid!=null&&subgrid.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(iso==null){
            throw "Error: MarchingSquares requires an iso function to operate";
        }
        if(bounds==null){
            throw "Error: MarchingSquares requires an AABB to define bounds of surface extraction";
        }
        if(cellsize==null){
            throw "Error: MarchingSquares requires a Vec2 to define cell size for surface extraction";
        }
        if(cellsize.x<=0||cellsize.y<=0){
            throw "Error: MarchingSquares cannot operate with non-positive cell dimensions";
        }
        if(quality<0){
            throw "Error: MarchingSquares cannot use a negative quality value for interpolation";
        }
        if(subgrid!=null&&(subgrid.x<=0||subgrid.y<=0)){
            throw "Error: MarchingSquares cannot with non-positive sub-grid dimensions";
        }
        #end
        var ret=(output!=null?output:new GeomPolyList());
        if(subgrid==null){
            ZPP_MarchingSquares.run(iso,bounds.x,bounds.y,bounds.max.x,bounds.max.y,cellsize,quality,combine,ret);
        }
        else{
            var xp=bounds.width/subgrid.x;
            var yp=bounds.height/subgrid.y;
            var xn:Int=(#if flash9 untyped __int__(xp)#else Std.int(xp)#end);
            var yn:Int=(#if flash9 untyped __int__(yp)#else Std.int(yp)#end);
            if(xn!=xp)xn++;
            if(yn!=yp)yn++;
            for(x in 0...xn){
                var x0=bounds.x+subgrid.x*x;
                var x1=if(x==xn-1)bounds.max.x else(x0+subgrid.x);
                for(y in 0...yn){
                    var y0=bounds.y+subgrid.y*y;
                    var y1=if(y==yn-1)bounds.max.y else(y0+subgrid.y);
                    ZPP_MarchingSquares.run(iso,x0,y0,x1,y1,cellsize,quality,combine,ret);
                }
            }
        }
        ({
            if(({
                cellsize.zpp_inner.weak;
            })){
                cellsize.dispose();
                true;
            }
            else{
                false;
            }
        });
        if(subgrid!=null){
            ({
                if(({
                    subgrid.zpp_inner.weak;
                })){
                    subgrid.dispose();
                    true;
                }
                else{
                    false;
                }
            });
        }
        return ret;
    }
}
