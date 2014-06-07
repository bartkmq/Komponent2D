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
 * A general MxN dimensional matrix.
 * <br/><br/>
 * This object is not often used in Nape :)
 */
@:final#if nape_swc@:keep #end
class MatMN{
    /**
     * @private
     */
    public var zpp_inner:ZPP_MatMN=null;
    /**
     * The number of rows in the matrix.
     */
    #if nape_swc@:isVar #end
    public var rows(get_rows,never):Int;
    inline function get_rows():Int{
        return zpp_inner.m;
    }
    /**
     * The number of columns in the matrix.
     */
    #if nape_swc@:isVar #end
    public var cols(get_cols,never):Int;
    inline function get_cols():Int{
        return zpp_inner.n;
    }
    /**
     * Access element at index.
     *
     * @param row The row of the matrix to access.
     * @param col the column of the matrix to access.
     * @return The element at given (row,col) index.
     * @throws # If access is out of range.
     */
    #if NAPE_NO_INLINE#else inline #end
    public function x(row:Int,col:Int):Float{
        #if(!NAPE_RELEASE_BUILD)
        if(row<0||col<0||row>=rows||col>=cols){
            throw "Error: MatMN indices out of range";
        }
        #end
        return zpp_inner.x[(row*cols)+col];
    }
    /**
     * Set element at index.
     *
     * @param row The row of the matrix to set.
     * @param col The column of the matrix to set.
     * @param x The value to set at given (row,col) index.
     * @return The value of matrix at given index after set. (Always
     *         equal to the x parameter)
     * @throws # If index is out of range.
     */
    #if NAPE_NO_INLINE#else inline #end
    public function setx(row:Int,col:Int,x:Float):Float{
        #if(!NAPE_RELEASE_BUILD)
        if(row<0||col<0||row>=rows||col>=cols){
            throw "Error: MatMN indices out of range";
        }
        #end
        return zpp_inner.x[(row*cols)+col]=x;
    }
    /**
     * Construct a new Matrix.
     *
     * @param rows The number of rows in matrix.
     * @param cols The number of columns in matrix.
     * @return The constructed Matrix.
     * @throws # If rows or columns is negative or 0.
     */
    #if flib@:keep function flibopts_0(){}
    #end
    public function new(rows:Int,cols:Int){
        #if(!NAPE_RELEASE_BUILD)
        if(rows<=0||cols<=0){
            throw "Error: MatMN::dimensions cannot be < 1";
        }
        #end
        zpp_inner=new ZPP_MatMN(rows,cols);
        zpp_inner.outer=this;
    }
    /**
     * @private
     */
    @:keep public function toString(){
        var ret="{ ";
        var fst=true;
        for(i in 0...rows){
            if(!fst)ret+="; ";
            fst=false;
            for(j in 0...cols)ret+=x(i,j)+" ";
        }
        ret+="}";
        return ret;
    }
    /**
     * Transpose matrix, returning a new Matrix.
     *
     * @return The transposed matrix.
     */
    public function transpose():MatMN{
        var ret=new MatMN(cols,rows);
        for(i in 0...rows){
            for(j in 0...cols)ret.setx(j,i,x(i,j));
        }
        return ret;
    }
    /**
     * Multiple this matrix with another.
     * <br/><br/>
     * This operation is only valid if the number of columns
     * in this matrix, is equal to the number of rows in the input
     * matrix.
     * <br/>
     * The result of the multiplication is returned as a new matrix.
     *
     * @param matrix The matrix to multiple with.
     * @return The result of the multiplication
     * @throws If matrix dimensions are not compatible.
     */
    public function mul(matrix:MatMN):MatMN{
        var y=matrix;
        #if(!NAPE_RELEASE_BUILD)
        if(cols!=y.rows){
            throw "Error: Matrix dimensions aren't compatible";
        }
        #end
        var ret=new MatMN(rows,y.cols);
        for(i in 0...rows){
            for(j in 0...y.cols){
                var v=0.0;
                for(k in 0...cols)v+=x(i,k)*y.x(k,j);
                ret.setx(i,j,v);
            }
        }
        return ret;
    }
}
