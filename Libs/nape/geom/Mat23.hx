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
/**
 * 2D Matrix class representing affine transformations:
 * <pre>
 * [ a  b  tx ]
 * [ c  d  ty ]
 * [ 0  0  1  ]
 * </pre>
 *
 * Note that in AS3, flash.geom.Matrix has 'b' and 'c' swapped! so if you are
 * converting between flash.geom.Matrix and nape.geom.Mat23 you should use the
 * methods provided to avoid any mistakes with this.
 *
 */
@:final#if nape_swc@:keep #end
class Mat23{
    /**
     * @private
     */
    public var zpp_inner:ZPP_Mat23=null;
    /**
     * The (1,1) entry in Mat23:
     * <br/><br/>
     * <pre>
     * [ a  .  . ]
     * [ .  .  . ]
     * </pre>
     *
     * @default 1
     */
    #if nape_swc@:isVar #end
    public var a(get_a,set_a):Float;
    inline function get_a():Float{
        return zpp_inner.a;
    }
    inline function set_a(a:Float):Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if((a!=a)){
                throw "Error: Mat23::"+"a"+" cannot be NaN";
            }
            #end
            zpp_inner.a=a;
            zpp_inner.invalidate();
        }
        return get_a();
    }
    /**
     * The (1,2) entry in Mat23:
     * <br/><br/>
     * <pre>
     * [ .  b  . ]
     * [ .  .  . ]
     * </pre>
     *
     * @default 0
     */
    #if nape_swc@:isVar #end
    public var b(get_b,set_b):Float;
    inline function get_b():Float{
        return zpp_inner.b;
    }
    inline function set_b(b:Float):Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if((b!=b)){
                throw "Error: Mat23::"+"b"+" cannot be NaN";
            }
            #end
            zpp_inner.b=b;
            zpp_inner.invalidate();
        }
        return get_b();
    }
    /**
     * The (2,1) entry in Mat23:
     * <br/><br/>
     * <pre>
     * [ .  .  . ]
     * [ c  .  . ]
     * </pre>
     *
     * @default 0
     */
    #if nape_swc@:isVar #end
    public var c(get_c,set_c):Float;
    inline function get_c():Float{
        return zpp_inner.c;
    }
    inline function set_c(c:Float):Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if((c!=c)){
                throw "Error: Mat23::"+"c"+" cannot be NaN";
            }
            #end
            zpp_inner.c=c;
            zpp_inner.invalidate();
        }
        return get_c();
    }
    /**
     * The (2,2) entry in Mat23:
     * <br/><br/>
     * <pre>
     * [ .  .  . ]
     * [ .  d  . ]
     * </pre>
     *
     * @default 1
     */
    #if nape_swc@:isVar #end
    public var d(get_d,set_d):Float;
    inline function get_d():Float{
        return zpp_inner.d;
    }
    inline function set_d(d:Float):Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if((d!=d)){
                throw "Error: Mat23::"+"d"+" cannot be NaN";
            }
            #end
            zpp_inner.d=d;
            zpp_inner.invalidate();
        }
        return get_d();
    }
    /**
     * The (1,3) entry in Mat23 which represents x translation
     * <br/><br/>
     * <pre>
     * [ .  .  tx ]
     * [ .  .  .  ]
     * </pre>
     *
     * @default 0
     */
    #if nape_swc@:isVar #end
    public var tx(get_tx,set_tx):Float;
    inline function get_tx():Float{
        return zpp_inner.tx;
    }
    inline function set_tx(tx:Float):Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if((tx!=tx)){
                throw "Error: Mat23::"+"tx"+" cannot be NaN";
            }
            #end
            zpp_inner.tx=tx;
            zpp_inner.invalidate();
        }
        return get_tx();
    }
    /**
     * The (2,3) entry in Mat23 which represents y translation
     * <br/><br/>
     * <pre>
     * [ .  .  .  ]
     * [ .  .  ty ]
     * </pre>
     *
     * @default 0
     */
    #if nape_swc@:isVar #end
    public var ty(get_ty,set_ty):Float;
    inline function get_ty():Float{
        return zpp_inner.ty;
    }
    inline function set_ty(ty:Float):Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if((ty!=ty)){
                throw "Error: Mat23::"+"ty"+" cannot be NaN";
            }
            #end
            zpp_inner.ty=ty;
            zpp_inner.invalidate();
        }
        return get_ty();
    }
    /**
     * Construct new Mat23.
     * <br/><br/>
     * <pre>
     * [ a  b  tx ]
     * [ c  d  ty ]
     * </pre>
     *
     * @param a  The (1,1) entry in matrix (default 1)
     * @param b  The (1,2) entry in matrix (default 0)
     * @param c  The (2,1) entry in matrix (default 0)
     * @param d  The (2,2) entry in matrix (default 1)
     * @param tx The (1,3) entry in matrix (default 0)
     * @param ty The (2,3) entry in matrix (default 0)
     * @return   The newly constructed Mat23.
     */
    public function new(a:Float=1.0,b:Float=0.0,c:Float=0.0,d:Float=1.0,tx:Float=0.0,ty:Float=0.0){
        zpp_inner=ZPP_Mat23.get();
        zpp_inner.outer=this;
        this.a=a;
        this.b=b;
        this.tx=tx;
        this.c=c;
        this.d=d;
        this.ty=ty;
    }
    /**
     * Produce copy of this Mat23
     *
     * @return The new Mat23 representing copy of this.
     */
    #if nape_swc@:keep #end
    public function copy(){
        return new Mat23(a,b,c,d,tx,ty);
    }
    /**
     * Set values of matrix from another.
     *
     * @param matrix The matrix to copy values from.
     * @return       A reference to this Mat23.
     * @throws # if matrix argument is null.
     */
    public function set(matrix:Mat23):Mat23{
        #if(!NAPE_RELEASE_BUILD)
        if(matrix==null){
            throw "Error: Cannot set form null matrix";
        }
        #end
        zpp_inner.set(matrix.zpp_inner);
        zpp_inner.invalidate();
        return this;
    }
    /**
     * Set values of matrix from numbers.
     * <br/><br/>
     * So that: <code>mat.setAs(...)</code> is
     * semantically equivalent to: <code>mat.set(new Mat23(...))</code>
     * <br/><br/>
     * @param a  The value to which the (1,1) entry will be set (default 1)
     * @param b  The value to which the (1,2) entry will be set (default 0)
     * @param c  The value to which the (2,1) entry will be set (default 0)
     * @param d  The value to which the (2,2) entry will be set (default 1)
     * @param tx The value to which the (1,3) entry will be set (default 0)
     * @param ty The value to which the (2,3) entry will be set (default 0)
     * @return   A reference to this Mat23.
     */
    public function setAs(a:Float=1.0,b:Float=0.0,c:Float=0.0,d:Float=1.0,tx:Float=0.0,ty:Float=0.0):Mat23{
        zpp_inner.setas(a,b,c,d,tx,ty);
        zpp_inner.invalidate();
        return this;
    }
    /**
     * Reset matrix to identity.
     * <br/><br/>
     * Equivalent to calling setAs with default argument values.
     * <br/><br/>
     * @return A reference to this Mat23.
     */
    public#if NAPE_NO_INLINE#else inline #end
    function reset():Mat23{
        return setAs();
    }
    #if(flash9||openfl||nme)/**
     * Create a Mat23 matrix from a given AS3 flash.geom.Matrix.
     * <br/><br/>
     * This method should be used in preference to doing so manually
     * as the allocation of matrix entries to name is different and
     * it is easy to make this mistake!
     * <br/><br/>
     * This method is only available on <code>flash</code> and
     * <code>openfl||nme</code> targets.
     *
     * @param matrix The AS3 Matrix to create Mat23 from. This value must
     *               not be null.
     * @return       The constructed Mat23 matching AS3 Matrix.
     */
    #if nape_swc@:keep #end
    public static function fromMatrix(matrix:flash.geom.Matrix):Mat23{
        var m=matrix;
        return new Mat23(m.a,m.c,m.b,m.d,m.tx,m.ty);
    }
    /**
     * Create an AS3 flash.geom.Matrix from this Mat23.
     * <br/><br/>
     * This method should be used in preference to doing so manually
     * as the allocation of matrix entries to name is different and
     * it is easy to make this mistake!
     *
     * @param output If supplied, this Matrix will have its properties
     *               populated insteaad of creating a new Matrix.
     * @preturn The constructed AS3 Matrix.
     */
    #if nape_swc@:keep #end
    public function toMatrix(output:flash.geom.Matrix=null):flash.geom.Matrix{
        if(output==null)output=new flash.geom.Matrix();
        output.a=a;
        output.b=c;
        output.c=b;
        output.d=d;
        output.tx=tx;
        output.ty=ty;
        return output;
    }
    #end
    /**
     * Construct a Mat23 representing a clockwise rotation.
     *
     * <pre>
     * [ cos angle  -sin angle  0 ]
     * [ sin angle   cos angle  0 ]
     * </pre>
     *
     * @param angle The clockwise rotation in radians
     * @return      The rotation matrix.
     */
    #if nape_swc@:keep #end
    public static function rotation(angle:Float):Mat23{
        #if(!NAPE_RELEASE_BUILD)
        if((angle!=angle)){
            throw "Error: Cannot create rotation matrix with NaN angle";
        }
        #end
        var cos=Math.cos(angle);
        var sin=Math.sin(angle);
        return new Mat23(cos,-sin,sin,cos,0,0);
    }
    /**
     * Construct a Mat23 representing a translation
     *
     * <pre>
     * [ 1  0  tx ]
     * [ 0  1  ty ]
     * </pre>
     *
     * @param tx The x translation.
     * @param ty The y translation.
     * @return   The translation matrix.
     */
    #if nape_swc@:keep #end
    public static function translation(tx:Float,ty:Float):Mat23{
        return new Mat23(1,0,0,1,tx,ty);
    }
    /**
     * Construct a Mat23 representing a scaling
     *
     * <pre>
     * [ sx  0  0 ]
     * [ 0  sy  0 ]
     * </pre>
     *
     * @param sx The x factor of scaling.
     * @param sy The y factor of scaling.
     * @return   The scaling matrix.
     */
    #if nape_swc@:keep #end
    public static function scale(sx:Float,sy:Float):Mat23{
        return new Mat23(sx,0,0,sy,0,0);
    }
    /**
     * (readonly) The determinant of this matrix.
     * <br/><br/>
     * This represents the factor of change in area
     * for a region of the plane after transformation by matrix.
     * <br/><br/>
     * A determinant of 0 signifies that the matrix is not invertible.
     * <br/><br/>
     * A negative determinant signifies that for example, a clockwise wound
     * polygon would be transformed into a counter-clockwise polygon.
     *
     * @default 1
     */
    #if nape_swc@:isVar #end
    public var determinant(get_determinant,never):Float;
    inline function get_determinant():Float{
        return(a*d)-(b*c);
    }
    /**
     * Determine if the matrix is singular.
     * This check is based on computing the condition number of the matrix
     * by the Frobenius norm, and comparing against 2 / epsilon.
     * <br/><br/>
     * If matrix is singular, then inversion of the matrix cannot be performed
     *
     * @return True, if matrix is singular.
     */
    #if nape_swc@:keep #end
    public function singular():Bool{
        var norm=(a*a)+(b*b)+(c*c)+(d*d);
        var limit=determinant;
        if(limit<0)limit=-limit;
        return(norm>(Config.illConditionedThreshold*limit));
    }
    /**
     * Compute the inverse of this matrix, returning the inverse in a new
     * Mat23 object.
     * <br/><br/>
     * The inverse is such that mat.concat(mat.inverse()) is the identity
     * matrix, as well as mat.inverse().concat(mat)
     *
     * @return The inverse matrix.
     * @throws # If matrix is singular.
     */
    #if nape_swc@:keep #end
    public function inverse():Mat23{
        #if(!NAPE_RELEASE_BUILD)
        if(singular()){
            throw "Error: Matrix is singular and cannot be inverted";
        }
        #end
        var idet=1.0/determinant;
        return new Mat23((d*idet),(-b*idet),(-c*idet),(a*idet),((b*ty)-(d*tx))*idet,((c*tx)-(a*ty))*idet);
    }
    /**
     * Compute the transpose of this matrix, returning the transpose in a new
     * Mat23 object.
     * <br/><br/>
     * Technically speaking, we cannot transpose a matrix if the tx/ty values
     * are non-zero as the implicit bottom row of matrix must be (0, 0, 1)
     * so the tx/ty values of output matrix are set so that should the main
     * 2x2 block of the matrix be orthogonal (Representing a rotation), then
     * the transpose will be able to act as the matrix inverse.
     * <pre>
     * var mat = Mat23.rotation(..).concat(Mat23.translation(...));
     * trace(mat.concat(mat.transpose())); // Identity matrix
     * trace(mat.concat(mat.inverse())); // Identity matrix
     * </pre>
     * If the main 2x2 block of matrix is 'not' orthogonal, then the transpose
     * will not be equal to the inverse.
     *
     * @return The transposed matrix.
     */
    #if nape_swc@:keep #end
    public function transpose():Mat23{
        return new Mat23(a,c,b,d,-a*tx-c*ty,-b*tx-d*ty);
    }
    /**
     * Concatenate matrices (left-multiplication), returning new Mat23.
     * <br/><br/>
     * <code>mat1.concat(mat2)</code> is the transformation that first
     * performs transformation represented by mat1, followed by transformation
     * represented by mat2.
     * <br/>
     *
     * @param matrix Matrix to concatenate with.
     * @return       The result of the concatenation.
     * @throws # If matrix argument is null.
     */
    #if nape_swc@:keep #end
    public function concat(matrix:Mat23):Mat23{
        var m=matrix;
        #if(!NAPE_RELEASE_BUILD)
        if(m==null){
            throw "Error: Cannot concatenate with null Mat23";
        }
        #end
        return new Mat23(((m.a*a)+(m.b*c)),((m.a*b)+(m.b*d)),((m.c*a)+(m.d*c)),((m.c*b)+(m.d*d)),(m.a*tx)+(m.b*ty)+m.tx,(m.c*tx)+(m.d*ty)+m.ty);
    }
    /**
     * Transform a Vec2 by this matrix in new Vec2.
     * <br/><br/>
     * The Vec2 object will be allocated form the global object pool.
     *
     * @param point         The Vec2 to transform by this matrix.
     * @param noTranslation If true, then the input Vec2 will be treat as a
     *                      vector, rather than a point with the tx/ty values
     *                      treat as 0. (default false)
     * @param weak          If true, then the allocated Vec2 will be
     *                      automatically released to global object pool when
     *                      used as an argument to a Nape function.
     * @return              The result of the transformation as a newly
     *                      allocated (possibly weak) Vec2. (default false)
     * @throws # If point argument is null.
     */
    #if nape_swc@:keep #end
    public function transform(point:Vec2,noTranslation:Bool=false,weak:Bool=false):Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(point!=null&&point.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(point==null){
            throw "Error: Cannot transform null Vec2";
        }
        #end
        var ret:Vec2;
        if(noTranslation){
            ret=Vec2.get((point.x*a)+(point.y*b),(point.x*c)+(point.y*d),weak);
        }
        else{
            ret=Vec2.get((point.x*a)+(point.y*b)+tx,(point.x*c)+(point.y*d)+ty,weak);
        }
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
     * Perform inverse transformation with Vec2, returning new Vec2.
     * <br/><br/>
     * The matrix inverse will be performed implicitly and should this
     * method be called many times for the same Mat23, it would be better
     * to instead compute the matrix inverse only once.
     * <br/><br/>
     * The new Vec2 will be allocated from the global object pool.
     *
     * @param point         The Vec2 to transform.
     * @param noTranslation If true then the input Vec2 will be treat as a
     *                      vector instead of a point, treating the tx/ty
     *                      values of this Mat23 as though they were 0.
     *                      (default false)
     * @param weak          If true, then the allocated Vec2 will be
     *                      automatically released to global object pool when
     *                      used as an argument to a Nape function.
     * @return              The result of the transformation as a newly
     *                      allocated (possibly weak) Vec2. (default false)
     * @throws # If matrix is singular.
     * @throws # If point argument is null.
     */
    #if nape_swc@:keep #end
    public function inverseTransform(point:Vec2,noTranslation:Bool=false,weak:Bool=false):Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(point!=null&&point.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(point==null){
            throw "Error: Cannot transform null Vec2";
        }
        if(singular()){
            throw "Error: Matrix is singular and inverse transformation cannot be performed";
        }
        #end
        var idet=1.0/determinant;
        var ret:Vec2;
        if(noTranslation){
            ret=Vec2.get(((point.x*d)-(point.y*b))*idet,((point.y*a)-(point.x*c))*idet,weak);
        }
        else{
            var dx=point.x-tx;
            var dy=point.y-ty;
            ret=Vec2.get(((dx*d)-(dy*b))*idet,((dy*a)-(dx*c))*idet,weak);
        }
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
     * @private
     */
    @:keep public function toString():String{
        return "{ a: "+a+" b: "+b+" c: "+c+" d: "+d+" tx: "+tx+" ty: "+ty+" }";
    }
    /**
     * Determine if matrix is equiorthogonal
     * <br/><br/>
     * This is a term I invented after
     * failing to find an existing name. It describes that this matrix maps
     * circles into other circles (of not necessarigly the same radius). In
     * otherwords the matrix can be decomposed into a rotation, translation
     * and scaling of equal x/y factors.
     * <br/><br/>
     * This property is required for any Mat23 that is used to transform a
     * Circle, or any Body containing a Circle, or to transform a Debug view.
     * <br/><br/>
     * This is a weaker property than orthogonality which describes a mapping
     * to a circle of equal radius.
     * <br/><br/>
     * Mathematically speaking a matrix is equiorthogonal iff.
     * <code>transpose(M) * M = kI</code> for some non-zero scalar k.
     *
     * @return True if matrix is equiorthogonal.
     */
    #if nape_swc@:keep #end
    public function equiorthogonal():Bool{
        if(singular()){
            return false;
        }
        else{
            var x=(a*b)+(c*d);
            if((x*x)<Config.epsilon){
                var y=(a*a)+(b*b)-(c*c)-(d*d);
                return(y*y)<Config.epsilon;
            }
            else{
                return false;
            }
        }
    }
    /**
     * Determine if matrix is orthogonal
     * <br/><br/>
     * This property describes a matrix
     * which maps circles into other circles of equal radius. In otherwords
     * the matrix can be decomposed into a rotation and a translation.
     * <br/><br/>
     * Mathematically speaking a matrix is orthogonal iff.
     * <code>transpose(M) * M = I</code>.
     *
     * @return True if matrix is orthogonal.
     */
    #if nape_swc@:keep #end
    public function orthogonal():Bool{
        var x=(a*b)+(c*d);
        if((x*x)<Config.epsilon){
            var y=(a*a)+(b*b)-1;
            var z=(c*c)+(d*d)-1;
            return(y*y)<Config.epsilon&&(z*z)<Config.epsilon;
        }
        else{
            return false;
        }
    }
    /**
     * Equiorthogonalise matrix.
     * <br/><br/>
     * We do this by finding the 'nearest' orthogonal matrix;
     * scaling the basis vectors of matrix to their mean length
     * and applying an equal and opposite rotation to each basis vector to
     * make them perpendicular.
     *
     * @return A reference to this Mat23.
     * @throws # If matrix is singular.
     */
    #if nape_swc@:keep #end
    public function equiorthogonalise():Mat23{
        if(!equiorthogonal()){
            var k1=Math.sqrt((a*a)+(c*c));
            var k2=Math.sqrt((b*b)+(d*d));
            #if(!NAPE_RELEASE_BUILD)
            if((k1*k1)<Config.epsilon||(k2*k2)<Config.epsilon){
                throw "Error: Matrix is singular and cannot be "+"equiorthogonal"+"ised";
            }
            #end
            var k=(k1+k2)/2;
            k1=k/k1;
            k2=k/k2;
            a*=k1;
            c*=k1;
            b*=k2;
            d*=k2;
            var dot=(a*b)+(c*d);
            var ang=0.25*Math.PI-0.5*Math.acos(dot/(k*k));
            if(determinant>0){
                ang=-ang;
            }
            var sin=Math.sin(ang);
            var cos=Math.cos(ang);
            var a2=(a*cos)-(c*sin);
            var b2=(b*cos)+(d*sin);
            c=(c*cos)+(a*sin);
            a=a2;
            d=(d*cos)-(b*sin);
            b=b2;
            zpp_inner.invalidate();
        }
        return this;
    }
    /**
     * Orthogonalise matrix.
     * <br/><br/>
     * We do this by finding the 'nearest' orthogonal matrix;
     * normalising the basis vectors of matrix
     * and applying an equal and opposite rotation to each basis vector to
     * make them perpendicular.
     *
     * @return A reference to this Mat23.
     * @throws # If matrix is singular.
     */
    #if nape_swc@:keep #end
    public function orthogonalise():Mat23{
        if(!orthogonal()){
            var k1=Math.sqrt((a*a)+(c*c));
            var k2=Math.sqrt((b*b)+(d*d));
            #if(!NAPE_RELEASE_BUILD)
            if((k1*k1)<Config.epsilon||(k2*k2)<Config.epsilon){
                throw "Error: Matrix is singular and cannot be "+"orthogonal"+"ised";
            }
            #end
            var k=1;
            k1=k/k1;
            k2=k/k2;
            a*=k1;
            c*=k1;
            b*=k2;
            d*=k2;
            var dot=(a*b)+(c*d);
            var ang=0.25*Math.PI-0.5*Math.acos(dot/(k*k));
            if(determinant>0){
                ang=-ang;
            }
            var sin=Math.sin(ang);
            var cos=Math.cos(ang);
            var a2=(a*cos)-(c*sin);
            var b2=(b*cos)+(d*sin);
            c=(c*cos)+(a*sin);
            a=a2;
            d=(d*cos)-(b*sin);
            b=b2;
            zpp_inner.invalidate();
        }
        return this;
    }
}
