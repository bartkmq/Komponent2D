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
 * Axis Aligned Bounding Box (AABB)
 * <br/><br/>
 * Note that in many cases of an AABB object being returned by a Nape function
 * the AABB object will be marked internally as an 'immutable' AABB. This will
 * always be documented and trying to mutate such an AABB will result in an
 * error being thrown.
 */
@:final#if nape_swc@:keep #end
class AABB{
    /**
     * @private
     */
    public var zpp_inner:ZPP_AABB=null;
    /**
     * Construct a new AABB.
     * <br/><br/>
     * As input width/height are permitted to be negative it is not guaranteed
     * that the resultant AABB will have the same parameters as those
     * specified as the AABB parameters are guaranteed to always have positive
     * width/height, and for x/y to always be the top-left corner.
     *
     * @param x      The x coordinate of the top-left corner of the AABB.
     *               (default 0)
     * @param y      The y coordinate of the top-left corner of the AABB
     *               (default 0)
     * @param width  The width of the AABB. This value may be negative.
     *               (default 0)
     * @param height The height of the AABB. This value may be negative.
     *               (default 0)
     * @return       The newly constructed AABB object.
     */
    public function new(x:Float=0,y:Float=0,width:Float=0,height:Float=0){
        #if(!NAPE_RELEASE_BUILD)
        if((x!=x)||(y!=y)){
            throw "Error: AABB position cannot be NaN";
        }
        if((width!=width)||(height!=height)){
            throw "Error: AABB dimensions cannot be NaN";
        }
        #end
        zpp_inner=ZPP_AABB.get(x,y,x+width,y+height);
        zpp_inner.outer=this;
    }
    /**
     * Produce a copy of this AABB.
     * <br/><br/>
     * As would be expected, if you produce a copy of an 'immutable' AABB then
     * the copy will be 'mutable'.
     *
     * @return The copy of this AABB.
     */
    #if nape_swc@:keep #end
    public function copy(){
        zpp_inner.validate();
        return zpp_inner.copy().wrapper();
    }
    #if(flash9||openfl||nme)
    /**
     * Construct an AABB from an AS3 Rectangle object.
     * <br/><br/>
     * This method is only available on <code>flash</code> and
     * <code>openfl||nme</code> targets.
     *
     * @param rect The AS3 Rectangle to construct AABB from, this value
     *             must not be null.
     * @return The constructed AABB matching the input Rectangle.
     * @throws # If the input rectangle is null.
     */
    #if nape_swc@:keep #end
    public static function fromRect(rect:flash.geom.Rectangle):AABB{
        #if(!NAPE_RELEASE_BUILD)
        if(rect==null){
            throw "Error: Cannot create AABB from null Rectangle";
        }
        #end
        return new AABB(rect.x,rect.y,rect.width,rect.height);
    }
    /**
     * Create an AS3 Rectangle object from AABB.
     * <br/><br/>
     * This method is available only on <code>flash</code> and
     * <code>openfl||nme</code> targets.
     *
     * @return The AS3 Rectangle object representing AABB.
     */
    #if nape_swc@:keep #end
    public function toRect():flash.geom.Rectangle{
        return new flash.geom.Rectangle(x,y,width,height);
    }
    #end
    /**
     * The minimum bounds for the AABB.
     * <br/><br/>
     * Euivalent to the top-left corner.
     * <br/>
     * This Vec2 is intrinsically linked to the AABB so that modifications
     * to this object are reflected in changes to the AABB and vice-versa.
     * <br/><br/>
     * If the AABB is immutable, then this Vec2 will also be immutable.
     * <br/><br/>
     * This value can be set with the = operator, equivalent to performing
     * <code>aabb.min.set(value)</code>.
     * @default (0, 0)
     */
    #if nape_swc@:isVar #end
    public var min(get_min,set_min):Vec2;
    inline function get_min():Vec2{
        return zpp_inner.getmin();
    }
    inline function set_min(min:Vec2):Vec2{
        {
            {
                #if(!NAPE_RELEASE_BUILD)
                if(min!=null&&min.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                #end
            };
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner._immutable){
                throw "Error: AABB is immutable";
            }
            if(min==null){
                throw "Error: Cannot assign null to AABB::"+"min";
            }
            if((x!=x)||(y!=y)){
                throw "Error: AABB::"+"min"+" components cannot be NaN";
            }
            if("min"=="min"){
                if(min.x>max.x)throw "Error: Assignment would cause negative width";
                if(min.y>max.y)throw "Error: Assignment would cause negative height";
            }
            else{
                if(min.x<min.x)throw "Error: Assignment would cause negative width";
                if(min.y<min.y)throw "Error: Assignment would cause negative height";
            }
            #end
            this.min.set(min);
        }
        return get_min();
    }
    /**
     * The maximum bounds for the AABB.
     * <br/><br/>
     * Euivalent to the bottom-right corner.
     * <br/>
     * This Vec2 is intrinsically linked to the AABB so that modifications
     * to this object are reflected in changes to the AABB and vice-versa.
     * <br/><br/>
     * If the AABB is immutable, then this Vec2 will also be immutable.
     * <br/><br/>
     * This value can be set with the = operator, equivalent to performing
     * <code>aabb.max.set(value)</code>.
     * @default (0, 0)
     */
    #if nape_swc@:isVar #end
    public var max(get_max,set_max):Vec2;
    inline function get_max():Vec2{
        return zpp_inner.getmax();
    }
    inline function set_max(max:Vec2):Vec2{
        {
            {
                #if(!NAPE_RELEASE_BUILD)
                if(max!=null&&max.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                #end
            };
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner._immutable){
                throw "Error: AABB is immutable";
            }
            if(max==null){
                throw "Error: Cannot assign null to AABB::"+"max";
            }
            if((x!=x)||(y!=y)){
                throw "Error: AABB::"+"max"+" components cannot be NaN";
            }
            if("max"=="min"){
                if(max.x>max.x)throw "Error: Assignment would cause negative width";
                if(max.y>max.y)throw "Error: Assignment would cause negative height";
            }
            else{
                if(max.x<min.x)throw "Error: Assignment would cause negative width";
                if(max.y<min.y)throw "Error: Assignment would cause negative height";
            }
            #end
            this.max.set(max);
        }
        return get_max();
    }
    /**
     * The x coordinate of the AABB's top-left corner.
     * <br/><br/>
     * Equivalent to accessing/mutating min.x.
     * @default 0
     */
    #if nape_swc@:isVar #end
    public var x(get_x,set_x):Float;
    inline function get_x():Float{
        zpp_inner.validate();
        return zpp_inner.minx;
    }
    inline function set_x(x:Float):Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner._immutable){
                throw "Error: AABB is immutable";
            }
            #end
            if(this.x!=x){
                #if(!NAPE_RELEASE_BUILD)
                if((x!=x))throw "Error: AABB::"+"x"+" cannot be NaN";
                #end
                zpp_inner.maxx+=x-zpp_inner.minx;
                zpp_inner.minx=x;
                zpp_inner.invalidate();
            }
        }
        return get_x();
    }
    /**
     * The y coordinate of the AABB's top-left corner.
     * <br/><br/>
     * Equivalent to accessing/mutating min.y.
     * @default 0
     */
    #if nape_swc@:isVar #end
    public var y(get_y,set_y):Float;
    inline function get_y():Float{
        zpp_inner.validate();
        return zpp_inner.miny;
    }
    inline function set_y(y:Float):Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner._immutable){
                throw "Error: AABB is immutable";
            }
            #end
            if(this.y!=y){
                #if(!NAPE_RELEASE_BUILD)
                if((y!=y))throw "Error: AABB::"+"y"+" cannot be NaN";
                #end
                zpp_inner.maxy+=y-zpp_inner.miny;
                zpp_inner.miny=y;
                zpp_inner.invalidate();
            }
        }
        return get_y();
    }
    /**
     * width of AABB.
     * <br/><br/>
     * This value is and must always be positive.
     * @default 0
     */
    #if nape_swc@:isVar #end
    public var width(get_width,set_width):Float;
    inline function get_width():Float{
        zpp_inner.validate();
        return zpp_inner.width();
    }
    inline function set_width(width:Float):Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner._immutable){
                throw "Error: AABB is immutable";
            }
            #end
            if(this.width!=width){
                #if(!NAPE_RELEASE_BUILD)
                if((width!=width)){
                    throw "Error: AABB::"+"width"+" cannot be NaN";
                }
                if(width<0){
                    throw "Error: AABB::"+"width"+" ("+width+") must be >= 0";
                }
                #end
                zpp_inner.maxx=this.x+width;
                zpp_inner.invalidate();
            }
        }
        return get_width();
    }
    /**
     * height of AABB.
     * <br/><br/>
     * This value is and must always be positive.
     * @default 0
     */
    #if nape_swc@:isVar #end
    public var height(get_height,set_height):Float;
    inline function get_height():Float{
        zpp_inner.validate();
        return zpp_inner.height();
    }
    inline function set_height(height:Float):Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner._immutable){
                throw "Error: AABB is immutable";
            }
            #end
            if(this.height!=height){
                #if(!NAPE_RELEASE_BUILD)
                if((height!=height)){
                    throw "Error: AABB::"+"height"+" cannot be NaN";
                }
                if(height<0){
                    throw "Error: AABB::"+"height"+" ("+height+") must be >= 0";
                }
                #end
                zpp_inner.maxy=this.y+height;
                zpp_inner.invalidate();
            }
        }
        return get_height();
    }
    /**
     * @private
     */
    @:keep public function toString(){
        zpp_inner.validate();
        return zpp_inner.toString();
    }
}
