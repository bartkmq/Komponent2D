package nape.phys;
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
/**
 * Compound represents a grouping of physics objects into a single object.
 * <br/><br/>
 * This compound owns its constituents and works in the callback system and with
 * respect to adding/removing from a Space as a single object.
 * <pre>
 *       ____Cmp1_____
 *      /    /    &#92;   &#92;
 * Body1 Body2--Joint Cmp2
 *   |     |        &#92;  |
 * Shp1  Shp2        Body3
 *                     |
 *                    Shp3
 * </pre>
 * For example if you have a complex car built with several bodies and
 * constraints you might store this in a Compound providing an easy way
 * of removing/adding/copying the Car as well as being able to get a single
 * callback for when the car collides with something.
 * <br/><br/>
 * When you add a compound to a Space, all of it's constituents get added
 * and furthermore, those constituents cannot be added seperately.
 */
@:final#if nape_swc@:keep #end
class Compound extends Interactor{
    /**
     * @private
     */
    public var zpp_inner:ZPP_Compound=null;
    /**
     * List of bodies directly owned by this Compound.
     * <br/><br/>
     * This list does not include those bodies belonging to sub-compounds.
     *
     * @default []
     */
    #if nape_swc@:isVar #end
    public var bodies(get_bodies,never):BodyList;
    inline function get_bodies():BodyList{
        return zpp_inner.wrap_bodies;
    }
    /**
     * List of constraints directly owned by this Compound.
     * <br/><br/>
     * This list does not include those constraints belonging to sub-compounds.
     *
     * @default []
     */
    #if nape_swc@:isVar #end
    public var constraints(get_constraints,never):ConstraintList;
    inline function get_constraints():ConstraintList{
        return zpp_inner.wrap_constraints;
    }
    /**
     * List of compounds directly owned by this Compound.
     * <br/><br/>
     * This list does not include those compounds belonging to sub-compounds.
     *
     * @default []
     */
    #if nape_swc@:isVar #end
    public var compounds(get_compounds,never):CompoundList;
    inline function get_compounds():CompoundList{
        return zpp_inner.wrap_compounds;
    }
    /**
     * Compound that this compound belongs to.
     *
     * @default null
     */
    #if nape_swc@:isVar #end
    public var compound(get_compound,set_compound):Null<Compound>;
    inline function get_compound():Null<Compound>{
        return if(zpp_inner.compound==null)null else zpp_inner.compound.outer;
    }
    inline function set_compound(compound:Null<Compound>):Null<Compound>{
        {
            zpp_inner.immutable_midstep("Compound::compound");
            if(this.compound!=compound){
                if(this.compound!=null)this.compound.compounds.remove(this);
                if(compound!=null)compound.compounds.add(this);
            }
        }
        return get_compound();
    }
    /**
     * Space this compound belongs to.
     * <br/><br/>
     * This value is immutable when this compound belongs to another parent Compound.
     *
     * @default null
     */
    #if nape_swc@:isVar #end
    public var space(get_space,set_space):Null<Space>;
    inline function get_space():Null<Space>{
        return if(zpp_inner.space==null)null else zpp_inner.space.outer;
    }
    inline function set_space(space:Null<Space>):Null<Space>{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner.compound!=null)throw "Error: Cannot set the space of an inner Compound, only the root Compound space can be set";
            #end
            zpp_inner.immutable_midstep("Compound::space");
            if(this.space!=space){
                if(this.space!=null)this.space.compounds.remove(this);
                if(space!=null)space.compounds.add(this);
            }
        }
        return get_space();
    }
    /**
     * Construct a new Compound.
     *
     * @result The constructed Compound.
     */
    public function new(){
        #if(!NAPE_RELEASE_BUILD)
        try{
            super();
        }
        catch(e:Dynamic){}
        #end
        #if NAPE_RELEASE_BUILD 
        super();
        #end
        zpp_inner=new ZPP_Compound();
        zpp_inner.outer=this;
        zpp_inner.outer_i=this;
        zpp_inner_i=zpp_inner;
        zpp_inner.insert_cbtype(CbType.ANY_COMPOUND.zpp_inner);
    }
    /**
     * @private
     */
    @:keep public override function toString(){
        return "Compound"+this.id;
    }
    /**
     * Produce an exact copy of this Compound.
     * <br/><br/>
     * This copy will remap owned constraints so that their body properties
     * refer to the newly copied bodies also owned by this compound.
     * <br/><br/>
     * If this compound tree contains any constraints that make references
     * to outside of this compound; then these properties will be made null.
     * <pre>
     *       ____Cmp1____               [Cmp2.copy()]
     *      /    /        &#92;
     * Body1 Body2___     Cmp2        null    Cmp2'
     *   |     |     &#92;     /  &#92;         &#92;    /    &#92;
     * Shp1  Shp2     Joint--Body3       Joint'--Body3'
     *                         |                   |
     *                        Shp3               Shp3'
     * </pre>
     * For instance if copying Cmp1 then all is well, but if we copy Cmp2 the
     * copied Joint will have one of it's body references null as that body is
     * not owned directly, or indirectly by the compound.
     */
    public function copy(){
        return zpp_inner.copy();
    }
    /**
     * Breaking compound apart in-place.
     * <br/><br/>
     * This method will destroy the compound, moving all of it's components
     * to the assigned Space if this is the root compound, otherwise to the
     * parent compound.
     * <br/><br/>
     * Apart from being easier than doing this manually it also means that we
     * do not have to temporarigly remove objects from the space meaning that
     * things like PreListener ignored interactions will be unaffected.
     */
    public function breakApart(){
        zpp_inner.breakApart();
    }
    /**
     * Method to iterate over all bodies contained directly or indirectly by
     * this Compound.
     *
     * @param lambda The method to apply to each Body.
     * @throws # If lambda is null.
     */
    public function visitBodies(lambda:Body->Void){
        #if(!NAPE_RELEASE_BUILD)
        if(lambda==null)throw "Error: lambda cannot be null for Compound::visitBodies";
        #end
        for(b in bodies)lambda(b);
        for(c in compounds)c.visitBodies(lambda);
    }
    /**
     * Method to iterate over all constraints contained directly or indirectly by
     * this Compound.
     *
     * @param lambda The method to apply to each Constraint.
     * @throws # If lambda is null.
     */
    public function visitConstraints(lambda:Constraint->Void){
        #if(!NAPE_RELEASE_BUILD)
        if(lambda==null)throw "Error: lambda cannot be null for Compound::visitConstraints";
        #end
        for(c in constraints)lambda(c);
        for(c in compounds)c.visitConstraints(lambda);
    }
    /**
     * Method to iterate over all compounds contained directly or indirectly by
     * this Compound.
     *
     * @param lambda The method to apply to each Compound.
     * @throws # If lambda is null.
     */
    public function visitCompounds(lambda:Compound->Void){
        #if(!NAPE_RELEASE_BUILD)
        if(lambda==null)throw "Error: lambda cannot be null for Compound::visitConstraints";
        #end
        for(c in compounds){
            lambda(c);
            c.visitCompounds(lambda);
        }
    }
    /**
     * Compute centre of mass of Compound.
     *
     * @param weak If true, the returned Vec2 will be automatically released
     *             to the object pool when passed as an argument to a Nape
     *             function. (default false)
     * @return The centre of mass of compound.
     * @throws # If Compound has no Bodies contained directly or indirectly
     *           that contain at least one Shape.
     */
    public function COM(weak:Bool=false){
        var ret=Vec2.get(0,0,weak);
        var total=0.0;
        visitBodies(function(b){
            if(!b.shapes.empty()){
                ret.addeq(b.worldCOM.mul(b.mass,true));
                total+=b.mass;
            }
        });
        #if(!NAPE_RELEASE_BUILD)
        if(total==0.0)throw "Error: COM of an empty Compound is undefined silly";
        #end
        ret.muleq(1/total);
        return ret;
    }
    /**
     * Translate entire compound.
     * <br/><br/>
     * This is equivalent to: <code>compound.visitBodies(function (b) b.translate(translation))</code>
     *
     * @param translation The translation to apply to the Compound.
     * @return A reference to this Compound.
     * @throws # If translation is null or disposed of.
     * @throws # If any Body in the compound is static, and this compound is in a Space.
     */
    public function translate(translation:Vec2){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(translation!=null&&translation.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(translation==null)throw "Error: Cannot translate by null Vec2";
        #end
        var weak=translation.zpp_inner.weak;
        translation.zpp_inner.weak=false;
        visitBodies(function(b)b.position.addeq(translation));
        translation.zpp_inner.weak=weak;
        ({
            if(({
                translation.zpp_inner.weak;
            })){
                translation.dispose();
                true;
            }
            else{
                false;
            }
        });
        return this;
    }
    /**
     * Rotate entire compound about a point.
     * <br/><br/>
     * This is equivalent to: <code>compound.visitBodies(function (b) b.rotate(centre, angle))</code>
     *
     * @param centre The centre of rotation in world coordinates.
     * @param angle The clockwise angle of rotation in radians.
     * @return A reference to this Compound.
     * @throws # If centre is null or disposed of.
     * @throws # If any Body in the compound is static, and this compound is in a Space.
     */
    public function rotate(centre:Vec2,angle:Float){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(centre!=null&&centre.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(centre==null)throw "Error: Cannot rotate about a null Vec2";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if((angle!=angle))throw "Error: Cannot rotate by NaN radians";
        #end
        var weak=centre.zpp_inner.weak;
        centre.zpp_inner.weak=false;
        visitBodies(function(b)b.rotate(centre,angle));
        centre.zpp_inner.weak=weak;
        ({
            if(({
                centre.zpp_inner.weak;
            })){
                centre.dispose();
                true;
            }
            else{
                false;
            }
        });
        return this;
    }
}
