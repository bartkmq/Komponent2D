package zpp_nape.phys;
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
class ZPP_Compound extends ZPP_Interactor{
    public var outer:Compound=null;
    public var bodies:ZNPList_ZPP_Body=null;
    public var constraints:ZNPList_ZPP_Constraint=null;
    public var compounds:ZNPList_ZPP_Compound=null;
    public var wrap_bodies:BodyList=null;
    public var wrap_constraints:ConstraintList=null;
    public var wrap_compounds:CompoundList=null;
    public var depth:Int=0;
    public var compound:ZPP_Compound=null;
    public var space:ZPP_Space=null;
    public function __imutable_midstep(name:String){
        #if(!NAPE_RELEASE_BUILD)
        if(space!=null&&space.midstep)throw "Error: "+name+" cannot be set during space step()";
        #end
    }
    public function addedToSpace(){
        __iaddedToSpace();
    }
    public function removedFromSpace(){
        __iremovedFromSpace();
    }
    public function breakApart(){
        if(space!=null){
            __iremovedFromSpace();
            space.nullInteractorType(this);
        }
        if(compound!=null)compound.compounds.remove(this);
        else if(space!=null)space.compounds.remove(this);
        {
            while(!bodies.empty()){
                var b=bodies.pop_unsafe();
                {
                    if((b.compound=compound)!=null)compound.bodies.add(b);
                    else if(space!=null)space.bodies.add(b);
                    if(space!=null)space.freshInteractorType(b);
                };
            }
        };
        {
            while(!constraints.empty()){
                var c=constraints.pop_unsafe();
                {
                    if((c.compound=compound)!=null)compound.constraints.add(c);
                    else if(space!=null)space.constraints.add(c);
                };
            }
        };
        {
            while(!compounds.empty()){
                var c=compounds.pop_unsafe();
                {
                    if((c.compound=compound)!=null)compound.compounds.add(c);
                    else if(space!=null)space.compounds.add(c);
                    if(space!=null)space.freshInteractorType(c);
                };
            }
        };
        compound=null;
        space=null;
    }
    
    private function bodies_adder(x:Body){
        {}
        if(x.zpp_inner.compound!=this){
            if(x.zpp_inner.compound!=null)x.zpp_inner.compound.wrap_bodies.remove(x);
            else if(x.zpp_inner.space!=null)x.zpp_inner.space.wrap_bodies.remove(x);
            x.zpp_inner.compound=this;
            {};
            if(space!=null)space.addBody(x.zpp_inner);
            return true;
        }
        else return false;
    }
    private function bodies_subber(x:Body){
        x.zpp_inner.compound=null;
        {};
        if(space!=null)space.remBody(x.zpp_inner);
    }
    #if(!NAPE_RELEASE_BUILD)
    private function bodies_modifiable(){
        immutable_midstep("Compound::"+"bodies");
    }
    #end
    private function constraints_adder(x:Constraint){
        {}
        if(x.zpp_inner.compound!=this){
            if(x.zpp_inner.compound!=null)x.zpp_inner.compound.wrap_constraints.remove(x);
            else if(x.zpp_inner.space!=null)x.zpp_inner.space.wrap_constraints.remove(x);
            x.zpp_inner.compound=this;
            {};
            if(space!=null)space.addConstraint(x.zpp_inner);
            return true;
        }
        else return false;
    }
    private function constraints_subber(x:Constraint){
        x.zpp_inner.compound=null;
        {};
        if(space!=null)space.remConstraint(x.zpp_inner);
    }
    #if(!NAPE_RELEASE_BUILD)
    private function constraints_modifiable(){
        immutable_midstep("Compound::"+"constraints");
    }
    #end
    private function compounds_adder(x:Compound){
        #if(!NAPE_RELEASE_BUILD)
        var cur=this;
        while(cur!=null&&cur!=x.zpp_inner)cur=cur.compound;
        if(cur==x.zpp_inner){
            throw "Error: Assignment would cause a cycle in the Compound tree: assigning "+x.toString()+".compound = "+outer.toString();
            return false;
        }
        #end
        if(x.zpp_inner.compound!=this){
            if(x.zpp_inner.compound!=null)x.zpp_inner.compound.wrap_compounds.remove(x);
            else if(x.zpp_inner.space!=null)x.zpp_inner.space.wrap_compounds.remove(x);
            x.zpp_inner.compound=this;
            x.zpp_inner.depth=depth+1;
            if(space!=null)space.addCompound(x.zpp_inner);
            return true;
        }
        else return false;
    }
    private function compounds_subber(x:Compound){
        x.zpp_inner.compound=null;
        x.zpp_inner.depth=1;
        if(space!=null)space.remCompound(x.zpp_inner);
    }
    #if(!NAPE_RELEASE_BUILD)
    private function compounds_modifiable(){
        immutable_midstep("Compound::"+"compounds");
    }
    #end
    public function new(){
        super();
        icompound=this;
        depth=1;
        var me=this;
        bodies=new ZNPList_ZPP_Body();
        wrap_bodies=ZPP_BodyList.get(bodies);
        wrap_bodies.zpp_inner.adder=bodies_adder;
        wrap_bodies.zpp_inner.subber=bodies_subber;
        #if(!NAPE_RELEASE_BUILD)
        wrap_bodies.zpp_inner._modifiable=bodies_modifiable;
        #end
        constraints=new ZNPList_ZPP_Constraint();
        wrap_constraints=ZPP_ConstraintList.get(constraints);
        wrap_constraints.zpp_inner.adder=constraints_adder;
        wrap_constraints.zpp_inner.subber=constraints_subber;
        #if(!NAPE_RELEASE_BUILD)
        wrap_constraints.zpp_inner._modifiable=constraints_modifiable;
        #end
        compounds=new ZNPList_ZPP_Compound();
        wrap_compounds=ZPP_CompoundList.get(compounds);
        wrap_compounds.zpp_inner.adder=compounds_adder;
        wrap_compounds.zpp_inner.subber=compounds_subber;
        #if(!NAPE_RELEASE_BUILD)
        wrap_compounds.zpp_inner._modifiable=compounds_modifiable;
        #end
    }
    public function copy(dict:Array<ZPP_CopyHelper>=null,todo:Array<ZPP_CopyHelper>=null):Compound{
        var root=dict==null;
        if(dict==null)dict=new Array<ZPP_CopyHelper>();
        if(todo==null)todo=new Array<ZPP_CopyHelper>();
        var ret=new Compound();
        {
            var cx_ite=compounds.begin();
            while(cx_ite!=null){
                var c=cx_ite.elem();
                {
                    var cc=c.copy(dict,todo);
                    cc.compound=ret;
                };
                cx_ite=cx_ite.next;
            }
        };
        {
            var cx_ite=bodies.begin();
            while(cx_ite!=null){
                var b=cx_ite.elem();
                {
                    var bc=b.outer.copy();
                    dict.push(ZPP_CopyHelper.dict(b.id,bc));
                    bc.compound=ret;
                };
                cx_ite=cx_ite.next;
            }
        };
        {
            var cx_ite=constraints.begin();
            while(cx_ite!=null){
                var c=cx_ite.elem();
                {
                    var cc=c.copy(dict,todo);
                    cc.compound=ret;
                };
                cx_ite=cx_ite.next;
            }
        };
        if(root){
            while(todo.length>0){
                var xcb=todo.pop();
                for(idc in dict){
                    if(idc.id==xcb.id){
                        xcb.cb(idc.bc);
                        break;
                    }
                }
            }
        }
        copyto(ret);
        return ret;
    }
}
