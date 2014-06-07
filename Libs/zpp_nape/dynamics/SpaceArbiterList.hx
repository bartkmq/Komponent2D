package zpp_nape.dynamics;
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
class ZPP_SpaceArbiterList extends ArbiterList{
    public var space:ZPP_Space=null;
    public var _length:Int=0;
    public var zip_length:Bool=false;
    public function new(){
        super();
        at_index_0=0;
        at_index_1=0;
        at_index_2=0;
        at_index_3=0;
        zip_length=true;
        _length=0;
        lengths=new Array<Int>();
        for(i in 0...4)lengths.push(0);
    }
    public override function zpp_gl(){
        zpp_vm();
        if(zip_length){
            _length=0;
            var ind=0;
            {
                {
                    var len=0;
                    {
                        var cx_ite=space.c_arbiters_true.begin();
                        while(cx_ite!=null){
                            var i=cx_ite.elem();
                            if(i.active)len++;
                            cx_ite=cx_ite.next;
                        }
                    };
                    lengths[ind++]=len;
                    _length+=len;
                };
                {
                    var len=0;
                    {
                        var cx_ite=space.c_arbiters_false.begin();
                        while(cx_ite!=null){
                            var i=cx_ite.elem();
                            if(i.active)len++;
                            cx_ite=cx_ite.next;
                        }
                    };
                    lengths[ind++]=len;
                    _length+=len;
                };
                {
                    var len=0;
                    {
                        var cx_ite=space.f_arbiters.begin();
                        while(cx_ite!=null){
                            var i=cx_ite.elem();
                            if(i.active)len++;
                            cx_ite=cx_ite.next;
                        }
                    };
                    lengths[ind++]=len;
                    _length+=len;
                };
                {
                    var len=0;
                    {
                        var cx_ite=space.s_arbiters.begin();
                        while(cx_ite!=null){
                            var i=cx_ite.elem();
                            if(i.active)len++;
                            cx_ite=cx_ite.next;
                        }
                    };
                    lengths[ind++]=len;
                    _length+=len;
                };
            };
            zip_length=false;
        }
        return _length;
    }
    public var lengths:Array<Int>=null;
    public var ite_0:ZNPNode_ZPP_ColArbiter=null;
    public var ite_1:ZNPNode_ZPP_ColArbiter=null;
    public var ite_2:ZNPNode_ZPP_FluidArbiter=null;
    public var ite_3:ZNPNode_ZPP_SensorArbiter=null;
    public var at_index_0:Int=0;
    public var at_index_1:Int=0;
    public var at_index_2:Int=0;
    public var at_index_3:Int=0;
    public override function zpp_vm(){
        var modified=false;
        {
            if(space.c_arbiters_true.modified){
                modified=true;
                space.c_arbiters_true.modified=false;
            };
            if(space.c_arbiters_false.modified){
                modified=true;
                space.c_arbiters_false.modified=false;
            };
            if(space.f_arbiters.modified){
                modified=true;
                space.f_arbiters.modified=false;
            };
            if(space.s_arbiters.modified){
                modified=true;
                space.s_arbiters.modified=false;
            };
        };
        if(modified){
            zip_length=true;
            _length=0;
            ite_0=null;
            ite_1=null;
            ite_2=null;
            ite_3=null;
        }
    }
    public override function push(obj:Arbiter):Bool{
        #if(!NAPE_RELEASE_BUILD)
        throw "Error: ArbiterList is immutable";
        #end
        return false;
    }
    public override function pop():Arbiter{
        #if(!NAPE_RELEASE_BUILD)
        throw "Error: ArbiterList is immutable";
        #end
        return null;
    }
    public override function unshift(obj:Arbiter):Bool{
        #if(!NAPE_RELEASE_BUILD)
        throw "Error: ArbiterList is immutable";
        #end
        return false;
    }
    public override function shift():Arbiter{
        #if(!NAPE_RELEASE_BUILD)
        throw "Error: ArbiterList is immutable";
        #end
        return null;
    }
    public override function remove(obj:Arbiter):Bool{
        #if(!NAPE_RELEASE_BUILD)
        throw "Error: ArbiterList is immutable";
        #end
        return false;
    }
    public override function clear(){
        #if(!NAPE_RELEASE_BUILD)
        throw "Error: ArbiterList is immutable";
        #end
    }
    public override function at(index:Int):Arbiter{
        zpp_vm();
        #if(!NAPE_RELEASE_BUILD)
        if(index<0||index>=length)throw "Error: Index out of bounds";
        #end
        var ret:Arbiter=null;
        var accum_length=0;
        
        if(ret==null){
            if(index<accum_length+lengths[0]){
                var offset=index-accum_length;
                if(offset<at_index_0||ite_0==null){
                    at_index_0=0;
                    ite_0=space.c_arbiters_true.begin();
                    while(true){
                        var x=ite_0.elem();
                        if(x.active)break;
                        ite_0=ite_0.next;
                    }
                }
                while(at_index_0!=offset){
                    at_index_0++;
                    ite_0=ite_0.next;
                    while(true){
                        var x=ite_0.elem();
                        if(x.active)break;
                        ite_0=ite_0.next;
                    }
                }
                ret=ite_0.elem().wrapper();
            }
            else accum_length+=lengths[0];
        }
        if(ret==null){
            if(index<accum_length+lengths[1]){
                var offset=index-accum_length;
                if(offset<at_index_1||ite_1==null){
                    at_index_1=0;
                    ite_1=space.c_arbiters_false.begin();
                    while(true){
                        var x=ite_1.elem();
                        if(x.active)break;
                        ite_1=ite_1.next;
                    }
                }
                while(at_index_1!=offset){
                    at_index_1++;
                    ite_1=ite_1.next;
                    while(true){
                        var x=ite_1.elem();
                        if(x.active)break;
                        ite_1=ite_1.next;
                    }
                }
                ret=ite_1.elem().wrapper();
            }
            else accum_length+=lengths[1];
        }
        if(ret==null){
            if(index<accum_length+lengths[2]){
                var offset=index-accum_length;
                if(offset<at_index_2||ite_2==null){
                    at_index_2=0;
                    ite_2=space.f_arbiters.begin();
                    while(true){
                        var x=ite_2.elem();
                        if(x.active)break;
                        ite_2=ite_2.next;
                    }
                }
                while(at_index_2!=offset){
                    at_index_2++;
                    ite_2=ite_2.next;
                    while(true){
                        var x=ite_2.elem();
                        if(x.active)break;
                        ite_2=ite_2.next;
                    }
                }
                ret=ite_2.elem().wrapper();
            }
            else accum_length+=lengths[2];
        }
        if(ret==null){
            if(index<accum_length+lengths[3]){
                var offset=index-accum_length;
                if(offset<at_index_3||ite_3==null){
                    at_index_3=0;
                    ite_3=space.s_arbiters.begin();
                    while(true){
                        var x=ite_3.elem();
                        if(x.active)break;
                        ite_3=ite_3.next;
                    }
                }
                while(at_index_3!=offset){
                    at_index_3++;
                    ite_3=ite_3.next;
                    while(true){
                        var x=ite_3.elem();
                        if(x.active)break;
                        ite_3=ite_3.next;
                    }
                }
                ret=ite_3.elem().wrapper();
            }
            else accum_length+=lengths[3];
        }
        return ret;
    }
}
