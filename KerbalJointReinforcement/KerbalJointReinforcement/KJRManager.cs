/*
    Developers: Michael Ferrara (aka Ferram4), Meiru

    This file is part of Kerbal Joint Reinforcement.

    Kerbal Joint Reinforcement is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Kerbal Joint Reinforcement is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Kerbal Joint Reinforcement.  If not, see <http://www.gnu.org/licenses/>.
*/

using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using CompoundParts;

namespace KerbalJointReinforcement
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class KJRManager : MonoBehaviour
    {
        List<Vessel> updatedVessels;
        HashSet<Vessel> vesselOffRails;
        Dictionary<Vessel, List<Joint>> vesselJointStrengthened;
        KJRMultiJointManager multiJointManager;

        public void Awake()
        {
            KJRJointUtils.LoadConstants();
            updatedVessels = new List<Vessel>();
            vesselOffRails = new HashSet<Vessel>();
            vesselJointStrengthened = new Dictionary<Vessel, List<Joint>>();
            multiJointManager = new KJRMultiJointManager();
        }

        public void Start()
        {
            if(!CompatibilityChecker.IsCompatible())
                return;

            GameEvents.onVesselCreate.Add(OnVesselCreate);
            GameEvents.onVesselWasModified.Add(OnVesselWasModified);

            GameEvents.onVesselGoOffRails.Add(OnVesselOffRails);
    
            GameEvents.onVesselGoOnRails.Add(OnVesselOnRails);
            GameEvents.onVesselDestroy.Add(OnVesselOnRails);

            GameEvents.onPartDestroyed.Add(OnPartDestroyed);
            GameEvents.onPartDie.Add(OnPartDestroyed);
        }

        public void OnDestroy()
        {
            if(!CompatibilityChecker.IsCompatible())
                return;

            GameEvents.onVesselCreate.Remove(OnVesselCreate);
            GameEvents.onVesselWasModified.Remove(OnVesselWasModified);

            GameEvents.onVesselGoOffRails.Remove(OnVesselOffRails);
    
            GameEvents.onVesselGoOnRails.Remove(OnVesselOnRails);
            GameEvents.onVesselDestroy.Remove(OnVesselOnRails);

            GameEvents.onPartDestroyed.Remove(OnPartDestroyed);
            GameEvents.onPartDie.Remove(OnPartDestroyed);

            if(InputLockManager.GetControlLock("KJRLoadLock") == ControlTypes.ALL_SHIP_CONTROLS)
                InputLockManager.RemoveControlLock("KJRLoadLock");
            updatedVessels = null;
            vesselOffRails = null;
            vesselJointStrengthened = null;

            multiJointManager = null;
        }

        private void OnVesselCreate(Vessel v)
        {
            multiJointManager.VerifyVesselJoints(v);
        }

        private void OnVesselWasModified(Vessel v)
        {
            if((object)v == null || v.isEVA)
                return; 
            
            multiJointManager.VerifyVesselJoints(v);

            if(KJRJointUtils.debug)
            {
                StringBuilder debugString = new StringBuilder();
                debugString.AppendLine("KJR: Modified vessel " + v.id + " (" + v.GetName() + ")");
                debugString.AppendLine(System.Environment.StackTrace);
                debugString.AppendLine("Now contains: ");
                foreach(Part p in v.Parts)
                    debugString.AppendLine("  " + p.partInfo.name + " (" + p.flightID + ")");
                Debug.Log(debugString);
            }

            updatedVessels.Remove(v);
            RunVesselJointUpdateFunction(v);
        }

        // this function should be called by all compatible modules when
        // they call Vessel.CycleAllAutoStrut for AutoStruts
        public void CycleAllAutoStrut(Vessel v)
        {
            OnVesselWasModified(v);
        }

        private void OnVesselOffRails(Vessel v)
        {
            if((object)v == null || v.isEVA)
                return; 
            
            RunVesselJointUpdateFunction(v);

            if(!vesselOffRails.Contains(v) && v.precalc.isEasingGravity)
            {
                Debug.Log("KJR easing " + v.vesselName);

                vesselOffRails.Add(v);

                for(int i = 0; i < v.Parts.Count; ++i)
                {
                    Part p = v.Parts[i];
                    p.crashTolerance = p.crashTolerance * 10000f;
                    if(p.attachJoint)
                        p.attachJoint.SetUnbreakable(true, false);

                    Joint[] partJoints = p.GetComponents<Joint>();

                    if(p.Modules.Contains<LaunchClamp>())
                    {
                        foreach(Joint j in partJoints)
                            if(j.connectedBody == null)
                            {
                                GameObject.Destroy(j);
                                KJRJointUtils.ConnectLaunchClampToGround(p);
                                break;
                            }
                    }
                }
            }
        }

        private void OnVesselOnRails(Vessel v)
        {
            if((object)v == null)
                return;

            if(updatedVessels.Contains(v))
            {
                if(vesselOffRails.Contains(v))
                {
                    foreach(Part p in v.Parts)
                    {
                        p.crashTolerance = p.crashTolerance / 10000;
                        if(p.attachJoint)
                            p.attachJoint.SetUnbreakable(false, false);
                    }

                    vesselOffRails.Remove(v);
                }
                vesselJointStrengthened.Remove(v);
                updatedVessels.Remove(v);
            }
        }

        private void OnPartDestroyed(Part p)
        {
            multiJointManager.RemovePartJoints(p);
        }

        private void RunVesselJointUpdateFunction(Vessel v)
        {
            if(KJRJointUtils.debug)
            {
                Debug.Log("KJR: Processing vessel " + v.id + " (" + v.GetName() + "); root " +
                            v.rootPart.partInfo.name + " (" + v.rootPart.flightID + ")");
            }

            bool bReinforced = false;

            foreach(Part p in v.Parts)
            {
                if(KJRJointUtils.reinforceAttachNodes)
                {
                    if((p.parent != null) && (p.physicalSignificance == Part.PhysicalSignificance.FULL))
                    {
                        bReinforced = true;
                        UpdatePartJoint(p);
                    }
                }

                if(KJRJointUtils.reinforceDecouplersFurther)
                {
                    if((p.Modules.Contains<ModuleDecouple>() || p.Modules.Contains<ModuleAnchoredDecoupler>())
                        && !p.Modules.Contains<KJRDecouplerReinforcementModule>())
                    {
                        KJRJointUtils.AddDecouplerJointReinforcementModule(p);
                        continue;
                    }
                }

                if(KJRJointUtils.reinforceLaunchClampsFurther)
                {
                    if((p.parent != null) && p.Modules.Contains<LaunchClamp>()
                        && !p.Modules.Contains<KJRLaunchClampReinforcementModule>())
                    {
                        p.breakingForce = Mathf.Infinity;
                        p.breakingTorque = Mathf.Infinity;
                        p.mass = Mathf.Max(p.mass, (p.parent.mass + p.parent.GetResourceMass()) * 0.01f);          //We do this to make sure that there is a mass ratio of 100:1 between the clamp and what it's connected to.  This helps counteract some of the wobbliness simply, but also allows some give and springiness to absorb the initial physics kick
                        if(KJRJointUtils.debug)
                            Debug.Log("KJR: Launch Clamp Break Force / Torque increased");

                        KJRJointUtils.AddLaunchClampReinforcementModule(p);
                    }
                }
            }

            if(bReinforced)
                updatedVessels.Add(v);

            if(KJRJointUtils.reinforceAttachNodes && KJRJointUtils.multiPartAttachNodeReinforcement)
                MultiPartJointTreeChildren(v);
        }

        public void FixedUpdate()
        {
            if(FlightGlobals.ready && FlightGlobals.Vessels != null)
            {
                for(int i = 0; i < updatedVessels.Count; ++i)
                {
                    Vessel v = updatedVessels[i];
                    if(v == null || !vesselOffRails.Contains(v))
                        continue;

                    if(!v.precalc.isEasingGravity)
                    {
                        foreach(Part p in v.Parts)
                        {
                            p.crashTolerance = p.crashTolerance / 10000f;
                            if(p.attachJoint)
                                p.attachJoint.SetUnbreakable(false, false);
                        }

                        vesselOffRails.Remove(v);
                    }
                }
            }
        }

        private void UpdatePartJoint(Part p)
        {
            if(p.rb == null || p.attachJoint == null || !KJRJointUtils.IsJointAdjustmentAllowed(p))
                return;

            if(p.attachMethod == AttachNodeMethod.LOCKED_JOINT)
            {
                if(KJRJointUtils.debug)
                {
                    Debug.Log("KJR: Already processed part before: " + p.partInfo.name + " (" + p.flightID + ") -> " +
                                p.parent.partInfo.name + " (" + p.parent.flightID + ")");
                }
            }

            List<ConfigurableJoint> jointList;

            if(p.Modules.Contains<CModuleStrut>())
            {
                CModuleStrut s = p.Modules.GetModule<CModuleStrut>();

                if(!(s.jointTarget == null || s.jointRoot == null))
                {
                    jointList = s.strutJoint.joints;

                    if(jointList != null)
                    {
                        for(int i = 0; i < jointList.Count; i++)
                        {
                            ConfigurableJoint j = jointList[i];

                            if(j == null)
                                continue;

                            JointDrive strutDrive = j.angularXDrive;
                            strutDrive.positionSpring = KJRJointUtils.decouplerAndClampJointStrength;
                            strutDrive.maximumForce = KJRJointUtils.decouplerAndClampJointStrength;
                            j.xDrive = j.yDrive = j.zDrive = j.angularXDrive = j.angularYZDrive = strutDrive;

                            j.xMotion = j.yMotion = j.zMotion = ConfigurableJointMotion.Locked;
                            j.angularXMotion = j.angularYMotion = j.angularZMotion = ConfigurableJointMotion.Locked;

                            //float scalingFactor = (s.jointTarget.mass + s.jointTarget.GetResourceMass() + s.jointRoot.mass + s.jointRoot.GetResourceMass()) * 0.01f;

                            j.breakForce = KJRJointUtils.decouplerAndClampJointStrength;
                            j.breakTorque = KJRJointUtils.decouplerAndClampJointStrength;
                        }

                        p.attachMethod = AttachNodeMethod.LOCKED_JOINT;
                    }
                }
            }
            
            jointList = p.attachJoint.joints;

            if(jointList == null)
                return;

            StringBuilder debugString = new StringBuilder();

            bool addAdditionalJointToParent = KJRJointUtils.multiPartAttachNodeReinforcement;
            //addAdditionalJointToParent &= !(p.Modules.Contains("LaunchClamp") || (p.parent.Modules.Contains("ModuleDecouple") || p.parent.Modules.Contains("ModuleAnchoredDecoupler")));
            addAdditionalJointToParent &= !p.Modules.Contains<CModuleStrut>();

            if(p.GetComponent<IJointLockState>() == null) // exclude those actions from joints that can be dynamically unlocked
            {
                float partMass = p.mass + p.GetResourceMass();
                for(int i = 0; i < jointList.Count; i++)
                {
                    ConfigurableJoint j = jointList[i];
                    if(j == null)
                        continue;

                    String jointType = j.GetType().Name;
                    Rigidbody connectedBody = j.connectedBody;

                    Part connectedPart = connectedBody.GetComponent<Part>() ?? p.parent;
                    float parentMass = connectedPart.mass + connectedPart.GetResourceMass();

                    if(partMass < KJRJointUtils.massForAdjustment || parentMass < KJRJointUtils.massForAdjustment)
                    {
                        if(KJRJointUtils.debug)
                            Debug.Log("KJR: Part mass too low, skipping: " + p.partInfo.name + " (" + p.flightID + ")");

                        continue;
                    }                
                
                    // Check attachment nodes for better orientation data
                    AttachNode attach = p.FindAttachNodeByPart(p.parent);
                    AttachNode p_attach = p.parent.FindAttachNodeByPart(p);
                    AttachNode node = attach ?? p_attach;

                    if(node == null)
                    {
                        // Check if it's a pair of coupled docking ports
                        var dock1 = p.Modules.GetModule<ModuleDockingNode>();
                        var dock2 = p.parent.Modules.GetModule<ModuleDockingNode>();

                        //Debug.Log(dock1 + " " + (dock1 ? ""+dock1.dockedPartUId : "?") + " " + dock2 + " " + (dock2 ? ""+dock2.dockedPartUId : "?"));

                        if(dock1 && dock2 && (dock1.dockedPartUId == p.parent.flightID || dock2.dockedPartUId == p.flightID))
                        {
                            attach = p.FindAttachNode(dock1.referenceAttachNode);
                            p_attach = p.parent.FindAttachNode(dock2.referenceAttachNode);
                            node = attach ?? p_attach;
                        }
                    }

                    // If still no node and apparently surface attached, use the normal one if it's there
                    if(node == null && p.attachMode == AttachModes.SRF_ATTACH)
                        node = attach = p.srfAttachNode;

                    if(KJRJointUtils.debug)
                    {
                        debugString.AppendLine("Original joint from " + p.partInfo.title + " to " + p.parent.partInfo.title);
                        debugString.AppendLine("  " + p.partInfo.name + " (" + p.flightID + ") -> " + p.parent.partInfo.name + " (" + p.parent.flightID + ")");
                        debugString.AppendLine("");
                        debugString.AppendLine(p.partInfo.title + " Inertia Tensor: " + p.rb.inertiaTensor + " " + p.parent.partInfo.name + " Inertia Tensor: " + connectedBody.inertiaTensor);
                        debugString.AppendLine("");


                        debugString.AppendLine("Std. Joint Parameters");
                        debugString.AppendLine("Connected Body: " + p.attachJoint.Joint.connectedBody);
                        debugString.AppendLine("Attach mode: " + p.attachMode + " (was " + jointType + ")");
                        if(attach != null)
                            debugString.AppendLine("Attach node: " + attach.id + " - " + attach.nodeType + " " + attach.size);
                        if(p_attach != null)
                            debugString.AppendLine("Parent node: " + p_attach.id + " - " + p_attach.nodeType + " " + p_attach.size);
                        debugString.AppendLine("Anchor: " + p.attachJoint.Joint.anchor);
                        debugString.AppendLine("Axis: " + p.attachJoint.Joint.axis);
                        debugString.AppendLine("Sec Axis: " + p.attachJoint.Joint.secondaryAxis);
                        debugString.AppendLine("Break Force: " + p.attachJoint.Joint.breakForce);
                        debugString.AppendLine("Break Torque: " + p.attachJoint.Joint.breakTorque);
                        debugString.AppendLine("");

                        debugString.AppendLine("Joint Motion Locked: " + Convert.ToString(p.attachJoint.Joint.xMotion == ConfigurableJointMotion.Locked));

                        debugString.AppendLine("X Drive");
                        debugString.AppendLine("Position Spring: " + p.attachJoint.Joint.xDrive.positionSpring);
                        debugString.AppendLine("Position Damper: " + p.attachJoint.Joint.xDrive.positionDamper);
                        debugString.AppendLine("Max Force: " + p.attachJoint.Joint.xDrive.maximumForce);
                        debugString.AppendLine("");

                        debugString.AppendLine("Y Drive");
                        debugString.AppendLine("Position Spring: " + p.attachJoint.Joint.yDrive.positionSpring);
                        debugString.AppendLine("Position Damper: " + p.attachJoint.Joint.yDrive.positionDamper);
                        debugString.AppendLine("Max Force: " + p.attachJoint.Joint.yDrive.maximumForce);
                        debugString.AppendLine("");

                        debugString.AppendLine("Z Drive");
                        debugString.AppendLine("Position Spring: " + p.attachJoint.Joint.zDrive.positionSpring);
                        debugString.AppendLine("Position Damper: " + p.attachJoint.Joint.zDrive.positionDamper);
                        debugString.AppendLine("Max Force: " + p.attachJoint.Joint.zDrive.maximumForce);
                        debugString.AppendLine("");

                        debugString.AppendLine("Angular X Drive");
                        debugString.AppendLine("Position Spring: " + p.attachJoint.Joint.angularXDrive.positionSpring);
                        debugString.AppendLine("Position Damper: " + p.attachJoint.Joint.angularXDrive.positionDamper);
                        debugString.AppendLine("Max Force: " + p.attachJoint.Joint.angularXDrive.maximumForce);
                        debugString.AppendLine("");

                        debugString.AppendLine("Angular YZ Drive");
                        debugString.AppendLine("Position Spring: " + p.attachJoint.Joint.angularYZDrive.positionSpring);
                        debugString.AppendLine("Position Damper: " + p.attachJoint.Joint.angularYZDrive.positionDamper);
                        debugString.AppendLine("Max Force: " + p.attachJoint.Joint.angularYZDrive.maximumForce);
                        debugString.AppendLine("");


                        //Debug.Log(debugString.ToString());
                    }


                    float breakForce = Math.Min(p.breakingForce, connectedPart.breakingForce) * KJRJointUtils.breakForceMultiplier;
                    float breakTorque = Math.Min(p.breakingTorque, connectedPart.breakingTorque) * KJRJointUtils.breakTorqueMultiplier;
                    Vector3 anchor = j.anchor;
                    Vector3 connectedAnchor = j.connectedAnchor;
                    Vector3 axis = j.axis;

                    float radius = 0;
                    float area = 0;
                    float momentOfInertia = 0;

                    if(node != null)
                    {
                        // Part that owns the node. For surface attachment,
                        // this can only be parent if docking flips hierarchy.
                        Part main = (node == attach) ? p : p.parent;

                        // Orientation and position of the node in owner's local coords
                        Vector3 ndir = node.orientation.normalized;
                        Vector3 npos = node.position + node.offset;

                        // And in the current part's local coords
                        Vector3 dir = axis = p.transform.InverseTransformDirection(main.transform.TransformDirection(ndir));

                        if(node.nodeType == AttachNode.NodeType.Surface)
                        {
                            // Guessed main axis; for parts with stack nodes should be the axis of the stack
                            Vector3 up = KJRJointUtils.GuessUpVector(main).normalized;

                            // if guessed up direction is same as node direction, it's basically stack
                            // for instance, consider a radially-attached docking port
                            if(Mathf.Abs(Vector3.Dot(up, ndir)) > 0.9f)
                            {
                                radius = Mathf.Min(KJRJointUtils.CalculateRadius(main, ndir), KJRJointUtils.CalculateRadius(connectedPart, ndir));
                                if(radius <= 0.001)
                                    radius = node.size * 1.25f;
                                area = Mathf.PI * radius * radius;                      //Area of cylinder
                                momentOfInertia = area * radius * radius / 4;           //Moment of Inertia of cylinder
                            }
                            else
                            {
                                // x along surface, y along ndir normal to surface, z along surface & main axis (up)
                                var size1 = KJRJointUtils.CalculateExtents(main, ndir, up);

                                var size2 = KJRJointUtils.CalculateExtents(connectedPart, ndir, up);

                                // use average of the sides, since we don't know which one is used for attaching
                                float width1 = (size1.x + size1.z) / 2;
                                float width2 = (size2.x + size2.z) / 2;
                                if(size1.y * width1 > size2.y * width2)
                                {
                                    area = size1.y * width1;
                                    radius = Mathf.Max(size1.y, width1);
                                }
                                else
                                {
                                    area = size2.y * width2;
                                    radius = Mathf.Max(size2.y, width2);
                                }

                                momentOfInertia = area * radius / 12;          //Moment of Inertia of a rectangle bending along the longer length
                            }
                        }
                        else
                        {
                            radius = Mathf.Min(KJRJointUtils.CalculateRadius(p, dir), KJRJointUtils.CalculateRadius(connectedPart, dir));
                            if(radius <= 0.001)
                                radius = node.size * 1.25f;
                            area = Mathf.PI * radius * radius;                      //Area of cylinder
                            momentOfInertia = area * radius * radius / 4;           //Moment of Inertia of cylinder
                        }
                    }
                    //Assume part is attached along its "up" cross section; use a cylinder to approximate properties
                    else if(p.attachMode == AttachModes.STACK)
                    {
                        radius = Mathf.Min(KJRJointUtils.CalculateRadius(p, Vector3.up), KJRJointUtils.CalculateRadius(connectedPart, Vector3.up));
                        if(radius <= 0.001)
                            radius = node.size * 1.25f;
                        area = Mathf.PI * radius * radius;                      //Area of cylinder
                        momentOfInertia = area * radius * radius / 4;           //Moment of Inertia of cylinder
                    }
                    else if(p.attachMode == AttachModes.SRF_ATTACH)
                    {                    
                        // x,z sides, y along main axis
                        Vector3 up1 = KJRJointUtils.GuessUpVector(p);
                        var size1 = KJRJointUtils.CalculateExtents(p, up1);

                        Vector3 up2 = KJRJointUtils.GuessUpVector(connectedPart);
                        var size2 = KJRJointUtils.CalculateExtents(connectedPart, up2);

                        // use average of the sides, since we don't know which one is used for attaching
                        float width1 = (size1.x + size1.z) / 2;
                        float width2 = (size2.x + size2.z) / 2;
                        if(size1.y * width1 > size2.y * width2)
                        {
                            area = size1.y * width1;
                            radius = Mathf.Max(size1.y, width1);
                        }
                        else
                        {
                            area = size2.y * width2;
                            radius = Mathf.Max(size2.y, width2);
                        }
                        momentOfInertia = area * radius / 12;          //Moment of Inertia of a rectangle bending along the longer length
                    }

                    if(KJRJointUtils.useVolumeNotArea)                //If using volume, raise al stiffness-affecting parameters to the 1.5 power
                    {
                        area = Mathf.Pow(area, 1.5f);
                        momentOfInertia = Mathf.Pow(momentOfInertia, 1.5f);
                    }


                    breakForce = Mathf.Max(KJRJointUtils.breakStrengthPerArea * area, breakForce);
                    breakTorque = Mathf.Max(KJRJointUtils.breakTorquePerMOI * momentOfInertia, breakTorque);

                    JointDrive angDrive = j.angularXDrive;
                    angDrive.positionSpring = Mathf.Max(momentOfInertia * KJRJointUtils.angularDriveSpring, angDrive.positionSpring);
                    angDrive.positionDamper = Mathf.Max(momentOfInertia * KJRJointUtils.angularDriveDamper * 0.1f, angDrive.positionDamper);
                    angDrive.maximumForce = breakTorque;
                    /*float moi_avg = p.rb.inertiaTensor.magnitude;

                    moi_avg += (p.transform.localToWorldMatrix.MultiplyPoint(p.CoMOffset) - p.parent.transform.position).sqrMagnitude * p.rb.mass;

                    if(moi_avg * 2f / drive.positionDamper < 0.08f)
                    {
                        drive.positionDamper = moi_avg / (0.04f);

                        drive.positionSpring = drive.positionDamper * drive.positionDamper / moi_avg;
                    }*/
                    j.angularXDrive = j.angularYZDrive = j.slerpDrive = angDrive;

                    JointDrive linDrive = j.xDrive;
                    linDrive.maximumForce = breakForce;
                    j.xDrive = j.yDrive = j.zDrive = linDrive;

                    j.linearLimit = j.angularYLimit = j.angularZLimit = j.lowAngularXLimit = j.highAngularXLimit
                        = new SoftJointLimit { limit = 0, bounciness = 0 };
                    j.linearLimitSpring = j.angularYZLimitSpring = j.angularXLimitSpring
                        = new SoftJointLimitSpring { spring = 0, damper = 0 };

                    j.targetAngularVelocity = Vector3.zero;
                    j.targetVelocity = Vector3.zero;
                    j.targetRotation = Quaternion.identity;
                    j.targetPosition = Vector3.zero;

                    j.breakForce = breakForce;
                    j.breakTorque = breakTorque;
                    p.attachJoint.SetBreakingForces(j.breakForce, j.breakTorque);

                    p.attachMethod = AttachNodeMethod.LOCKED_JOINT;

                    if(KJRJointUtils.debug)
                    {
                        debugString.AppendLine("Updated joint from " + p.partInfo.title + " to " + p.parent.partInfo.title);
                        debugString.AppendLine("  " + p.partInfo.name + " (" + p.flightID + ") -> " + p.parent.partInfo.name + " (" + p.parent.flightID + ")");
                        debugString.AppendLine("");
                        debugString.AppendLine(p.partInfo.title + " Inertia Tensor: " + p.rb.inertiaTensor + " " + p.parent.partInfo.name + " Inertia Tensor: " + connectedBody.inertiaTensor);
                        debugString.AppendLine("");


                        debugString.AppendLine("Std. Joint Parameters");
                        debugString.AppendLine("Connected Body: " + p.attachJoint.Joint.connectedBody);
                        debugString.AppendLine("Attach mode: " + p.attachMode + " (was " + jointType + ")");
                        if(attach != null)
                            debugString.AppendLine("Attach node: " + attach.id + " - " + attach.nodeType + " " + attach.size);
                        if(p_attach != null)
                            debugString.AppendLine("Parent node: " + p_attach.id + " - " + p_attach.nodeType + " " + p_attach.size);
                        debugString.AppendLine("Anchor: " + p.attachJoint.Joint.anchor);
                        debugString.AppendLine("Axis: " + p.attachJoint.Joint.axis);
                        debugString.AppendLine("Sec Axis: " + p.attachJoint.Joint.secondaryAxis);
                        debugString.AppendLine("Break Force: " + p.attachJoint.Joint.breakForce);
                        debugString.AppendLine("Break Torque: " + p.attachJoint.Joint.breakTorque);
                        debugString.AppendLine("");

                        debugString.AppendLine("Joint Motion Locked: " + Convert.ToString(p.attachJoint.Joint.xMotion == ConfigurableJointMotion.Locked));

                        debugString.AppendLine("Angular Drive");
                        debugString.AppendLine("Position Spring: " + angDrive.positionSpring);
                        debugString.AppendLine("Position Damper: " + angDrive.positionDamper);
                        debugString.AppendLine("Max Force: " + angDrive.maximumForce);
                        debugString.AppendLine("");

                        debugString.AppendLine("Cross Section Properties");
                        debugString.AppendLine("Radius: " + radius);
                        debugString.AppendLine("Area: " + area);
                        debugString.AppendLine("Moment of Inertia: " + momentOfInertia);
                    }
                }
            }

            if(addAdditionalJointToParent && p.parent.parent != null
            && KJRJointUtils.IsJointAdjustmentAllowed(p.parent)          // verify that parent is not an excluded part
            && KJRJointUtils.IsJointAdjustmentAllowed(p.parent.parent))  // verify that parent of parent (our target part) is not an excluded part
            {
                /*if(ValidDecoupler(p) || ValidDecoupler(p.parent))
                    continue;*/

                ConfigurableJoint j = p.attachJoint.Joint; // second steps uses the first/main joint as reference

                Part newConnectedPart = p.parent.parent;

                bool massRatioBelowThreshold = false;
                int numPartsFurther = 0;

                float partMaxMass = KJRJointUtils.MaximumPossiblePartMass(p);
                List<Part> partsCrossed = new List<Part>();
                List<Part> possiblePartsCrossed = new List<Part>();

                partsCrossed.Add(p);
                partsCrossed.Add(p.parent);
                partsCrossed.Add(newConnectedPart);

                Rigidbody connectedRb = newConnectedPart.rb;

                // search the first part with an acceptable mass/mass ration to this part (joints work better then)
                do
                {
                    float massRat1 = (partMaxMass < newConnectedPart.mass) ? (newConnectedPart.mass / partMaxMass) : (partMaxMass / newConnectedPart.mass);

                    if(massRat1 <= KJRJointUtils.stiffeningExtensionMassRatioThreshold)
                        massRatioBelowThreshold = true;
                    else
                    {
                        float maxMass = KJRJointUtils.MaximumPossiblePartMass(newConnectedPart);
                        float massRat2 = (p.mass < maxMass) ? (maxMass / p.mass) : (p.mass / maxMass);
                        
                        if(massRat2 <= KJRJointUtils.stiffeningExtensionMassRatioThreshold)
                            massRatioBelowThreshold = true;
                        else
                        {
                            if((newConnectedPart.parent == null)
                            || !KJRJointUtils.IsJointAdjustmentAllowed(newConnectedPart.parent))
                                break;

                            newConnectedPart = newConnectedPart.parent;

                            if(newConnectedPart.rb == null)
                                possiblePartsCrossed.Add(newConnectedPart);
                            else
                            {
                                connectedRb = newConnectedPart.rb;
                                partsCrossed.AddRange(possiblePartsCrossed);
                                partsCrossed.Add(newConnectedPart);
                                possiblePartsCrossed.Clear();
                            }

                            numPartsFurther++;
                        }
                    }

                } while(!massRatioBelowThreshold);// && numPartsFurther < 5);

                if(connectedRb != null && !multiJointManager.CheckMultiJointBetweenParts(p, newConnectedPart))
                {
                    ConfigurableJoint newJoint;

                    if((p.mass >= newConnectedPart.mass) || (p.rb == null))
                    {
                        newJoint = p.gameObject.AddComponent<ConfigurableJoint>();
                        newJoint.connectedBody = connectedRb;
                    }
                    else
                    {
                        newJoint = connectedRb.gameObject.AddComponent<ConfigurableJoint>();
                        newJoint.connectedBody = p.rb;
                    }

                    newJoint.axis = Vector3.right;
                    newJoint.secondaryAxis = Vector3.forward;

                    newJoint.anchor = Vector3.zero;
                //  newJoint.autoConfigureConnectedAnchor = false;
                //  newJoint.connectedAnchor = newJoint.connectedBody.transform.InverseTransformPoint(newJoint.transform.position);

                    newJoint.angularXDrive = newJoint.angularYZDrive = newJoint.slerpDrive = j.angularXDrive;

                    newJoint.xDrive = j.xDrive;
                    newJoint.yDrive = j.yDrive;
                    newJoint.zDrive = j.zDrive;

                    newJoint.linearLimit = newJoint.angularYLimit = newJoint.angularZLimit = newJoint.lowAngularXLimit = newJoint.highAngularXLimit
                        = new SoftJointLimit { limit = 0, bounciness = 0 };

                    newJoint.breakForce = j.breakForce;
                    newJoint.breakTorque = j.breakTorque;

                    for(int k = 0; k < partsCrossed.Count; k++)
                        multiJointManager.RegisterMultiJoint(partsCrossed[k], newJoint);
                }
            }

            if(KJRJointUtils.debug)
                Debug.Log(debugString.ToString());
        }

        private void MultiPartJointBuildJoint(Part p, Part linkPart)
        {
            if(multiJointManager.CheckMultiJointBetweenParts(p, linkPart) || !multiJointManager.TrySetValidLinkedSet(p, linkPart))
                return;

            multiJointManager.RegisterMultiJointBetweenParts(p, linkPart, KJRJointUtils.BuildJoint(p, linkPart));
        }

        public void MultiPartJointTreeChildren(Vessel v)
        {
            if(v.Parts.Count <= 1)
                return;

            Dictionary<Part, List<Part>> childPartsToConnectByRoot = new Dictionary<Part,List<Part>>();

            for(int i = 0; i < v.Parts.Count; ++i)
            {
                Part p = v.Parts[i];

                bool bEndPoint = (p.children.Count == 0);

                if(!bEndPoint && !KJRJointUtils.IsJointAdjustmentAllowed(p) && p.parent && (p.parent.vessel == v))
                {
                    p = p.parent;

                    bEndPoint = true;
                    for(int j = 0; j < p.children.Count; j++)
                    {
                        if(KJRJointUtils.IsJointAdjustmentAllowed(p.children[j]))
                        { bEndPoint = false; break; }
                    }
                }

                if(bEndPoint && !p.Modules.Contains("LaunchClamp") && KJRJointUtils.MaximumPossiblePartMass(p) > KJRJointUtils.massForAdjustment)
                {
                    if(p.rb == null && p.Rigidbody != null)
                        p = p.RigidBodyPart;

                    Part root = p;
                    while(root.parent && (root.parent.vessel == v) && KJRJointUtils.IsJointAdjustmentAllowed(root))
                        root = root.parent;

                    List<Part> childPartsToConnect;
                    if(!childPartsToConnectByRoot.TryGetValue(root, out childPartsToConnect))
                    {
                        childPartsToConnect = new List<Part>();
                        childPartsToConnectByRoot.Add(root, childPartsToConnect);
                    }

                    childPartsToConnect.Add(p);
                }
            }

            foreach(Part root in childPartsToConnectByRoot.Keys)
            {
                List<Part> childPartsToConnect = childPartsToConnectByRoot[root];

                for(int i = 0; i < childPartsToConnect.Count; ++i)
                {
                    Part p = childPartsToConnect[i];

                    if(!p.rb)
                        continue;

                    Part linkPart = childPartsToConnect[i + 1 >= childPartsToConnect.Count ? 0 : i + 1];

                    if(!linkPart.Rigidbody || p.rb == linkPart.Rigidbody)
                        continue;

                    MultiPartJointBuildJoint(p, linkPart);


                    int part2Index = i + childPartsToConnect.Count / 2;
                    if(part2Index >= childPartsToConnect.Count)
                        part2Index -= childPartsToConnect.Count;

                    Part linkPart2 = childPartsToConnect[part2Index];

                    if(!linkPart2.Rigidbody || p.rb == linkPart2.Rigidbody)
                        continue;

                    MultiPartJointBuildJoint(p, linkPart2);


                    if(!root.Rigidbody || p.rb == root.Rigidbody)
                        continue;

                    MultiPartJointBuildJoint(p, root);
                }
            }
        }
    }
}
