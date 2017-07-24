/*
Kerbal Joint Reinforcement, v3.3.3
Copyright 2015, Michael Ferrara, aka Ferram4

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
            if (!CompatibilityChecker.IsCompatible())
                return;

            GameEvents.onVesselWasModified.Add(OnVesselWasModified);
            GameEvents.onVesselGoOffRails.Add(OnVesselOffRails);
            GameEvents.onVesselGoOnRails.Add(OnVesselOnRails);
            GameEvents.onVesselDestroy.Add(OnVesselOnRails);
        }

        public void OnDestroy()
        {
            if (!CompatibilityChecker.IsCompatible())
                return;

            GameEvents.onVesselWasModified.Remove(OnVesselWasModified);
            GameEvents.onVesselGoOffRails.Remove(OnVesselOffRails);
            GameEvents.onVesselGoOnRails.Remove(OnVesselOnRails);
            GameEvents.onVesselDestroy.Remove(OnVesselOnRails);

            if (InputLockManager.GetControlLock("KJRLoadLock") == ControlTypes.ALL_SHIP_CONTROLS)
                InputLockManager.RemoveControlLock("KJRLoadLock");
            updatedVessels = null;
            vesselOffRails = null;
            vesselJointStrengthened = null;

            multiJointManager.OnDestroy();
            multiJointManager = null;
        }

        private void OnVesselWasModified(Vessel v)
        {
            if ((object)v == null || v.isEVA)
                return; 
            
            if (KJRJointUtils.debug)
            {
                StringBuilder debugString = new StringBuilder();
                debugString.AppendLine("KJR: Modified vessel " + v.id + " (" + v.GetName() + ")");
                debugString.AppendLine(System.Environment.StackTrace);
                debugString.AppendLine("Now contains: ");
                foreach (Part p in v.Parts)
                    debugString.AppendLine("  " + p.partInfo.name + " (" + p.flightID + ")");
                Debug.Log(debugString);
            }

            updatedVessels.Remove(v);
            RunVesselJointUpdateFunction(v);
        }

        private void OnVesselOffRails(Vessel v)
        {
            if ((object)v == null || v.isEVA)
                return; 
            
            RunVesselJointUpdateFunction(v);
            if (!vesselOffRails.Contains(v) && v.precalc.isEasingGravity)
            {
                Debug.Log("KJR easing " + v.vesselName);
                vesselOffRails.Add(v);
                List<Joint> jointList = new List<Joint>();
                for (int i = 0; i < v.Parts.Count; ++i)
                {
                    Part p = v.Parts[i];
                    p.crashTolerance = p.crashTolerance * 10000f;
                    if (p.attachJoint)
                        p.attachJoint.SetUnbreakable(true, false);

                    Joint[] partJoints = p.GetComponents<Joint>();

                    if (p.Modules.Contains<LaunchClamp>())
                    {
                        foreach (Joint j in partJoints)
                            if (j.connectedBody == null)
                            {
                                jointList.Remove(j);
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
            if ((object)v == null)
                return;

            if (updatedVessels.Contains(v))
            {
                if (vesselOffRails.Contains(v))
                {
                    foreach (Part p in v.Parts)
                    {
                        p.crashTolerance = p.crashTolerance / 10000;
                        if (p.attachJoint)
                            p.attachJoint.SetUnbreakable(false, false);
                    }

                    vesselOffRails.Remove(v);
                }
                vesselJointStrengthened.Remove(v);
                updatedVessels.Remove(v);
            }
        }

        private void RunVesselJointUpdateFunction(Vessel v)
        {
            if (KJRJointUtils.debug)
            {
                Debug.Log("KJR: Processing vessel " + v.id + " (" + v.GetName() + "); root " +
                            v.rootPart.partInfo.name + " (" + v.rootPart.flightID + ")");
            }

            bool child_parts = false;
            bool success = false;

            foreach (Part p in v.Parts)
            {
                if (p.parent != null && p.physicalSignificance == Part.PhysicalSignificance.FULL)
                {
                    child_parts = true;

                    if (p.attachJoint != null)
                    {
                        success = true;

                        if (KJRJointUtils.reinforceAttachNodes)
                            UpdatePartJoint(p);
                    }
                }

                if (KJRJointUtils.reinforceDecouplersFurther)
                    if (ValidDecoupler(p))
                    {
                        KJRJointUtils.AddDecouplerJointReinforcementModule(p);
                        continue;
                    }

                if (KJRJointUtils.reinforceLaunchClampsFurther)
                    if (p.Modules.Contains<LaunchClamp>() && p.parent != null)
                    {
                        p.breakingForce = Mathf.Infinity;
                        p.breakingTorque = Mathf.Infinity;
                        p.mass = Mathf.Max(p.mass, (p.parent.mass + p.parent.GetResourceMass()) * 0.01f);          //We do this to make sure that there is a mass ratio of 100:1 between the clamp and what it's connected to.  This helps counteract some of the wobbliness simply, but also allows some give and springiness to absorb the initial physics kick
                        if (KJRJointUtils.debug)
                            Debug.Log("KJR: Launch Clamp Break Force / Torque increased");

                        if (!p.Modules.Contains<KJRLaunchClampReinforcementModule>())
                            KJRJointUtils.AddLaunchClampReinforcementModule(p);
                    }
            }

            if (KJRJointUtils.reinforceAttachNodes && KJRJointUtils.multiPartAttachNodeReinforcement)
                MultiPartJointTreeChildren(v);

            if (success || !child_parts)
                updatedVessels.Add(v);
        }

        private bool ValidDecoupler(Part p)
        {
            return (p.Modules.Contains<ModuleDecouple>() || p.Modules.Contains<ModuleAnchoredDecoupler>()) && !p.Modules.Contains<KJRDecouplerReinforcementModule>();
        }

        public void FixedUpdate()
        {
            if (FlightGlobals.ready && FlightGlobals.Vessels != null)
            {
                for(int i = 0; i < updatedVessels.Count; ++i)
                {
                    Vessel v = updatedVessels[i];
                    if (v == null || !vesselOffRails.Contains(v))
                        continue;

                    if (!v.precalc.isEasingGravity)
                    {
                        foreach (Part p in v.Parts)
                        {
                            p.crashTolerance = p.crashTolerance / 10000f;
                            if (p.attachJoint)
                                p.attachJoint.SetUnbreakable(false, false);
                        }

                        vesselOffRails.Remove(v);
                    }
                }
            }
        }

        private void UpdatePartJoint(Part p)
        {
            if (!KJRJointUtils.JointAdjustmentValid(p) || p.rb == null || p.attachJoint == null)
                return;

            if (p.attachMethod == AttachNodeMethod.LOCKED_JOINT)
            {
                if (KJRJointUtils.debug)
                {
                    Debug.Log("KJR: Already processed part before: " + p.partInfo.name + " (" + p.flightID + ") -> " +
                              p.parent.partInfo.name + " (" + p.parent.flightID + ")");
                }

                return;
            }
            List<ConfigurableJoint> jointList;
            if (p.Modules.Contains<CModuleStrut>())
            {
                CModuleStrut s = p.Modules.GetModule<CModuleStrut>();

                if (!(s.jointTarget == null || s.jointRoot == null))
                {
                    jointList = KJRJointUtils.GetJointListFromAttachJoint(s.strutJoint);

                    if (jointList != null)
                    {
                        for (int i = 0; i < jointList.Count; i++)
                        {
                            ConfigurableJoint j = jointList[i];

                            if (j == null)
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
            

            jointList = KJRJointUtils.GetJointListFromAttachJoint(p.attachJoint);
            if (jointList == null)
                return;

            StringBuilder debugString = new StringBuilder();

            bool addAdditionalJointToParent = KJRJointUtils.multiPartAttachNodeReinforcement;
            //addAdditionalJointToParent &= !(p.Modules.Contains("LaunchClamp") || (p.parent.Modules.Contains("ModuleDecouple") || p.parent.Modules.Contains("ModuleAnchoredDecoupler")));
            addAdditionalJointToParent &= !p.Modules.Contains<CModuleStrut>();

            float partMass = p.mass + p.GetResourceMass();
            for (int i = 0; i < jointList.Count; i++)
            {
                ConfigurableJoint j = jointList[i];
                if (j == null)
                    continue;

                String jointType = j.GetType().Name;
                Rigidbody connectedBody = j.connectedBody;

                Part connectedPart = connectedBody.GetComponent<Part>() ?? p.parent;
                float parentMass = connectedPart.mass + connectedPart.GetResourceMass();

                if (partMass < KJRJointUtils.massForAdjustment || parentMass < KJRJointUtils.massForAdjustment)
                {
                    if (KJRJointUtils.debug)
                    {
                        Debug.Log("KJR: Part mass too low, skipping: " + p.partInfo.name + " (" + p.flightID + ")");
                    }

                    continue;
                }                
                
                // Check attachment nodes for better orientation data
                AttachNode attach = p.FindAttachNodeByPart(p.parent);
                AttachNode p_attach = p.parent.FindAttachNodeByPart(p);
                AttachNode node = attach ?? p_attach;

                if (node == null)
                {
                    // Check if it's a pair of coupled docking ports
                    var dock1 = p.Modules.GetModule<ModuleDockingNode>();
                    var dock2 = p.parent.Modules.GetModule<ModuleDockingNode>();

                    //Debug.Log(dock1 + " " + (dock1 ? ""+dock1.dockedPartUId : "?") + " " + dock2 + " " + (dock2 ? ""+dock2.dockedPartUId : "?"));

                    if (dock1 && dock2 && (dock1.dockedPartUId == p.parent.flightID || dock2.dockedPartUId == p.flightID))
                    {
                        attach = p.FindAttachNode(dock1.referenceAttachNode);
                        p_attach = p.parent.FindAttachNode(dock2.referenceAttachNode);
                        node = attach ?? p_attach;
                    }
                }

                // If still no node and apparently surface attached, use the normal one if it's there
                if (node == null && p.attachMode == AttachModes.SRF_ATTACH)
                    node = attach = p.srfAttachNode;

                if (KJRJointUtils.debug)
                {
                    debugString.AppendLine("Original joint from " + p.partInfo.title + " to " + p.parent.partInfo.title);
                    debugString.AppendLine("  " + p.partInfo.name + " (" + p.flightID + ") -> " + p.parent.partInfo.name + " (" + p.parent.flightID + ")");
                    debugString.AppendLine("");
                    debugString.AppendLine(p.partInfo.title + " Inertia Tensor: " + p.rb.inertiaTensor + " " + p.parent.partInfo.name + " Inertia Tensor: " + connectedBody.inertiaTensor);
                    debugString.AppendLine("");


                    debugString.AppendLine("Std. Joint Parameters");
                    debugString.AppendLine("Connected Body: " + p.attachJoint.Joint.connectedBody);
                    debugString.AppendLine("Attach mode: " + p.attachMode + " (was " + jointType + ")");
                    if (attach != null)
                        debugString.AppendLine("Attach node: " + attach.id + " - " + attach.nodeType + " " + attach.size);
                    if (p_attach != null)
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

                if (node != null)
                {
                    // Part that owns the node. For surface attachment,
                    // this can only be parent if docking flips hierarchy.
                    Part main = (node == attach) ? p : p.parent;

                    // Orientation and position of the node in owner's local coords
                    Vector3 ndir = node.orientation.normalized;
                    Vector3 npos = node.position + node.offset;

                    // And in the current part's local coords
                    Vector3 dir = axis = p.transform.InverseTransformDirection(main.transform.TransformDirection(ndir));

                    if (node.nodeType == AttachNode.NodeType.Surface)
                    {
                        // Guessed main axis; for parts with stack nodes should be the axis of the stack
                        Vector3 up = KJRJointUtils.GuessUpVector(main).normalized;

                        // if guessed up direction is same as node direction, it's basically stack
                        // for instance, consider a radially-attached docking port
                        if (Mathf.Abs(Vector3.Dot(up, ndir)) > 0.9f)
                        {
                            radius = Mathf.Min(KJRJointUtils.CalculateRadius(main, ndir), KJRJointUtils.CalculateRadius(connectedPart, ndir));
                            if (radius <= 0.001)
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
                            if (size1.y * width1 > size2.y * width2)
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
                        if (radius <= 0.001)
                            radius = node.size * 1.25f;
                        area = Mathf.PI * radius * radius;                      //Area of cylinder
                        momentOfInertia = area * radius * radius / 4;           //Moment of Inertia of cylinder
                    }
                }
                //Assume part is attached along its "up" cross section; use a cylinder to approximate properties
                else if (p.attachMode == AttachModes.STACK)
                {
                    radius = Mathf.Min(KJRJointUtils.CalculateRadius(p, Vector3.up), KJRJointUtils.CalculateRadius(connectedPart, Vector3.up));
                    if (radius <= 0.001)
                        radius = node.size * 1.25f;
                    area = Mathf.PI * radius * radius;                      //Area of cylinder
                    momentOfInertia = area * radius * radius / 4;           //Moment of Inertia of cylinder
                }
                else if (p.attachMode == AttachModes.SRF_ATTACH)
                {                    
                    // x,z sides, y along main axis
                    Vector3 up1 = KJRJointUtils.GuessUpVector(p);
                    var size1 = KJRJointUtils.CalculateExtents(p, up1);

                    Vector3 up2 = KJRJointUtils.GuessUpVector(connectedPart);
                    var size2 = KJRJointUtils.CalculateExtents(connectedPart, up2);

                    // use average of the sides, since we don't know which one is used for attaching
                    float width1 = (size1.x + size1.z) / 2;
                    float width2 = (size2.x + size2.z) / 2;
                    if (size1.y * width1 > size2.y * width2)
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

                if (KJRJointUtils.useVolumeNotArea)                //If using volume, raise al stiffness-affecting parameters to the 1.5 power
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

                if (moi_avg * 2f / drive.positionDamper < 0.08f)
                {
                    drive.positionDamper = moi_avg / (0.04f);

                    drive.positionSpring = drive.positionDamper * drive.positionDamper / moi_avg;
                }*/
                j.angularXDrive = j.angularYZDrive = j.slerpDrive = angDrive;

                JointDrive linDrive = j.xDrive;
                linDrive.maximumForce = breakForce;
                j.xDrive = j.yDrive = j.zDrive = linDrive;

                SoftJointLimit lim = new SoftJointLimit();
                lim.limit = 0;
                lim.bounciness = 0;

                SoftJointLimitSpring limSpring = new SoftJointLimitSpring();

                limSpring.spring = 0;
                limSpring.damper = 0;

                j.linearLimit = j.angularYLimit = j.angularZLimit = j.lowAngularXLimit = j.highAngularXLimit = lim;
                j.linearLimitSpring = j.angularYZLimitSpring = j.angularXLimitSpring = limSpring;

                j.targetAngularVelocity = Vector3.zero;
                j.targetVelocity = Vector3.zero;
                j.targetRotation = Quaternion.identity;
                j.targetPosition = Vector3.zero;

                j.breakForce = breakForce;
                j.breakTorque = breakTorque;
                p.attachJoint.SetBreakingForces(j.breakForce, j.breakTorque);

                p.attachMethod = AttachNodeMethod.LOCKED_JOINT;

                if (addAdditionalJointToParent && p.parent.parent != null)
                {
                    addAdditionalJointToParent = false;
                    if (!KJRJointUtils.JointAdjustmentValid(p.parent) || !KJRJointUtils.JointAdjustmentValid(p.parent.parent))
                        continue;

                    /*if (ValidDecoupler(p) || ValidDecoupler(p.parent))
                        continue;*/
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

                    do
                    {
                        float massRat1, massRat2;
                        massRat1 = partMaxMass / newConnectedPart.mass;
                        if (massRat1 < 1)
                            massRat1 = 1 / massRat1;

                        massRat2 = p.mass / KJRJointUtils.MaximumPossiblePartMass(newConnectedPart);
                        if (massRat2 < 1)
                            massRat2 = 1 / massRat2;

                        if (massRat1 > KJRJointUtils.stiffeningExtensionMassRatioThreshold || massRat2 > KJRJointUtils.stiffeningExtensionMassRatioThreshold)
                        {
                            if (newConnectedPart.parent != null)
                            {
                                if (!KJRJointUtils.JointAdjustmentValid(newConnectedPart.parent))
                                    break;

                                newConnectedPart = newConnectedPart.parent;
                                if (newConnectedPart.rb == null)
                                    possiblePartsCrossed.Add(newConnectedPart);
                                else
                                {
                                    connectedRb = newConnectedPart.rb;
                                    partsCrossed.AddRange(possiblePartsCrossed);
                                    partsCrossed.Add(newConnectedPart);
                                    possiblePartsCrossed.Clear();
                                }
                            }
                            else
                                break;
                            numPartsFurther++;
                        }
                        else
                            massRatioBelowThreshold = true;
                    } while (!massRatioBelowThreshold);// && numPartsFurther < 5);

                    if (connectedRb != null && !multiJointManager.CheckMultiJointBetweenParts(p, newConnectedPart))
                    {

                        ConfigurableJoint newJoint = p.gameObject.AddComponent<ConfigurableJoint>();

                        newJoint.connectedBody = connectedRb;
                        newJoint.axis = Vector3.right;
                        newJoint.secondaryAxis = Vector3.forward;
                        newJoint.anchor = Vector3.zero;
                        newJoint.connectedAnchor = p.transform.worldToLocalMatrix.MultiplyPoint(newConnectedPart.transform.position);

                        //if(massRatioBelowThreshold)
                        //{

                        newJoint.angularXDrive = newJoint.angularYZDrive = newJoint.slerpDrive = j.angularXDrive;

                        newJoint.xDrive = j.xDrive;
                        newJoint.yDrive = j.yDrive;
                        newJoint.zDrive = j.zDrive;

                        newJoint.linearLimit = newJoint.angularYLimit = newJoint.angularZLimit = newJoint.lowAngularXLimit = newJoint.highAngularXLimit = lim;

                        /*newJoint.targetAngularVelocity = Vector3.zero;
                        newJoint.targetVelocity = Vector3.zero;
                        newJoint.targetRotation = Quaternion.identity;
                        newJoint.targetPosition = Vector3.zero;*/
                        /*}
                        else
                        {
                            newJoint.xMotion = newJoint.yMotion = newJoint.zMotion = ConfigurableJointMotion.Locked;
                            newJoint.angularXMotion = newJoint.angularYMotion = newJoint.angularZMotion = ConfigurableJointMotion.Locked;
                        }*/

                        newJoint.breakForce = breakForce;
                        newJoint.breakTorque = breakTorque;

                        //jointList.Add(newJoint);
                        for (int k = 0; k < partsCrossed.Count; k++)
                            multiJointManager.RegisterMultiJoint(partsCrossed[k], newJoint);
                    }

                    /*if(p.symmetryCounterparts != null && p.symmetryCounterparts.Count > 0)
                    {
                        Part linkPart = null;
                        Vector3 center = p.transform.position;
                        float cross = float.NegativeInfinity;
                        for(int k = 0; k < p.symmetryCounterparts.Count; k++)
                        {
                            center += p.symmetryCounterparts[k].transform.position;
                        }
                        center /= (p.symmetryCounterparts.Count + 1);

                        for(int k = 0; k < p.symmetryCounterparts.Count; k++)
                        {
                            Part counterPart = p.symmetryCounterparts[k];
                            if (counterPart.parent == p.parent && counterPart.rb != null)
                            {
                                float tmpCross = Vector3.Dot(Vector3.Cross(center - p.transform.position, counterPart.transform.position - p.transform.position), p.transform.up);
                                if(tmpCross > cross)
                                {
                                    cross = tmpCross;
                                    linkPart = counterPart;
                                }
                            }
                        }
                        if (linkPart)
                        {
                            Rigidbody rigidBody = linkPart.rb;
                            if (!linkPart.rb)
                                continue;
                            ConfigurableJoint newJoint;

                            newJoint = p.gameObject.AddComponent<ConfigurableJoint>();

                            newJoint.connectedBody = rigidBody;
                            newJoint.anchor = Vector3.zero;
                            newJoint.axis = Vector3.right;
                            newJoint.secondaryAxis = Vector3.forward;
                            newJoint.breakForce = KJRJointUtils.decouplerAndClampJointStrength;
                            newJoint.breakTorque = KJRJointUtils.decouplerAndClampJointStrength;

                            newJoint.xMotion = newJoint.yMotion = newJoint.zMotion = ConfigurableJointMotion.Locked;
                            newJoint.angularXMotion = newJoint.angularYMotion = newJoint.angularZMotion = ConfigurableJointMotion.Locked;

                            multiJointManager.RegisterMultiJoint(p, newJoint);
                            multiJointManager.RegisterMultiJoint(linkPart, newJoint);
                        }
                    }*/
                }

                if (KJRJointUtils.debug)
                {
                    debugString.AppendLine("Updated joint from " + p.partInfo.title + " to " + p.parent.partInfo.title);
                    debugString.AppendLine("  " + p.partInfo.name + " (" + p.flightID + ") -> " + p.parent.partInfo.name + " (" + p.parent.flightID + ")");
                    debugString.AppendLine("");
                    debugString.AppendLine(p.partInfo.title + " Inertia Tensor: " + p.rb.inertiaTensor + " " + p.parent.partInfo.name + " Inertia Tensor: " + connectedBody.inertiaTensor);
                    debugString.AppendLine("");


                    debugString.AppendLine("Std. Joint Parameters");
                    debugString.AppendLine("Connected Body: " + p.attachJoint.Joint.connectedBody);
                    debugString.AppendLine("Attach mode: " + p.attachMode + " (was " + jointType + ")");
                    if (attach != null)
                        debugString.AppendLine("Attach node: " + attach.id + " - " + attach.nodeType + " " + attach.size);
                    if (p_attach != null)
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
            if (KJRJointUtils.debug)
                Debug.Log(debugString.ToString());
        }

        public void MultiPartJointTreeChildren(Vessel v)
        {
            if (v.Parts.Count <= 1)
                return;

            List<Part> childPartsToConnect = new List<Part>();

            for(int i = 0; i < v.Parts.Count; ++i)
            {
                Part p = v.Parts[i];
                if (p.children.Count == 0 && !p.Modules.Contains("LaunchClamp") && KJRJointUtils.MaximumPossiblePartMass(p) > KJRJointUtils.massForAdjustment)
                {
                    if(p.rb == null && p.Rigidbody != null)
                    {
                        p = p.RigidBodyPart;
                    }
                    childPartsToConnect.Add(p);
                }
            }


            Rigidbody rootRb = v.rootPart.Rigidbody;

            for(int i = 0; i < childPartsToConnect.Count; ++i)
            {
                Part p = childPartsToConnect[i];
                Part linkPart = childPartsToConnect[i + 1 >= childPartsToConnect.Count ? 0 : i + 1];

                Rigidbody rigidBody = linkPart.Rigidbody;
                if (!p.rb || !rigidBody || p.rb == rigidBody)
                    continue;

                if (!multiJointManager.CheckMultiJointBetweenParts(p, linkPart) && multiJointManager.TrySetValidLinkedSet(p, linkPart))
                {
                    ConfigurableJoint betweenChildJoint;

                    betweenChildJoint = p.gameObject.AddComponent<ConfigurableJoint>();

                    betweenChildJoint.connectedBody = rigidBody;
                    betweenChildJoint.anchor = Vector3.zero;
                    betweenChildJoint.axis = Vector3.right;
                    betweenChildJoint.secondaryAxis = Vector3.forward;
                    betweenChildJoint.breakForce = KJRJointUtils.decouplerAndClampJointStrength;
                    betweenChildJoint.breakTorque = KJRJointUtils.decouplerAndClampJointStrength;

                    betweenChildJoint.xMotion = betweenChildJoint.yMotion = betweenChildJoint.zMotion = ConfigurableJointMotion.Locked;
                    betweenChildJoint.angularXMotion = betweenChildJoint.angularYMotion = betweenChildJoint.angularZMotion = ConfigurableJointMotion.Locked;

                    multiJointManager.RegisterMultiJointBetweenParts(p, linkPart, betweenChildJoint);
                    //multiJointManager.RegisterMultiJoint(p, betweenChildJoint);
                    //multiJointManager.RegisterMultiJoint(linkPart, betweenChildJoint);
                }

                Part linkPart2;

                int part2Index = i + childPartsToConnect.Count / 2;
                if (part2Index >= childPartsToConnect.Count)
                    part2Index -= childPartsToConnect.Count;

                linkPart2 = childPartsToConnect[part2Index];
                rigidBody = linkPart2.Rigidbody;

                if (!p.rb || !rigidBody || p.rb == rigidBody)
                    continue;

                if (!multiJointManager.CheckMultiJointBetweenParts(p, linkPart2) && multiJointManager.TrySetValidLinkedSet(p, linkPart2))
                {
                    ConfigurableJoint betweenChildJoint2;

                    betweenChildJoint2 = p.gameObject.AddComponent<ConfigurableJoint>();

                    betweenChildJoint2.connectedBody = rigidBody;
                    betweenChildJoint2.anchor = Vector3.zero;
                    betweenChildJoint2.axis = Vector3.right;
                    betweenChildJoint2.secondaryAxis = Vector3.forward;
                    betweenChildJoint2.breakForce = KJRJointUtils.decouplerAndClampJointStrength;
                    betweenChildJoint2.breakTorque = KJRJointUtils.decouplerAndClampJointStrength;

                    betweenChildJoint2.xMotion = betweenChildJoint2.yMotion = betweenChildJoint2.zMotion = ConfigurableJointMotion.Locked;
                    betweenChildJoint2.angularXMotion = betweenChildJoint2.angularYMotion = betweenChildJoint2.angularZMotion = ConfigurableJointMotion.Locked;

                    multiJointManager.RegisterMultiJointBetweenParts(p, linkPart2, betweenChildJoint2);
                    //multiJointManager.RegisterMultiJoint(p, betweenChildJoint2);
                    //multiJointManager.RegisterMultiJoint(linkPart2, betweenChildJoint2);
                }


                if (!rootRb || p.rb == rootRb)
                    continue;

                if (multiJointManager.CheckMultiJointBetweenParts(p, v.rootPart) && multiJointManager.TrySetValidLinkedSet(p, v.rootPart))
                {

                    ConfigurableJoint toRootJoint;

                    toRootJoint = p.gameObject.AddComponent<ConfigurableJoint>();

                    toRootJoint.connectedBody = rootRb;
                    toRootJoint.anchor = Vector3.zero;
                    toRootJoint.axis = Vector3.right;
                    toRootJoint.secondaryAxis = Vector3.forward;
                    toRootJoint.breakForce = KJRJointUtils.decouplerAndClampJointStrength;
                    toRootJoint.breakTorque = KJRJointUtils.decouplerAndClampJointStrength;

                    toRootJoint.xMotion = toRootJoint.yMotion = toRootJoint.zMotion = ConfigurableJointMotion.Locked;
                    toRootJoint.angularXMotion = toRootJoint.angularYMotion = toRootJoint.angularZMotion = ConfigurableJointMotion.Locked;

                    multiJointManager.RegisterMultiJointBetweenParts(p, v.rootPart, toRootJoint);
                    //multiJointManager.RegisterMultiJoint(p, toRootJoint);
                    //multiJointManager.RegisterMultiJoint(v.rootPart, toRootJoint);
                }
            }
        }
    }
}