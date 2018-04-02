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

namespace KerbalJointReinforcement
{
    //This class adds extra joints between the parts connected to a decoupler to stiffen up the connection
    public class KJRDecouplerReinforcementModule : PartModule
    {
        protected List<ConfigurableJoint> joints = new List<ConfigurableJoint>();
        protected List<Part> neighbours = new List<Part>();
        private PartModule decoupler = null;
        //private bool radiallyAttached = false;

        public override void OnAwake()
        {
            base.OnAwake();

            foreach (PartModule m in part.Modules)
                if (m is ModuleDecouple)
                {
                    decoupler = m;
                    break;
                }
                else if (m is ModuleAnchoredDecoupler)
                {
                    decoupler = m;
                    break;
                }
        }

        public override void OnStart(PartModule.StartState state)
        {
            base.OnStart(state);

            if (part.parent == null)
                return;

            //if (part.attachMode == AttachModes.SRF_ATTACH)
            //    radiallyAttached = true;
        }


        public void OnPartUnpack()
        {
            if (part.parent == null || part.children.Count == 0)
                return;
            if (decoupler is ModuleDecouple && (decoupler as ModuleDecouple).isDecoupled)
                return;
            if (decoupler is ModuleAnchoredDecoupler && (decoupler as ModuleAnchoredDecoupler).isDecoupled)
                return;

            AddExtraJoints();
        }

        private void OnVesselWasModified(Vessel v)
        {
            foreach (Part p in neighbours)
            {
                if (p.vessel == part.vessel)
                    continue;

                if (KJRJointUtils.debug)
                    Debug.Log("Decoupling part " + part.partInfo.title + "; destroying all extra joints");

                BreakAllInvalidJointsAndRebuild();
                break;
            }
        }

        private void AddExtraJoints()
        {
            List<Part> childParts = new List<Part>();
            List<Part> parentParts = new List<Part>();

            parentParts = KJRJointUtils.DecouplerPartStiffeningList(part.parent, false, true);
            foreach (Part p in part.children)
            {
                childParts.AddRange(KJRJointUtils.DecouplerPartStiffeningList(p, true, true));
                if (!childParts.Contains(p))
                    childParts.Add(p);
            }

            neighbours.Clear();
            neighbours.AddRange(parentParts);
            neighbours.AddRange(childParts);
            neighbours.Remove(part);

            parentParts.Add(part);

            StringBuilder debugString = null;

            if (KJRJointUtils.debug)
            {
                debugString = new StringBuilder();
                debugString.AppendLine(parentParts.Count + " parts above decoupler to be connected to " + childParts.Count + " below decoupler.");
                debugString.AppendLine("The following joints added by " + part.partInfo.title + " to increase stiffness:");
            }

            foreach (Part p in parentParts)
            {
                if (p == null || p.rb == null || p.Modules.Contains("ProceduralFairingDecoupler") || !KJRJointUtils.JointAdjustmentValid(p))
                    continue;
                foreach (Part q in childParts)
                {
                    if (q == null || q.rb == null || q.Modules.Contains("ProceduralFairingDecoupler") || p == q || !KJRJointUtils.JointAdjustmentValid(q))
                        continue;

                    StrutConnectParts(p, q);

                    if (KJRJointUtils.debug)
                        debugString.AppendLine(p.partInfo.title + " connected to part " + q.partInfo.title);
                }
            }

            if (joints.Count > 0)
            {
                GameEvents.onVesselWasModified.Add(OnVesselWasModified);
                GameEvents.onVesselCreate.Add(OnVesselWasModified);
            }

            if (KJRJointUtils.debug)
                Debug.Log(debugString.ToString());
        }

        public void OnPartPack()
        {
            if (joints.Count > 0)
            {
                GameEvents.onVesselWasModified.Remove(OnVesselWasModified);
                GameEvents.onVesselCreate.Remove(OnVesselWasModified);
            }

            foreach (ConfigurableJoint j in joints)
                GameObject.Destroy(j);

            joints.Clear();
            neighbours.Clear();
        }

        public void OnDestroy()
        {
            if (joints.Count > 0)
            {
                GameEvents.onVesselWasModified.Remove(OnVesselWasModified);
                GameEvents.onVesselCreate.Remove(OnVesselWasModified);
            }
        }

        private void BreakAllInvalidJointsAndRebuild()
        {
            GameEvents.onVesselWasModified.Remove(OnVesselWasModified);
            GameEvents.onVesselCreate.Remove(OnVesselWasModified);

            foreach (ConfigurableJoint j in joints)
                GameObject.Destroy(j);

            joints.Clear();

            var vessels = new List<Vessel>();

            foreach (Part n in neighbours)
            {
                if (n.vessel == null || vessels.Contains(n.vessel))
                    continue;

                vessels.Add(n.vessel);

                foreach (Part p in n.vessel.Parts)
                {
                    if (p.Modules.Contains<LaunchClamp>())
                        continue;

                    ConfigurableJoint[] possibleConnections = p.GetComponents<ConfigurableJoint>();

                    if (possibleConnections == null)
                        continue;

                    foreach (ConfigurableJoint j in possibleConnections)
                    {
                        if (j.connectedBody == null)
                        {
                            GameObject.Destroy(j);
                            continue;
                        }
                        Part cp = j.connectedBody.GetComponent<Part>();
                        if (cp != null && cp.vessel != p.vessel)
                            GameObject.Destroy(j);
                    }
                }
            }
            neighbours.Clear();

            if (part.parent == null || part.children.Count == 0)
                return;
            if (decoupler is ModuleDecouple && (decoupler as ModuleDecouple).isDecoupled)
                return;
            if (decoupler is ModuleAnchoredDecoupler && (decoupler as ModuleAnchoredDecoupler).isDecoupled)
                return;

            AddExtraJoints();
        }

        private void StrutConnectParts(Part partWithJoint, Part partConnectedByJoint)
        {
            Rigidbody rigidBody = partConnectedByJoint.rb;
            float breakForce = KJRJointUtils.decouplerAndClampJointStrength;
            float breakTorque = KJRJointUtils.decouplerAndClampJointStrength;
            Vector3 anchor, axis;

            anchor = Vector3.zero;
            axis = Vector3.right;

            ConfigurableJoint newJoint;

            newJoint = partWithJoint.gameObject.AddComponent<ConfigurableJoint>();

            newJoint.connectedBody = rigidBody;
            newJoint.anchor = anchor;
            newJoint.connectedAnchor = partWithJoint.transform.worldToLocalMatrix.MultiplyPoint(partConnectedByJoint.transform.position);
            newJoint.axis = axis;
            newJoint.secondaryAxis = Vector3.forward;
            newJoint.breakForce = breakForce;
            newJoint.breakTorque = breakTorque;

            newJoint.xMotion = newJoint.yMotion = newJoint.zMotion = ConfigurableJointMotion.Locked;
            newJoint.angularXMotion = newJoint.angularYMotion = newJoint.angularZMotion = ConfigurableJointMotion.Locked;


            newJoint.breakForce = breakForce;
            newJoint.breakTorque = breakTorque;

            joints.Add(newJoint);

        }
    }

}
